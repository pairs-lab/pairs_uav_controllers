/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>

#include <common.h>

#include <pairs_uav_managers/controller.h>

#include <pairs_lib/profiler.h>
#include <pairs_lib/attitude_converter.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/subscribe_handler.h>

//}

namespace pairs_uav_controllers
{

namespace failsafe_controller
{

/* class FailsafeController //{ */

class FailsafeController : public pairs_uav_managers::Controller {

public:
  bool initialize(const ros::NodeHandle &nh, std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                  std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers);

  bool activate(const ControlOutput &last_control_output);
  void deactivate(void);

  void updateInactive(const pairs_msgs::UavState &uav_state, const std::optional<pairs_msgs::TrackerCommand> &tracker_command);

  ControlOutput updateActive(const pairs_msgs::UavState &uav_state, const pairs_msgs::TrackerCommand &tracker_command);

  const pairs_msgs::ControllerStatus getStatus();

  void switchOdometrySource(const pairs_msgs::UavState &new_uav_state);

  void resetDisturbanceEstimators(void);

  const pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr setConstraints(const pairs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &cmd);

  double getHeadingSafely(const geometry_msgs::Quaternion &quaternion);

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t>  common_handlers_;
  std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers_;

  // | ----------------------- parameters ----------------------- |

  double _descend_speed_;
  double _descend_acceleration_;

  double _kq_;
  double _kw_;

  // | ------------------- remember uav state ------------------- |

  pairs_msgs::UavState uav_state_;
  std::mutex         mutex_uav_state_;

  // | --------------------- throttle control --------------------- |

  double _uav_mass_;
  double uav_mass_difference_;

  double hover_throttle_;

  double _throttle_decrease_rate_;
  double _initial_throttle_percentage_;

  // | ----------------------- yaw control ---------------------- |

  double heading_setpoint_;

  pairs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_hw_api_orientation_;

  // | ------------------ activation and output ----------------- |

  ControlOutput last_control_output_;
  ControlOutput activation_control_output_;

  ros::Time         last_update_time_;
  std::atomic<bool> first_iteration_ = true;

  // | ------------------------ profiler ------------------------ |

  pairs_lib::Profiler profiler_;
  bool              _profiler_enabled_ = false;

  // | ----------------------- constraints ---------------------- |

  pairs_msgs::DynamicsConstraints constraints_;
  std::mutex                    mutex_constraints_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

bool FailsafeController::initialize(const ros::NodeHandle &nh, std::shared_ptr<pairs_uav_managers::control_manager::CommonHandlers_t> common_handlers,
                                    std::shared_ptr<pairs_uav_managers::control_manager::PrivateHandlers_t> private_handlers) {

  nh_ = nh;

  common_handlers_  = common_handlers;
  private_handlers_ = private_handlers;

  _uav_mass_ = common_handlers->getMass();

  ros::Time::waitForValid();

  // | ---------- loading params using the parent's nh ---------- |

  pairs_lib::ParamLoader param_loader_parent(common_handlers->parent_nh, "ControlManager");

  param_loader_parent.loadParam("enable_profiler", _profiler_enabled_);

  if (!param_loader_parent.loadedSuccessfully()) {
    ROS_ERROR("[FailsafeController]: Could not load all parameters!");
    return false;
  }

  // | -------------------- loading my params ------------------- |

  private_handlers->param_loader->addYamlFile(ros::package::getPath("pairs_uav_controllers") + "/config/private/failsafe_controller.yaml");
  private_handlers->param_loader->addYamlFile(ros::package::getPath("pairs_uav_controllers") + "/config/public/failsafe_controller.yaml");

  const std::string yaml_namespace = "pairs_uav_controllers/failsafe_controller/";

  private_handlers->param_loader->loadParam(yaml_namespace + "throttle_output/throttle_decrease_rate", _throttle_decrease_rate_);
  private_handlers->param_loader->loadParam(yaml_namespace + "throttle_output/initial_throttle_percentage", _initial_throttle_percentage_);

  private_handlers->param_loader->loadParam(yaml_namespace + "attitude_controller/gains/kp", _kq_);

  private_handlers->param_loader->loadParam(yaml_namespace + "rate_controller/gains/kp", _kw_);

  private_handlers->param_loader->loadParam(yaml_namespace + "velocity_output/descend_speed", _descend_speed_);
  private_handlers->param_loader->loadParam(yaml_namespace + "acceleration_output/descend_acceleration", _descend_acceleration_);

  if (!private_handlers->param_loader->loadedSuccessfully()) {
    ROS_ERROR("[FailsafeController]: Could not load all parameters!");
    return false;
  }

  _descend_speed_        = std::abs(_descend_speed_);
  _descend_acceleration_ = std::abs(_descend_acceleration_);

  uav_mass_difference_ = 0;

  // | ----------------------- subscribers ---------------------- |

  pairs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "FailsafeController";
  shopts.no_message_timeout = pairs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_hw_api_orientation_ = pairs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, "/" + common_handlers->uav_name + "/" + "hw_api/orientation");

  // | ----------- calculate the default hover throttle ----------- |

  hover_throttle_ = pairs_lib::quadratic_throttle_model::forceToThrottle(common_handlers_->throttle_model, _uav_mass_ * common_handlers_->g);

  // | ------------------------ profiler ------------------------ |

  profiler_ = pairs_lib::Profiler(common_handlers->parent_nh, "FailsafeController", _profiler_enabled_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[FailsafeController]: initialized");

  is_initialized_ = true;

  return true;
}

//}

/* activate() //{ */

bool FailsafeController::activate(const ControlOutput &last_control_output) {

  auto uav_state = pairs_lib::get_mutexed(mutex_uav_state_, uav_state_);

  if (!last_control_output.control_output) {

    ROS_WARN("[FailsafeController]: activated without getting the last controller's command");

    return false;

  } else {

    // | -------------- calculate the initial heading ------------- |

    if (sh_hw_api_orientation_.getMsg()) {

      auto hw_api_orientation = sh_hw_api_orientation_.getMsg();

      heading_setpoint_ = getHeadingSafely(hw_api_orientation->quaternion);

      ROS_INFO("[FailsafeController]: activated with heading = %.2f rad", heading_setpoint_);

    } else {

      ROS_ERROR("[FailsafeController]: missing orientation from HW API, activated with heading = 0 rad");

      heading_setpoint_ = 0;
    }

    activation_control_output_ = last_control_output;

    if (last_control_output.diagnostics.mass_estimator) {
      uav_mass_difference_ = last_control_output.diagnostics.mass_difference;
    } else {
      uav_mass_difference_ = 0;
    }

    activation_control_output_.diagnostics.controller_enforcing_constraints = false;

    hover_throttle_ = _initial_throttle_percentage_ * pairs_lib::quadratic_throttle_model::forceToThrottle(
                                                          common_handlers_->throttle_model, (_uav_mass_ + uav_mass_difference_) * common_handlers_->g);

    ROS_INFO("[FailsafeController]: activated with uav_mass_difference %.2f kg.", uav_mass_difference_);
  }

  first_iteration_ = true;

  is_active_ = true;

  return true;
}

//}

/* deactivate() //{ */

void FailsafeController::deactivate(void) {

  is_active_           = false;
  first_iteration_     = false;
  uav_mass_difference_ = 0;

  ROS_INFO("[FailsafeController]: deactivated");
}

//}

/* updateInactive() //{ */

void FailsafeController::updateInactive(const pairs_msgs::UavState &uav_state, [[maybe_unused]] const std::optional<pairs_msgs::TrackerCommand> &tracker_command) {

  pairs_lib::set_mutexed(mutex_uav_state_, uav_state, uav_state_);

  last_update_time_ = ros::Time::now();

  first_iteration_ = false;
}

//}

/* //{ updateWhenAcctive() */

FailsafeController::ControlOutput FailsafeController::updateActive(const pairs_msgs::UavState &                       uav_state,
                                                                   [[maybe_unused]] const pairs_msgs::TrackerCommand &tracker_command) {

  pairs_lib::Routine    profiler_routine = profiler_.createRoutine("update");
  pairs_lib::ScopeTimer timer = pairs_lib::ScopeTimer("FailsafeController::update", common_handlers_->scope_timer.logger, common_handlers_->scope_timer.enabled);

  {
    std::scoped_lock lock(mutex_uav_state_);

    uav_state_ = uav_state;
  }

  if (!is_active_) {
    return ControlOutput();
  }

  auto constraints = pairs_lib::get_mutexed(mutex_constraints_, constraints_);

  // | -------------------- calculate the dt -------------------- |

  double dt;

  if (first_iteration_) {
    dt               = 0.01;
    first_iteration_ = false;
  } else {
    dt = (ros::Time::now() - last_update_time_).toSec();
  }

  last_update_time_ = ros::Time::now();

  hover_throttle_ -= _throttle_decrease_rate_ * dt;

  if (!std::isfinite(hover_throttle_)) {
    hover_throttle_ = 0;
    ROS_ERROR("[FailsafeController]: NaN detected in variable 'hover_throttle', setting it to 0 and returning!!!");
  } else if (hover_throttle_ > 1.0) {
    hover_throttle_ = 1.0;
  } else if (hover_throttle_ < 0.0) {
    hover_throttle_ = 0.0;
  }

  // | --------------- prepare the control output --------------- |

  FailsafeController::ControlOutput control_output;

  control_output.diagnostics.controller = "FailsafeController";

  auto highest_modality = common::getHighestOuput(common_handlers_->control_output_modalities);

  if (!highest_modality) {
    ROS_ERROR_THROTTLE(1.0, "[FailsafeController]: output modalities are empty! This error should never appear.");
    return control_output;
  }

  control_output.diagnostics.controller = "FailsafeController";

  // --------------------------------------------------------------
  // |                       position output                      |
  // --------------------------------------------------------------

  if (highest_modality.value() == common::POSITION) {
    ROS_INFO_THROTTLE(1.0, "[FailsafeController]: returning empty command, because we are at the position modality");
    return control_output;
  }

  // --------------------------------------------------------------
  // |                       velocity output                      |
  // --------------------------------------------------------------

  if (highest_modality.value() == common::VELOCITY_HDG) {

    pairs_msgs::HwApiVelocityHdgCmd vel_cmd;

    vel_cmd.header.stamp = ros::Time::now();

    vel_cmd.velocity.x = 0;
    vel_cmd.velocity.y = 0;
    vel_cmd.velocity.z = -_descend_speed_;
    vel_cmd.heading    = heading_setpoint_;

    ROS_DEBUG_THROTTLE(1.0, "[FailsafeController]: returning velocity+hdg output");
    control_output.control_output = vel_cmd;
    return control_output;
  }

  if (highest_modality.value() == common::VELOCITY_HDG_RATE) {

    pairs_msgs::HwApiVelocityHdgRateCmd vel_cmd;

    vel_cmd.header.stamp = ros::Time::now();

    vel_cmd.velocity.x   = 0;
    vel_cmd.velocity.y   = 0;
    vel_cmd.velocity.z   = -_descend_speed_;
    vel_cmd.heading_rate = 0.0;

    ROS_DEBUG_THROTTLE(1.0, "[FailsafeController]: returning velocity+hdg rate output");
    control_output.control_output = vel_cmd;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                     acceleration output                    |
  // --------------------------------------------------------------

  if (highest_modality.value() == common::ACCELERATION_HDG) {

    pairs_msgs::HwApiAccelerationHdgCmd acc_cmd;

    acc_cmd.header.stamp = ros::Time::now();

    acc_cmd.acceleration.x = 0;
    acc_cmd.acceleration.y = 0;
    acc_cmd.acceleration.z = -_descend_acceleration_;
    acc_cmd.heading        = heading_setpoint_;

    ROS_DEBUG_THROTTLE(1.0, "[FailsafeController]: returning acceleration+hdg output");
    control_output.control_output = acc_cmd;
    return control_output;
  }

  if (highest_modality.value() == common::ACCELERATION_HDG_RATE) {

    pairs_msgs::HwApiAccelerationHdgRateCmd acc_cmd;

    acc_cmd.header.stamp = ros::Time::now();

    acc_cmd.acceleration.x = 0;
    acc_cmd.acceleration.y = 0;
    acc_cmd.acceleration.z = -_descend_acceleration_;
    acc_cmd.heading_rate   = 0.0;

    ROS_DEBUG_THROTTLE(1.0, "[FailsafeController]: returning acceleration+hdg rate output");
    control_output.control_output = acc_cmd;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                       attitude output                      |
  // --------------------------------------------------------------

  pairs_msgs::HwApiAttitudeCmd attitude_cmd;

  attitude_cmd.stamp       = ros::Time::now();
  attitude_cmd.orientation = pairs_lib::AttitudeConverter(0, 0, heading_setpoint_);
  attitude_cmd.throttle    = hover_throttle_;

  if (highest_modality.value() == common::ATTITUDE) {
    ROS_DEBUG_THROTTLE(1.0, "[FailsafeController]: returning attitude output");
    control_output.control_output = attitude_cmd;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                      attitude control                      |
  // --------------------------------------------------------------

  Eigen::Vector3d attitude_rate_saturation(constraints.roll_rate, constraints.pitch_rate, constraints.yaw_rate);
  Eigen::Vector3d rate_ff(0, 0, 0);
  Eigen::Vector3d Kq(_kq_, _kq_, _kq_);

  auto attitude_rate_command = common::attitudeController(uav_state, attitude_cmd, rate_ff, attitude_rate_saturation, Kq, false);

  if (highest_modality.value() == common::ATTITUDE_RATE) {
    ROS_DEBUG_THROTTLE(1.0, "[FailsafeController]: returning attitude rate output");
    control_output.control_output = attitude_rate_command;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                    attitude rate control                   |
  // --------------------------------------------------------------

  Eigen::Vector3d Kw = common_handlers_->detailed_model_params->inertia.diagonal() * _kw_;

  auto control_group_command = common::attitudeRateController(uav_state, attitude_rate_command.value(), Kw);

  if (highest_modality.value() == common::CONTROL_GROUP) {
    ROS_DEBUG_THROTTLE(1.0, "[FailsafeController]: returning control group output");
    control_output.control_output = control_group_command;
    return control_output;
  }

  // --------------------------------------------------------------
  // |                            mixer                           |
  // --------------------------------------------------------------

  pairs_msgs::HwApiActuatorCmd actuator_cmd = common::actuatorMixer(control_group_command.value(), common_handlers_->detailed_model_params->control_group_mixer);

  ROS_DEBUG_THROTTLE(1.0, "[FailsafeController]: returning actuators output");
  control_output.control_output = actuator_cmd;
  return control_output;
}

//}

/* getStatus() //{ */

const pairs_msgs::ControllerStatus FailsafeController::getStatus() {

  pairs_msgs::ControllerStatus controller_status;

  controller_status.active = is_active_;

  return controller_status;
}

//}

/* switchOdometrySource() //{ */

void FailsafeController::switchOdometrySource([[maybe_unused]] const pairs_msgs::UavState &new_uav_state) {
}

//}

/* resetDisturbanceEstimators() //{ */

void FailsafeController::resetDisturbanceEstimators(void) {
}

//}

/* setConstraints() //{ */

const pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr FailsafeController::setConstraints([
    [maybe_unused]] const pairs_msgs::DynamicsConstraintsSrvRequest::ConstPtr &constraints) {

  if (!is_initialized_) {
    return pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new pairs_msgs::DynamicsConstraintsSrvResponse());
  }

  pairs_lib::set_mutexed(mutex_constraints_, constraints->constraints, constraints_);

  ROS_INFO("[FailsafeController]: updating constraints");

  pairs_msgs::DynamicsConstraintsSrvResponse res;
  res.success = true;
  res.message = "constraints updated";

  return pairs_msgs::DynamicsConstraintsSrvResponse::ConstPtr(new pairs_msgs::DynamicsConstraintsSrvResponse(res));
}

//}

/* getHeadingSafely() //{ */

double FailsafeController::getHeadingSafely(const geometry_msgs::Quaternion &quaternion) {

  try {
    return pairs_lib::AttitudeConverter(quaternion).getHeading();
  }
  catch (...) {
  }

  try {
    return pairs_lib::AttitudeConverter(quaternion).getYaw();
  }
  catch (...) {
  }

  return 0;
}

//}

}  // namespace failsafe_controller

}  // namespace pairs_uav_controllers

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(pairs_uav_controllers::failsafe_controller::FailsafeController, pairs_uav_managers::Controller)
