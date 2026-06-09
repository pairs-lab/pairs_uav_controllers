#ifndef PAIRS_UAV_CONTROLLERS_COMMON_H
#define PAIRS_UAV_CONTROLLERS_COMMON_H

#include <eigen3/Eigen/Eigen>
#include <pairs_uav_managers/control_manager/common_handlers.h>
#include <pairs_uav_managers/controller.h>
#include <pairs_lib/attitude_converter.h>

namespace pairs_uav_controllers
{

namespace common
{

enum CONTROL_OUTPUT
{
  ACTUATORS_CMD,
  CONTROL_GROUP,
  ATTITUDE_RATE,
  ATTITUDE,
  ACCELERATION_HDG_RATE,
  ACCELERATION_HDG,
  VELOCITY_HDG_RATE,
  VELOCITY_HDG,
  POSITION
};

Eigen::Vector3d orientationError(const Eigen::Matrix3d& R, const Eigen::Matrix3d& Rd);

std::optional<Eigen::Vector3d> sanitizeDesiredForce(const rclcpp::Node::SharedPtr& node, const Eigen::Vector3d& desired_force, const double& tilt_over_limit, const double& tilt_saturation, const std::string& node_name);

Eigen::Matrix3d so3transform(const rclcpp::Node::SharedPtr& node, const Eigen::Vector3d& body_z, const ::Eigen::Vector3d& heading, const bool& preserve_heading);

std::optional<CONTROL_OUTPUT> getLowestOuput(const pairs_uav_managers::control_manager::ControlOutputModalities_t& outputs);

std::optional<CONTROL_OUTPUT> getHighestOuput(const pairs_uav_managers::control_manager::ControlOutputModalities_t& outputs);

std::optional<pairs_msgs::msg::HwApiAttitudeRateCmd> attitudeController(const rclcpp::Node::SharedPtr& node, const pairs_msgs::msg::UavState& uav_state, const pairs_msgs::msg::HwApiAttitudeCmd& reference, const Eigen::Vector3d& ff_rate, const Eigen::Vector3d& rate_saturation, const Eigen::Vector3d& gains, const bool& parasitic_heading_rate_compensation);

std::optional<pairs_msgs::msg::HwApiControlGroupCmd> attitudeRateController(const rclcpp::Node::SharedPtr& node, const pairs_msgs::msg::UavState& uav_state, const pairs_msgs::msg::HwApiAttitudeRateCmd& reference, const Eigen::Vector3d& gains);


pairs_msgs::msg::HwApiActuatorCmd actuatorMixer(const rclcpp::Node::SharedPtr& node, const pairs_msgs::msg::HwApiControlGroupCmd& ctrl_group_cmd, const Eigen::MatrixXd& mixer);

/* throttle extraction //{ */

std::optional<double> extractThrottle(const pairs_uav_managers::Controller::ControlOutput& control_output);

struct HwApiCmdExtractThrottleVisitor
{
  std::optional<double> operator()(const pairs_msgs::msg::HwApiActuatorCmd& msg) {

    double throttle = 0;

    if (msg.motors.size() == 0) {
      return std::nullopt;
    }

    throttle = 0;

    for (size_t i = 0; i < msg.motors.size(); i++) {
      throttle += msg.motors[i];
    };

    throttle /= msg.motors.size();

    return throttle;
  }
  std::optional<double> operator()(const pairs_msgs::msg::HwApiControlGroupCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()(const pairs_msgs::msg::HwApiAttitudeCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()(const pairs_msgs::msg::HwApiAttitudeRateCmd& msg) {
    return msg.throttle;
  }
  std::optional<double> operator()([[maybe_unused]] const pairs_msgs::msg::HwApiAccelerationHdgRateCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const pairs_msgs::msg::HwApiAccelerationHdgCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const pairs_msgs::msg::HwApiVelocityHdgRateCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const pairs_msgs::msg::HwApiVelocityHdgCmd& msg) {
    return std::nullopt;
  }
  std::optional<double> operator()([[maybe_unused]] const pairs_msgs::msg::HwApiPositionCmd& msg) {
    return std::nullopt;
  }
};

//}

}  // namespace common

}  // namespace pairs_uav_controllers

#endif  // PAIRS_UAV_CONTROLLERS_COMMON_H
