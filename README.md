# pairs_uav_controllers

The reference flight controllers of the PAIRS UAV stack. Each controller takes the
desired reference from the control manager together with the current state estimate
and produces a low-level attitude/attitude-rate/thrust command for the autopilot.
They are shipped as `pluginlib` plugins of the `pairs_uav_managers::Controller`
interface, so the control manager can load and switch between them at runtime.

## Contents

Controller plugins (base class `pairs_uav_managers::Controller`):

- `pairs_uav_controllers::se3_controller::Se3Controller` — geometric SE(3) feedback controller (the default workhorse).
- `pairs_uav_controllers::mpc_controller::MpcController` — model-predictive controller; it depends on the prebuilt `pairs_mpc_solvers` package for the underlying MPC solver.
- `pairs_uav_controllers::failsafe_controller::FailsafeController` — emergency controller used to land safely when the active controller fails.
- `pairs_uav_controllers::midair_activation_controller::MidairActivationController` — brings the control pipeline online while the UAV is already airborne.

## Branches

- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 2 Jazzy)

```bash
sudo apt install ros-jazzy-pairs-uav-controllers
```

The controllers are loaded automatically by the control manager during UAV
bring-up; there are no standalone launch files in this package.

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_controllers` package; the
original copyright is retained in [LICENSE](LICENSE).
