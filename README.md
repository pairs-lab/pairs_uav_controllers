# pairs_uav_controllers

The reference flight controllers of the PAIRS UAV stack. Each controller takes the
desired reference from the control manager together with the current state estimate
and produces a low-level attitude/attitude-rate/thrust command for the autopilot.
They are shipped as `pluginlib` plugins of the `pairs_uav_managers::Controller`
interface, so the control manager can load and switch between them at runtime.

## Contents

Controller plugins (base class `pairs_uav_managers::Controller`):

- `pairs_uav_controllers/Se3Controller` — geometric SE(3) feedback controller (the default workhorse).
- `pairs_uav_controllers/MpcController` — model-predictive controller; links the vendored `libMpcControllerSolver.so` (prebuilt for `x64` and `arm64` under `lib/`).
- `pairs_uav_controllers/FailsafeController` — emergency controller used to land safely when the active controller fails.
- `pairs_uav_controllers/MidairActivationController` — brings the control pipeline online while the UAV is already airborne.

The SE(3) and MPC controllers expose tunable gains via `dynamic_reconfigure`
(`cfg/se3_controller.cfg`, `cfg/mpc_controller.cfg`).

## Branches

- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 1 Noetic)

```bash
sudo apt install ros-noetic-pairs-uav-controllers
```

The controllers are loaded automatically by the control manager during UAV
bring-up; there are no standalone launch files in this package.

## License

BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_controllers` package; the
original copyright is retained in [LICENSE](LICENSE).
