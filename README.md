# pairs_uav_controllers

Reference attitude/force controllers for the PAIRS UAV system, provided as
`pluginlib` plugins of the `pairs_uav_managers::Controller` interface:

| Plugin | Class |
|---|---|
| SE(3) controller | `pairs_uav_controllers/Se3Controller` |
| MPC controller | `pairs_uav_controllers/MpcController` |
| Failsafe controller | `pairs_uav_controllers/FailsafeController` |
| Midair activation controller | `pairs_uav_controllers/MidairActivationController` |

The MPC controller links against the prebuilt `libMpcControllerSolver.so`
(vendored under `lib/`, `x64` + `arm64`).

## Branches

- `ros1` — ROS 1 Noetic (catkin)
- `ros2` — ROS 2 Jazzy (ament_cmake)

## Install (ROS 1 Noetic)

```bash
sudo apt install ros-noetic-pairs-uav-controllers
```

## License

BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_controllers` package; the
original copyright is retained in [LICENSE](LICENSE).

Maintainer: Thanh Nguyen Canh <canhthanh@vnu.edu.vn>
