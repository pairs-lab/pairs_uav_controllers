# pairs_uav_controllers (ROS 2)

Reference attitude/force controllers for the PAIRS UAV system, provided as
`pluginlib` plugins of the `pairs_uav_managers::Controller` interface
(SE(3), MPC, Failsafe, Midair-activation).

The MPC controller uses the `pairs_mpc_solvers` package; its `Solver` is exposed
under the upstream `pairs_mpc_solvers` C++ namespace, kept to match the prebuilt
solver binary's ABI.

## Branches
- `ros2` — ROS 2 Jazzy (ament_cmake)
- `ros1` — ROS 1 Noetic (catkin)

## License
BSD 3-Clause. Derived from the CTU-MRS `pairs_uav_controllers` package; the
original copyright is retained in [LICENSE](LICENSE).