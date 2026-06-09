cmake_minimum_required(VERSION 3.15)
project(pairs_uav_controllers)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 1 == use rclcpp::Timer implementation
# 0 == use PAIRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[pairs_uav_controllers]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  pairs_msgs
  pairs_lib
  pluginlib
  geometry_msgs
  sensor_msgs
  pairs_uav_managers
  Eigen3
  pairs_mpc_solvers
)

set(LIBRARIES
  PairsUavControllers_MpcController
  PairsUavControllers_Se3Controller
  PairsUavControllers_FailsafeController
  PairsUavControllers_MidairActivationController
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${pairs_lib_INCLUDE_DIRS}
  ${pairs_uav_managers_INCLUDE_DIRS}
  ${pairs_mpc_solvers_INCLUDE_DIRS}
  ${pairs_uav_testing_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# MPC Controller

add_library(PairsUavControllers_MpcController SHARED
  src/mpc_controller.cpp
  src/common.cpp
  )

target_link_libraries(PairsUavControllers_MpcController
  rclcpp::rclcpp
  rclcpp_components::component
  ${pairs_msgs_TARGETS}
  pairs_lib::pairs_lib
  pluginlib::pluginlib
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  pairs_uav_managers::pairs_uav_managers
  Eigen3::Eigen
  ${pairs_mpc_solvers_TARGETS}
  pairs_uav_testing::pairs_uav_testing
  )

target_compile_definitions(PairsUavControllers_MpcController PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# SE3 Controller

add_library(PairsUavControllers_Se3Controller SHARED
  src/se3_controller.cpp
  src/common.cpp
  )

target_link_libraries(PairsUavControllers_Se3Controller
  rclcpp::rclcpp
  rclcpp_components::component
  ${pairs_msgs_TARGETS}
  pairs_lib::pairs_lib
  pluginlib::pluginlib
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  pairs_uav_managers::pairs_uav_managers
  Eigen3::Eigen
  ${pairs_mpc_solvers_TARGETS}
  pairs_uav_testing::pairs_uav_testing
  )

target_compile_definitions(PairsUavControllers_Se3Controller PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Failsafe Controller

add_library(PairsUavControllers_FailsafeController SHARED
  src/failsafe_controller.cpp
  src/common.cpp
  )

target_link_libraries(PairsUavControllers_FailsafeController
  rclcpp::rclcpp
  rclcpp_components::component
  ${pairs_msgs_TARGETS}
  pairs_lib::pairs_lib
  pluginlib::pluginlib
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  pairs_uav_managers::pairs_uav_managers
  Eigen3::Eigen
  ${pairs_mpc_solvers_TARGETS}
  pairs_uav_testing::pairs_uav_testing
  )

target_compile_definitions(PairsUavControllers_FailsafeController PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

# Midair Activation Controller

add_library(PairsUavControllers_MidairActivationController SHARED
  src/midair_activation_controller.cpp
  src/common.cpp
  )

target_link_libraries(PairsUavControllers_MidairActivationController
  rclcpp::rclcpp
  rclcpp_components::component
  ${pairs_msgs_TARGETS}
  pairs_lib::pairs_lib
  pluginlib::pluginlib
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  pairs_uav_managers::pairs_uav_managers
  Eigen3::Eigen
  pairs_uav_testing::pairs_uav_testing
  )

target_compile_definitions(PairsUavControllers_MidairActivationController PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                       export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(pairs_uav_managers plugins.xml)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include
  DESTINATION .
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
