#!/usr/bin/env bash
#
# Runs INSIDE the ros:jazzy packaging container (see Dockerfile).
# Builds Debian packages for pairs_uav_controllers and its PAIRS dependencies,
# in dependency order, and copies the .deb files to /output.
#
# Dependency order:
#   pairs_msgs -> pairs_lib -> pairs_uav_hw_api -> pairs_mpc_solvers
#     -> pairs_uav_managers -> pairs_uav_controllers
#
# NOTE: pairs_mpc_solvers and pairs_uav_managers must also be present in /src on
# their ros2 branch for this build to succeed. Missing dependencies are reported.
#
# Mounts (handled by make.sh):
#   /src     -> the colcon src/ directory containing the pairs_* packages
#   /output  -> host directory where built .deb files are collected
#
set -eo pipefail

ROS_DISTRO_NAME="jazzy"
OS_NAME="ubuntu"
OS_VERSION="noble"     # Jazzy targets Ubuntu 24.04 (noble)

# ROS setup scripts reference unbound vars, so source before enabling nounset.
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

LOCAL_RULES="/src/pairs_uav_controllers/packaging/rosdep/pairs.yaml"
echo "yaml file://${LOCAL_RULES}" | sudo tee /etc/ros/rosdep/sources.list.d/10-pairs.list >/dev/null
rosdep update
sudo apt-get update

mkdir -p /output

build_one() {
  local pkg_dir="$1"
  local pkg_name; pkg_name="$(basename "$pkg_dir")"
  if [ ! -d "$pkg_dir" ]; then
    echo "WARNING: dependency '$pkg_name' not found at $pkg_dir — skipping (checked out on its ros2 branch?)." >&2
    return 0
  fi
  echo "=================================================================="
  echo " Building Debian package for: ${pkg_name}"
  echo "=================================================================="
  local work="/ws/${pkg_name}"
  rm -rf "$work"; cp -a "$pkg_dir" "$work"; cd "$work"; rm -rf debian obj-*
  rosdep install --from-paths . --ignore-src -r -y || true
  bloom-generate rosdebian --os-name "$OS_NAME" --os-version "$OS_VERSION" --ros-distro "$ROS_DISTRO_NAME"
  fakeroot debian/rules binary
  local deb
  for deb in /ws/ros-${ROS_DISTRO_NAME}-*"${pkg_name//_/-}"*.deb ../ros-${ROS_DISTRO_NAME}-*.deb; do
    [ -f "$deb" ] || continue
    cp -v "$deb" /output/
    apt-get install -y "$deb" || dpkg -i "$deb" || true
  done
}

build_one /src/pairs_msgs
build_one /src/pairs_lib
build_one /src/pairs_uav_hw_api
build_one /src/pairs_mpc_solvers
build_one /src/pairs_uav_managers
build_one /src/pairs_uav_controllers

echo "=================================================================="
echo " Done. Built packages in /output:"
ls -1 /output/*.deb 2>/dev/null || echo " (none)"
echo "=================================================================="
