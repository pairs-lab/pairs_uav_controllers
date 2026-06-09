#!/usr/bin/env bash
#
# Runs INSIDE the ros:noetic packaging container (see Dockerfile).
# Builds Debian packages for pairs_uav_controllers and its PAIRS dependencies,
# in dependency order, and copies the .deb files to /output.
#
# Dependency order:
#   pairs_msgs -> pairs_lib -> pairs_uav_hw_api -> pairs_uav_managers -> pairs_uav_controllers
#
# Mounts (handled by make.sh):
#   /src     -> the catkin src/ directory containing the pairs_* packages
#   /output  -> host directory where built .deb files are collected
#
set -eo pipefail

ROS_DISTRO_NAME="noetic"
OS_NAME="ubuntu"
OS_VERSION="focal"

source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

LOCAL_RULES="/src/pairs_uav_controllers/packaging/rosdep/pairs.yaml"
echo "yaml file://${LOCAL_RULES}" | sudo tee /etc/ros/rosdep/sources.list.d/10-pairs.list >/dev/null
rosdep update --include-eol-distros
sudo apt-get update

mkdir -p /output

build_one() {
  local pkg_dir="$1"
  local pkg_name; pkg_name="$(basename "$pkg_dir")"
  if [ ! -d "$pkg_dir" ]; then
    echo "ERROR: dependency '$pkg_name' not found at $pkg_dir — checked out in src/ on its ros1 branch?" >&2
    exit 1
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
build_one /src/pairs_uav_managers
build_one /src/pairs_uav_controllers

echo "=================================================================="
echo " Done. Built packages in /output:"
ls -1 /output/*.deb
echo "=================================================================="
