#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

ROS_DISTRO="${ROS_DISTRO:-humble}"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-1}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
SKIP_ROSDEP_KEYS="${SKIP_ROSDEP_KEYS:-}"

info() { echo -e "\033[1;34m[build]\033[0m $*"; }
warn() { echo -e "\033[1;33m[build]\033[0m $*"; }
error() { echo -e "\033[1;31m[build]\033[0m $*" >&2; }

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    error "Missing command: $1"
    return 1
  fi
}

setup_sudo() {
  if [ "${EUID}" -eq 0 ]; then
    SUDO=()
  elif command -v sudo >/dev/null 2>&1; then
    SUDO=(sudo)
  else
    error "This script needs apt/rosdep dependency installation, but sudo is not available."
    error "Run as root or install sudo first."
    exit 1
  fi
}

apt_install() {
  local missing=()
  local pkg
  for pkg in "$@"; do
    if ! dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null | grep -q "install ok installed"; then
      missing+=("$pkg")
    fi
  done

  if [ "${#missing[@]}" -eq 0 ]; then
    info "Base apt dependencies are already installed."
    return
  fi

  info "Installing base apt dependencies: ${missing[*]}"
  "${SUDO[@]}" apt-get update
  "${SUDO[@]}" DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${missing[@]}"
}

setup_rosdep() {
  if ! command -v rosdep >/dev/null 2>&1; then
    apt_install python3-rosdep
  fi

  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    info "Initializing rosdep."
    "${SUDO[@]}" rosdep init || true
  fi

  info "Updating rosdep database."
  rosdep update --rosdistro "$ROS_DISTRO"
}

install_project_dependencies() {
  local rosdep_args=(
    install
    --from-paths src
    --ignore-src
    --rosdistro "$ROS_DISTRO"
    -r
    -y
  )

  if [ -n "$SKIP_ROSDEP_KEYS" ]; then
    warn "Skipping rosdep keys: $SKIP_ROSDEP_KEYS"
    # shellcheck disable=SC2206
    local skip_keys=( $SKIP_ROSDEP_KEYS )
    rosdep_args+=(--skip-keys "${skip_keys[*]}")
  fi

  info "Checking and installing ROS/package dependencies from src/package.xml files."
  rosdep "${rosdep_args[@]}"
}

build_workspace() {
  local cmake_args=(
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE"
  )

  # KISS-Matcher's package exports kiss_matcher under this path in this workspace.
  # Keeping this argument preserves the previous build.sh behavior.
  if [ -d "$WORKSPACE_ROOT/install/kiss_matcher_ros/lib/cmake/kiss_matcher" ]; then
    cmake_args+=("-Dkiss_matcher_DIR=$WORKSPACE_ROOT/install/kiss_matcher_ros/lib/cmake/kiss_matcher")
  else
    cmake_args+=("-Dkiss_matcher_DIR=$WORKSPACE_ROOT/install/kiss_matcher_ros/lib/cmake/kiss_matcher")
  fi

  info "Building all packages in $WORKSPACE_ROOT."
  info "Build type: $BUILD_TYPE, parallel workers: $PARALLEL_WORKERS"
  colcon build \
    --symlink-install \
    --parallel-workers "$PARALLEL_WORKERS" \
    --cmake-args "${cmake_args[@]}"
}

main() {
  if [ ! -d src ]; then
    error "Cannot find src/ under $WORKSPACE_ROOT. Run this script from the Lidar_nav2_ws layout."
    exit 1
  fi

  setup_sudo

  apt_install \
    build-essential \
    cmake \
    git \
    pkg-config \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libomp-dev \
    libpcl-dev \
    libssl-dev \
    libunwind-dev \
    libusb-1.0-0-dev \
    nlohmann-json3-dev \
    qtbase5-private-dev \
    "ros-$ROS_DISTRO-camera-info-manager" \
    "ros-$ROS_DISTRO-cv-bridge" \
    "ros-$ROS_DISTRO-diagnostic-updater" \
    "ros-$ROS_DISTRO-gazebo-ros-pkgs" \
    "ros-$ROS_DISTRO-image-geometry" \
    "ros-$ROS_DISTRO-image-transport" \
    "ros-$ROS_DISTRO-nav2-bringup" \
    "ros-$ROS_DISTRO-pcl-conversions" \
    "ros-$ROS_DISTRO-pcl-ros" \
    "ros-$ROS_DISTRO-robot-state-publisher" \
    "ros-$ROS_DISTRO-slam-toolbox" \
    "ros-$ROS_DISTRO-tf-transformations" \
    "ros-$ROS_DISTRO-tf2-geometry-msgs" \
    "ros-$ROS_DISTRO-xacro"

  setup_rosdep
  install_project_dependencies

  info "Sourcing ROS $ROS_DISTRO."
  # Some upstream ROS setup files read unset variables under `set -u`.
  set +u
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  set -u

  export PATH="/opt/cmake/bin:/usr/lib/nvidia-cuda-toolkit/bin:$PATH"

  require_command colcon
  build_workspace

  info "Build finished."
  info "To use this workspace: source $WORKSPACE_ROOT/install/setup.bash"
}

main "$@"
