#!/usr/bin/env bash
# Prepare a Raspberry Pi 5 (Ubuntu 24.04 arm64) to run the Renkay car.
#
#   git clone https://github.com/strvaeq/Robotek-WRO2026.git && cd Robotek-WRO2026
#   bash tools/setup-pi.sh            # install and build (asks for sudo when needed)
#   bash tools/setup-pi.sh --check    # only inspect: installs nothing, moves nothing
#   bash tools/setup-pi.sh --dry-run  # print the commands without running them
#
# What it does (safe to re-run: finished steps are skipped):
#   1. checks the system (Ubuntu 24.04);
#   2. installs ROS 2 Jazzy base from the ROS apt repository and the car apt packages;
#   3. installs the udev rules (tools/99-renkay.rules) and adds you to the
#      dialout, video and gpio groups;
#   4. builds the workspace: colcon build --symlink-install.
# It does NOT clone third-party packages (ros_robot_controller_msgs,
# ros_robot_controller, ldlidar_stl_ros2): the commands are in SETUP.md section 5.
#
# SAFETY: this script starts no node and moves no motor or servo. The first time
# you drive the car, put it on blocks with the wheels off the ground.
set -euo pipefail

MODE=install
DRY=0
SKIP_BUILD=0
for arg in "$@"; do
  case "$arg" in
    --check) MODE=check ;;
    --dry-run) DRY=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    -h|--help) awk 'NR > 1 && /^#/ {print; next} NR > 1 {exit}' "$0"; exit 0 ;;
    *) echo "Unknown option: $arg (use --help)" >&2; exit 2 ;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SUDO=sudo
if [ "$(id -u)" -eq 0 ]; then SUDO=; fi
TARGET_USER="${SUDO_USER:-${USER:-$(id -un)}}"
DEPS_WS="${RENKAY_DEPS_WS:-$HOME/renkay_deps_ws}"

APT_PACKAGES=(
  git build-essential cmake curl rsync
  ros-jazzy-cv-bridge ros-jazzy-usb-cam ros-jazzy-tf2-ros
  python3-numpy python3-opencv python3-serial python3-yaml python3-lgpio v4l-utils
)

say() { echo ">>> $*"; }
warn() { echo "!!! $*" >&2; }
run() {   # run the command, or only print it with --dry-run
  if [ "$DRY" = 1 ]; then echo "+ $*"; else "$@"; fi
}
have_group() { getent group "$1" >/dev/null 2>&1; }
udev_running() { [ "$DRY" = 1 ] || pgrep -x systemd-udevd >/dev/null 2>&1; }

# ---------------------------------------------------------------------------------------------------------------
check_os() {
  local id="" version="" arch
  if [ -r /etc/os-release ]; then id="$(. /etc/os-release && echo "${ID:-}")"; version="$(. /etc/os-release && echo "${VERSION_ID:-}")"; fi
  arch="$(uname -m)"
  if [ "$id" = ubuntu ] && [ "$version" = 24.04 ]; then
    say "System: Ubuntu 24.04 ($arch)"
    if [ "$arch" != aarch64 ]; then warn "Not arm64 ($arch): it works, but the car is a Raspberry Pi 5 (aarch64)."; fi
  elif [ "$DRY" = 1 ] || [ "${RENKAY_FORCE:-0}" = 1 ]; then
    warn "System ${id:-unknown} ${version:-?}: only tested on Ubuntu 24.04. Continuing because of --dry-run or RENKAY_FORCE=1."
  else
    echo "This script is for Ubuntu 24.04 on the Raspberry Pi. Detected: ${id:-unknown} ${version:-?}." >&2
    echo "For a development laptop, install ROS 2 Jazzy manually (SETUP.md section 3)." >&2
    exit 1
  fi
}

# ---------------------------------------------------------------------------------------------------------------
install_ros() {
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    say "ROS 2 Jazzy already installed"
    return
  fi
  say "Adding the ROS 2 apt repository and installing ros-jazzy-ros-base"
  run $SUDO apt-get update
  run $SUDO apt-get install -y software-properties-common curl
  run $SUDO add-apt-repository -y universe
  run $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  local repo_line
  repo_line="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "${UBUNTU_CODENAME:-noble}") main"
  if [ "$DRY" = 1 ]; then
    echo "+ echo '$repo_line' | $SUDO tee /etc/apt/sources.list.d/ros2.list"
  else
    echo "$repo_line" | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi
  run $SUDO apt-get update
  run $SUDO apt-get install -y ros-jazzy-ros-base
}

install_apt() {
  say "Car apt packages: ${APT_PACKAGES[*]}"
  run $SUDO apt-get update
  run $SUDO apt-get install -y "${APT_PACKAGES[@]}"
}

# ---------------------------------------------------------------------------------------------------------------
install_udev() {
  local rules=/etc/udev/rules.d/99-renkay.rules
  if ! udev_running; then
    warn "systemd-udevd is not running (a container?): skipping the udev rules and the groups."
    return
  fi
  if [ -f "$rules" ] && cmp -s "$REPO/tools/99-renkay.rules" "$rules"; then
    say "udev rules already installed"
  else
    say "Installing udev rules ($rules)"
    run $SUDO install -m 644 "$REPO/tools/99-renkay.rules" "$rules"
    run $SUDO udevadm control --reload-rules
    run $SUDO udevadm trigger
  fi
  if [ "$(id -u)" -eq 0 ] && [ -z "${SUDO_USER:-}" ]; then
    warn "Running as root without SUDO_USER: cannot tell which user to add. Do it with: usermod -aG dialout,video,gpio USER"
    return
  fi
  if ! have_group gpio; then
    say "Creating the gpio group"
    run $SUDO groupadd -f gpio
  fi
  local missing=() group
  for group in dialout video gpio; do
    if ! id -nG "$TARGET_USER" 2>/dev/null | tr ' ' '\n' | grep -qx "$group"; then missing+=("$group"); fi
  done
  if [ ${#missing[@]} -gt 0 ]; then
    local list; list="$(IFS=,; echo "${missing[*]}")"
    say "Adding $TARGET_USER to groups: $list (log out and back in for it to apply)"
    run $SUDO usermod -aG "$list" "$TARGET_USER"
  else
    say "$TARGET_USER is already in dialout, video and gpio"
  fi
}

# ---------------------------------------------------------------------------------------------------------------
build_workspace() {
  if [ "$SKIP_BUILD" = 1 ]; then say "Skipping the build (--skip-build)"; return; fi
  say "Building: colcon build --symlink-install (several minutes on a Pi)"
  if [ "$DRY" = 1 ]; then
    echo "+ (cd $REPO && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install)"
    return
  fi
  (
    cd "$REPO"
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    if [ -f "$DEPS_WS/install/setup.bash" ]; then
      # shellcheck disable=SC1091
      source "$DEPS_WS/install/setup.bash"
    fi
    set -u
    colcon build --symlink-install
  )
}

next_steps() {
  cat <<NEXT

>>> Done. Next steps (on the Pi, from $REPO):
    1. If you were added to new groups, log out and back in.
    2. bash tools/setup-pi.sh --check          # board, LiDAR, camera and groups
    3. source install/setup.bash
    4. PUT THE CAR ON BLOCKS (wheels off the ground) the first time.
    5. ros2 launch renkay_bringup open.launch.py       # or obstacle.launch.py
       The RRC Lite button starts the run; press it again to stop.
NEXT
}

# ---------------------------------------------------------------------------------------------------------------
# --check: look only, change nothing
ok_count=0; bad_count=0
good() { echo "  ok      $*"; ok_count=$((ok_count+1)); }
bad() { echo "  MISSING $*"; bad_count=$((bad_count+1)); }
note() { echo "  note    $*"; }

first_existing() { local p; for p in "$@"; do if [ -e "$p" ]; then echo "$p"; return 0; fi; done; return 1; }

do_check() {
  say "Check (nothing is installed or moved)"
  local id="" version=""
  if [ -r /etc/os-release ]; then id="$(. /etc/os-release && echo "${ID:-}")"; version="$(. /etc/os-release && echo "${VERSION_ID:-}")"; fi
  if [ "$id" = ubuntu ] && [ "$version" = 24.04 ]; then good "Ubuntu 24.04 ($(uname -m))"; else bad "Ubuntu 24.04 (this system: ${id:-?} ${version:-?})"; fi
  if [ -f /opt/ros/jazzy/setup.bash ]; then good "ROS 2 Jazzy"; else bad "ROS 2 Jazzy (bash tools/setup-pi.sh)"; fi

  if [ -f "$REPO/install/setup.bash" ]; then
    good "workspace built (install/setup.bash)"
    local packages pkg
    packages="$(
      cd "$REPO"
      set +u
      source /opt/ros/jazzy/setup.bash 2>/dev/null
      if [ -f "$DEPS_WS/install/setup.bash" ]; then source "$DEPS_WS/install/setup.bash" 2>/dev/null; fi
      if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then source "$HOME/ros2_ws/install/setup.bash" 2>/dev/null; fi
      source install/setup.bash 2>/dev/null
      ros2 pkg list 2>/dev/null || true
    )"
    for pkg in renkay_open renkay_obstacle renkay_bringup ldlidar_stl_ros2 usb_cam ros_robot_controller ros_robot_controller_msgs; do
      if grep -qx "$pkg" <<<"$packages"; then good "package $pkg"; else bad "package $pkg (SETUP.md section 5, then rebuild)"; fi
    done
  else
    bad "workspace not built (bash tools/setup-pi.sh)"
  fi

  local board lidar camera
  if board="$(first_existing /dev/renkay_rrc /dev/ttyACM0)"; then good "RRC Lite board on $board"; else bad "RRC Lite board (USB 1a86:55d4, /dev/ttyACM0): use the data USB cable"; fi
  if lidar="$(first_existing /dev/renkay_lidar /dev/ttyUSB0)"; then good "LD19 LiDAR on $lidar"; else bad "LD19 LiDAR (USB 10c4:ea60, /dev/ttyUSB0): check the cable"; fi
  if camera="$(first_existing /dev/v4l/by-id/*-video-index0)"; then good "camera on $camera"
  elif [ -e /dev/video0 ]; then note "camera only as /dev/video0 (on the Pi 5 that number can belong to another device): ls /dev/v4l/by-id/"
  else bad "USB camera (/dev/v4l/by-id/...-video-index0)"; fi
  if [ -e /dev/gpiochip4 ]; then good "motor GPIO (/dev/gpiochip4)"; else bad "/dev/gpiochip4 (TB6612FNG through lgpio; another Pi model changes the chip)"; fi

  local module
  for module in lgpio serial cv2 numpy yaml; do
    if python3 -c "import $module" 2>/dev/null; then good "python3-$module"; else bad "python3-$module (sudo apt install python3-$module)"; fi
  done

  local group
  for group in dialout video gpio; do
    if [ "$(id -u)" -eq 0 ]; then note "running as root: groups are not checked"; break
    elif id -nG 2>/dev/null | tr ' ' '\n' | grep -qx "$group"; then good "you are in group $group"; else bad "not in group $group (sudo usermod -aG $group \$USER, then log in again)"; fi
  done
  if [ -f /etc/udev/rules.d/99-renkay.rules ]; then good "udev rules installed"; else note "udev rules not installed (bash tools/setup-pi.sh)"; fi
  echo
  echo ">>> $ok_count ok, $bad_count to fix"
  [ "$bad_count" -eq 0 ]
}

# ---------------------------------------------------------------------------------------------------------------
if [ "$MODE" = check ]; then
  do_check
  exit $?
fi

check_os
install_ros
install_apt
install_udev
build_workspace
if [ "$DRY" = 1 ]; then say "End of --dry-run: nothing was executed."; else next_steps; fi
