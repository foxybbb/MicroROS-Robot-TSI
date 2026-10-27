#!/usr/bin/env bash
# Lab 5 setup: installs everything needed for mapping and navigation and builds
# the three workspaces of this repository.
#
# Requirements: Ubuntu 22.04, ROS 2 Humble already installed in /opt/ros/humble,
# this repository cloned (any location).
#
#   ./helper_scripts/install_lab5.sh            # install + build + edit ~/.bashrc
#   ./helper_scripts/install_lab5.sh --no-apt   # skip apt (packages already there)
#   ROS_DOMAIN_ID=21 ./helper_scripts/install_lab5.sh   # domain written to ~/.bashrc
#
# Safe to run again: apt skips installed packages, colcon rebuilds only what
# changed, and the ~/.bashrc block is replaced, not duplicated.

set -eo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOMAIN="${ROS_DOMAIN_ID:-21}"
BASHRC="${BASHRC:-$HOME/.bashrc}"   # override only for testing
DO_APT=1
[ "${1:-}" = "--no-apt" ] && DO_APT=0

say()  { printf '\n\033[1;34m== %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------- checks
say "Checking the system"
. /etc/os-release
[ "$VERSION_ID" = "22.04" ] || echo "WARNING: tested on Ubuntu 22.04, this is $PRETTY_NAME"
[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble not found in /opt/ros/humble"
echo "repository: $REPO"
echo "ROS_DOMAIN_ID for ~/.bashrc: $DOMAIN"

# ---------------------------------------------------------------- apt
if [ "$DO_APT" = 1 ]; then
    say "Installing ROS packages (sudo password may be asked)"
    sudo apt-get update
    sudo apt-get install -y \
        ros-humble-navigation2 ros-humble-nav2-bringup \
        ros-humble-cartographer-ros \
        ros-humble-robot-localization \
        ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
        ros-humble-xacro ros-humble-joy \
        ros-humble-tf2-tools ros-humble-tf-transformations \
        ros-humble-rqt-graph ros-humble-topic-tools \
        ros-humble-rviz2 \
        python3-pykdl python3-numpy python3-colcon-common-extensions
fi

# ---------------------------------------------------------------- build
# setup.bash files use unset variables; do not run them under `set -u`.
source /opt/ros/humble/setup.bash

build() {   # build <workspace dir> [extra colcon args...]
    local ws="$1"; shift
    say "Building $ws"
    cd "$REPO/$ws"
    if ! colcon build "$@" 2>&1 | tail -n 25; then
        die "colcon build failed in $ws (see output above)"
    fi
    source "$REPO/$ws/install/setup.bash"
}

# Order matters: yahboomcar_ws uses imu_filter_madgwick and slam_gmapping.
build additional_ws/imu_ws
build additional_ws/gmapping_ws
build yahboomcar_ws

# ---------------------------------------------------------------- ~/.bashrc
say "Updating $BASHRC"
BEGIN="# >>> MicroROS-Robot-TSI >>>"
END="# <<< MicroROS-Robot-TSI <<<"
touch "$BASHRC"
sed -i "/^$BEGIN\$/,/^$END\$/d" "$BASHRC"
cat >> "$BASHRC" <<EOF
$BEGIN
export REPO=$REPO
export ROS_DOMAIN_ID=$DOMAIN
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    source \$REPO/additional_ws/imu_ws/install/setup.bash
    source \$REPO/additional_ws/gmapping_ws/install/setup.bash
    source \$REPO/yahboomcar_ws/install/setup.bash
fi
$END
EOF
echo "added the block between '$BEGIN' and '$END'"

# ---------------------------------------------------------------- check
say "Checking the installation"
fail=0
for p in yahboomcar_bringup yahboomcar_nav yahboomcar_ctrl yahboomcar_description \
         imu_filter_madgwick slam_gmapping robot_localization cartographer_ros \
         nav2_bringup nav2_map_server; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then
        printf '  ok       %s\n' "$p"
    else
        printf '  MISSING  %s\n' "$p"; fail=1
    fi
done
command -v docker >/dev/null || echo "  note     docker not found: install it with helper_scripts/install_docker.sh (needed for the micro-ROS agent)"

[ "$fail" = 0 ] || die "some packages are missing"
say "Done. Open a NEW terminal (or run: source ~/.bashrc) before starting the lab."
