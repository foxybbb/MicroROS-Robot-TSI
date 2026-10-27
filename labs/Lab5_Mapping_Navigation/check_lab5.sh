#!/usr/bin/env bash
# Lab 5 pre-flight checker. Prints PASS / WARN / FAIL for each item.
#
#   ./check_lab5.sh robot      robot connected: topics, scan rate, points per scan
#   ./check_lab5.sh bringup    + IMU filter, odometry, TF from bringup
#   ./check_lab5.sh slam       + /map and map -> odom (Gmapping or Cartographer running)
#   ./check_lab5.sh nav        + Nav2 servers active, AMCL pose
#
# Each level includes the checks of the levels above it. Default: bringup.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
LEVEL="${1:-bringup}"

source /opt/ros/humble/setup.bash
for ws in additional_ws/imu_ws additional_ws/gmapping_ws yahboomcar_ws; do
    [ -f "$REPO/$ws/install/setup.bash" ] && source "$REPO/$ws/install/setup.bash"
done

pass=0; warn=0; fail=0
ok()   { echo "  PASS  $*"; pass=$((pass + 1)); }
wrn()  { echo "  WARN  $*"; warn=$((warn + 1)); }
bad()  { echo "  FAIL  $*"; fail=$((fail + 1)); }

# --spin-time: give DDS discovery time over Wi-Fi (the default 0.5 s misses topics)
TOPICS="$(timeout 15 ros2 topic list --no-daemon --spin-time 3 2>/dev/null)"
has_topic() { grep -qx "$1" <<<"$TOPICS"; }

check_topic() {
    if has_topic "$1"; then ok "topic $1"; else bad "topic $1 missing${2:+ — $2}"; fi
}

check_tf() {
    if timeout 6 ros2 run tf2_ros tf2_echo "$1" "$2" 2>/dev/null | grep -q Translation; then
        ok "TF $1 -> $2"
    else
        bad "TF $1 -> $2 not available${3:+ — $3}"
    fi
}

check_active() {
    state="$(timeout 6 ros2 lifecycle get "/$1" 2>/dev/null)"
    if grep -q '^active' <<<"$state"; then ok "Nav2 $1 active"; else bad "Nav2 $1 not active (${state:-no answer})"; fi
}

echo "Lab 5 check — level: $LEVEL, ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

echo "[setup]"
for p in yahboomcar_bringup yahboomcar_nav slam_gmapping imu_filter_madgwick robot_localization nav2_bringup cartographer_ros; do
    if ros2 pkg prefix "$p" >/dev/null 2>&1; then ok "package $p"; else bad "package $p not found — see README Part 0"; fi
done

echo "[robot]"
for t in /scan /imu /odom_raw /cmd_vel; do
    check_topic "$t" "is the agent running and ROS_DOMAIN_ID equal to the board's?"
done
if has_topic /scan; then
    rate="$(timeout 8 ros2 topic hz /scan --window 20 2>/dev/null | awk '/average rate/ {r=$3} END {print r}')"
    if [ -n "$rate" ]; then ok "/scan rate ${rate} Hz"; else wrn "/scan rate could not be measured"; fi
    points="$(timeout 8 ros2 topic echo /scan --once --field ranges 2>/dev/null | tr ',' '\n' | grep -c '[0-9]')"
    if [ "${points:-0}" -eq 0 ]; then
        wrn "could not read a /scan message"
    elif [ "$points" -lt 1440 ]; then
        ok "$points points per scan (Gmapping needs < 1440)"
    else
        bad "$points points per scan — Gmapping will crash, use Cartographer"
    fi
fi

if [ "$LEVEL" != robot ]; then
    echo "[bringup]"
    check_topic /imu/data "imu_filter_madgwick not running — start bringup"
    if has_topic /odom; then
        ok "topic /odom (Nav2 odom_topic)"
    elif has_topic /odometry/filtered; then
        wrn "no /odom, only /odometry/filtered — run './run_lab5.sh relay' before Nav2"
    else
        bad "no /odom and no /odometry/filtered — EKF not running"
    fi
    check_tf odom base_footprint "EKF (robot_localization) not publishing TF"
    check_tf base_footprint base_link "yahboomcar_description not running"
    check_tf base_link imu_frame "static TF from bringup missing"
fi

if [ "$LEVEL" = slam ] || [ "$LEVEL" = nav ]; then
    echo "[slam/map]"
    check_topic /map "no SLAM or map_server running"
    check_tf base_link laser_frame "static TF from the SLAM/Nav2 launch file missing"
    check_tf map odom "SLAM (or AMCL) not publishing map -> odom"
fi

if [ "$LEVEL" = nav ]; then
    echo "[nav2]"
    for n in map_server amcl planner_server controller_server bt_navigator behavior_server waypoint_follower velocity_smoother; do
        check_active "$n"
    done
    if timeout 6 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
        ok "/amcl_pose received"
    else
        wrn "no /amcl_pose yet — set the initial pose with 2D Pose Estimate"
    fi
fi

echo
echo "Result: $pass passed, $warn warnings, $fail failed"
[ "$fail" -eq 0 ]
