#!/usr/bin/env bash
# Lab 5 helper: one short command per lab step, each meant for its own terminal.
#
#   ./run_lab5.sh agent [udp|serial]     micro-ROS agent (default udp, port 8090)
#   ./run_lab5.sh bringup                IMU filter, EKF, URDF, TF
#   ./run_lab5.sh rviz                   RViz with yahboomcar_nav/rviz/view.rviz
#   ./run_lab5.sh gmapping               Gmapping SLAM
#   ./run_lab5.sh cartographer           Cartographer SLAM
#   ./run_lab5.sh teleop [keyboard|joy]  manual driving (default keyboard)
#   ./run_lab5.sh save <name>            save /map as maps/<name>.{pgm,yaml}
#   ./run_lab5.sh relay                  relay /odometry/filtered -> /odom (only if /odom is missing)
#   ./run_lab5.sh nav <name|path.yaml> [params.yaml]   Nav2 on a saved map
#   ./run_lab5.sh stop                   publish a zero /cmd_vel once
#   ./run_lab5.sh tf                     TF tree PDF (view_frames) in the current directory
#   ./run_lab5.sh graph                  rqt_graph
#
# Environment: ROS_DOMAIN_ID (default 21), AGENT_PORT (8090), AGENT_DEV (/dev/ttyUSB0).

set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAPS="$REPO/yahboomcar_ws/src/yahboomcar_nav/maps"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"

source_ws() {
    # setup.bash scripts reference unset variables, so relax -u style checks here.
    set +e
    source /opt/ros/humble/setup.bash
    for ws in additional_ws/imu_ws additional_ws/gmapping_ws yahboomcar_ws; do
        if [ -f "$REPO/$ws/install/setup.bash" ]; then
            source "$REPO/$ws/install/setup.bash"
        else
            echo "[run_lab5] warning: $ws is not built (see README, Part 0)" >&2
        fi
    done
    set -e
}

resolve_map() {
    case "$1" in
        "") echo "usage: $0 nav <map name or path to .yaml> [params.yaml]" >&2; exit 1 ;;
        /*.yaml|./*.yaml|../*.yaml) realpath "$1" ;;
        *.yaml) echo "$MAPS/$1" ;;
        *) echo "$MAPS/$1.yaml" ;;
    esac
}

cmd="${1:-}"
shift || true
echo "[run_lab5] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

case "$cmd" in
    agent)
        mode="${1:-udp}"
        if [ "$mode" = serial ]; then
            args=(serial --dev "${AGENT_DEV:-/dev/ttyUSB0}" -b 921600 -v4)
        else
            args=(udp4 --port "${AGENT_PORT:-8090}" -v4)
        fi
        exec docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host \
            microros/micro-ros-agent:humble "${args[@]}"
        ;;
    bringup)
        source_ws
        exec ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
        ;;
    rviz)
        source_ws
        exec ros2 launch yahboomcar_nav display_launch.py
        ;;
    gmapping)
        source_ws
        exec ros2 launch yahboomcar_nav map_gmapping_launch.py
        ;;
    cartographer)
        source_ws
        exec ros2 launch yahboomcar_nav map_cartographer_launch.py
        ;;
    teleop)
        source_ws
        if [ "${1:-keyboard}" = joy ]; then
            ros2 run joy joy_node &
            trap 'kill $! 2>/dev/null' EXIT
            ros2 run yahboomcar_ctrl yahboom_joy
        else
            exec ros2 run yahboomcar_ctrl yahboom_keyboard
        fi
        ;;
    save)
        name="${1:?usage: $0 save <map name>}"
        source_ws
        ros2 launch yahboomcar_nav save_map_launch.py map_path:="$MAPS/$name"
        ls -l "$MAPS/$name".pgm "$MAPS/$name".yaml
        ;;
    relay)
        source_ws
        exec ros2 run topic_tools relay /odometry/filtered /odom
        ;;
    nav)
        map="$(resolve_map "${1:-}")"
        [ -f "$map" ] || { echo "[run_lab5] map not found: $map" >&2; exit 1; }
        source_ws
        extra=()
        if [ -n "${2:-}" ]; then
            extra=(params_file:="$(realpath "$2")")
        fi
        echo "[run_lab5] map: $map"
        exec ros2 launch yahboomcar_nav navigation_dwb_launch.py maps:="$map" "${extra[@]}"
        ;;
    stop)
        source_ws
        exec ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
        ;;
    tf)
        source_ws
        exec ros2 run tf2_tools view_frames
        ;;
    graph)
        source_ws
        exec ros2 run rqt_graph rqt_graph
        ;;
    *)
        sed -n '2,17p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
        exit 1
        ;;
esac
