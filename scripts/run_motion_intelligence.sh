#!/usr/bin/env bash
# Usage:
#   ./scripts/run_motion_intelligence.sh \
#       --fastdds <true|false> \
#       --is-sim <true|false> \
#       --joy-topic </joy|/xlab/hr/joy_state_debug> \
#       --joy-type <keyboard|game_controller|robot_remote_control>

echo "====================================================="
## Strict mode(exit immediately on any command failure) ##
set -eo pipefail

## cd root ##
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

## parameters ##
V=V1
ROS_SETUP=/opt/ros/humble/setup.bash
FASTDDS_SUBNET=192.168.42.
FASTDDS=""
IS_SIM=""
JOY_TOPIC=""
JOY_TYPE=""

## parse arguments ##
while [ $# -gt 0 ]; do
    case "$1" in
        --fastdds)
            FASTDDS="$2"
            shift 2
            ;;
        --fastdds=*)
            FASTDDS="${1#--fastdds=}"
            shift
            ;;
        --is-sim)
            IS_SIM="$2"
            shift 2
            ;;
        --is-sim=*)
            IS_SIM="${1#--is-sim=}"
            shift
            ;;
        --joy-topic)
            JOY_TOPIC="$2"
            shift 2
            ;;
        --joy-topic=*)
            JOY_TOPIC="${1#--joy-topic=}"
            shift
            ;;
        --joy-type)
            JOY_TYPE="$2"
            shift 2
            ;;
        --joy-type=*)
            JOY_TYPE="${1#--joy-type=}"
            shift
            ;;
        *)
            echo "unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

# Input parameter restrictions
USAGE="usage: $0 --fastdds <true|false> [--is-sim <true|false>] [--joy-topic /joy|/xlab/hr/joy_state_debug] [--joy-type <keyboard|game_controller|robot_remote_control>]"
case "$FASTDDS" in
    true|false) ;;
    *)
        echo "$USAGE" >&2
        exit 1
        ;;
esac
if [ -n "$IS_SIM" ]; then
    case "$IS_SIM" in
        true|false) ;;
        *)
            echo "$USAGE" >&2
            exit 1
            ;;
    esac
fi
if [ -n "$JOY_TYPE" ]; then
    case "$JOY_TYPE" in
        keyboard|game_controller|robot_remote_control) ;;
        *)
            echo "$USAGE" >&2
            exit 1
            ;;
    esac
fi
if [ -n "$JOY_TOPIC" ]; then
    case "$JOY_TOPIC" in
        /joy|/xlab/hr/joy_state_debug) ;;
        *)
            echo "$USAGE" >&2
            exit 1
            ;;
    esac
fi

## path ##
SOURCE_DIR="$REPO_ROOT/source"
BUILD_DIR="$SOURCE_DIR/build"
BIN="$BUILD_DIR/motion_intelligence"
BASE_YAML="$SOURCE_DIR/config/vita_boy/$V/base.yaml"

## optional overrides ##
# Modify base.yaml
if [ -n "$IS_SIM" ] || [ -n "$JOY_TOPIC" ] || [ -n "$JOY_TYPE" ]; then
    if [ ! -f "$BASE_YAML" ]; then
        echo "!! base.yaml not found: $BASE_YAML" >&2
        exit 1
    fi
    echo "==> patching $BASE_YAML"
    if [ -n "$IS_SIM" ]; then
        echo "    is_sim -> $IS_SIM"
        sed -i -E "s|^(\s*is_sim:\s*)[a-z]+|\1${IS_SIM}|" "$BASE_YAML"
    fi
    if [ -n "$JOY_TOPIC" ]; then
        echo "    joy_topic -> $JOY_TOPIC"
        sed -i -E "s|^(\s*joy_topic:\s*)\"[^\"]*\"|\1\"${JOY_TOPIC}\"|" "$BASE_YAML"
    fi
    if [ -n "$JOY_TYPE" ]; then
        echo "    joy_type -> $JOY_TYPE"
        sed -i -E "s|^(\s*joy_type:\s*)\"[^\"]*\"|\1\"${JOY_TYPE}\"|" "$BASE_YAML"
    fi
fi

## fastdds ##
if [ "$FASTDDS" = "true" ]; then
    FASTDDS_IP="$(ip -o -4 addr show scope global 2>/dev/null \
        | awk -v p="${FASTDDS_SUBNET//./\\.}" '{sub(/\/.*/,"",$4)} $4 ~ "^"p {print $4; exit}')"
    if [ -z "$FASTDDS_IP" ]; then
        echo "!! --fastdds true but no local IP found on ${FASTDDS_SUBNET}0/24 -- check the network cable / static IP, or edit FASTDDS_SUBNET in this script" >&2
        exit 1
    fi
    FASTDDS_TEMPLATE="$SCRIPT_DIR/fastdds_profile.xml.template"
    FASTDDS_PROFILE="$SCRIPT_DIR/run_motion_intelligence_fastdds.xml"
    if [ ! -f "$FASTDDS_TEMPLATE" ]; then
        echo "!! fastdds template not found: $FASTDDS_TEMPLATE" >&2
        exit 1
    fi
    sed "s|__FASTDDS_IP__|${FASTDDS_IP}|" "$FASTDDS_TEMPLATE" > "$FASTDDS_PROFILE"
    export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_PROFILE"
    echo "==> FASTRTPS_DEFAULT_PROFILES_FILE=$FASTDDS_PROFILE (DDS restricted to $FASTDDS_IP)"
fi

## source ROS2
echo "==> ROS 2: $ROS_SETUP"
source "$ROS_SETUP"

# build project (cmake + make)
echo "==> build project (cmake + make)"
mkdir -p "$BUILD_DIR"
[ -f "$BUILD_DIR/CMakeCache.txt" ] || cmake -S "$SOURCE_DIR" -B "$BUILD_DIR"
make -C "$BUILD_DIR" -j"$(nproc)"

if [ ! -x "$BIN" ]; then
    echo "!! build finished but $BIN is missing" >&2
    exit 1
fi

## clean JOY_PID ##
JOY_PID=""
cleanup() {
    if [ -n "$JOY_PID" ] && kill -0 "$JOY_PID" 2>/dev/null; then
        kill "$JOY_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

## run joy node ##
case "$JOY_TYPE" in
    game_controller)
        echo "==> input: f710 (ros2 run joy game_controller_node)"
        ros2 run joy game_controller_node &
        JOY_PID=$!
        ;;
    robot_remote_control)
        echo "==> input: remote control (joy state published by the robot itself)"
        ;;
    keyboard|*) # keyboard or other JOY_TYPE
        echo "==> input: keyboard (this terminal) -- w/s/a/d move, q/e turn,"
        ;;
esac

## run motion_intelligence ##
cd "$BUILD_DIR"
echo "==> ./motion_intelligence -v $V"
./motion_intelligence -v "$V"
