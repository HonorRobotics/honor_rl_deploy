#!/usr/bin/env bash
# =============================================================================
# build_sdk_msgs.sh — build honor_robot_sdk's common/ message packages and
#                     source them into the current shell.
#
# Usage:
#   source ./scripts/build_sdk_msgs.sh --honor-sdk /path/to/sdk            # build
#   source ./scripts/build_sdk_msgs.sh --honor-sdk /path/to/sdk rebuild    # force a clean rebuild
#   source ./scripts/build_sdk_msgs.sh --honor-sdk /path/to/sdk clean      # clean the SDK common build, then return
# =============================================================================

echo "====================================================="
## enforce sourced ##
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    echo "ERROR: source this script, do not execute it directly:" >&2
    echo "  source ${0}" >&2
    exit 1
fi

## SDK variable ##
_BSM_SCRIPT_DIR="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_BSM_REPO_ROOT="$(builtin cd "$_BSM_SCRIPT_DIR/.." && pwd)"
_BSM_ARG=""
_BSM_SDK_ARG=""

## clean variable ##
_bsm_cleanup_vars() {
    unset _BSM_SCRIPT_DIR _BSM_REPO_ROOT _BSM_SDK _BSM_ROS _BSM_COMMON _BSM_OVERLAY _BSM_ARG _BSM_SDK_ARG _p
}

## parse args ##
# [--honor-sdk /path] [rebuild|clean]
while [ $# -gt 0 ]; do
    case "$1" in
        rebuild|clean)
            _BSM_ARG="$1"
            shift
            ;;
        --honor-sdk)
            _BSM_SDK_ARG="$2"
            shift 2
            ;;
        --honor-sdk=*)
            _BSM_SDK_ARG="${1#--honor-sdk=}"
            shift
            ;;
        *)
            echo "[build_sdk_msgs] ERROR: unknown argument: $1" >&2
            _bsm_cleanup_vars; return 1
            ;;
    esac
done

## SDK path ##
_BSM_SDK="${_BSM_SDK_ARG:-${HONOR_SDK:-$(builtin cd "$_BSM_REPO_ROOT/.." && pwd)/honor_robot_sdk}}"
export HONOR_SDK="$_BSM_SDK"
_BSM_ROS="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
_BSM_COMMON="${_BSM_SDK}/common"
_BSM_OVERLAY="${_BSM_COMMON}/build_dist/common/setup.bash"

## write into ~/.bashrc ##
# persist $HONOR_SDK + auto-source ROS 2/messages into ~/.bashrc
# skipped if ~/.bashrc already has this block, so this never duplicates
if [ -f "$HOME/.bashrc" ] && ! grep -q '^export HONOR_SDK=' "$HOME/.bashrc"; then
    {
        echo ""
        echo "# >>> honor_robot_sdk (added by build_sdk_msgs.sh) >>>"
        echo "export HONOR_SDK=\"$_BSM_SDK\""
        echo "[ -f \"$_BSM_ROS\" ] && source \"$_BSM_ROS\""
        echo "[ -f \"$_BSM_OVERLAY\" ] && source \"$_BSM_OVERLAY\""
        echo "# <<< honor_robot_sdk <<<"
    } >> "$HOME/.bashrc"
    echo "[build_sdk_msgs] appended HONOR_SDK + ROS 2 + SDK message auto-source to ~/.bashrc (new terminals will have ros2/interaction_msgs ready)"
fi

## sanity checks ##
if [ ! -d "$_BSM_COMMON" ]; then
    echo "[build_sdk_msgs] ERROR: honor_robot_sdk/common not found: $_BSM_COMMON" >&2
    echo "                 pass it explicitly: source ./scripts/build_sdk_msgs.sh --honor-sdk /path/to/honor_robot_sdk" >&2
    _bsm_cleanup_vars; return 1
fi
if [ ! -f "$_BSM_ROS" ]; then
    echo "[build_sdk_msgs] ERROR: ROS 2 not found: $_BSM_ROS" >&2
    _bsm_cleanup_vars; return 1
fi

## activate uv ##
if [ -f "$_BSM_REPO_ROOT/.venv/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$_BSM_REPO_ROOT/.venv/bin/activate"
    echo "[build_sdk_msgs] venv: $VIRTUAL_ENV  ($(python3 -V 2>&1))"
else
    echo "[build_sdk_msgs] WARNING: .venv not found, run ./scripts/install.sh first" >&2
    echo "                 falling back to system python3 (must provide empy / rosidl_generator_*)" >&2
    # fallback: drop conda from PATH so its python 3.12 does not take precedence
    PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v -E 'miniforge|/conda|anaconda' | paste -sd ':')"
    export PATH
    unset PYTHONPATH PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE
fi

## source ROS setup ##
source "$_BSM_ROS"

## clean SDK build ##
if [ "$_BSM_ARG" = "clean" ]; then
    ( cd "$_BSM_COMMON" && bash ./build.sh clean )
    echo "[build_sdk_msgs] cleaned build artifacts under $_BSM_COMMON"
    _bsm_cleanup_vars; return 0
fi

## build the SDK ##
# only when "_BSM_OVERLAY" or "rebuild" build sdk， else not build
if [ "$_BSM_ARG" = "rebuild" ] || [ ! -f "$_BSM_OVERLAY" ]; then
    echo "[build_sdk_msgs] building $_BSM_COMMON ..."
    if ! ( cd "$_BSM_COMMON" && BUILD_OFFLINE=1 bash ./build.sh all ); then
        echo "[build_sdk_msgs] ERROR: SDK common/ build failed, see the log above" >&2
        _bsm_cleanup_vars; return 1
    fi
else
    echo "[build_sdk_msgs] build artifacts already present, skipping build (force: source ./scripts/build_sdk_msgs.sh rebuild)"
fi

## source the overlay ##
# source common/build_dist/common/setup.bash
if [ ! -f "$_BSM_OVERLAY" ]; then
    echo "[build_sdk_msgs] ERROR: overlay was not generated: $_BSM_OVERLAY" >&2
    _bsm_cleanup_vars; return 1
fi
source "$_BSM_OVERLAY"

## INFO ##
echo ""
echo "[build_sdk_msgs] OK — SDK message packages sourced into the current shell:"
for _p in interaction_msgs robot_msgs camera_msgs sys_monitor_msgs; do
    echo "    - $_p"
done
echo ""
echo "  build this project:"
echo "    cmake -S source -B source/build && make -C source/build -j\$(nproc)"
echo "  verify:  ros2 interface show interaction_msgs/msg/LowCommand"
echo "  overlay: $_BSM_OVERLAY"
echo ""

## clean variable ##
_bsm_cleanup_vars
echo "====================================================="
echo "==> build and install honor robot sdk successfully."
echo "====================================================="
