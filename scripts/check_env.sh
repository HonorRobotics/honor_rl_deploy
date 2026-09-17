#!/usr/bin/env bash
# Environment check for the C++ controller build.
#
# Verifies every dependency the CMake build needs (see source/CMakeLists.txt):
#   - build toolchain : cmake >= 3.10, make, gcc/g++ with C++17
#   - python / ros2   : uv, .venv (python 3.10), colcon
#   - ROS 2 Humble    : setup.bash + rclcpp / std_msgs / sensor_msgs /
#                       nav2_util / cv_bridge / ament_cmake
#   - system libs     : Eigen3, yaml-cpp, zlib, OpenCV, glog, libcurl
#   - thirdparty      : bundled ONNX Runtime for this arch
#
# Exit code: 0 if the environment is OK (warnings allowed), 1 if anything failed.
#
# Env overrides:
#   ROS_SETUP   ROS 2 setup.bash           (default: /opt/ros/humble/setup.bash)
#   PYTHON      expected uv venv python    (default: 3.10)
#   CMAKE_MIN   minimum cmake version      (default: 3.10)

echo "====================================================="
## Strict mode(exit immediately on any command failure) ##
set -uo pipefail

## cd root ##
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

## terminal INFO ##
if [ -t 1 ]; then
    RED=$'\033[0;31m'; GREEN=$'\033[32m'; YELLOW=$'\033[33m'; BOLD=$'\033[1m'; NC=$'\033[0m'
else
    RED=; GREEN=; YELLOW=; BOLD=; NC=
fi
PASS=0 WARN=0 FAIL=0
pass() { printf '  [%sPASS%s] %s\n' "$GREEN" "$NC" "$1"; PASS=$((PASS + 1)); }
warn() { printf '  [%sWARN%s] %s\n' "$YELLOW" "$NC" "$1"; WARN=$((WARN + 1)); }
fail() { printf '  [%sFAIL%s] %s\n' "$RED" "$NC" "$1"; FAIL=$((FAIL + 1)); }
section() { printf '\n%s== %s%s\n' "$BOLD" "$1" "$NC"; }
version_ge() { [ "$(printf '%s\n%s\n' "$1" "$2" | sort -V | head -n1)" = "$2" ]; }

## check OS ##
section "Operating system"
if [ -r /etc/os-release ]; then
    . /etc/os-release
    if [ "${ID:-}" = "ubuntu" ] && [ "${VERSION_ID:-}" = "22.04" ]; then
        pass "Ubuntu 22.04 (${PRETTY_NAME:-})"
    else
        warn "expected Ubuntu 22.04, found ${PRETTY_NAME:-unknown} (build may still work)"
    fi
else
    warn "cannot read /etc/os-release"
fi

## check build toolchain ##
CMAKE_MIN="${CMAKE_MIN:-3.10}" # minimum cmake version
# cmake
section "Build toolchain"
if command -v cmake >/dev/null 2>&1; then
    cmake_ver="$(cmake --version | head -n1 | awk '{print $3}')"
    if version_ge "$cmake_ver" "$CMAKE_MIN"; then
        pass "cmake $cmake_ver (>= $CMAKE_MIN)"
    else
        fail "cmake $cmake_ver is older than the required $CMAKE_MIN"
    fi
else
    fail "cmake not found  ->  sudo apt-get install -y cmake"
fi
# make
command -v make >/dev/null 2>&1 && pass "make $(make --version | head -n1 | awk '{print $NF}')" \
    || fail "make not found  ->  sudo apt-get install -y build-essential"
# gcc/g++
if command -v gcc >/dev/null 2>&1 && command -v g++ >/dev/null 2>&1; then
    pass "gcc/g++ $(gcc -dumpfullversion 2>/dev/null || gcc -dumpversion)"
    tmp="$(mktemp -d)"
    if echo 'int main(){return 0;}' | g++ -std=c++17 -x c++ - -o "$tmp/a.out" 2>"$tmp/err"; then
        pass "g++ compiles C++17"
    else
        fail "g++ cannot compile C++17: $(tr '\n' ' ' <"$tmp/err")"
    fi
    rm -rf "$tmp"
else
    fail "gcc/g++ not found  ->  sudo apt-get install -y build-essential"
fi
# pkg-config
command -v pkg-config >/dev/null 2>&1 && pass "pkg-config $(pkg-config --version)" \
    || fail "pkg-config not found  ->  sudo apt-get install -y pkg-config"

## check uv ##
PYTHON="${PYTHON:-3.10}" # python version
# check uv and uv’s colcon install
section "Python / colcon tooling"
command -v uv >/dev/null 2>&1 && pass "uv $(uv --version | awk '{print $2}')" \
    || fail "uv not found  ->  https://docs.astral.sh/uv/getting-started/installation/"

VENV="$REPO_ROOT/.venv"
if [ -d "$VENV" ] && [ -x "$VENV/bin/python" ]; then
    venv_py="$("$VENV/bin/python" -c 'import platform; print(platform.python_version())' 2>/dev/null)"
    case "$venv_py" in
        "$PYTHON"|"$PYTHON".*) pass ".venv python $venv_py" ;;
        *) warn ".venv python $venv_py (expected $PYTHON)" ;;
    esac
    colcon_bin=""
    command -v colcon >/dev/null 2>&1 && colcon_bin="colcon"
    [ -x "$VENV/bin/colcon" ] && colcon_bin="$VENV/bin/colcon"
    if [ -n "$colcon_bin" ]; then
        pass "colcon present ($colcon_bin)"
    else
        fail "colcon not found  ->  ./scripts/install.sh  (installs colcon-common-extensions in .venv)"
    fi
else
    fail ".venv missing or incomplete  ->  ./scripts/install.sh"
fi

## check ROS 2 ##
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}" # ros setup
section "ROS 2 Humble"
if [ -f "$ROS_SETUP" ]; then
    pass "ROS 2 setup: $ROS_SETUP"
    ros_share="$(dirname "$ROS_SETUP")/share"
    for pkg in rclcpp std_msgs sensor_msgs nav2_util cv_bridge ament_cmake; do
        if [ -d "$ros_share/$pkg" ]; then
            pass "ros pkg: $pkg"
        else
            fail "ros pkg missing: $pkg  ->  sudo apt-get install -y ros-humble-${pkg//_/-}"
        fi
    done
else
    fail "ROS 2 not found at $ROS_SETUP  (set ROS_SETUP or install ros-humble-desktop)"
fi

## check system libraries ##
section "System libraries (pkg-config)"
declare -A LIBS=(
    [eigen3]="libeigen3-dev"
    [yaml-cpp]="libyaml-cpp-dev"
    [zlib]="zlib1g-dev"
    [opencv4]="libopencv-dev"
    [libglog]="libgoogle-glog-dev"
    [libcurl]="libcurl4-openssl-dev"
)
for mod in "${!LIBS[@]}"; do
    if pkg-config --exists "$mod" 2>/dev/null; then
        pass "$mod $(pkg-config --modversion "$mod" 2>/dev/null)"
    else
        fail "$mod not found  ->  sudo apt-get install -y ${LIBS[$mod]}"
    fi
done

## check thirdparty ONNX Runtime ##
section "Thirdparty ONNX Runtime"
case "$(uname -m)" in
    aarch64|arm64) ort_dir="$REPO_ROOT/source/thirdparty/onnxruntime-linux-aarch64-1.22.0" ;;
    *)             ort_dir="$REPO_ROOT/source/thirdparty/onnxruntime-linux-x64-1.22.0" ;;
esac
ort_lib="$ort_dir/lib/libonnxruntime.so.1.22.0"
if [ -f "$ort_lib" ] && [ -f "$ort_dir/include/onnxruntime_cxx_api.h" ]; then
    pass "onnxruntime 1.22.0 for $(uname -m)"
else
    fail "bundled onnxruntime missing at $ort_dir  (re-clone / restore thirdparty/)"
fi

## summary ##
printf '\n%s== Summary%s\n' "$BOLD" "$NC"
printf '  %sPASS %d%s   %sWARN %d%s   %sFAIL %d%s\n' \
    "$GREEN" "$PASS" "$NC" "$YELLOW" "$WARN" "$NC" "$RED" "$FAIL" "$NC"

echo "====================================================="
if [ "$FAIL" -gt 0 ]; then
    printf '\n%sEnvironment NOT ready%s - fix the FAIL items above, then re-run.\n' "$RED" "$NC"
    exit 1
fi
if [ "$WARN" -gt 0 ]; then
    printf '\n%sEnvironment OK with warnings%s - the build should work.\n' "$YELLOW" "$NC"
    exit 0
fi
printf '\n%sEnvironment OK%s - ready to build.\n' "$GREEN" "$NC"
exit 0
