#!/usr/bin/env bash
# Install dependencies: uv env + build libraries + ros2 extension.
#
# Before running the script, please install UV and Ros2.
# The installation address is as follows:
# - uv      https://docs.astral.sh/uv/getting-started/installation/
# - ROS 2   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

echo "====================================================="
## Strict mode(exit immediately on any command failure) ##
set -euo pipefail

## cd root ##
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

## check ros and uv ##
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}" # ros setup
PYTHON="${PYTHON:-3.10}" # python version

# check ros2 install
if [ ! -f "$ROS_SETUP" ]; then
    echo "Error: ROS 2 not found at $ROS_SETUP" >&2
    echo "       Install ROS 2 Humble: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html" >&2
    echo "       or point ROS_SETUP at your setup.bash." >&2
    exit 1
fi
echo "==> ROS 2: $ROS_SETUP"

# check uv install
if ! command -v uv >/dev/null 2>&1; then
    echo "Error: uv not found." >&2
    echo "       Install it: https://docs.astral.sh/uv/getting-started/installation/" >&2
    exit 1
fi
echo "==> uv: $(uv --version)"
# create uv environment
if [ ! -d "$REPO_ROOT/.venv" ]; then
    echo "==> uv venv --python $PYTHON"
    uv venv --python "$PYTHON"
fi
# source uv environment
set +u
source "$REPO_ROOT/.venv/bin/activate"
set -u
echo "==> Environment: $VIRTUAL_ENV"
echo "==> Python: $(python -V 2>&1)"

## install build libraries ##
echo "==> build libraries"
sudo apt-get install -y \
    libgoogle-glog-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    zlib1g-dev \
    libopencv-dev

## ros2 extension install ##
echo "==> ros2 extension"
uv pip install -U colcon-common-extensions
uv pip install catkin_pkg numpy lark setuptools==58.2.0
uv pip uninstall -y empy || true          # do not abort if empy is not installed
uv pip install empy==3.3.4
uv pip install matplotlib

## dev tooling ##
echo "==> dev tooling"
uv tool install clang-format || echo "==> WARNING: failed to install clang-format"

echo "==> Dependencies installed successfully."
echo "====================================================="
