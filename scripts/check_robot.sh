#!/usr/bin/env bash
# Connectivity check for the real robot — network, SSH, service, ROS 2 topics.
#
# Runs three layers, each independent:
#   1. network : local IP on the robot subnet, ping, SSH port
#   2. ssh     : log in, check motion_intelligence.service
#   3. ROS 2   : ros2 daemon reachable, the low_state / low_cmd / imu / joy
#                topics from base.yaml are present, low_state is publishing
#
# Exit code: 0 if the robot is reachable (warnings allowed), 1 if a check failed.
#
# Usage:
#   ./scripts/check_robot.sh \
#       --joy-check <true|false> \
#       --joy-topic </xlab/hr/joy_state_debug|/joy>
#
#   Everything else below is a fixed parameter -- edit here if needed.

echo "====================================================="
## Strict mode(exit immediately on any command failure) ##
set -uo pipefail

## cd root ##
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

## parameters ##
ROBOT_IP=192.168.42.201
ROBOT_USER=hihonor
ROBOT_PASS=hihonor
HOST_IP=192.168.42.
V=V1
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
HZ_SECS=3

## joy-check and --joy-topic are required ##
JOY_CHECK=""
JOY_TOPIC=""
USAGE="usage: $0 --joy-check <true|false> --joy-topic </joy|/xlab/hr/joy_state_debug>"
while [ $# -gt 0 ]; do
    case "$1" in
        --joy-check) JOY_CHECK="$2"; shift 2 ;;
        --joy-check=*) JOY_CHECK="${1#--joy-check=}"; shift ;;
        --joy-topic) JOY_TOPIC="$2"; shift 2 ;;
        --joy-topic=*) JOY_TOPIC="${1#--joy-topic=}"; shift ;;
        *)
            echo "$USAGE" >&2
            exit 1
            ;;
    esac
done
if [ -z "$JOY_TOPIC" ]; then
    echo "$USAGE" >&2
    exit 1
fi
case "$JOY_CHECK" in
    true|false) ;;
    *)
        echo "$USAGE" >&2
        exit 1
        ;;
esac

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

# read base.yaml
BASE_YAML="$REPO_ROOT/source/config/vita_boy/$V/base.yaml"
yaml_topic() {
    [ -f "$BASE_YAML" ] || return 1
    awk -v k="$1" '$1 == k":" { v = $2; gsub(/["'\'']/, "", v); print v; exit }' "$BASE_YAML"
}

printf '%sRobot connectivity check%s  ->  %s@%s  (config %s)\n' \
    "$BOLD" "$NC" "$ROBOT_USER" "$ROBOT_IP" "$V"

## network ##
section "Network"
# ip
subnet="${ROBOT_IP%.*}."
local_ips="$(ip -o -4 addr show scope global 2>/dev/null | awk '{sub(/\/.*/,"",$4); print $4}')"
if printf '%s\n' "$local_ips" | grep -q "^${HOST_IP//./\\.}"; then
    pass "local IP on ${HOST_IP}x (have: $(echo $local_ips | tr '\n' ' '))"
else
    fail "no local IP on ${subnet}0/24 (have: $(echo $local_ips | tr '\n' ' ')) — check the network cable / static IP"
fi

if ping -c 2 -W 2 "$ROBOT_IP" >/dev/null 2>&1; then
    pass "ping $ROBOT_IP"
else
    fail "cannot ping $ROBOT_IP — robot powered on? cable connected?"
fi

if timeout 3 bash -c "exec 3<>/dev/tcp/$ROBOT_IP/22" 2>/dev/null; then
    pass "SSH port 22 open on $ROBOT_IP"
else
    fail "SSH port 22 closed/filtered on $ROBOT_IP"
fi

# ssh + service
section "SSH & service"
SSH_OPTS="-o ConnectTimeout=6 -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR"
ssh_run() { ssh $SSH_OPTS -o BatchMode=yes "$ROBOT_USER@$ROBOT_IP" "$@"; }
if command -v sshpass >/dev/null 2>&1; then
    ssh_run() { sshpass -p "$ROBOT_PASS" ssh $SSH_OPTS "$ROBOT_USER@$ROBOT_IP" "$@"; }
fi

# check motion_intelligence.service
if out="$(ssh_run 'echo ok' 2>/dev/null)" && [ "$out" = "ok" ]; then
    pass "ssh $ROBOT_USER@$ROBOT_IP"
    state="$(ssh_run 'systemctl is-active motion_intelligence.service' 2>/dev/null || true)"
    case "$state" in
        inactive) pass "motion_intelligence.service: inactive (stopped, ready for your own controller)" ;;
        active)   fail "motion_intelligence.service: active — stop it before running your own controller (systemctl stop motion_intelligence.service)" ;;
        failed)   warn "motion_intelligence.service: failed — check: journalctl -u motion_intelligence.service" ;;
        "")       warn "could not query motion_intelligence.service" ;;
        *)        warn "motion_intelligence.service: $state" ;;
    esac
else
    if command -v sshpass >/dev/null 2>&1; then
        fail "ssh login failed (user/pass? edit ROBOT_PASS in this script)"
    else
        warn "ssh needs a key or 'sshpass' (apt-get install sshpass, or set up an SSH key) — skipped service check"
    fi
fi

## check ROS 2 ##
section "ROS 2"
if [ ! -f "$ROS_SETUP" ]; then
    fail "ROS 2 not found at $ROS_SETUP — run this from a machine with ROS 2, or edit ROS_SETUP in this script"
else
    set +u                       # ROS setup scripts trip nounset
    source "$ROS_SETUP"
    set -u
    if ! ros2 interface show interaction_msgs/msg/LowState >/dev/null 2>&1; then
        warn "honor_robot_sdk messages not on the ROS 2 path — run 'source ./scripts/build_sdk_msgs.sh' in this terminal first (custom msg topics below may fail to subscribe)"
    fi
    # topic check
    topics=""
    for _attempt in 1 2 3; do
        _t="$(timeout 8 ros2 topic list 2>/dev/null || true)"
        topics="$(printf '%s\n%s\n' "$topics" "$_t" | sed '/^$/d' | sort -u)"
        [ "$_attempt" -lt 3 ] && sleep 2
    done
    unset _attempt _t
    if [ -z "$topics" ]; then
        fail "no ROS 2 topics visible — DDS discovery blocked (domain id / RMW / firewall / not on same subnet)"
    else
        n="$(printf '%s\n' "$topics" | grep -c .)"
        pass "ros2 topic list: $n topics visible"
        for key in lowstate_topic lowcmd_topic imu_topic; do
            t="$(yaml_topic "$key")"
            [ -z "$t" ] && { warn "$key not found in $(basename "$BASE_YAML")"; continue; }
            if printf '%s\n' "$topics" | grep -qx "$t"; then
                pass "topic present: $t"
            else
                fail "topic missing: $t ($key)"
            fi
        done

        if [ "$JOY_CHECK" = "false" ]; then
            pass "joy topic check skipped (--joy-check false) — use keyboard or F710"
        elif printf '%s\n' "$topics" | grep -qx "$JOY_TOPIC"; then
            pass "topic present: $JOY_TOPIC (joy_topic, sim2real)"
        else
            fail "topic missing: $JOY_TOPIC (joy_topic, sim2real)"
        fi

        ls_topic="$(yaml_topic lowstate_topic)"
        if [ -n "$ls_topic" ] && printf '%s\n' "$topics" | grep -qx "$ls_topic"; then
            hz="$(timeout $((HZ_SECS + 3)) ros2 topic hz -w 20 "$ls_topic" 2>/dev/null | grep -m1 'average rate' | awk '{print $3}')"
            if [ -n "$hz" ]; then
                pass "$ls_topic publishing (~${hz} Hz)"
            else
                fail "$ls_topic listed but no data in ${HZ_SECS}s — state bridge / robot controller down"
            fi
        fi
    fi
fi

## summary ##
printf '\n%s== Summary%s\n' "$BOLD" "$NC"
printf '  %sPASS %d%s   %sWARN %d%s   %sFAIL %d%s\n' \
    "$GREEN" "$PASS" "$NC" "$YELLOW" "$WARN" "$NC" "$RED" "$FAIL" "$NC"

echo "====================================================="
if [ "$FAIL" -gt 0 ]; then
    printf '\n%sChecks failed%s — fix the FAIL items above.\n' "$RED" "$NC"
    exit 1
fi
if [ "$WARN" -gt 0 ]; then
    printf '\n%sRobot reachable with warnings%s.\n' "$YELLOW" "$NC"
    exit 0
fi
printf '\n%sRobot reachable%s — network, SSH and ROS 2 all OK.\n' "$GREEN" "$NC"
exit 0
