#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="/home/vinbui/vinh_ws"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${1:-${SCRIPT_DIR}/motomini_bag_${TIMESTAMP}}"

# Temporarily disable unbound variable check for ROS setup scripts
set +u
if compgen -G "/opt/ros/*/setup.bash" > /dev/null; then
    # Use the first installed ROS distribution found on this machine.
    source "$(compgen -G "/opt/ros/*/setup.bash" | head -n 1)"
fi

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
    source "${WORKSPACE_ROOT}/install/setup.bash"
fi
set -u

mkdir -p "$(dirname "${OUTPUT_DIR}")"

echo "Recording selected /motomini/* topics to: ${OUTPUT_DIR}"
echo "Stop with Ctrl+C"

exec ros2 bag record \
    -o "${OUTPUT_DIR}" \
    /motomini/target_pose \
    /motomini/target_vel \
    /motomini/feedback \
    /motomini/feedback_vel \
    /motomini/collision_distance