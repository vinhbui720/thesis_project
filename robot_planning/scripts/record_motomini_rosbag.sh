#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="/home/vinbui/vinh_ws"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${1:-${SCRIPT_DIR}/motomini_bag_${TIMESTAMP}}"

if compgen -G "/opt/ros/*/setup.bash" > /dev/null; then
    # Use the first installed ROS distribution found on this machine.
    source "$(compgen -G "/opt/ros/*/setup.bash" | head -n 1)"
fi

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
    source "${WORKSPACE_ROOT}/install/setup.bash"
fi

mkdir -p "$(dirname "${OUTPUT_DIR}")"

echo "Recording /motomini/* topics to: ${OUTPUT_DIR}"
echo "Stop with Ctrl+C"

exec ros2 bag record \
    -o "${OUTPUT_DIR}" \
    -e '^/motomini(/.*)?$'