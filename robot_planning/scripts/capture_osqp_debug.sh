#!/bin/bash
# Helper script to run planner with debug output and capture OSQP logs

set -e

WORKSPACE_ROOT="/home/vinbui/vinh_ws"
OUTPUT_DIR="${1:-./_osqp_debug}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DEBUG_LOG="${OUTPUT_DIR}/osqp_debug_${TIMESTAMP}.log"

# Create output directory
mkdir -p "${OUTPUT_DIR}"

echo "=========================================="
echo "OSQP Optimizer Debug Capture"
echo "=========================================="
echo "Output directory: ${OUTPUT_DIR}"
echo "Log file: ${DEBUG_LOG}"
echo ""

# Setup ROS2 environment
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"

# Run planning node with debug output captured
echo "Starting MotoMini planning node with debug enabled..."
echo "(OSQP iteration data will be logged)"
echo ""

# Run the debug GUI node which triggers optimization
python3 "${WORKSPACE_ROOT}/src/thesis_project/robot_planning/scripts/motomini_optimizer_debug_gui.py" \
    --node-filter motomini_planning_node-1 \
    2>&1 | tee "${DEBUG_LOG}"

echo ""
echo "=========================================="
echo "Debug capture complete!"
echo "Log saved: ${DEBUG_LOG}"
echo ""
echo "Next step: Parse and plot OSQP convergence"
echo "  python3 plot_osqp_convergence.py ${DEBUG_LOG} ${OUTPUT_DIR}/plots"
echo "=========================================="
