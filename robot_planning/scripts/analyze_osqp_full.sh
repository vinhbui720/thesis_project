#!/bin/bash
# Master script: Capture OSQP debug logs and generate plots in one go

set -e

WORKSPACE_ROOT="/home/vinbui/vinh_ws"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR="${1:-./_osqp_analysis_${TIMESTAMP}}"

mkdir -p "${OUTPUT_DIR}"

echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║       OSQP Optimizer Analysis Pipeline                 ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""
echo "📁 Output directory: ${OUTPUT_DIR}"
echo ""

# Setup environment
source /opt/ros/humble/setup.bash || echo "WARNING: ROS2 setup not found"
source "${WORKSPACE_ROOT}/install/setup.bash" || echo "WARNING: Workspace setup not found"

# Step 1: Capture logs
LOG_FILE="${OUTPUT_DIR}/osqp_debug_${TIMESTAMP}.log"
echo "📝 Step 1/3: Capturing OSQP debug output to file..."
echo "   Log file: ${LOG_FILE}"
echo ""

python3 "${WORKSPACE_ROOT}/src/thesis_project/robot_planning/scripts/motomini_optimizer_debug_gui.py" \
    --node-filter motomini_planning_node-1 \
    2>&1 | tee "${LOG_FILE}" || true

echo ""
echo "✅ Debug capture complete"
echo ""

# Step 2: Parse and generate plots
PLOTS_DIR="${OUTPUT_DIR}/plots"
echo "📊 Step 2/3: Parsing log and generating plots..."
echo "   Plots directory: ${PLOTS_DIR}"
echo ""

python3 "${WORKSPACE_ROOT}/src/thesis_project/robot_planning/scripts/plot_osqp_convergence.py" \
    "${LOG_FILE}" \
    "${PLOTS_DIR}" || {
    echo "⚠️  Plot generation failed. Try installing dependencies:"
    echo "    pip3 install matplotlib numpy"
}

echo ""
echo "✅ Plot generation complete"
echo ""

# Step 3: Summary
echo "📋 Step 3/3: Analysis Summary"
echo "════════════════════════════════════════════════════════"
echo ""
echo "Generated files:"
ls -lh "${OUTPUT_DIR}"/ 2>/dev/null || echo "  (check output directory)"

if [ -d "${PLOTS_DIR}" ]; then
    ls -lh "${PLOTS_DIR}"/ 2>/dev/null
fi

echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "✅ Analysis Complete!"
echo "╚════════════════════════════════════════════════════════╝"
echo ""
echo "📌 Next steps:"
echo ""
echo "   1. View convergence plots:"
echo "      eog ${PLOTS_DIR}/osqp_convergence.png"
echo "      eog ${PLOTS_DIR}/osqp_summary.png"
echo ""
echo "   2. Analyze raw data (JSON):"
echo "      cat ${PLOTS_DIR}/osqp_iterations.json | python3 -m json.tool | less"
echo ""
echo "   3. Custom plotting (Python):"
echo "      python3 << 'EOF'"
echo "import json"
echo "with open('${PLOTS_DIR}/osqp_iterations.json') as f:"
echo "    data = json.load(f)"
echo "    # Your custom analysis here"
echo "EOF"
echo ""
echo "📚 Full documentation: ${WORKSPACE_ROOT}/OSQP_DEBUG_GUIDE.md"
echo ""
