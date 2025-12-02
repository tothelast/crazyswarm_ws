#!/bin/bash
# Script to run Crazyswarm2 experiments and save logs to repo root

# Get the directory where this script is located (repo root)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LOG_DIR="${SCRIPT_DIR}/logs"

# Create logs directory if it doesn't exist
mkdir -p "${LOG_DIR}"

# Generate timestamp for this run
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# Log files
FULL_LOG="${LOG_DIR}/latest_flight_full.log"
FILTERED_LOG="${LOG_DIR}/latest_flight_filtered.log"
TASK_LOG="${LOG_DIR}/task_${TIMESTAMP}.log"

# Default values
SCRIPT_NAME="affine_transformation"
GUI="false"
BACKEND="cpp"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --script)
            SCRIPT_NAME="$2"
            shift 2
            ;;
        --gui)
            GUI="$2"
            shift 2
            ;;
        --backend)
            BACKEND="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--script SCRIPT_NAME] [--gui true|false] [--backend cpp|cflib|sim]"
            exit 1
            ;;
    esac
done

echo "=========================================="
echo "Starting Affine Transformation Flight"
echo "Timestamp: ${TIMESTAMP}"
echo "=========================================="
echo "Full log:     ${FULL_LOG}"
echo "Filtered log: ${FILTERED_LOG}"
echo "Task log:     ${TASK_LOG}"
echo "=========================================="
echo ""

# Run the launch command with filtering
ros2 launch affine_transformation launch.py \
    script:="${SCRIPT_NAME}" \
    gui:="${GUI}" \
    backend:="${BACKEND}" \
    2>&1 | tee "${FULL_LOG}" | while IFS= read -r line; do

        # Filter out verbose system messages
        if echo "$line" | grep -qE "SYS:|IMU:|DECK_|MTR-DRV:|EEPROM:|STORAGE:|STAB:|NRF51|Free heap|self-test|I2C connection|BMI088|BMP388|Gyro|Accel|brushed motor"; then
            continue
        fi

        # Filter out repetitive parameter updates (keep only important ones)
        if echo "$line" | grep -q "Update parameter" && ! echo "$line" | grep -qE "controller|estimator|extPosStdDev|extQuatStdDev"; then
            continue
        fi

        # Filter out TOC cache messages
        if echo "$line" | grep -qE "TOC: found cache|TOC:.*entries with CRC"; then
            continue
        fi

        # Print filtered line to console
        echo "$line"

        # Save filtered line to filtered log
        echo "$line" >> "${FILTERED_LOG}"

        # Extract task-specific messages for task log
        if echo "$line" | grep -qE "AFFINE|TAKEOFF|HOVER|CONTROL LOOP|LANDING|STABILIZ|Flight complete|✅|⚠️|❌|ERROR|WARN"; then
            echo "$line" >> "${TASK_LOG}"
        fi

        # Extract control loop metrics
        if echo "$line" | grep -qE "Control loop complete|iterations|Duration:|Rate:|gains:"; then
            echo "$line" >> "${TASK_LOG}"
        fi

        # Extract connection milestones
        if echo "$line" | grep -q "Requesting memories..."; then
            drone=$(echo "$line" | grep -oP '\[cf\d+\]' | tr -d '[]')
            echo -e "\033[1;32m[INFO] ✅ ${drone} CONNECTED AND INITIALIZED!\033[0m"
            echo "[INFO] ✅ ${drone} CONNECTED AND INITIALIZED!" >> "${TASK_LOG}"
        fi

        # Extract controller/estimator confirmations
        if echo "$line" | grep -qE "CONTROLLER: Using|ESTIMATOR: Using"; then
            echo "$line" >> "${TASK_LOG}"
        fi
    done

# Print summary
echo ""
echo "=========================================="
echo "Flight Complete!"
echo "=========================================="
echo "Full log (all messages):     ${FULL_LOG}"
echo "Filtered log (reduced):      ${FILTERED_LOG}"
echo "Task log (key events only):  ${TASK_LOG}"
echo "=========================================="
echo ""
echo "Quick view of task log:"
echo "----------------------------------------"
tail -30 "${TASK_LOG}"
echo "=========================================="

