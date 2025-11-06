#!/bin/bash
# Script to run Crazyswarm2 experiments and save logs to repo root

# Get the directory where this script is located (repo root)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LOG_DIR="${SCRIPT_DIR}/logs"

# Create logs directory if it doesn't exist
mkdir -p "${LOG_DIR}"

# Use fixed log filename (overwrites previous)
LOG_FILE="${LOG_DIR}/latest_flight.log"

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

# Run the launch command with enhanced output and save to log file
ros2 launch affine_transformation launch.py \
    script:="${SCRIPT_NAME}" \
    gui:="${GUI}" \
    backend:="${BACKEND}" \
    2>&1 | while IFS= read -r line; do
        # Print the original line
        echo "$line"

        # Check for connection milestones and print highlighted messages
        if echo "$line" | grep -q "Requesting memories..."; then
            drone=$(echo "$line" | grep -oP '\[cf\d+\]' | tr -d '[]')
            echo -e "\033[1;32m[INFO] ✅ ${drone} CONNECTED AND INITIALIZED!\033[0m"
        fi
    done | tee "${LOG_FILE}"

# Print summary
echo ""
echo "=========================================="
echo "Flight complete! Log: ${LOG_FILE}"
echo "=========================================="

