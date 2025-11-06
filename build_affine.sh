#!/bin/bash
# Script to rebuild the affine_transformation package

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=========================================="
echo "Building affine_transformation package..."
echo "=========================================="

cd "${SCRIPT_DIR}"

# Build only the affine_transformation package
colcon build --packages-select affine_transformation --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✅ Build successful!"
    echo "=========================================="
    echo ""
    echo "You can now run the experiment with:"
    echo "  ./run_and_log.sh"
else
    echo ""
    echo "=========================================="
    echo "❌ Build failed!"
    echo "=========================================="
    exit 1
fi

