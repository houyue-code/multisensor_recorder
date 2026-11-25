
#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$PROJECT_DIR"

if [[ "$(id -u)" -eq 0 ]]; then
    echo "Please run as a normal user."
    exit 1
fi

echo "========================================"
echo " Multisensor Recorder"
echo "========================================"

# ========================================
# Load setup functions
# ========================================

source "$PROJECT_DIR/setup.sh"

# ========================================
# Detect ROS2 version
# ========================================

detect_ros

# ========================================
# Check and install dependencies
# ========================================

if ! check_environment; then

    echo "[Setup] Missing dependencies detected."

    setup_environment

    if ! check_environment; then

        echo "Error: Environment setup incomplete."

        exit 1

    fi

fi

echo "[OK] Environment is ready."

# ========================================
# Configure USB permissions
# ========================================

configure_permissions

# ========================================
# Activate USB permissions automatically
# ========================================

if ! id -nG | grep -qw dialout || \
   ! id -nG | grep -qw video; then

    if [[ "${1:-}" == "--groups-refreshed" ]]; then

        echo "Error: USB permissions could not be activated."
        echo "Please log out and log in again."

        exit 1

    fi

    echo "[Setup] Activating USB permissions..."

    exec sg dialout -c \
        'exec sg video -c "exec bash ./start.sh --groups-refreshed"'

fi

echo "[OK] USB permissions activated."

# ========================================
# Load ROS2 environment
# ========================================

load_environment

# ========================================
# DDS configuration
# ========================================

export RMW_IMPLEMENTATION="${MULTISENSOR_RMW:-rmw_fastrtps_cpp}"

if [[ "$RMW_IMPLEMENTATION" == "rmw_fastrtps_cpp" ]]; then

    unset CYCLONEDDS_URI

fi

echo "[OK] DDS: $RMW_IMPLEMENTATION"

# ========================================
# Qt configuration
# ========================================

export RCUTILS_COLORIZED_OUTPUT=0

if [[ -n "${DISPLAY:-}" ]]; then

    export QT_QPA_PLATFORM=xcb

fi

# ========================================
# Create data directory
# ========================================

mkdir -p "$HOME/experiment_data"

# ========================================
# Start application
# ========================================

echo "[OK] Starting Multisensor Recorder..."
echo "[OK] ROS2 version: $DISTRO"

exec /usr/bin/python3 "$PROJECT_DIR/export.py"
