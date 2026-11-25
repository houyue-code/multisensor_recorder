
#!/bin/bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ========================================
# Detect Ubuntu and ROS2 version
# ========================================

detect_ros() {

    source /etc/os-release

    if [[ "$ID" != "ubuntu" ]]; then
        echo "Error: Ubuntu is required."
        return 1
    fi

    case "$VERSION_ID" in

        "22.04")
            DISTRO="humble"
            ;;

        "24.04")
            DISTRO="jazzy"
            ;;

        *)
            echo "Error: Unsupported Ubuntu version."
            echo "Supported: Ubuntu 22.04 and 24.04."
            return 1
            ;;

    esac

    if [[ ! -f "/opt/ros/$DISTRO/setup.bash" ]]; then
        echo "Error: ROS2 $DISTRO is not installed."
        return 1
    fi

    if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != "$DISTRO" ]]; then
        echo "Error: Incorrect ROS2 environment."
        echo "Please open a clean terminal."
        return 1
    fi

    WORKSPACE="$HOME/.local/share/multisensor_recorder/$DISTRO/ros2_ws"

    echo "[OK] Ubuntu $VERSION_ID"
    echo "[OK] ROS2 $DISTRO"
}

# ========================================
# Load ROS2 environment
# ========================================

load_environment() {

    source "/opt/ros/$DISTRO/setup.bash" || return 1

    source "$WORKSPACE/install/local_setup.bash" || return 1
}

# ========================================
# Check installed dependencies
# ========================================

check_environment() (

    [[ -f "$WORKSPACE/install/local_setup.bash" ]] || return 1

    load_environment || return 1

    for package in \
        leptrino_force_sensor \
        usb_cam \
        rosbag2_storage_mcap \
        rmw_cyclonedds_cpp \
        rmw_fastrtps_cpp
    do

        ros2 pkg prefix "$package" \
            >/dev/null 2>&1 || return 1

    done

    /usr/bin/python3 - <<'PY' >/dev/null 2>&1 || return 1
import rclpy
import rosbag2_py
import cv2
import numpy
import yaml
import serial
import PyQt5
import cv_bridge

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Image

assert "mcap" in rosbag2_py.get_registered_writers()
PY

)

# ========================================
# Configure USB permissions
# ========================================

configure_permissions() {

    CURRENT_USER="$(id -un)"

    if ! id -nG "$CURRENT_USER" | grep -qw dialout || \
       ! id -nG "$CURRENT_USER" | grep -qw video; then

        echo "[Setup] Configuring USB permissions..."

        sudo usermod -aG dialout,video "$CURRENT_USER"

    fi
}

# ========================================
# Install required packages
# ========================================

setup_environment() (

    source "/opt/ros/$DISTRO/setup.bash"

    echo "[Setup] Installing ROS2 $DISTRO dependencies..."

    sudo apt update

    sudo apt install -y \
        git \
        build-essential \
        cmake \
        python3-colcon-common-extensions \
        python3-pyqt5 \
        python3-opencv \
        python3-numpy \
        python3-yaml \
        python3-serial \
        v4l-utils \
        "ros-$DISTRO-ament-cmake" \
        "ros-$DISTRO-rclcpp" \
        "ros-$DISTRO-rclpy" \
        "ros-$DISTRO-geometry-msgs" \
        "ros-$DISTRO-sensor-msgs" \
        "ros-$DISTRO-std-srvs" \
        "ros-$DISTRO-usb-cam" \
        "ros-$DISTRO-cv-bridge" \
        "ros-$DISTRO-rosbag2" \
        "ros-$DISTRO-rosbag2-py" \
        "ros-$DISTRO-rosbag2-storage-mcap" \
        "ros-$DISTRO-rmw-cyclonedds-cpp" \
        "ros-$DISTRO-rmw-fastrtps-cpp"

    # ====================================
    # Install Leptrino driver
    # ====================================

    mkdir -p "$WORKSPACE/src"

    DRIVER="$WORKSPACE/src/leptrino_force_sensor"

    OLD_DRIVER="$HOME/ros2_ws/src/leptrino_force_sensor"

    if [[ ! -d "$DRIVER" ]]; then

        if [[ -f "$OLD_DRIVER/package.xml" ]]; then

            echo "[Setup] Copying existing Leptrino source..."

            cp -a "$OLD_DRIVER" "$DRIVER"

        else

            echo "[Setup] Downloading Leptrino driver..."

            git clone \
                --branch humble_devel \
                https://github.com/open-rdc/leptrino_force_sensor.git \
                "$DRIVER"

        fi

    fi

    # ====================================
    # Build driver
    # ====================================

    cd "$WORKSPACE"

    colcon build \
        --packages-select leptrino_force_sensor

    # ====================================
    # Configure permissions and directories
    # ====================================

    configure_permissions

    mkdir -p "$HOME/experiment_data"

    chmod +x "$PROJECT_DIR/start.sh"
    chmod +x "$PROJECT_DIR/setup.sh"

    echo "[OK] Setup completed."

)

# ========================================
# Run setup directly
# ========================================

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then

    if [[ "$(id -u)" -eq 0 ]]; then
        echo "Please run as a normal user."
        exit 1
    fi

    detect_ros

    setup_environment

    echo "Run bash start.sh to launch the application."

fi
