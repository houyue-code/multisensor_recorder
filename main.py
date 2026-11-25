
import sys
import os
import signal
import subprocess
import time
import threading
import queue
import glob
import shlex

from collections import deque

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Image

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QMessageBox,
    QProgressBar,
    QPlainTextEdit,
    QComboBox,
    QSplitter,
    QInputDialog,
    QSizePolicy
)

class MainWindow(QWidget):

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Multisensor Recorder")
        self.resize(1200, 780)

        self.node = Node("multisensor_recorder")

        self.processes = {
            "force": None,
            "camera": None
        }

        self.subscriptions = {
            "force": None,
            "camera": None
        }

        self.log_queue = queue.Queue()

        self.started = {
            "force": 0.0,
            "camera": 0.0
        }

        self.last = {
            "force": 0.0,
            "camera": 0.0
        }

        self.generation = {
            "force": 0,
            "camera": 0
        }

        self.calibrated = False
        self.calibrated_at = 0.0

        self.force_values = None
        self.image = None

        self.frame_times = deque(maxlen=100)

        self.image_error = ""
        self.closing = False

        # Main layout
        layout = QVBoxLayout(self)

        splitter = QSplitter(Qt.Horizontal)

        splitter.addWidget(
            self.build_force_panel()
        )

        splitter.addWidget(
            self.build_camera_panel()
        )

        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)

        layout.addWidget(splitter, 3)

        # Driver log
        log_group = QGroupBox("Driver Log")

        log_layout = QVBoxLayout(log_group)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)

        self.log.document().setMaximumBlockCount(500)

        log_layout.addWidget(self.log)

        layout.addWidget(log_group, 1)

        # Update timer
        self.timer = QTimer(self)

        self.timer.timeout.connect(self.update)

        self.timer.start(20)

        self.append_log(
            "[System] USB port auto-detection enabled."
        )

        self.append_log(
            f"[System] Running: {os.path.realpath(__file__)}"
        )

    # ==========================================
    # Force sensor panel
    # ==========================================

    def build_force_panel(self):

        group = QGroupBox("Force Sensor - Leptrino")

        layout = QVBoxLayout(group)

        self.force_status = QLabel(
            "Status: Disconnected"
        )

        self.force_message = QLabel(
            "Connect the sensor to start calibration."
        )

        self.force_message.setWordWrap(True)

        self.progress = QProgressBar()

        self.progress.setRange(0, 0)
        self.progress.hide()

        layout.addWidget(self.force_status)
        layout.addWidget(self.force_message)
        layout.addWidget(self.progress)

        # Connection buttons
        row = QHBoxLayout()

        self.force_connect = QPushButton(
            "Connect"
        )

        self.force_recalibrate = QPushButton(
            "Recalibrate"
        )

        self.force_disconnect = QPushButton(
            "Disconnect"
        )

        self.force_connect.clicked.connect(
            self.connect_force
        )

        self.force_recalibrate.clicked.connect(
            self.recalibrate
        )

        self.force_disconnect.clicked.connect(
            lambda: self.stop("force")
        )

        for button in (
            self.force_connect,
            self.force_recalibrate,
            self.force_disconnect
        ):

            row.addWidget(button)

        self.force_recalibrate.setEnabled(False)

        self.force_disconnect.setEnabled(False)

        layout.addLayout(row)

        # Force and torque values
        grid = QGridLayout()

        self.force_labels = []

        names = (
            "Fx (N)",
            "Fy (N)",
            "Fz (N)",
            "Mx (N·m)",
            "My (N·m)",
            "Mz (N·m)"
        )

        for i, name in enumerate(names):

            title = QLabel(name)

            value = QLabel("--")

            value.setAlignment(
                Qt.AlignCenter
            )

            value.setStyleSheet(
                "font-size: 25px;"
                "border: 1px solid gray;"
                "padding: 8px;"
            )

            self.force_labels.append(value)

            grid.addWidget(title, i, 0)
            grid.addWidget(value, i, 1)

        layout.addLayout(grid)

        layout.addStretch()

        return group

    # ==========================================
    # USB camera panel
    # ==========================================

    def build_camera_panel(self):

        group = QGroupBox("USB Camera")

        layout = QVBoxLayout(group)

        self.camera_status = QLabel(
            "Status: Disconnected"
        )

        layout.addWidget(self.camera_status)

        grid = QGridLayout()

        # USB camera device
        self.device = QComboBox()

        self.device.setEditable(True)

        self.refresh = QPushButton("Refresh")

        self.refresh.clicked.connect(
            self.refresh_devices
        )

        self.refresh_devices()

        # Resolution
        self.resolution = QComboBox()

        self.resolution.addItems([
            "640 x 480",
            "640 x 360",
            "352 x 288",
            "320 x 240",
            "800 x 600"
        ])

        # Frame rate
        self.fps = QComboBox()

        self.fps.addItems([
            "15",
            "25",
            "30"
        ])

        grid.addWidget(
            QLabel("Device"), 0, 0
        )

        grid.addWidget(
            self.device, 0, 1
        )

        grid.addWidget(
            self.refresh, 0, 2
        )

        grid.addWidget(
            QLabel("Resolution"), 1, 0
        )

        grid.addWidget(
            self.resolution, 1, 1
        )

        grid.addWidget(
            QLabel("Frame rate (FPS)"), 2, 0
        )

        grid.addWidget(
            self.fps, 2, 1
        )

        layout.addLayout(grid)

        # Camera connection buttons
        row = QHBoxLayout()

        self.camera_connect = QPushButton(
            "Connect"
        )

        self.camera_disconnect = QPushButton(
            "Disconnect"
        )

        self.camera_connect.clicked.connect(
            self.connect_camera
        )

        self.camera_disconnect.clicked.connect(
            lambda: self.stop("camera")
        )

        self.camera_disconnect.setEnabled(False)

        row.addWidget(self.camera_connect)
        row.addWidget(self.camera_disconnect)

        layout.addLayout(row)

        # Received frame rate
        self.actual_fps = QLabel(
            "Received FPS: --"
        )

        layout.addWidget(self.actual_fps)

        # Camera preview
        self.preview = QLabel(
            "No camera image"
        )

        self.preview.setAlignment(
            Qt.AlignCenter
        )

        self.preview.setMinimumSize(
            320, 240
        )

        self.preview.setStyleSheet(
            "background: #202020;"
            "color: white;"
        )

        self.preview.setSizePolicy(
            QSizePolicy.Ignored,
            QSizePolicy.Ignored
        )

        layout.addWidget(self.preview, 1)

        return group

    # ==========================================
    # Refresh camera devices
    # ==========================================

    def refresh_devices(self):

        current = self.device.currentText()

        devices = sorted(
            glob.glob("/dev/video*")
        )

        self.device.clear()

        self.device.addItems(
            devices or ["/dev/video0"]
        )

        if current and current in devices:

            self.device.setCurrentText(current)

    # ==========================================
    # Update device status
    # ==========================================

    def status(self, kind, text, color="gray"):

        if kind == "force":

            label = self.force_status

        else:

            label = self.camera_status

        label.setText(
            "Status: " + text
        )

        label.setStyleSheet(
            "font-size: 16px;"
            f"color: {color};"
        )

    # ==========================================
    # Driver log
    # ==========================================

    def append_log(self, message):

        self.log.appendPlainText(message)

    # ==========================================
    # Launch ROS2 driver
    # ==========================================

    def launch(
        self,
        kind,
        command,
        msg_type,
        topic,
        callback
    ):

        if self.processes[kind] is not None:

            return False

        # Check existing publishers
        if self.node.count_publishers(topic):

            QMessageBox.warning(
                self,
                "Topic already active",
                f"{topic} already has a publisher.\n"
                "Stop the existing driver first."
            )

            return False

        self.generation[kind] += 1

        generation = self.generation[kind]

        self.started[kind] = time.monotonic()

        self.last[kind] = 0.0

        env = os.environ.copy()

        env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"

        env["RCUTILS_COLORIZED_OUTPUT"] = "0"

        try:

            self.subscriptions[kind] = (
                self.node.create_subscription(
                    msg_type,
                    topic,
                    callback,
                    qos_profile_sensor_data
                )
            )

            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
                bufsize=1,
                start_new_session=True,
                env=env
            )

            self.processes[kind] = process

            threading.Thread(
                target=self.read_logs,
                args=(
                    kind,
                    generation,
                    process
                ),
                daemon=True
            ).start()

            self.append_log(
                f"[{kind.capitalize()}] Starting driver..."
            )

            return True

        except Exception as exc:

            self.stop(kind)

            self.status(
                kind,
                "Connection failed",
                "red"
            )

            self.append_log(
                f"[{kind.capitalize()}] {exc}"
            )

            return False

    # ==========================================
    # Read driver output
    # ==========================================

    def read_logs(
        self,
        kind,
        generation,
        process
    ):

        try:

            with process.stdout as stream:

                for line in stream:

                    self.log_queue.put(
                        (
                            kind,
                            generation,
                            line.rstrip()
                        )
                    )

        except Exception as exc:

            self.log_queue.put(
                (
                    kind,
                    generation,
                    str(exc)
                )
            )

    # ==========================================
    # Connect force sensor
    # ==========================================

    def connect_force(self):

        if self.processes["force"] is not None:

            return

        result = QMessageBox.information(
            self,
            "Force Sensor Calibration",
            "Keep the sensor stationary and unloaded.\n"
            "Do not apply external force until "
            "calibration is complete.\n\n"
            "Press OK when ready.",
            QMessageBox.Ok | QMessageBox.Cancel
        )

        if result != QMessageBox.Ok:

            return

        self.start_force()

    # ==========================================
    # Automatically detect USB serial port
    # ==========================================

    def detect_force_port(self):

        ports = sorted(
            set(
                glob.glob("/dev/ttyACM[0-9]*")
                + glob.glob("/dev/ttyUSB[0-9]*")
            )
        )

        self.append_log(
            "[Force] Detected USB serial ports: "
            + (", ".join(ports) or "none")
        )

        # No serial device
        if not ports:

            self.status(
                "force",
                "Connection failed",
                "red"
            )

            QMessageBox.warning(
                self,
                "Force Sensor",
                "No USB serial device found.\n"
                "Check the USB cable and reconnect."
            )

            return None

        # Automatically select a single port
        if len(ports) == 1:

            port = ports[0]

        # Select manually if multiple ports exist
        else:

            port, ok = QInputDialog.getItem(
                self,
                "Force Sensor",
                "Select the Leptrino USB port:",
                ports,
                0,
                False
            )

            if not ok:

                return None

        # Check device permissions
        if not os.access(
            port,
            os.R_OK | os.W_OK
        ):

            self.status(
                "force",
                "Permission denied",
                "red"
            )

            QMessageBox.warning(
                self,
                "Force Sensor",
                f"Cannot access {port}.\n\n"
                "Run the following command:\n\n"
                "sudo usermod -aG dialout $USER\n\n"
                "Then log out and log in again."
            )

            return None

        self.append_log(
            f"[Force] Selected serial port: {port}"
        )

        return port

    # ==========================================
    # Start force sensor driver
    # ==========================================

    def start_force(self):

        if self.processes["force"] is not None:

            return

        # Detect the port on every connection
        port = self.detect_force_port()

        if port is None:

            return

        self.calibrated = False

        self.force_values = None

        # Pass the detected serial port to ROS2
        command = [
            "ros2",
            "run",
            "leptrino_force_sensor",
            "leptrino_force_sensor_node",
            "--ros-args",
            "-p",
            f"port:={port}"
        ]

        self.append_log(
            "[Force] Command: "
            + shlex.join(command)
        )

        success = self.launch(
            "force",
            command,
            WrenchStamped,
            "/force_data",
            self.force_callback
        )

        if success:

            self.force_connect.setEnabled(False)

            self.force_disconnect.setEnabled(True)

            self.force_recalibrate.setEnabled(False)

            self.progress.show()

            self.status(
                "force",
                "Calibrating...",
                "orange"
            )

            self.force_message.setText(
                "Do not apply any external force."
            )

    # ==========================================
    # Recalibrate force sensor
    # ==========================================

    def recalibrate(self):

        result = QMessageBox.question(
            self,
            "Recalibrate",
            "Keep the sensor stationary and unloaded.\n"
            "Restart calibration?",
            QMessageBox.Yes | QMessageBox.No
        )

        if result == QMessageBox.Yes:

            self.stop("force")

            self.start_force()

    # ==========================================
    # Receive force sensor data
    # ==========================================

    def force_callback(self, msg):

        if not self.calibrated:

            return

        if self.processes["force"] is None:

            return

        f = msg.wrench.force
        t = msg.wrench.torque

        self.force_values = (
            f.x,
            f.y,
            f.z,
            t.x,
            t.y,
            t.z
        )

        self.last["force"] = time.monotonic()

    # ==========================================
    # Connect USB camera
    # ==========================================

    def connect_camera(self):

        device = self.device.currentText().strip()

        if not os.path.exists(device):

            QMessageBox.warning(
                self,
                "Camera",
                "Camera device not found: " + device
            )

            return

        width, height = map(
            int,
            self.resolution.currentText().split(" x ")
        )

        fps = float(
            self.fps.currentText()
        )

        if width == 800 and fps != 15:

            QMessageBox.warning(
                self,
                "Camera",
                "800 x 600 YUYV supports 15 FPS "
                "on the tested camera."
            )

            return

        self.frame_times.clear()

        self.image = None

        self.image_error = ""

        command = [
            "ros2",
            "run",
            "usb_cam",
            "usb_cam_node_exe",
            "--ros-args",
            "-p",
            f"video_device:={device}",
            "-p",
            f"image_width:={width}",
            "-p",
            f"image_height:={height}",
            "-p",
            f"framerate:={fps}",
            "-p",
            "pixel_format:=yuyv"
        ]

        self.append_log(
            "[Camera] Command: "
            + shlex.join(command)
        )

        success = self.launch(
            "camera",
            command,
            Image,
            "/image_raw",
            self.camera_callback
        )

        if success:

            self.set_camera_controls(True)

            self.status(
                "camera",
                "Connecting...",
                "orange"
            )

            self.preview.setText(
                "Waiting for image..."
            )

    # ==========================================
    # Enable / disable camera settings
    # ==========================================

    def set_camera_controls(self, active):

        self.camera_connect.setEnabled(
            not active
        )

        self.camera_disconnect.setEnabled(
            active
        )

        for control in (
            self.device,
            self.refresh,
            self.resolution,
            self.fps
        ):

            control.setEnabled(not active)

    # ==========================================
    # Receive camera images
    # ==========================================

    def camera_callback(self, msg):

        if self.processes["camera"] is None:

            return

        try:

            encoding = msg.encoding.lower()

            channels = {
                "yuv422_yuy2": 2,
                "yuyv": 2,
                "yuy2": 2,
                "yuv422": 2,
                "rgb8": 3,
                "bgr8": 3,
                "mono8": 1,
                "rgba8": 4,
                "bgra8": 4
            }[encoding]

            rows = np.frombuffer(
                msg.data,
                dtype=np.uint8
            ).reshape(
                msg.height,
                msg.step
            )

            raw = np.ascontiguousarray(
                rows[:, :msg.width * channels]
            )

            raw = raw.reshape(
                msg.height,
                msg.width,
                channels
            )

            conversion_codes = {
                "yuv422_yuy2":
                    cv2.COLOR_YUV2RGB_YUY2,

                "yuyv":
                    cv2.COLOR_YUV2RGB_YUY2,

                "yuy2":
                    cv2.COLOR_YUV2RGB_YUY2,

                "yuv422":
                    cv2.COLOR_YUV2RGB_UYVY,

                "bgr8":
                    cv2.COLOR_BGR2RGB,

                "mono8":
                    cv2.COLOR_GRAY2RGB,

                "rgba8":
                    cv2.COLOR_RGBA2RGB,

                "bgra8":
                    cv2.COLOR_BGRA2RGB
            }

            if encoding == "rgb8":

                rgb = raw

            else:

                rgb = cv2.cvtColor(
                    raw,
                    conversion_codes[encoding]
                )

            self.image = QImage(
                rgb.data,
                msg.width,
                msg.height,
                rgb.strides[0],
                QImage.Format_RGB888
            ).copy()

            now = time.monotonic()

            self.last["camera"] = now

            self.frame_times.append(now)

            self.image_error = ""

        except Exception as exc:

            error = (
                f"Image conversion failed "
                f"({msg.encoding}): {exc}"
            )

            if error != self.image_error:

                self.append_log(
                    "[Camera] " + error
                )

            self.image_error = error

    # ==========================================
    # Stop device driver
    # ==========================================

    def stop(self, kind):

        subscription = self.subscriptions[kind]

        if subscription is not None:

            self.node.destroy_subscription(
                subscription
            )

            self.subscriptions[kind] = None

        process = self.processes[kind]

        self.processes[kind] = None

        if process is not None:

            try:

                os.killpg(
                    process.pid,
                    signal.SIGINT
                )

            except ProcessLookupError:

                pass

            deadline = time.monotonic() + 3

            while time.monotonic() < deadline:

                process.poll()

                try:

                    os.killpg(
                        process.pid,
                        0
                    )

                except ProcessLookupError:

                    break

                time.sleep(0.05)

            else:

                try:

                    os.killpg(
                        process.pid,
                        signal.SIGKILL
                    )

                except ProcessLookupError:

                    pass

            process.wait()

            self.append_log(
                f"[{kind.capitalize()}] Driver stopped."
            )

        self.last[kind] = 0.0

        self.status(
            kind,
            "Disconnected"
        )

        # Reset force sensor display
        if kind == "force":

            self.calibrated = False

            self.force_values = None

            self.progress.hide()

            self.force_message.setText(
                "Connect the sensor to start calibration."
            )

            for label in self.force_labels:

                label.setText("--")

            self.force_connect.setEnabled(True)

            self.force_disconnect.setEnabled(False)

            self.force_recalibrate.setEnabled(False)

        # Reset camera display
        else:

            self.image = None

            self.frame_times.clear()

            self.preview.clear()

            self.preview.setText(
                "No camera image"
            )

            self.actual_fps.setText(
                "Received FPS: --"
            )

            self.set_camera_controls(False)

    # ==========================================
    # Update GUI and ROS2 subscriptions
    # ==========================================

    def update(self):

        if self.closing:

            return

        # Read driver logs
        while True:

            try:

                kind, generation, line = (
                    self.log_queue.get_nowait()
                )

            except queue.Empty:

                break

            if generation != self.generation[kind]:

                continue

            self.append_log(
                f"[{kind.capitalize()}] {line}"
            )

            # Detect completed calibration
            if (
                kind == "force"
                and self.processes[kind] is not None
                and "force sensor calibration completed"
                    in line.lower()
            ):

                self.calibrated = True

                self.calibrated_at = (
                    time.monotonic()
                )

                self.force_message.setText(
                    "Calibration completed. "
                    "Waiting for data..."
                )

        # Receive ROS2 messages
        for _ in range(8):

            rclpy.spin_once(
                self.node,
                timeout_sec=0
            )

        now = time.monotonic()

        for kind in (
            "force",
            "camera"
        ):

            process = self.processes[kind]

            if process is None:

                continue

            # Detect stopped driver
            if process.poll() is not None:

                self.stop(kind)

                self.status(
                    kind,
                    "Driver stopped - check log",
                    "red"
                )

                continue

            # Force sensor calibration
            if (
                kind == "force"
                and not self.calibrated
            ):

                if now - self.started[kind] > 60:

                    self.stop(kind)

                    self.status(
                        kind,
                        "Calibration timeout",
                        "red"
                    )

                continue

            # Wait for first message
            if not self.last[kind]:

                if (
                    kind == "camera"
                    and self.image_error
                ):

                    self.status(
                        kind,
                        "Image conversion error",
                        "red"
                    )

                else:

                    self.status(
                        kind,
                        "Waiting for data...",
                        "orange"
                    )

                if kind == "force":

                    start = self.calibrated_at

                else:

                    start = self.started[kind]

                if now - start > 15:

                    self.stop(kind)

                    self.status(
                        kind,
                        "No data - check log",
                        "red"
                    )

                continue

            # Detect interrupted data
            if now - self.last[kind] > 2:

                self.status(
                    kind,
                    "No data",
                    "red"
                )

                if kind == "force":

                    for label in self.force_labels:

                        label.setText("--")

                else:

                    self.preview.clear()

                    self.preview.setText(
                        "No recent camera image"
                    )

                    self.actual_fps.setText(
                        "Received FPS: --"
                    )

                continue

            # Device connected
            self.status(
                kind,
                "Connected",
                "green"
            )

            # Update force sensor values
            if kind == "force":

                self.progress.hide()

                self.force_message.setText(
                    "Calibration completed. "
                    "Ready for measurement."
                )

                self.force_recalibrate.setEnabled(
                    True
                )

                if self.force_values is not None:

                    for i, (label, value) in enumerate(
                        zip(
                            self.force_labels,
                            self.force_values
                        )
                    ):

                        if i < 3:

                            label.setText(
                                f"{value:.3f}"
                            )

                        else:

                            label.setText(
                                f"{value:.4f}"
                            )

            # Update camera preview
            else:

                if self.image is not None:

                    pixmap = QPixmap.fromImage(
                        self.image
                    )

                    pixmap = pixmap.scaled(
                        self.preview.size(),
                        Qt.KeepAspectRatio,
                        Qt.FastTransformation
                    )

                    self.preview.setPixmap(
                        pixmap
                    )

                # Calculate received frame rate
                if len(self.frame_times) > 1:

                    dt = (
                        self.frame_times[-1]
                        - self.frame_times[0]
                    )

                    if dt > 0:

                        fps = (
                            len(self.frame_times) - 1
                        ) / dt

                        self.actual_fps.setText(
                            f"Received FPS: {fps:.1f}"
                        )

    # ==========================================
    # Close application safely
    # ==========================================

    def closeEvent(self, event):

        self.closing = True

        self.timer.stop()

        self.stop("camera")

        self.stop("force")

        self.node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()

        event.accept()

# ==========================================
# Main program
# ==========================================

if __name__ == "__main__":

    rclpy.init()

    app = QApplication(sys.argv)

    window = MainWindow()

    window.show()

    sys.exit(app.exec_())