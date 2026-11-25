
import sys
import os
import signal
import time
import queue
import threading
import subprocess
import yaml

from pathlib import Path
from datetime import datetime

import rclpy

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QLineEdit,
    QFileDialog,
    QCheckBox,
    QMessageBox
)

from main import MainWindow

class RecordingWindow(MainWindow):

    def __init__(self):
        super().__init__()

        self.record_process = None
        self.record_queue = queue.Queue()

        self.record_start = 0
        self.record_requested = 0
        self.record_stopping = False

        self.record_path = None
        self.record_topics = []
        self.subscribed_topics = set()

        self.layout().insertWidget(
            1, self.create_recording_panel()
        )

        self.record_timer = QTimer(self)

        self.record_timer.timeout.connect(
            self.update_recording
        )

        self.record_timer.start(200)

    # =====================================
    # Recording panel
    # =====================================

    def create_recording_panel(self):

        group = QGroupBox("Data Recording")
        layout = QVBoxLayout()

        # Save directory
        path_layout = QHBoxLayout()

        self.save_directory = QLineEdit(
            str(Path.home() / "experiment_data")
        )

        self.browse_button = QPushButton("Browse")

        self.browse_button.clicked.connect(
            self.select_directory
        )

        path_layout.addWidget(
            QLabel("Save Directory")
        )

        path_layout.addWidget(self.save_directory)
        path_layout.addWidget(self.browse_button)

        layout.addLayout(path_layout)

        # Experiment name
        name_layout = QHBoxLayout()

        self.experiment_name = QLineEdit()

        self.experiment_name.setPlaceholderText(
            "Leave empty for automatic naming"
        )

        name_layout.addWidget(
            QLabel("Experiment Name")
        )

        name_layout.addWidget(self.experiment_name)

        layout.addLayout(name_layout)

        # Recording devices
        sensor_layout = QHBoxLayout()

        self.record_force = QCheckBox("Force Sensor")
        self.record_camera = QCheckBox("USB Camera")

        self.record_force.setChecked(True)
        self.record_camera.setChecked(True)

        sensor_layout.addWidget(
            QLabel("Recording Devices")
        )

        sensor_layout.addWidget(self.record_force)
        sensor_layout.addWidget(self.record_camera)
        sensor_layout.addStretch()

        layout.addLayout(sensor_layout)

        # Experiment information
        self.experiment_label = QLabel(
            "Experiment: --"
        )

        layout.addWidget(self.experiment_label)

        # Recording status and time
        status_layout = QHBoxLayout()

        self.record_status = QLabel(
            "Status: Ready"
        )

        self.record_time = QLabel("00:00:00")

        self.record_time.setStyleSheet(
            "font-size: 26px; font-weight: bold;"
        )

        status_layout.addWidget(self.record_status)
        status_layout.addStretch()

        status_layout.addWidget(
            QLabel("Recording Time")
        )

        status_layout.addWidget(self.record_time)

        layout.addLayout(status_layout)

        # Start and stop buttons
        button_layout = QHBoxLayout()

        self.start_record_button = QPushButton(
            "Start Recording"
        )

        self.stop_record_button = QPushButton(
            "Stop Recording"
        )

        self.start_record_button.clicked.connect(
            self.start_recording
        )

        self.stop_record_button.clicked.connect(
            self.stop_recording
        )

        self.start_record_button.setMinimumHeight(40)
        self.stop_record_button.setMinimumHeight(40)

        self.stop_record_button.setEnabled(False)

        button_layout.addWidget(
            self.start_record_button
        )

        button_layout.addWidget(
            self.stop_record_button
        )

        layout.addLayout(button_layout)

        layout.addWidget(
            QLabel("Storage Format: ROS2 Bag (MCAP)")
        )

        group.setLayout(layout)

        return group

    # =====================================
    # Recording status
    # =====================================

    def set_record_status(self, text, color):

        self.record_status.setText(
            f"Status: {text}"
        )

        self.record_status.setStyleSheet(
            f"color: {color}; font-weight: bold;"
        )

    # =====================================
    # Select save directory
    # =====================================

    def select_directory(self):

        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Save Directory",
            self.save_directory.text()
        )

        if directory:

            self.save_directory.setText(directory)

    # =====================================
    # Generate experiment name
    # =====================================

    def generate_experiment_path(self, directory):

        custom_name = (
            self.experiment_name.text().strip()
        )

        # Manual naming
        if custom_name:

            invalid_characters = '<>:"/\\|?*'

            if (
                custom_name in (".", "..")
                or any(
                    char in custom_name
                    for char in invalid_characters
                )
                or any(
                    ord(char) < 32
                    for char in custom_name
                )
            ):

                raise ValueError(
                    "Invalid experiment name."
                )

            path = directory / custom_name

            if os.path.lexists(str(path)):

                raise ValueError(
                    f"Experiment '{custom_name}' "
                    "already exists.\n"
                    "Please enter a different name."
                )

            return path

        # Automatic naming
        date = datetime.now().strftime("%Y-%m-%d")

        number = 1

        while True:

            name = f"{date}_test_{number:03d}"

            path = directory / name

            if not os.path.lexists(str(path)):

                return path

            number += 1

    # =====================================
    # Check sensor readiness
    # =====================================

    def sensors_ready(self):

        now = time.monotonic()

        # Support both MainWindow structures
        if hasattr(self, "processes"):

            force_process = self.processes.get("force")
            camera_process = self.processes.get("camera")

            force_last = self.last.get("force", 0)
            camera_last = self.last.get("camera", 0)

            calibrated = self.calibrated

        else:

            force_process = getattr(
                self, "force_process", None
            )

            camera_process = getattr(
                self, "camera_process", None
            )

            force_last = getattr(
                self, "force_last_received", 0
            )

            camera_last = getattr(
                self, "camera_last_received", 0
            )

            calibrated = getattr(
                self, "calibration_completed", False
            )

        # Force sensor
        if self.record_force.isChecked():

            if (
                force_process is None
                or force_process.poll() is not None
                or not calibrated
                or not force_last
                or now - force_last > 1
            ):

                return False

        # USB camera
        if self.record_camera.isChecked():

            if (
                camera_process is None
                or camera_process.poll() is not None
                or not camera_last
                or now - camera_last > 2
            ):

                return False

        return True

    # =====================================
    # Read recording process logs
    # =====================================

    def read_record_logs(self, process, log_queue):

        try:

            with process.stdout as stream:

                for line in stream:

                    log_queue.put(line.rstrip())

        except Exception as e:

            log_queue.put(str(e))

    # =====================================
    # Launch rosbag recording process
    # =====================================

    def launch_record_process(self, command):

        env = os.environ.copy()

        env["RCUTILS_LOGGING_BUFFERED_STREAM"] = "0"

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

        threading.Thread(
            target=self.read_record_logs,
            args=(process, self.record_queue),
            daemon=True
        ).start()

        return process

    # =====================================
    # Start recording
    # =====================================

    def start_recording(self):

        if self.record_process is not None:
            return

        if not (
            self.record_force.isChecked()
            or self.record_camera.isChecked()
        ):

            QMessageBox.warning(
                self,
                "Recording",
                "Select at least one sensor."
            )

            return

        if not self.sensors_ready():

            QMessageBox.warning(
                self,
                "Recording",
                "Selected sensors are not ready.\n"
                "Connect the sensors and complete calibration."
            )

            return

        topics = []

        if self.record_force.isChecked():

            topics.append("/force_data")

        if self.record_camera.isChecked():

            topics.extend([
                "/image_raw",
                "/camera_info"
            ])

        try:

            # Save directory
            directory_text = (
                self.save_directory.text().strip()
            )

            if not directory_text:

                raise ValueError(
                    "Please enter a save directory."
                )

            directory = Path(
                directory_text
            ).expanduser().resolve()

            directory.mkdir(
                parents=True,
                exist_ok=True
            )

            # Generate experiment path
            self.record_path = (
                self.generate_experiment_path(directory)
            )

            self.record_topics = topics

            self.subscribed_topics.clear()

            self.record_queue = queue.Queue()

            # ROS2 recording command
            command = [
                "ros2",
                "bag",
                "record",
                "-s",
                "mcap",
                "-o",
                str(self.record_path)
            ]

            # QoS settings
            qos_path = (
                Path(__file__).parent / "qos.yaml"
            )

            if qos_path.exists():

                command.extend([
                    "--qos-profile-overrides-path",
                    str(qos_path)
                ])

            if os.environ.get("ROS_DISTRO") == "humble":

                command.extend(topics)

            else:

                command.extend(
                    ["--topics"] + topics
                )

            # Start recording process
            self.record_process = (
                self.launch_record_process(command)
            )

            self.record_requested = time.monotonic()

            self.record_start = 0

            self.record_stopping = False

            self.experiment_label.setText(
                f"Experiment: {self.record_path.name}"
            )

            self.record_time.setText("00:00:00")

            self.set_record_status(
                "Starting recording...",
                "orange"
            )

            self.set_recording_controls(True)

            self.append_log(
                f"[Recording] Output: {self.record_path}"
            )

        except Exception as e:

            if self.record_process is not None:
                self.stop_recording()
            else:
                self.set_recording_controls(False)

            QMessageBox.critical(
                self,
                "Recording Error",
                str(e)
            )

    # =====================================
    # Stop recording
    # =====================================

    def stop_recording(self):

        if self.record_process is None:
            return

        if self.record_stopping:
            return

        self.record_stopping = True

        self.stop_record_button.setEnabled(False)

        self.set_record_status(
            "Saving data...",
            "orange"
        )

        try:

            os.killpg(
                self.record_process.pid,
                signal.SIGINT
            )

        except ProcessLookupError:

            pass

    # =====================================
    # Update recording status
    # =====================================

    def update_recording(self):

        # Read process logs
        while True:

            try:

                line = self.record_queue.get_nowait()

            except queue.Empty:

                break

            self.append_log(
                f"[Recording] {line}"
            )

            for topic in self.record_topics:

                if f"Subscribed to topic '{topic}'" in line:

                    self.subscribed_topics.add(topic)

        if self.record_process is None:

            return

        # Recording process completed
        if self.record_process.poll() is not None:

            self.finish_recording()

            return

        if self.record_stopping:

            return

        startup_elapsed = (
            time.monotonic() - self.record_requested
        )

        # Check sensor connection
        if (
            startup_elapsed > 3
            and not self.sensors_ready()
        ):

            self.append_log(
                "[Recording] Sensor data interrupted."
            )

            self.stop_recording()

            return

        # Wait for topic subscriptions
        if self.record_start == 0:

            if set(self.record_topics).issubset(
                self.subscribed_topics
            ):

                self.record_start = time.monotonic()

                self.set_record_status(
                    "Recording",
                    "red"
                )

            elif startup_elapsed > 20:

                self.append_log(
                    "[Recording] Topic subscription timeout."
                )

                self.stop_recording()

            return

        # Update recording time
        elapsed = int(
            time.monotonic() - self.record_start
        )

        hours = elapsed // 3600

        minutes = (elapsed % 3600) // 60

        seconds = elapsed % 60

        self.record_time.setText(
            f"{hours:02d}:{minutes:02d}:{seconds:02d}"
        )

    # =====================================
    # Finish recording
    # =====================================

    def finish_recording(self):

        exit_code = self.record_process.returncode

        requested_stop = self.record_stopping

        self.record_process = None

        self.record_stopping = False

        self.set_recording_controls(False)

        metadata = self.record_path / "metadata.yaml"

        try:

            if not metadata.exists():

                raise RuntimeError(
                    "metadata.yaml not found."
                )

            with metadata.open(
                encoding="utf-8"
            ) as f:

                data = yaml.safe_load(f)

            info = data["rosbag2_bagfile_information"]

            counts = {
                item["topic_metadata"]["name"]:
                item["message_count"]
                for item in info["topics_with_message_count"]
            }

            self.append_log(
                f"[Recording] Saved: {self.record_path}"
            )

            # Display saved message counts
            for topic in self.record_topics:

                count = counts.get(topic, 0)

                self.append_log(
                    f"[Recording] {topic}: "
                    f"{count} messages"
                )

            # Calculate average saved camera FPS
            duration_ns = info.get(
                "duration", {}
            ).get(
                "nanoseconds", 0
            )

            duration_sec = duration_ns / 1e9

            if "/image_raw" in self.record_topics:

                image_count = counts.get(
                    "/image_raw", 0
                )

                if duration_sec > 0:

                    average_fps = (
                        image_count / duration_sec
                    )

                    self.append_log(
                        f"[Recording] Average saved FPS: "
                        f"{average_fps:.2f}"
                    )

            # Verify that each selected topic contains data
            complete = all(
                counts.get(topic, 0) > 0
                for topic in self.record_topics
            )

            normal_exit = (
                requested_stop
                and exit_code in (
                    0, 130, -signal.SIGINT
                )
            )

            if complete and normal_exit:

                self.set_record_status(
                    "Saved - messages verified",
                    "green"
                )

            else:

                self.set_record_status(
                    "Saved - check data",
                    "orange"
                )

        except Exception as e:

            self.set_record_status(
                "Save verification failed",
                "red"
            )

            self.append_log(
                f"[Recording] Error: {e}"
            )

    # =====================================
    # Lock settings during recording
    # =====================================

    def set_recording_controls(self, active):

        self.start_record_button.setEnabled(
            not active
        )

        self.stop_record_button.setEnabled(
            active
        )

        self.save_directory.setEnabled(
            not active
        )

        self.browse_button.setEnabled(
            not active
        )

        self.experiment_name.setEnabled(
            not active
        )

        self.record_force.setEnabled(
            not active
        )

        self.record_camera.setEnabled(
            not active
        )

        self.force_status.parentWidget().setEnabled(
            not active
        )

        self.camera_status.parentWidget().setEnabled(
            not active
        )

    # =====================================
    # Close application safely
    # =====================================

    def closeEvent(self, event):

        if self.record_process is not None:

            self.stop_recording()

            event.ignore()

            QMessageBox.information(
                self,
                "Saving Data",
                "Please wait until data saving is completed,\n"
                "then close the application again."
            )

            return

        self.record_timer.stop()

        super().closeEvent(event)

# =============================================
# Main program
# =============================================

if __name__ == "__main__":

    app = QApplication(sys.argv)

    rclpy.init()

    window = RecordingWindow()

    window.show()

    sys.exit(app.exec_())