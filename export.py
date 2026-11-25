
import sys
import csv
import cv2
import numpy as np

from pathlib import Path

import rclpy
import rosbag2_py

from rclpy.serialization import deserialize_message
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QFileDialog,
    QMessageBox,
    QGroupBox
)

from recording import RecordingWindow

# =========================================
# Export worker
# =========================================

class ExportWorker(QThread):

    progress = pyqtSignal(str)
    completed = pyqtSignal(str)
    failed = pyqtSignal(str)

    def __init__(self, bag_path):
        super().__init__()
        self.bag_path = Path(bag_path)

    def run(self):

        output = None

        try:

            # Create output folder
            output = self.bag_path / "export"

            number = 1

            while output.exists():

                output = (
                    self.bag_path
                    / f"export_{number:03d}"
                )

                number += 1

            partial = output.with_name(
                output.name + ".partial"
            )

            while partial.exists():

                partial = partial.with_name(
                    partial.name + "_new"
                )

            partial.mkdir()

            images_dir = partial / "images"
            images_dir.mkdir()

            # Open ROS2 bag
            reader = rosbag2_py.SequentialReader()

            storage_options = (
                rosbag2_py.StorageOptions(
                    uri=str(self.bag_path),
                    storage_id="mcap"
                )
            )

            converter_options = (
                rosbag2_py.ConverterOptions(
                    "cdr", "cdr"
                )
            )

            reader.open(
                storage_options,
                converter_options
            )

            bridge = CvBridge()

            force_count = 0
            image_count = 0

            force_csv = partial / "force.csv"

            camera_csv = (
                partial / "camera_timestamps.csv"
            )

            with open(
                force_csv, "w", newline=""
            ) as force_file, open(
                camera_csv, "w", newline=""
            ) as camera_file:

                force_writer = csv.writer(force_file)
                camera_writer = csv.writer(camera_file)

                # Force CSV header
                force_writer.writerow([
                    "timestamp_ns",
                    "bag_timestamp_ns",
                    "Fx_N",
                    "Fy_N",
                    "Fz_N",
                    "Mx_Nm",
                    "My_Nm",
                    "Mz_Nm"
                ])

                # Camera CSV header
                camera_writer.writerow([
                    "filename",
                    "timestamp_ns",
                    "bag_timestamp_ns"
                ])

                # Read all recorded messages
                while reader.has_next():

                    topic, data, bag_time = (
                        reader.read_next()
                    )

                    # =========================
                    # Export force data
                    # =========================

                    if topic == "/force_data":

                        msg = deserialize_message(
                            data, WrenchStamped
                        )

                        timestamp = (
                            msg.header.stamp.sec
                            * 1000000000
                            + msg.header.stamp.nanosec
                        )

                        force_writer.writerow([
                            timestamp,
                            bag_time,
                            msg.wrench.force.x,
                            msg.wrench.force.y,
                            msg.wrench.force.z,
                            msg.wrench.torque.x,
                            msg.wrench.torque.y,
                            msg.wrench.torque.z
                        ])

                        force_count += 1

                    # =========================
                    # Export camera images
                    # =========================

                    elif topic == "/image_raw":

                        msg = deserialize_message(
                            data, Image
                        )

                        timestamp = (
                            msg.header.stamp.sec
                            * 1000000000
                            + msg.header.stamp.nanosec
                        )

                        encoding = msg.encoding.lower()

                        # Convert YUYV image
                        if encoding in [
                            "yuv422_yuy2",
                            "yuyv",
                            "yuy2"
                        ]:

                            raw = np.frombuffer(
                                msg.data,
                                dtype=np.uint8
                            )

                            raw = raw.reshape(
                                msg.height,
                                msg.step
                            )

                            raw = raw[
                                :, :msg.width * 2
                            ]

                            raw = np.ascontiguousarray(
                                raw
                            )

                            raw = raw.reshape(
                                msg.height,
                                msg.width,
                                2
                            )

                            image = cv2.cvtColor(
                                raw,
                                cv2.COLOR_YUV2BGR_YUY2
                            )

                        else:

                            image = bridge.imgmsg_to_cv2(
                                msg,
                                desired_encoding="bgr8"
                            )

                        filename = (
                            f"{image_count + 1:06d}.png"
                        )

                        image_path = images_dir / filename

                        success = cv2.imwrite(
                            str(image_path), image
                        )

                        if not success:
                            raise RuntimeError(
                                f"Failed to save {filename}"
                            )

                        camera_writer.writerow([
                            filename,
                            timestamp,
                            bag_time
                        ])

                        image_count += 1

                        if image_count % 50 == 0:

                            self.progress.emit(
                                f"Exporting: "
                                f"{image_count} images..."
                            )

            del reader

            if force_count == 0 and image_count == 0:

                raise RuntimeError(
                    "No force or image data found."
                )

            # Rename folder after successful export
            partial.rename(output)

            result = (
                f"Export completed successfully.\n\n"
                f"Force samples: {force_count}\n"
                f"Images: {image_count}\n\n"
                f"Output directory:\n{output}"
            )

            self.completed.emit(result)

        except Exception as e:

            self.failed.emit(str(e))

# =========================================
# Main window with export function
# =========================================

class ExportWindow(RecordingWindow):

    def __init__(self):
        super().__init__()

        self.export_worker = None

        group = QGroupBox("Data Export")
        layout = QHBoxLayout()

        self.export_status = QLabel(
            "Ready to export"
        )

        self.export_button = QPushButton(
            "Export Data"
        )

        self.export_button.setMinimumHeight(40)

        self.export_button.clicked.connect(
            self.start_export
        )

        layout.addWidget(self.export_status, 1)
        layout.addWidget(self.export_button)

        group.setLayout(layout)

        self.layout().insertWidget(2, group)

    # =====================================
    # Select and export experiment data
    # =====================================

    def start_export(self):

        if self.record_process is not None:
            return

        if self.export_worker is not None:
            return

        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Experiment Folder",
            self.save_directory.text()
        )

        if not directory:
            return

        bag_path = Path(directory)

        if not (bag_path / "metadata.yaml").exists():

            QMessageBox.warning(
                self,
                "Invalid Folder",
                "Please select an experiment folder "
                "containing metadata.yaml."
            )

            return

        self.export_button.setEnabled(False)
        self.start_record_button.setEnabled(False)

        self.export_status.setText(
            "Exporting data..."
        )

        self.export_worker = ExportWorker(bag_path)

        self.export_worker.progress.connect(
            self.export_status.setText
        )

        self.export_worker.completed.connect(
            self.export_completed
        )

        self.export_worker.failed.connect(
            self.export_failed
        )

        self.export_worker.finished.connect(
            self.export_finished
        )

        self.export_worker.start()

    # =====================================
    # Export completed
    # =====================================

    def export_completed(self, message):

        self.export_status.setText(
            "Export completed"
        )

        self.append_log(
            f"[Export] {message}"
        )

        QMessageBox.information(
            self,
            "Export Completed",
            message
        )

    # =====================================
    # Export failed
    # =====================================

    def export_failed(self, message):

        self.export_status.setText(
            "Export failed"
        )

        self.append_log(
            f"[Export] Error: {message}"
        )

        QMessageBox.critical(
            self,
            "Export Error",
            message
        )

    def export_finished(self):

        self.export_worker.deleteLater()
        self.export_worker = None

        self.export_button.setEnabled(True)
        self.start_record_button.setEnabled(True)

    # =====================================
    # Disable export during recording
    # =====================================

    def set_recording_controls(self, active):

        super().set_recording_controls(active)

        if hasattr(self, "export_button"):

            self.export_button.setEnabled(
                not active
                and self.export_worker is None
            )

    # =====================================
    # Close application safely
    # =====================================

    def closeEvent(self, event):

        if self.export_worker is not None:

            event.ignore()

            QMessageBox.warning(
                self,
                "Exporting",
                "Please wait until export is completed."
            )

            return

        super().closeEvent(event)

# =========================================
# Main program
# =========================================

if __name__ == "__main__":

    app = QApplication(sys.argv)

    rclpy.init()

    window = ExportWindow()
    window.show()

    sys.exit(app.exec_())