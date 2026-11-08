# ros2 service call /mission3/capture std_srvs/srv/Trigger "{}"

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Int32MultiArray, Bool
from cv_bridge import CvBridge

import json
from pathlib import Path
from datetime import datetime

from std_srvs.srv import Trigger

import cv2
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.Dictionary_get(
            cv2.aruco.DICT_4X4_50
        )

        self.parameters =  cv2.aruco.DetectorParameters_create()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.captured_ids = set()
        self.detection_counts = {}
        self.required_detection_frames = 5
        
        self.storage_path = Path("arucoData")

        self.image_path = self.storage_path / "images"
        self.metadata_path = self.storage_path / "metadata"

        self.image_path.mkdir(parents=True, exist_ok=True)
        self.metadata_path.mkdir(parents=True, exist_ok=True)

        # Latest detection
        self.latest_frame = None
        self.latest_ids = None
        self.latest_corners = None


        self.logging_requested = False
        self.mission_active = False

        self.log_directory = Path('arucoData/logs')
        self.log_directory.mkdir(parents=True, exist_ok=True)

        self.scanning_enabled = False

        self.scanning_subscription = self.create_subscription(
            Bool,
            '/mission3/scanning_enabled',
            self.scanning_callback,
            10
        )

        self.logging_subscription = self.create_subscription(
            Bool,
            '/mission3/logging_enabled',
            self.logging_callback,
            10
        )

        self.mission_subscription = self.create_subscription(
            Bool,
            '/navigation/mission_active',
            self.mission_callback,
            10
        )

        self.detection_log = self.log_directory / 'detections.jsonl'
        self.capture_service = self.create_service(
            Trigger,
            '/mission3/capture',
            self.capture_callback
        )

        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        self.info_sub = self.create_subscription(
            CameraInfo,
            'camera/camera_info',
            self.info_callback,
            10
        )
        self.get_logger().info('ArUco Detector initialized!')

    def info_callback(self, msg):
        self.camera_matrix = np.array(
            msg.k
        ).reshape(3,3)
        self.dist_coeffs = np.array(
            msg.d
        )

    def logging_callback(self, msg: Bool):
        self.logging_requested = msg.data
        self.get_logger().info(
            f'Logging requested: {self.logging_requested}'
        )


    def mission_callback(self, msg: Bool):
        self.mission_active = msg.data

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.parameters
        )

        output = frame.copy()

        self.latest_frame = frame.copy()

        self.latest_ids = (
            ids.flatten().tolist()
            if ids is not None
            else None
        )

        self.latest_corners = (
            [corner[0].tolist() for corner in corners]
            if ids is not None
            else None
        )

        currently_detected = (
            set(self.latest_ids)
            if self.latest_ids is not None
            else set()
        )


        if ids is not None:
            cv2.aruco.drawDetectedMarkers(
                output,
                corners,
                ids
            )


        if (
            self.scanning_enabled
            and self.logging_requested
            and self.mission_active
            and ids is not None
        ):
            self.log_detections(ids, corners)

            for i, marker_id in enumerate(ids.flatten()):

                marker_corners = corners[i][0]

                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])

                self.get_logger().info(
                    f'ID: {marker_id} | '
                    f'Center: ({center_x:.1f}, {center_y:.1f})'
                )

        # -----------------------------------------------
        # AUTONOMOUS CAPTURE
        # -----------------------------------------------

        if self.scanning_enabled and self.mission_active:

            # Reset counters for markers no longer visible.
            for marker_id in list(self.detection_counts.keys()):

                if marker_id not in currently_detected:
                    self.detection_counts[marker_id] = 0

            # Count consecutive detections.
            for marker_id in currently_detected:

                if marker_id in self.captured_ids:
                    continue

                self.detection_counts[marker_id] = (
                    self.detection_counts.get(marker_id, 0) + 1
                )

                if (
                    self.detection_counts[marker_id]
                    >= self.required_detection_frames
                ):

                    success, message = self.save_capture()

                    if success:
                        self.captured_ids.add(marker_id)

                        self.get_logger().info(
                            f'AUTONOMOUS CAPTURE: Marker {marker_id}'
                        )

                    else:
                        self.get_logger().warning(message)

        else:
            # Prevent stale detection counts between missions.
            self.detection_counts.clear()


        cv2.imshow(
            'Mission 3 - ArUco Detection',
            output
        )

        cv2.waitKey(1)
    def scanning_callback(self, msg: Bool):
        self.scanning_enabled = msg.data
        self.get_logger().info(
            f'Scanning enabled: {self.scanning_enabled}'
        )


    def save_capture(self):
        if self.latest_frame is None:
            return False, "No camera frame available."

        if self.latest_ids is None:
            return False, "No ArUco markers detected."

        # 3. Generate unique filenames
        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )

        filename = f"capture_{timestamp}"

        image_file = self.image_path / f"{filename}.png"
        metadata_file = self.metadata_path / f"{filename}.json"

        try:
            success = cv2.imwrite(
                str(image_file),
                self.latest_frame
            )

            if not success:
                raise IOError("Failed to write image.")

            saved_image = cv2.imread(str(image_file))

            if saved_image is None:
                raise IOError("Saved image cannot be opened.")

            gray = cv2.cvtColor(
                saved_image,
                cv2.COLOR_BGR2GRAY
            )

            _, verified_ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.parameters
            )

            if verified_ids is None:
                raise ValueError("Saved image contains no readable markers.")

            verified_ids = verified_ids.flatten().tolist()

            if sorted(verified_ids) != sorted(self.latest_ids):
                raise ValueError("Marker IDs do not match the original detection.")

            metadata = {
                "capture_id": filename,
                "timestamp": datetime.now().isoformat(),
                "image": str(image_file),
                "image_width": int(saved_image.shape[1]),
                "image_height": int(saved_image.shape[0]),
                "detected_markers": [
                    {
                        "id": int(marker_id),
                        "corners": corners
                    }
                    for marker_id, corners in zip(
                        self.latest_ids,
                        self.latest_corners
                    )
                ],
                "camera_matrix": (
                    self.camera_matrix.tolist()
                    if self.camera_matrix is not None
                    else None
                ),
                "distortion_coefficients": (
                    self.dist_coeffs.tolist()
                    if self.dist_coeffs is not None
                    else None
                )
            }

            temp_metadata = metadata_file.with_suffix(".json.tmp")

            with open(temp_metadata, "w") as file:
                json.dump(metadata, file, indent=4)

            with open(temp_metadata, "r") as file:
                json.load(file)

            temp_metadata.replace(metadata_file)

            self.get_logger().info(
                f"Successfully saved {filename}"
            )

            return True, f"Capture successful! Detected IDs: {verified_ids}"
        except Exception as error:

            image_file.unlink(missing_ok=True)
            metadata_file.unlink(missing_ok=True)

            return False, f"Capture failed: {error}"

    
    def capture_callback(self, request, response):
        success, message = self.save_capture()

        response.success = success
        response.message = message

        return response

    def log_detections(self, ids, corners):
        if not self.logging_requested or not self.mission_active:
            return

        timestamp = datetime.now().isoformat(
            timespec='milliseconds'
        )

        with self.detection_log.open('a') as file:

            for marker_id, marker_corners in zip(ids, corners):

                entry = {
                    'timestamp': timestamp,
                    'marker_id': int(marker_id),
                    'corners': marker_corners.tolist()
                }

                file.write(json.dumps(entry) + '\n')
    

    
def main(args = None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()