# ros2 service call /mission3/capture std_srvs/srv/Trigger "{}"

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
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

        self.storage_path = Path("arucoData")

        self.image_path = self.storage_path / "images"
        self.metadata_path = self.storage_path / "metadata"

        self.image_path.mkdir(parents=True, exist_ok=True)
        self.metadata_path.mkdir(parents=True, exist_ok=True)

        # Latest detection
        self.latest_frame = None
        self.latest_ids = None
        self.latest_corners = None

        # Capture service
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
            parameters = self.parameters
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

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(
                output,
                corners,
                ids
            )
            

            for i, marker_id in enumerate(ids.flatten()):

                marker_corners = corners[i][0]

                center_x = np.mean(marker_corners[:, 0])
                center_y = np.mean(marker_corners[:, 1])

                self.get_logger().info(
                    f'ID: {marker_id} | '
                    f'Center: ({center_x:.1f}, {center_y:.1f})'
                )

        cv2.imshow('Mission 3 - ArUco Detection', output)
        cv2.waitKey(1)
    def capture_callback(self, request, response):

        # 1. Check whether a camera frame exists
        if self.latest_frame is None:
            response.success = False
            response.message = "No camera frame available."
            return response

        # 2. Check whether any marker was detected
        if self.latest_ids is None:
            response.success = False
            response.message = "No ArUco markers detected."
            return response

        # 3. Generate unique filenames
        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )

        filename = f"capture_{timestamp}"

        image_file = self.image_path / f"{filename}.png"
        metadata_file = self.metadata_path / f"{filename}.json"

        try:

            # 4. Save original image
            success = cv2.imwrite(
                str(image_file),
                self.latest_frame
            )

            if not success:
                raise IOError("Failed to write image.")

            # 5. Reopen and validate image
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

            # 6. Prepare metadata
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

            # 7. Write metadata to temporary file first
            temp_metadata = metadata_file.with_suffix(".json.tmp")

            with open(temp_metadata, "w") as file:
                json.dump(metadata, file, indent=4)

            # Verify JSON is readable
            with open(temp_metadata, "r") as file:
                json.load(file)

            # Commit metadata
            temp_metadata.replace(metadata_file)

            # 8. Only now report success
            self.get_logger().info(
                f"Successfully saved {filename}"
            )

            response.success = True
            response.message = (
                f"Capture successful! "
                f"Detected IDs: {verified_ids}"
            )

        except Exception as error:

            # Avoid leaving incomplete recordings
            image_file.unlink(missing_ok=True)
            metadata_file.unlink(missing_ok=True)

            self.get_logger().error(str(error))

            response.success = False
            response.message = f"Capture failed: {error}"

        return response

    
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
        rclpy.shutdown

if __name__ == "__main__":
    main()