import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from ultralytics import YOLO

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

# cv2.imshow is unavailable in headless environments.
_HEADLESS = os.environ.get('DISPLAY') is None


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, 10)
        # fixed_rgbd_camera.sdf type="camera" sensor publishes camera_info
        # at {topic}/camera_info = /camera/image/camera_info (not /camera/camera_info)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/image/camera_info', self._camera_info_callback, 1)

        self.result_pub = self.create_publisher(String, '/vision/detection_results', 10)

        self.latest_depth_image = None

        # Camera intrinsics — updated from /camera/camera_info; defaults match
        # Ignition Gazebo RGBD sensor with fov=1.047rad, 640×480.
        self._fx = 554.25
        self._fy = 554.25
        self._cx = 320.0
        self._cy = 240.0
        self._intrinsics_ready = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Loading YOLOv8 Model')
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('Loading Complete!')

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        if self._intrinsics_ready:
            return
        K = msg.k  # row-major 3×3: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self._fx = float(K[0])
        self._fy = float(K[4])
        self._cx = float(K[2])
        self._cy = float(K[5])
        self._intrinsics_ready = True
        self.get_logger().info(
            f'Camera intrinsics received: fx={self._fx:.2f} fy={self._fy:.2f} '
            f'cx={self._cx:.2f} cy={self._cy:.2f}')

    def depth_callback(self, msg):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth 변환 에러: {e}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            results = self.model(cv_image, verbose=False)
            annotated_frame = results[0].plot()

            frame_detections = []

            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                cv2.circle(annotated_frame, (u, v), 5, (0, 255, 0), -1)

                cls_id = int(box.cls[0])
                class_name = self.model.names[cls_id]
                confidence = float(box.conf[0])

                distance = 0.0
                X_cam, Y_cam, Z_cam = 0.0, 0.0, 0.0
                X_base, Y_base, Z_base = None, None, None

                if self.latest_depth_image is not None:
                    if (0 <= v < self.latest_depth_image.shape[0] and
                            0 <= u < self.latest_depth_image.shape[1]):
                        distance = self.latest_depth_image[v, u]
                        if np.isnan(distance) or np.isinf(distance):
                            distance = -1.0

                if distance > 0:
                    X_cam = float((u - self._cx) * distance / self._fx)
                    Y_cam = float((v - self._cy) * distance / self._fy)
                    Z_cam = float(distance)

                    try:
                        t = self.tf_buffer.lookup_transform(
                            'base_link',
                            'camera_link',
                            rclpy.time.Time()
                        )
                        p_cam = PointStamped()
                        p_cam.point.x = X_cam
                        p_cam.point.y = Y_cam
                        p_cam.point.z = Z_cam

                        p_base = tf2_geometry_msgs.do_transform_point(p_cam, t)

                        X_base = float(round(p_base.point.x, 3))
                        Y_base = float(round(p_base.point.y, 3))
                        Z_base = float(round(p_base.point.z, 3))
                    except Exception as e:
                        self.get_logger().debug(f'TF lookup failed: {e}')

                # D9: exclude objects where base_link coordinates are unavailable.
                if X_base is None:
                    continue

                text = f'{distance:.2f}m'
                cv2.putText(annotated_frame, text, (u + 10, v - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                frame_detections.append({
                    'class_name': str(class_name),
                    'confidence': float(round(confidence, 3)),
                    'center_2d': {'u': int(u), 'v': int(v)},
                    'distance_m': float(round(distance, 3)),
                    'position_3d_camera_frame': {
                        'X': float(round(X_cam, 3)),
                        'Y': float(round(Y_cam, 3)),
                        'Z': float(round(Z_cam, 3)),
                    },
                    'position_3d_base_frame': {
                        'X': X_base,
                        'Y': Y_base,
                        'Z': Z_base,
                    },
                })

            if frame_detections:
                result_dict = {
                    'timestamp_ns': self.get_clock().now().nanoseconds,
                    'num_detections': len(frame_detections),
                    'objects': frame_detections,
                }
                json_string = json.dumps(result_dict, ensure_ascii=False)
                msg_out = String()
                msg_out.data = json_string
                self.result_pub.publish(msg_out)
                self.get_logger().info(
                    f'Published JSON:\n{json.dumps(result_dict, indent=2)}')

            if not _HEADLESS:
                cv2.imshow('Gazebo YOLO Detection', annotated_frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'이미지 변환 및 처리 에러: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if not _HEADLESS:
            cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
