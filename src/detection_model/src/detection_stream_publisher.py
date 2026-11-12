#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class DetectionStreamPublisher(Node):
    """Draw detections on images and publish the annotated stream."""

    def __init__(self) -> None:
        super().__init__('detection_stream_publisher')

        self.declare_parameter('image_topic', 'camera/image')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('output_topic', '/detection_stream')
        self.declare_parameter('score_threshold', 0.0)

        self.image_topic = self.get_parameter('image_topic').value
        self.detections_topic = self.get_parameter('detections_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.score_threshold = float(self.get_parameter('score_threshold').value)

        self.bridge = CvBridge()
        qos = QoSProfile(depth=10)

        self.stream_pub = self.create_publisher(Image, self.output_topic, qos)

        self.get_logger().info(
            f"Subscribing to '{self.image_topic}' + '{self.detections_topic}' and publishing '{self.output_topic}'."
        )

        self.image_sub = Subscriber(self, Image, self.image_topic, qos_profile=qos)
        self.detections_sub = Subscriber(self, Detection2DArray, self.detections_topic, qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.detections_sub],
            queue_size=15,
            slop=0.15,
            allow_headerless=False,
        )
        self.sync.registerCallback(self.synced_callback)

    def synced_callback(self, image_msg: Image, detections_msg: Detection2DArray) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to convert incoming image: {exc}')
            return

        height, width = frame.shape[:2]

        for detection in detections_msg.detections:
            bbox = detection.bbox
            cx = bbox.center.position.x
            cy = bbox.center.position.y
            bw = bbox.size_x
            bh = bbox.size_y

            x_min = max(int(cx - bw / 2.0), 0)
            y_min = max(int(cy - bh / 2.0), 0)
            x_max = min(int(cx + bw / 2.0), width - 1)
            y_max = min(int(cy + bh / 2.0), height - 1)

            if x_max <= x_min or y_max <= y_min:
                continue

            score = detection.results[0].hypothesis.score if detection.results else 0.0
            if score < self.score_threshold:
                continue

            class_id = detection.results[0].hypothesis.class_id if detection.results else ''
            label = f'{class_id}:{score:.2f}' if class_id else f'{score:.2f}'

            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(
                frame,
                label,
                (x_min, max(y_min - 5, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

        output_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        output_msg.header = image_msg.header
        self.stream_pub.publish(output_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DetectionStreamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
