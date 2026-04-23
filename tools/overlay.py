#!/usr/bin/env python3
"""RGB-Depth alignment visual verification tool.

Usage:
    # PC 직연결 (raw, namespace oak → /oak/oak/...)
    python3 tools/overlay.py --namespace oak --subpath oak

    # TurtleBot4 원격 (compressed, namespace robot9 → /robot9/oakd/.../compressed)
    python3 tools/overlay.py --namespace robot9 --subpath oakd --compressed
"""
import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
import message_filters
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


# compressedDepth message format:
#   first 12 bytes: ConfigHeader (format enum + depthQuantA/B float32)
#   remaining bytes: PNG-encoded 16UC1 depth image
COMPRESSED_DEPTH_HEADER_SIZE = 12


class DepthOverlay(Node):
    def __init__(self, rgb_topic: str, depth_topic: str, alpha: float, compressed: bool):
        super().__init__('depth_overlay')
        self.bridge = CvBridge()
        self.alpha = alpha
        self.compressed = compressed

        if compressed:
            rgb_msg_type = CompressedImage
            depth_msg_type = CompressedImage
        else:
            rgb_msg_type = Image
            depth_msg_type = Image

        mode = 'compressed' if compressed else 'raw'
        self.get_logger().info(f'[{mode}] RGB: {rgb_topic}')
        self.get_logger().info(f'[{mode}] Depth: {depth_topic}')

        rgb_sub = message_filters.Subscriber(self, rgb_msg_type, rgb_topic)
        depth_sub = message_filters.Subscriber(self, depth_msg_type, depth_topic)

        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self._callback)

    def _decode_rgb(self, msg):
        if self.compressed:
            return cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        return self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _decode_depth(self, msg):
        if self.compressed:
            payload = np.frombuffer(msg.data[COMPRESSED_DEPTH_HEADER_SIZE:], np.uint8)
            return cv2.imdecode(payload, cv2.IMREAD_UNCHANGED)
        return self.bridge.imgmsg_to_cv2(msg, '16UC1')

    def _callback(self, rgb_msg, depth_msg):
        rgb = self._decode_rgb(rgb_msg)
        depth_raw = self._decode_depth(depth_msg)

        if rgb is None or depth_raw is None:
            self.get_logger().warn('Decode failed (None returned)')
            return

        depth_vis = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_raw, alpha=0.03), cv2.COLORMAP_JET)

        if rgb.shape[:2] != depth_vis.shape[:2]:
            depth_vis = cv2.resize(depth_vis, (rgb.shape[1], rgb.shape[0]))

        overlay = cv2.addWeighted(rgb, self.alpha, depth_vis, 1.0 - self.alpha, 0)

        rh, rw = rgb.shape[:2]
        dh, dw = depth_raw.shape[:2]
        label = f'RGB {rw}x{rh} | Depth {dw}x{dh}'
        cv2.putText(overlay, label, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow('RGB-Depth Overlay (q to quit)', overlay)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()


def build_topics(namespace: str, subpath: str, aligned: bool, compressed: bool):
    ns = f'/{namespace}' if namespace else ''
    base = f'{ns}/{subpath}'
    suffix = '/aligned' if aligned else ''
    if compressed:
        return (
            f'{base}/rgb/image_raw{suffix}/compressed',
            f'{base}/stereo/image_raw{suffix}/compressedDepth',
        )
    return (
        f'{base}/rgb/image_raw{suffix}',
        f'{base}/stereo/image_raw{suffix}',
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', default='',
                        help='Topic namespace prefix (default empty for PC; "robot9" for TurtleBot4)')
    parser.add_argument('--subpath', default='oakd',
                        help='Topic subpath: oakd (our C++ node / TurtleBot4) | oak (depthai_ros_driver PC)')
    parser.add_argument('--aligned', action='store_true',
                        help='Append /aligned suffix (for our oak_d_align_cpp node)')
    parser.add_argument('--alpha', type=float, default=0.6,
                        help='RGB blend weight (0.0~1.0)')
    parser.add_argument('--compressed', action='store_true',
                        help='Subscribe to compressed topics (WiFi-friendly for TurtleBot4)')
    parser.add_argument('--rgb-topic', default=None,
                        help='Override RGB topic path (bypass auto-construction)')
    parser.add_argument('--depth-topic', default=None,
                        help='Override Depth topic path (bypass auto-construction)')
    args, ros_args = parser.parse_known_args()

    if args.rgb_topic and args.depth_topic:
        rgb_topic, depth_topic = args.rgb_topic, args.depth_topic
    else:
        rgb_topic, depth_topic = build_topics(
            args.namespace, args.subpath, args.aligned, args.compressed)

    rclpy.init(args=ros_args)
    node = DepthOverlay(
        rgb_topic=rgb_topic,
        depth_topic=depth_topic,
        alpha=args.alpha,
        compressed=args.compressed,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
