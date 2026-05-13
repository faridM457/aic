#!/usr/bin/env python3
"""
Re-publishes /tf_static transforms to /tf at 2 Hz so host-side nodes
(which can receive /tf but not TRANSIENT_LOCAL /tf_static) can see them.
Runs inside the container via docker exec.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TFStaticRelay(Node):
    def __init__(self):
        super().__init__('tf_static_relay')

        transient_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self._pub = self.create_publisher(TFMessage, '/tf', 10)
        self._transforms = {}

        self._sub = self.create_subscription(
            TFMessage,
            '/tf_static',
            self._on_static_tf,
            transient_qos
        )

        self.create_timer(0.5, self._publish_cached)
        self.get_logger().info('TF static relay started')

    def _on_static_tf(self, msg):
        for tf in msg.transforms:
            key = (tf.header.frame_id, tf.child_frame_id)
            if key not in self._transforms:
                self.get_logger().info(
                    f'Cached: {tf.header.frame_id} -> {tf.child_frame_id}'
                )
            self._transforms[key] = tf

    def _publish_cached(self):
        if self._transforms:
            msg = TFMessage()
            msg.transforms = list(self._transforms.values())
            self._pub.publish(msg)


def main():
    rclpy.init()
    node = TFStaticRelay()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
