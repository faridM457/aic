#!/usr/bin/env python3
"""
Re-publishes /tf_static transforms to /tf at 2 Hz so host-side nodes can see them.

Runs INSIDE the eval container via docker exec. Zenoh does not bridge
TRANSIENT_LOCAL QoS (/tf_static) across the container/host boundary, so the
relay must run inside the container where it can receive cached static TF.
It re-publishes every transform to /tf (RELIABLE), which Zenoh does bridge,
making task_board frames visible to the host tf2 stack.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TFStaticRelay(Node):
    def __init__(self):
        super().__init__('aic_tf_static_relay')

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
    try:
        rclpy.init()
    except RuntimeError:
        pass  # already initialized
    node = TFStaticRelay()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
