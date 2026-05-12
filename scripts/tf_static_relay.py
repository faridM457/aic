#!/usr/bin/env python3
"""Re-publish /tf_static (TRANSIENT_LOCAL) to /tf (RELIABLE) for zenoh cross-container bridging.

Zenoh RMW does not bridge TRANSIENT_LOCAL QoS topics (/tf_static) across the
container/host boundary despite --network host.  This relay subscribes to
/tf_static inside the container and re-publishes every received transform to
/tf at 2 Hz, making task_board and cable frames visible to tf2_ros listeners
on the host so the aic_cheatcode teleop can leave its WAIT phase.

Run inside the eval container via collect_demos.sh Pane 2:
  docker exec aic_eval bash -c \
    'source /ws_aic/install/setup.bash && python3 <aic_dir>/scripts/tf_static_relay.py'
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfStaticRelay(Node):
    def __init__(self) -> None:
        super().__init__("tf_static_relay")
        self._transforms: dict[str, object] = {}  # keyed by child_frame_id

        qos_transient = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(TFMessage, "/tf_static", self._on_tf_static, qos_transient)
        self._pub = self.create_publisher(TFMessage, "/tf", 10)
        # Re-publish at 2 Hz so the host tf2_buffer keeps the transforms fresh.
        self.create_timer(0.5, self._publish)

    def _on_tf_static(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            self._transforms[t.child_frame_id] = t
        self.get_logger().info(
            f"tf_static_relay: caching {len(self._transforms)} static frames"
        )

    def _publish(self) -> None:
        if not self._transforms:
            return
        self._pub.publish(TFMessage(transforms=list(self._transforms.values())))


def main() -> None:
    rclpy.init()
    rclpy.spin(TfStaticRelay())


if __name__ == "__main__":
    main()
