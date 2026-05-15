#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

import math
import os
import time
from dataclasses import dataclass, field
from threading import Thread
from typing import Any, cast

import pyspacemouse
import rclpy
from geometry_msgs.msg import Twist
from lerobot.teleoperators import Teleoperator, TeleoperatorConfig
from lerobot.teleoperators.keyboard import (
    KeyboardEndEffectorTeleop,
    KeyboardEndEffectorTeleopConfig,
)
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot_teleoperator_devices import KeyboardJointTeleop, KeyboardJointTeleopConfig
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time as RosTime
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from .aic_robot import arm_joint_names
from .types import (
    JointMotionUpdateActionDict,
    MotionUpdateActionDict,
    PoseMotionUpdateActionDict,
)


@TeleoperatorConfig.register_subclass("aic_keyboard_joint")
@dataclass
class AICKeyboardJointTeleopConfig(KeyboardJointTeleopConfig):
    arm_action_keys: list[str] = field(
        default_factory=lambda: [f"{x}" for x in arm_joint_names]
    )
    high_command_scaling: float = 0.05
    low_command_scaling: float = 0.02


class AICKeyboardJointTeleop(KeyboardJointTeleop):
    def __init__(self, config: AICKeyboardJointTeleopConfig):
        super().__init__(config)

        self.config = config
        self._low_scaling = config.low_command_scaling
        self._high_scaling = config.high_command_scaling
        self._current_scaling = self._high_scaling

        self.curr_joint_actions: JointMotionUpdateActionDict = {
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": 0.0,
            "elbow_joint": 0.0,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        }

    @property
    def action_features(self) -> dict:
        return {"names": JointMotionUpdateActionDict.__annotations__}

    def _get_action_value(self, is_pressed: bool) -> float:
        return self._current_scaling if is_pressed else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()

        for key, is_pressed in self.current_pressed.items():

            if key == "u" and is_pressed:
                is_low_scaling = self._current_scaling == self._low_scaling
                self._current_scaling = (
                    self._high_scaling if is_low_scaling else self._low_scaling
                )
                print(f"Command scaling toggled to: {self._current_scaling}")
                continue

            val = self._get_action_value(is_pressed)

            if key == "q":
                self.curr_joint_actions["shoulder_pan_joint"] = val
            elif key == "a":
                self.curr_joint_actions["shoulder_pan_joint"] = -val
            elif key == "w":
                self.curr_joint_actions["shoulder_lift_joint"] = val
            elif key == "s":
                self.curr_joint_actions["shoulder_lift_joint"] = -val
            elif key == "e":
                self.curr_joint_actions["elbow_joint"] = val
            elif key == "d":
                self.curr_joint_actions["elbow_joint"] = -val
            elif key == "r":
                self.curr_joint_actions["wrist_1_joint"] = val
            elif key == "f":
                self.curr_joint_actions["wrist_1_joint"] = -val
            elif key == "t":
                self.curr_joint_actions["wrist_2_joint"] = val
            elif key == "g":
                self.curr_joint_actions["wrist_2_joint"] = -val
            elif key == "y":
                self.curr_joint_actions["wrist_3_joint"] = val
            elif key == "h":
                self.curr_joint_actions["wrist_3_joint"] = -val
            elif is_pressed:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        return cast(dict, self.curr_joint_actions)


@TeleoperatorConfig.register_subclass("aic_keyboard_ee")
@dataclass(kw_only=True)
class AICKeyboardEETeleopConfig(KeyboardEndEffectorTeleopConfig):
    high_command_scaling: float = 0.1
    low_command_scaling: float = 0.02


class AICKeyboardEETeleop(KeyboardEndEffectorTeleop):
    def __init__(self, config: AICKeyboardEETeleopConfig):
        super().__init__(config)
        self.config = config

        self._high_scaling = config.high_command_scaling
        self._low_scaling = config.low_command_scaling
        self._current_scaling = self._high_scaling

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
        }

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    def _get_action_value(self, is_pressed: bool) -> float:
        return self._current_scaling if is_pressed else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError()

        self._drain_pressed_keys()

        for key, is_pressed in self.current_pressed.items():

            if key == "t" and is_pressed:
                is_low_speed = self._current_scaling == self._low_scaling
                self._current_scaling = (
                    self._high_scaling if is_low_speed else self._low_scaling
                )
                print(f"Command scaling toggled to: {self._current_scaling}")
                continue

            val = self._get_action_value(is_pressed)

            if key == "w":
                self._current_actions["linear.y"] = -val
            elif key == "s":
                self._current_actions["linear.y"] = val
            elif key == "a":
                self._current_actions["linear.x"] = -val
            elif key == "d":
                self._current_actions["linear.x"] = val
            elif key == "r":
                self._current_actions["linear.z"] = -val
            elif key == "f":
                self._current_actions["linear.z"] = val
            elif key == "W":
                self._current_actions["angular.x"] = val
            elif key == "S":
                self._current_actions["angular.x"] = -val
            elif key == "A":
                self._current_actions["angular.y"] = -val
            elif key == "D":
                self._current_actions["angular.y"] = val
            elif key == "q":
                self._current_actions["angular.z"] = -val
            elif key == "e":
                self._current_actions["angular.z"] = val
            elif is_pressed:
                # If the key is pressed, add it to the misc_keys_queue
                # this will record key presses that are not part of the delta_x, delta_y, delta_z
                # this is useful for retrieving other events like interventions for RL, episode success, etc.
                self.misc_keys_queue.put(key)

        self.current_pressed.clear()

        return cast(dict, self._current_actions)


@TeleoperatorConfig.register_subclass("aic_spacemouse")
@dataclass(kw_only=True)
class AICSpaceMouseTeleopConfig(TeleoperatorConfig):
    operator_position_front: bool = True
    device: str | None = None  # only needed for multiple space mice
    command_scaling: float = 0.1


class AICSpaceMouseTeleop(Teleoperator):
    def __init__(self, config: AICSpaceMouseTeleopConfig):
        super().__init__(config)
        self.config = config
        self._is_connected = False
        self._device: pyspacemouse.SpaceMouseDevice | None = None

        self._current_actions: MotionUpdateActionDict = {
            "linear.x": 0.0,
            "linear.y": 0.0,
            "linear.z": 0.0,
            "angular.x": 0.0,
            "angular.y": 0.0,
            "angular.z": 0.0,
        }

    @property
    def name(self) -> str:
        return "aic_spacemouse"

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

    @property
    def feedback_features(self) -> dict:
        # TODO
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError()

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node("spacemouse_teleop")
        if calibrate:
            self._node.get_logger().warn(
                "Calibration not supported, ensure the robot is calibrated before running teleop."
            )

        self._device = pyspacemouse.open(
            dof_callback=None,
            # button_callback_arr=[
            #     pyspacemouse.ButtonCallback([0], self._button_callback),  # Button 1
            #     pyspacemouse.ButtonCallback([1], self._button_callback),  # Button 2
            # ],
            device=self.config.device,
        )

        if self._device is None:
            raise RuntimeError("Failed to open SpaceMouse device")

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(target=self._executor.spin)
        self._executor_thread.start()
        self._is_connected = True

    @property
    def is_calibrated(self) -> bool:
        # Calibration not supported
        return True

    def calibrate(self) -> None:
        # Calibration not supported
        pass

    def configure(self) -> None:
        pass

    def apply_deadband(self, value, threshold=0.02):
        return value if abs(value) > threshold else 0.0

    def get_action(self) -> dict[str, Any]:
        if not self.is_connected or not self._device:
            raise DeviceNotConnectedError()

        state = self._device.read()

        clean_x = self.apply_deadband(float(state.x))
        clean_y = self.apply_deadband(float(state.y))
        clean_z = self.apply_deadband(float(state.z))
        clean_roll = self.apply_deadband(float(state.roll))
        clean_pitch = self.apply_deadband(float(state.pitch))
        clean_yaw = self.apply_deadband(float(state.yaw))

        twist_msg = Twist()
        twist_msg.linear.x = clean_x**1 * self.config.command_scaling
        twist_msg.linear.y = -(clean_y**1) * self.config.command_scaling
        twist_msg.linear.z = -(clean_z**1) * self.config.command_scaling
        twist_msg.angular.x = -(clean_pitch**1) * self.config.command_scaling
        twist_msg.angular.y = clean_roll**1 * self.config.command_scaling  #
        twist_msg.angular.z = clean_yaw**1 * self.config.command_scaling

        if not self.config.operator_position_front:
            twist_msg.linear.x *= -1
            twist_msg.linear.y *= -1
            twist_msg.angular.x *= -1
            twist_msg.angular.y *= -1

        self._current_actions = {
            "linear.x": twist_msg.linear.x,
            "linear.y": twist_msg.linear.y,
            "linear.z": twist_msg.linear.z,
            "angular.x": twist_msg.angular.x,
            "angular.y": twist_msg.angular.y,
            "angular.z": twist_msg.angular.z,
        }

        return cast(dict, self._current_actions)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if self._device:
            self._device.close()
        self._is_connected = False
        pass


# ---------------------------------------------------------------------------
# AIC CheatCode Teleop — autonomous velocity-based insertion for recording
# ---------------------------------------------------------------------------

_TRIAL_DEFAULTS: dict[str, dict[str, str]] = {
    "t1": {
        "target_port_frame": "task_board/nic_card_mount_0/sfp_port_0_link",
        "cable_tip_frame": "cable_0/sfp_tip_link",
    },
    "t2": {
        "target_port_frame": "task_board/nic_card_mount_1/sfp_port_0_link",
        "cable_tip_frame": "cable_0/sfp_tip_link",
    },
    "t3": {
        "target_port_frame": "task_board/sc_port_1/sc_port_base_link",
        "cable_tip_frame": "cable_1/sc_tip_link",
    },
}


@TeleoperatorConfig.register_subclass("aic_cheatcode")
@dataclass(kw_only=True)
class AICCheatCodeTeleopConfig(TeleoperatorConfig):
    """Configuration for the autonomous CheatCode teleop used during demo collection."""

    trial_type: str = "t1"         # "t1", "t2", or "t3" — sets default frames
    target_port_frame: str = ""    # overrides trial_type when set
    cable_tip_frame: str = ""      # overrides trial_type when set

    approach_z_offset: float = 0.12    # m above port for the approach hover point
    approach_gain: float = 2.5         # P-gain: vel = gain * position_error
    approach_threshold: float = 0.012  # m: switch to DESCEND when error < this
    lateral_gain: float = 4.0          # P-gain for XY correction during descent
    descent_gain: float = 1.5          # P-gain for Z insertion target
    angular_gain: float = 2.0          # P-gain for plug-to-port orientation error
    insertion_depth_m: float = 0.006   # m below port frame used as final depth target
    done_xy_threshold: float = 0.007   # m: allow done when tip is centered at depth
    max_speed: float = 0.10            # m/s: hard clip on linear velocity components
    max_angular_speed: float = 0.5     # rad/s: hard clip on angular velocity components
    done_flag_path: str = "/tmp/aic_cheatcode_done"  # written when episode is complete


class AICCheatCodeTeleop(Teleoperator):
    """Autonomous CheatCode teleop for lerobot-record demo collection.

    Uses ground-truth TF (requires eval with ground_truth:=true) to drive the
    gripper toward the target port without any human input.

    State machine:
        WAIT     — TF frames not yet available; returns zero velocity.
        APPROACH — P-controller drives gripper/tcp toward (port + z_offset).
        DESCEND  — Constant -Z velocity with XY lateral correction.
        DONE     — Writes done_flag_path; returns zero velocity.

    The collect_demos.sh script watches done_flag_path and advances the episode
    via ``xdotool key Right`` when the flag appears.

    Usage with lerobot-record:
        pixi run lerobot-record \\
            --robot.type=aic_controller --robot.id=aic \\
            --robot.teleop_target_mode=cartesian \\
            --robot.teleop_frame_id=base_link \\
            --teleop.type=aic_cheatcode --teleop.id=aic \\
            --teleop.trial_type=t1 \\
            ...
    """

    def __init__(self, config: AICCheatCodeTeleopConfig) -> None:
        super().__init__(config)
        self.config = config

        defaults = _TRIAL_DEFAULTS.get(config.trial_type, _TRIAL_DEFAULTS["t1"])
        self._port_frame: str = config.target_port_frame or defaults["target_port_frame"]
        self._tip_frame: str = config.cable_tip_frame or defaults["cable_tip_frame"]
        cable_name = self._tip_frame.split("/", 1)[0]
        self._tip_frame_candidates: list[str] = [
            self._tip_frame,
            f"{cable_name}/link_10",
            f"{cable_name}/link_9",
            f"{cable_name}/link_11",
            f"{cable_name}/link_8",
            "gripper/tcp",
        ]
        self._active_tip_frame: str = self._tip_frame

        self._is_connected: bool = False
        self._node = None
        self._tf_buffer: Buffer | None = None
        self._scoring_tf_sub = None
        self._insertion_event_sub = None
        self._executor = None
        self._executor_thread: Thread | None = None

        self._phase: str = "WAIT"
        self._port_transform = None
        self._approach_step: int = 0
        self._z_offset: float = 0.2
        self._phase_started_at: float = time.monotonic()
        self._insertion_event_seen: bool = False
        self._last_status_log: float = 0.0
        self._tip_x_error_integrator: float = 0.0
        self._tip_y_error_integrator: float = 0.0
        self._max_integrator_windup: float = 0.05
        self._last_pose_action: PoseMotionUpdateActionDict | None = None

    @property
    def name(self) -> str:
        return "aic_cheatcode"

    @property
    def action_features(self) -> dict:
        return PoseMotionUpdateActionDict.__annotations__

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        if self._is_connected:
            raise DeviceAlreadyConnectedError()
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("aic_cheatcode_teleop")
        self._tf_buffer = Buffer()
        TransformListener(self._tf_buffer, self._node)
        self._scoring_tf_sub = self._node.create_subscription(
            TFMessage, '/scoring/tf', self._on_scoring_tf, 10)
        self._insertion_event_sub = self._node.create_subscription(
            String, "/scoring/insertion_event", self._on_insertion_event, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = Thread(target=self._executor.spin, daemon=True)
        self._executor_thread.start()
        self._is_connected = True
        if os.path.exists(self.config.done_flag_path):
            os.remove(self.config.done_flag_path)
        print(
            f"[aic_cheatcode] Connected. Watching TF frames:\n"
            f"  port : {self._port_frame}\n"
            f"  tip candidates: {', '.join(self._tip_frame_candidates)}\n"
            f"  gripper: gripper/tcp"
        )

    def _on_scoring_tf(self, msg: TFMessage) -> None:
        """Feed /scoring/tf transforms into the tf2 buffer for frame lookup.

        /scoring/tf uses RELIABLE QoS and is bridged by Zenoh from the container
        to the host. task_board frames are published here (not on /tf_static which
        uses TRANSIENT_LOCAL and is not bridged). Calling set_transform makes them
        available to _lookup_xyz via the normal tf2 buffer lookup.
        """
        for transform in msg.transforms:
            self._tf_buffer.set_transform(transform, 'scoring_tf')

    def _on_insertion_event(self, msg: String) -> None:
        event_port = msg.data.strip("/")
        target_port = self._port_frame.removeprefix("task_board/").removesuffix(
            "_link"
        )
        if event_port.endswith(target_port) or target_port.endswith(event_port):
            print(f"[aic_cheatcode] Insertion event received: {msg.data}")
            self._insertion_event_seen = True
        else:
            print(f"[aic_cheatcode] Ignoring insertion event for other port: {msg.data}")

    def _lookup_transform(self, frame: str):
        if self._tf_buffer is None:
            return None
        try:
            return self._tf_buffer.lookup_transform("base_link", frame, RosTime())
        except Exception:
            return None

    def _lookup_xyz(self, frame: str) -> tuple[float, float, float] | None:
        """Return (x, y, z) of frame in base_link, or None if unavailable."""
        tf = self._lookup_transform(frame)
        if tf is None:
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def _lookup_tip_xyz(self) -> tuple[float, float, float] | None:
        for frame in self._tip_frame_candidates:
            xyz = self._lookup_xyz(frame)
            if xyz is None:
                continue
            if frame != self._active_tip_frame:
                print(f"[aic_cheatcode] Using tip frame fallback: {frame}")
                self._active_tip_frame = frame
            return xyz
        return None

    def _clip_vel(self, v: float) -> float:
        return float(min(max(v, -self.config.max_speed), self.config.max_speed))

    def _clip_angular(self, v: float) -> float:
        return float(
            min(max(v, -self.config.max_angular_speed), self.config.max_angular_speed)
        )

    @staticmethod
    def _quat_multiply(
        a: tuple[float, float, float, float],
        b: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    @staticmethod
    def _quat_inverse(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
        w, x, y, z = q
        return (w, -x, -y, -z)

    @staticmethod
    def _quat_normalize(
        q: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        norm = math.sqrt(sum(v * v for v in q))
        if norm <= 0.0:
            return (1.0, 0.0, 0.0, 0.0)
        return tuple(v / norm for v in q)

    @classmethod
    def _quat_slerp(
        cls,
        q0: tuple[float, float, float, float],
        q1: tuple[float, float, float, float],
        fraction: float,
    ) -> tuple[float, float, float, float]:
        q0 = cls._quat_normalize(q0)
        q1 = cls._quat_normalize(q1)
        dot = sum(a * b for a, b in zip(q0, q1))
        if dot < 0.0:
            q1 = tuple(-v for v in q1)
            dot = -dot
        if dot > 0.9995:
            return cls._quat_normalize(
                tuple((1.0 - fraction) * a + fraction * b for a, b in zip(q0, q1))
            )
        theta_0 = math.acos(max(min(dot, 1.0), -1.0))
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * fraction
        s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta_0
        s1 = math.sin(theta) / sin_theta_0
        return cls._quat_normalize(tuple(s0 * a + s1 * b for a, b in zip(q0, q1)))

    @staticmethod
    def _pose_action(
        xyz: tuple[float, float, float],
        quat_wxyz: tuple[float, float, float, float],
    ) -> PoseMotionUpdateActionDict:
        return {
            "target_position.x": xyz[0],
            "target_position.y": xyz[1],
            "target_position.z": xyz[2],
            "target_orientation.x": quat_wxyz[1],
            "target_orientation.y": quat_wxyz[2],
            "target_orientation.z": quat_wxyz[3],
            "target_orientation.w": quat_wxyz[0],
        }

    def _hold_pose_action(self) -> dict[str, Any]:
        gripper_tf = self._lookup_transform("gripper/tcp")
        if gripper_tf is None:
            if self._last_pose_action is not None:
                return cast(dict, self._last_pose_action)
            return cast(dict, {
                "target_position.x": 0.0,
                "target_position.y": 0.0,
                "target_position.z": 0.0,
                "target_orientation.x": 0.0,
                "target_orientation.y": 0.0,
                "target_orientation.z": 0.0,
                "target_orientation.w": 1.0,
            })
        t = gripper_tf.transform.translation
        q = gripper_tf.transform.rotation
        action = self._pose_action((t.x, t.y, t.z), (q.w, q.x, q.y, q.z))
        self._last_pose_action = action
        return cast(dict, action)

    def _calc_gripper_pose_action(
        self,
        slerp_fraction: float = 1.0,
        position_fraction: float = 1.0,
        z_offset: float = 0.1,
        reset_xy_integrator: bool = False,
    ) -> dict[str, Any] | None:
        if self._port_transform is None:
            return None
        plug_tf = self._lookup_transform(self._active_tip_frame)
        gripper_tf = self._lookup_transform("gripper/tcp")
        if plug_tf is None or gripper_tf is None:
            return None

        port = self._port_transform
        q_port = (
            port.rotation.w,
            port.rotation.x,
            port.rotation.y,
            port.rotation.z,
        )
        q_plug_msg = plug_tf.transform.rotation
        q_plug = (q_plug_msg.w, q_plug_msg.x, q_plug_msg.y, q_plug_msg.z)
        # Match aic_example_policies.ros.CheatCode exactly; this is the
        # known-good pose alignment logic used by the ground-truth policy.
        q_plug_inv = (-q_plug[0], q_plug[1], q_plug[2], q_plug[3])
        q_diff = self._quat_multiply(q_port, q_plug_inv)

        q_gripper_msg = gripper_tf.transform.rotation
        q_gripper = (
            q_gripper_msg.w,
            q_gripper_msg.x,
            q_gripper_msg.y,
            q_gripper_msg.z,
        )
        q_gripper_target = self._quat_multiply(q_diff, q_gripper)
        q_gripper_slerp = self._quat_slerp(
            q_gripper, q_gripper_target, slerp_fraction
        )

        gripper_t = gripper_tf.transform.translation
        plug_t = plug_tf.transform.translation
        gripper_xyz = (gripper_t.x, gripper_t.y, gripper_t.z)
        plug_xyz = (plug_t.x, plug_t.y, plug_t.z)
        plug_tip_gripper_offset = (
            gripper_xyz[0] - plug_xyz[0],
            gripper_xyz[1] - plug_xyz[1],
            gripper_xyz[2] - plug_xyz[2],
        )

        tip_x_error = port.translation.x - plug_xyz[0]
        tip_y_error = port.translation.y - plug_xyz[1]

        if reset_xy_integrator:
            self._tip_x_error_integrator = 0.0
            self._tip_y_error_integrator = 0.0
        else:
            self._tip_x_error_integrator = min(
                max(
                    self._tip_x_error_integrator + tip_x_error,
                    -self._max_integrator_windup,
                ),
                self._max_integrator_windup,
            )
            self._tip_y_error_integrator = min(
                max(
                    self._tip_y_error_integrator + tip_y_error,
                    -self._max_integrator_windup,
                ),
                self._max_integrator_windup,
            )

        i_gain = 0.15
        target_x = port.translation.x + i_gain * self._tip_x_error_integrator
        target_y = port.translation.y + i_gain * self._tip_y_error_integrator
        target_z = port.translation.z + z_offset - plug_tip_gripper_offset[2]
        blend_xyz = (
            position_fraction * target_x + (1.0 - position_fraction) * gripper_xyz[0],
            position_fraction * target_y + (1.0 - position_fraction) * gripper_xyz[1],
            position_fraction * target_z + (1.0 - position_fraction) * gripper_xyz[2],
        )
        action = self._pose_action(blend_xyz, q_gripper_slerp)
        self._last_pose_action = action
        self._log_status(
            f"[aic_cheatcode] {self._phase} z_offset={z_offset:.4f} "
            f"xy_error=({tip_x_error:.4f},{tip_y_error:.4f}) "
            f"tip_frame={self._active_tip_frame}"
        )
        return cast(dict, action)

    def _set_phase(self, phase: str) -> None:
        self._phase = phase
        self._phase_started_at = time.monotonic()

    def _write_done_flag(self, reason: str) -> None:
        if not os.path.exists(self.config.done_flag_path):
            print(f"[aic_cheatcode] Done: {reason}")
            with open(self.config.done_flag_path, "w") as fh:
                fh.write(f"{reason}\n")

    def _log_status(self, message: str, period_s: float = 5.0) -> None:
        now = time.monotonic()
        if now - self._last_status_log >= period_s:
            print(message)
            self._last_status_log = now

    def get_action(self) -> dict[str, Any]:
        if not self._is_connected:
            raise DeviceNotConnectedError()

        cfg = self.config

        if self._insertion_event_seen:
            self._set_phase("DONE")
            self._write_done_flag("insertion_event")
            return self._hold_pose_action()

        # ------------------------------------------------------------------
        # WAIT: poll until TF frames are available
        # ------------------------------------------------------------------
        if self._phase == "WAIT":
            port_tf = self._lookup_transform(self._port_frame)
            port_xyz = self._lookup_xyz(self._port_frame)
            tip_xyz = self._lookup_tip_xyz()
            tcp_xyz = self._lookup_xyz("gripper/tcp")
            if port_tf is None or port_xyz is None or tip_xyz is None or tcp_xyz is None:
                self._log_status(
                    "[aic_cheatcode] Waiting for TF: "
                    f"port={port_xyz is not None} tip={tip_xyz is not None} "
                    f"tcp={tcp_xyz is not None}"
                )
                return self._hold_pose_action()
            self._port_transform = port_tf.transform
            self._approach_step = 0
            self._z_offset = 0.2
            self._set_phase("APPROACH")
            print(
                f"[aic_cheatcode] TF ready - port={port_xyz} tip={tip_xyz} "
                f"tip_frame={self._active_tip_frame} -> APPROACH"
            )

        # ------------------------------------------------------------------
        # APPROACH: match CheatCode.py by interpolating over 100 pose targets.
        # ------------------------------------------------------------------
        if self._phase == "APPROACH":
            if self._approach_step >= 100:
                self._set_phase("DESCEND")
                print("[aic_cheatcode] Finished approach -> DESCEND")
                return self._hold_pose_action()
            interp_fraction = self._approach_step / 100.0
            self._approach_step += 1
            action = self._calc_gripper_pose_action(
                slerp_fraction=interp_fraction,
                position_fraction=interp_fraction,
                z_offset=self._z_offset,
                reset_xy_integrator=True,
            )
            return action if action is not None else self._hold_pose_action()

        # ------------------------------------------------------------------
        # DESCEND: match CheatCode.py by walking z_offset down to -0.015.
        # ------------------------------------------------------------------
        if self._phase == "DESCEND":
            if self._z_offset < -0.015:
                self._set_phase("DONE")
                self._write_done_flag("cheatcode_descent_complete")
                return self._hold_pose_action()
            self._z_offset -= 0.0005
            action = self._calc_gripper_pose_action(z_offset=self._z_offset)
            return action if action is not None else self._hold_pose_action()

        # ------------------------------------------------------------------
        # DONE
        # ------------------------------------------------------------------
        return self._hold_pose_action()

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if self._node:
            self._node.destroy_node()
        if self._executor:
            self._executor.shutdown()
        if self._executor_thread:
            self._executor_thread.join(timeout=2.0)
        self._is_connected = False

    def reset(self) -> None:
        """Call between episodes to re-arm the state machine."""
        if os.path.exists(self.config.done_flag_path):
            os.remove(self.config.done_flag_path)
        self._phase = "WAIT"
        self._port_transform = None
        self._approach_step = 0
        self._z_offset = 0.2
        self._phase_started_at = time.monotonic()
        self._insertion_event_seen = False
        self._last_status_log = 0.0
        self._tip_x_error_integrator = 0.0
        self._tip_y_error_integrator = 0.0
        self._last_pose_action = None
        print("[aic_cheatcode] Reset for new episode")
