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
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from .aic_robot import arm_joint_names
from .types import JointMotionUpdateActionDict, MotionUpdateActionDict


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

    approach_z_offset: float = 0.15    # m above port for the approach hover point
    approach_gain: float = 2.5         # P-gain: vel = gain * position_error
    approach_threshold: float = 0.012  # m: switch to DESCEND when error < this
    descent_speed: float = 0.002       # m/s downward (base_link +Z is up)
    lateral_gain: float = 3.0          # P-gain for XY correction during descent
    max_depth_m: float = 0.032         # m: declare insertion done when depth reaches this
    max_speed: float = 0.08            # m/s: hard clip on all velocity components
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

        self._is_connected: bool = False
        self._node = None
        self._tf_buffer: Buffer | None = None
        self._scoring_tf_sub = None
        self._executor = None
        self._executor_thread: Thread | None = None

        self._phase: str = "WAIT"
        self._approach_target: tuple[float, float, float] | None = None
        self._descent_start_z: float | None = None

        self._zero: MotionUpdateActionDict = {
            "linear.x": 0.0, "linear.y": 0.0, "linear.z": 0.0,
            "angular.x": 0.0, "angular.y": 0.0, "angular.z": 0.0,
        }

    @property
    def name(self) -> str:
        return "aic_cheatcode"

    @property
    def action_features(self) -> dict:
        return MotionUpdateActionDict.__annotations__

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

    def _lookup_xyz(self, frame: str) -> tuple[float, float, float] | None:
        """Return (x, y, z) of frame in base_link, or None if unavailable."""
        if self._tf_buffer is None:
            return None
        try:
            tf = self._tf_buffer.lookup_transform("base_link", frame, RosTime())
            t = tf.transform.translation
            return (t.x, t.y, t.z)
        except Exception:
            return None

    def _clip_vel(self, v: float) -> float:
        return float(min(max(v, -self.config.max_speed), self.config.max_speed))

    def get_action(self) -> dict[str, Any]:
        if not self._is_connected:
            raise DeviceNotConnectedError()

        cfg = self.config

        # ------------------------------------------------------------------
        # WAIT: poll until TF frames are available
        # ------------------------------------------------------------------
        if self._phase == "WAIT":
            port_xyz = self._lookup_xyz(self._port_frame)
            tcp_xyz = self._lookup_xyz("gripper/tcp")
            if port_xyz is None or tcp_xyz is None:
                return cast(dict, self._zero)
            self._approach_target = (
                port_xyz[0],
                port_xyz[1],
                port_xyz[2] + cfg.approach_z_offset,
            )
            self._phase = "APPROACH"
            print(
                f"[aic_cheatcode] TF ready — port={port_xyz}  "
                f"hover_target={self._approach_target}  → APPROACH"
            )

        # ------------------------------------------------------------------
        # APPROACH: P-controller toward hover point above port
        # ------------------------------------------------------------------
        if self._phase == "APPROACH":
            tcp_xyz = self._lookup_xyz("gripper/tcp")
            if tcp_xyz is None or self._approach_target is None:
                return cast(dict, self._zero)
            tgt = self._approach_target
            ex = tgt[0] - tcp_xyz[0]
            ey = tgt[1] - tcp_xyz[1]
            ez = tgt[2] - tcp_xyz[2]
            dist = math.sqrt(ex * ex + ey * ey + ez * ez)
            if dist < cfg.approach_threshold:
                self._descent_start_z = tcp_xyz[2]
                self._phase = "DESCEND"
                print(
                    f"[aic_cheatcode] Arrived at hover (err={dist*1000:.1f}mm) "
                    f"start_z={self._descent_start_z:.4f}  → DESCEND"
                )
                return cast(dict, self._zero)
            g = cfg.approach_gain
            return cast(dict, {
                "linear.x": self._clip_vel(g * ex),
                "linear.y": self._clip_vel(g * ey),
                "linear.z": self._clip_vel(g * ez),
                "angular.x": 0.0, "angular.y": 0.0, "angular.z": 0.0,
            })

        # ------------------------------------------------------------------
        # DESCEND: constant -Z with XY correction toward port centre
        # ------------------------------------------------------------------
        if self._phase == "DESCEND":
            tcp_xyz = self._lookup_xyz("gripper/tcp")
            port_xyz = self._lookup_xyz(self._port_frame)
            if tcp_xyz is None or port_xyz is None or self._descent_start_z is None:
                return cast(dict, self._zero)
            depth = self._descent_start_z - tcp_xyz[2]
            if depth >= cfg.max_depth_m:
                self._phase = "DONE"
                print(
                    f"[aic_cheatcode] Insertion complete "
                    f"(depth={depth*1000:.1f}mm ≥ {cfg.max_depth_m*1000:.0f}mm)  → DONE"
                )
                with open(cfg.done_flag_path, "w") as fh:
                    fh.write("done\n")
                return cast(dict, self._zero)
            ex = port_xyz[0] - tcp_xyz[0]
            ey = port_xyz[1] - tcp_xyz[1]
            g = cfg.lateral_gain
            return cast(dict, {
                "linear.x": self._clip_vel(g * ex),
                "linear.y": self._clip_vel(g * ey),
                "linear.z": -cfg.descent_speed,   # negative = descend (base_link +Z up)
                "angular.x": 0.0, "angular.y": 0.0, "angular.z": 0.0,
            })

        # ------------------------------------------------------------------
        # DONE
        # ------------------------------------------------------------------
        return cast(dict, self._zero)

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
        self._approach_target = None
        self._descent_start_z = None
        print("[aic_cheatcode] Reset for new episode")
