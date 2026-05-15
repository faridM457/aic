from typing import TypedDict

MotionUpdateActionDict = TypedDict(
    "MotionUpdateActionDict",
    {
        "linear.x": float,
        "linear.y": float,
        "linear.z": float,
        "angular.x": float,
        "angular.y": float,
        "angular.z": float,
    },
)

PoseMotionUpdateActionDict = TypedDict(
    "PoseMotionUpdateActionDict",
    {
        "target_position.x": float,
        "target_position.y": float,
        "target_position.z": float,
        "target_orientation.x": float,
        "target_orientation.y": float,
        "target_orientation.z": float,
        "target_orientation.w": float,
    },
)

JointMotionUpdateActionDict = TypedDict(
    "JointMotionUpdateActionDict",
    {
        "shoulder_pan_joint": float,
        "shoulder_lift_joint": float,
        "elbow_joint": float,
        "wrist_1_joint": float,
        "wrist_2_joint": float,
        "wrist_3_joint": float,
    },
)
