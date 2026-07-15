#!/usr/bin/env python3
from typing import Dict
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

class CoppeliaToForwardPositionRobot2(Node):
    def __init__(self) -> None:
        super().__init__("coppelia_to_forward_position_robot2_ns")
        self.pub = self.create_publisher(
            Float64MultiArray,
            "/robot2/forward_position_controller/commands",
            10,
        )
        self.create_subscription(
            JointState,
            "/robot2/coppelia_joint_target",
            self.cb,
            10,
        )
        self.get_logger().info("robot2 bridge started")

    def cb(self, msg: JointState) -> None:
        name_to_pos: Dict[str, float] = dict(zip(msg.name, msg.position))
        try:
            ordered = [float(name_to_pos[name]) for name in JOINT_NAMES]
        except KeyError:
            self.get_logger().warn("Missing expected joint names")
            return
        out = Float64MultiArray()
        out.data = ordered
        self.pub.publish(out)

def main() -> None:
    rclpy.init()
    node = CoppeliaToForwardPositionRobot2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
