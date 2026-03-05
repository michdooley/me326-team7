#!/usr/bin/env python3
"""
Drop item: extend arm and open gripper into the bin.

Assumes:
- Start pose of the arm is unknown.
- The gripper is already holding an item.

Does:
1. Move arm to an extended pose over the bin (forward + slightly down).
2. Open gripper to drop the item.

Usage:
  # Terminal 1
  ros2 launch tidybot_bringup sim.launch.py

  # Terminal 2
  ros2 run tidybot_bringup drop_item_sim.py
"""

import rclpy
from rclpy.node import Node
import time

from tidybot_msgs.msg import ArmCommand
from std_msgs.msg import Float64MultiArray

# [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate] — arm extended forward, gripper over bin
DROP_POSITION = [0.0, 0.5, 0.6, 0.0, -0.5, 0.0]
MOVE_DURATION = 2.5


class DropItemNode(Node):
    def __init__(self, arm="right"):
        super().__init__("drop_item")
        self.arm = arm
        self.arm_pub = self.create_publisher(ArmCommand, f"/{arm}_arm/cmd", 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, f"/{arm}_gripper/cmd", 10)
        self.get_logger().info(f"Drop item: will extend {arm} arm then open gripper.")
        self.start_time = None
        self.timer = self.create_timer(0.02, self.tick)

    def tick(self):
        now = time.time()
        if self.start_time is None:
            self.start_time = now
            cmd = ArmCommand()
            cmd.joint_positions = DROP_POSITION
            cmd.duration = MOVE_DURATION
            self.arm_pub.publish(cmd)
            self.get_logger().info("Extending arm over bin...")
            return

        elapsed = now - self.start_time
        if elapsed < MOVE_DURATION + 0.3:
            # Keep holding (gripper stays closed until we open it)
            return

        if elapsed < MOVE_DURATION + 0.5:
            open_cmd = Float64MultiArray()
            open_cmd.data = [0.0]
            self.gripper_pub.publish(open_cmd)
            self.get_logger().info("Opening gripper — item dropped.")
            return

        if elapsed > MOVE_DURATION + 1.5:
            self.get_logger().info("Done.")
            raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = DropItemNode(arm="right")
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
