#!/usr/bin/env python3
"""
TidyBot2 Full Robot Test Script.

Sequentially tests all hardware components by reusing the individual test scripts:
1. Phoenix 6 mobile base (test_base_real.py)
2. Left arm (test_arms_real.py)
3. Right arm (test_arms_real.py)
4. Grippers (test_arms_real.py)
5. Pan-tilt camera system (test_pan_tilt_real.py)

Usage:
    # First, launch the full robot:
    ros2 launch tidybot_bringup real.launch.py

    # Then run this test:
    python3 test_full_robot.py
"""

import os
import sys
import time

import rclpy

# Add scripts directory to path so we can import sibling test scripts
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from test_base_real import TestBase
from test_arms_real import RobotTester
from test_pan_tilt_real import TestPanTilt


def main():
    print()
    print("#" * 60)
    print("#" + "       TidyBot2 Full Robot Hardware Test".center(58) + "#")
    print("#" * 60)
    print()
    print("This test will sequentially verify all hardware components:")
    print("  1. Phoenix 6 mobile base")
    print("  2. Left arm (via sim-compatible topics)")
    print("  3. Right arm (via sim-compatible topics)")
    print("  4. Grippers (via gripper_wrapper_node)")
    print("  5. Pan-tilt camera system")
    print()

    rclpy.init()

    tests_passed = 0
    tests_total = 0

    try:
        # ===== 1. BASE TEST =====
        print("=" * 50)
        print("DETECTING: Phoenix 6 Mobile Base")
        print("=" * 50)
        base_node = TestBase()
        # Give it a moment to detect odom
        start = time.time()
        while not base_node.odom_received and (time.time() - start) < 5.0:
            rclpy.spin_once(base_node, timeout_sec=0.1)

        if base_node.odom_received:
            tests_total += 1
            try:
                result = base_node.run_test()
                if result:
                    tests_passed += 1
            except Exception as e:
                print(f"Base test failed: {e}")
        else:
            print("Skipping base test (not detected)")
        base_node.destroy_node()

        # ===== 2 & 3. ARM TESTS + 4. GRIPPER TEST =====
        print()
        print("=" * 50)
        print("DETECTING: Arms & Grippers")
        print("=" * 50)
        arm_node = RobotTester()

        print("Waiting for joint states from both arms...")
        has_arms = arm_node.wait_for_joint_states(timeout=10.0)

        has_right = arm_node.right_joint_states is not None
        has_left = arm_node.left_joint_states is not None

        print(f"  Right arm: {'Yes' if has_right else 'No'}")
        print(f"  Left arm:  {'Yes' if has_left else 'No'}")

        if has_left:
            tests_total += 1
            try:
                arm_node.test_arm('left')
                tests_passed += 1
            except Exception as e:
                print(f"Left arm test failed: {e}")
        else:
            print("\nSkipping left arm test (not detected)")

        if has_right:
            tests_total += 1
            try:
                arm_node.test_arm('right')
                tests_passed += 1
            except Exception as e:
                print(f"Right arm test failed: {e}")
        else:
            print("\nSkipping right arm test (not detected)")

        if has_right or has_left:
            tests_total += 1
            try:
                arm_node.test_grippers()
                tests_passed += 1
            except Exception as e:
                print(f"Gripper test failed: {e}")
        else:
            print("\nSkipping gripper test (no arms detected)")

        arm_node.destroy_node()

        # ===== 5. PAN-TILT TEST =====
        print()
        print("=" * 50)
        print("DETECTING: Pan-Tilt Camera")
        print("=" * 50)
        pt_node = TestPanTilt()
        start = time.time()
        while not pt_node.state_received and (time.time() - start) < 3.0:
            rclpy.spin_once(pt_node, timeout_sec=0.1)

        if pt_node.state_received:
            tests_total += 1
            try:
                result = pt_node.run_test()
                if result:
                    tests_passed += 1
            except Exception as e:
                print(f"Pan-tilt test failed: {e}")
        else:
            print("Skipping pan-tilt test (not detected)")
        pt_node.destroy_node()

        # ===== SUMMARY =====
        print()
        print("#" * 60)
        print("#" + f"  Test Results: {tests_passed}/{tests_total} passed".center(58) + "#")
        print("#" * 60)
        print()

        if tests_total == 0:
            print("ERROR: No hardware components detected!")
            print("Make sure to launch the robot first:")
            print("  ros2 launch tidybot_bringup real.launch.py")
            return 1
        elif tests_passed == tests_total:
            print("All hardware tests PASSED!")
        else:
            print(f"WARNING: {tests_total - tests_passed} test(s) failed")

        return 0 if tests_passed == tests_total else 1

    except KeyboardInterrupt:
        print("\n\nTest cancelled by user")
        return 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
