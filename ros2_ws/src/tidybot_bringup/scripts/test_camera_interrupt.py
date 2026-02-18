#!/usr/bin/env python3
"""
Camera interrupt test.

Sends a large tilt command (tilt to maximum), waits 0.5 s, then immediately
sends the camera's CURRENT position as a new setpoint — to see whether the
position controller can be interrupted mid-motion.

Expected outcomes:
  - If interruption works: camera stops roughly where it was at t=0.5 s.
  - If interruption does NOT work: camera continues all the way to max tilt.

Usage:
    ros2 run tidybot_bringup test_camera_interrupt.py

Run with the simulation already started (sim.launch.py).
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class CameraInterruptTest(Node):
    def __init__(self):
        super().__init__('camera_interrupt_test')

        self._joint_states: dict = {}

        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10
        )
        self._pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10
        )

    def _js_cb(self, msg: JointState) -> None:
        for i, name in enumerate(msg.name):
            self._joint_states[name] = msg.position[i] if i < len(msg.position) else 0.0

    def _send(self, pan: float, tilt: float) -> None:
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self._pub.publish(msg)
        self.get_logger().info(f'  → sent pan={pan:.3f}, tilt={tilt:.3f}')

    def _spin(self, duration: float) -> None:
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self) -> None:
        log = self.get_logger()

        # Wait for joint state messages
        log.info('Waiting for /joint_states ...')
        self._spin(2.0)
        if not self._joint_states:
            log.error('No joint states received — is the simulation running?')
            return

        pan0  = self._joint_states.get('camera_pan',  0.0)
        tilt0 = self._joint_states.get('camera_tilt', 0.0)
        log.info(f'Starting position: pan={pan0:.3f}, tilt={tilt0:.3f}')

        # --- Step 1: first return camera to a known position (tilt=0) ---
        log.info('Step 1: homing camera to tilt=0 ...')
        self._send(0.0, 0.0)
        self._spin(2.0)
        log.info(f'  After homing: pan={self._joint_states.get("camera_pan", 0):.3f}, '
                 f'tilt={self._joint_states.get("camera_tilt", 0):.3f}')

        # --- Step 2: send a large tilt command (will take > 1 s to reach) ---
        TARGET_TILT = 1.4   # near maximum — takes ~1.5 s to reach
        log.info(f'Step 2: sending large tilt command (target={TARGET_TILT}) ...')
        self._send(0.0, TARGET_TILT)

        # --- Step 3: wait 0.5 s then read where camera actually is ---
        self._spin(0.5)
        mid_pan  = self._joint_states.get('camera_pan',  0.0)
        mid_tilt = self._joint_states.get('camera_tilt', 0.0)
        log.info(f'After 0.5 s: pan={mid_pan:.3f}, tilt={mid_tilt:.3f}')

        # --- Step 4: send an interrupt (freeze at current position) ---
        log.info('Step 4: sending INTERRUPT (freeze at current position) ...')
        self._send(mid_pan, mid_tilt)

        # --- Step 5: wait 1.5 s and check final position ---
        self._spin(1.5)
        final_pan  = self._joint_states.get('camera_pan',  0.0)
        final_tilt = self._joint_states.get('camera_tilt', 0.0)
        log.info(f'Final position: pan={final_pan:.3f}, tilt={final_tilt:.3f}')

        # --- Verdict ---
        drift = abs(final_tilt - mid_tilt)
        log.info('')
        log.info('=== RESULT ===')
        log.info(f'  Tilt at interrupt:  {mid_tilt:.3f} rad')
        log.info(f'  Tilt after 1.5 s:   {final_tilt:.3f} rad')
        log.info(f'  Drift after freeze: {drift:.3f} rad ({drift*57.3:.1f} deg)')
        if drift < 0.05:
            log.info('  >>> INTERRUPT WORKS: camera stopped at commanded position.')
        elif final_tilt > TARGET_TILT - 0.05:
            log.warn('  >>> INTERRUPT FAILED: camera drove to maximum tilt regardless.')
        else:
            log.warn(f'  >>> PARTIAL: camera drifted {drift:.3f} rad after freeze command.')

        # Return to home
        log.info('Returning to home (tilt=0) ...')
        self._send(0.0, 0.0)
        self._spin(2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInterruptTest()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
