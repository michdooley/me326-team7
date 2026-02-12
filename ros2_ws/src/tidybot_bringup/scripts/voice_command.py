#!/usr/bin/env python3
"""
Voice Command Module

Records audio via the microphone_node, sends to Google Cloud Speech-to-Text
for transcription, then uses an LLM (Gemini) to extract the target object
and action from the transcription.

Calls:
    /microphone/record (AudioRecord) - existing microphone service

Publishes (latched / transient_local QoS):
    /target_object (std_msgs/String) - the object to find (e.g. "apple")
    /user_command (std_msgs/String) - the action (e.g. "fetch", "place")

Environment:
    GOOGLE_CLOUD_PROJECT or GOOGLE_APPLICATION_CREDENTIALS must be set

Usage:
    # As a standalone script:
    ros2 run tidybot_bringup voice_command.py

    # Or import the function in another node:
    from voice_command import get_voice_command
    target, action = get_voice_command(self)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String
from tidybot_msgs.srv import AudioRecord


def get_voice_command(node: Node) -> tuple:
    """
    Record audio, transcribe via Google Cloud, extract target object and action.

    Args:
        node: A ROS2 node (used for logging and service calls)

    Returns:
        (target_object, action) tuple of strings, e.g. ("apple", "fetch")
    """
    # Step 1: Record audio via /microphone/record service
    # TODO: Call AudioRecord service to get audio data
    # record_client = node.create_client(AudioRecord, '/microphone/record')
    # ...

    # Step 2: Transcribe with Google Cloud Speech-to-Text
    # TODO: Send audio_data to Google Cloud STT
    # from google.cloud import speech
    # client = speech.SpeechClient()
    # ...
    transcript = ""

    # Step 3: Extract target object + action via LLM (Gemini)
    # TODO: Send transcript to Gemini with a prompt like:
    # "Extract the target object and action from this command: '{transcript}'"
    # Expected output: {"target_object": "apple", "action": "fetch"}
    target_object = ""
    action = ""

    node.get_logger().info(f'Voice command: target="{target_object}", action="{action}"')
    return target_object, action


class VoiceCommandNode(Node):
    """Standalone node that listens for a voice command and publishes the result."""

    def __init__(self):
        super().__init__('voice_command')

        # Latched publishers so late subscribers still get the message
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(String, '/target_object', latched_qos)
        self.command_pub = self.create_publisher(String, '/user_command', latched_qos)

        self.get_logger().info('Voice Command Node — waiting for audio...')

        # Run voice command after a short delay (let connections establish)
        self.create_timer(1.0, self.run_once)
        self._done = False

    def run_once(self):
        """Record and process voice command once."""
        if self._done:
            return
        self._done = True

        target_object, action = get_voice_command(self)

        if target_object:
            msg = String()
            msg.data = target_object
            self.target_pub.publish(msg)
            self.get_logger().info(f'Published target_object: {target_object}')

        if action:
            msg = String()
            msg.data = action
            self.command_pub.publish(msg)
            self.get_logger().info(f'Published user_command: {action}')


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
