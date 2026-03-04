#!/usr/bin/env python3
"""
Voice Command Node

Wraps Frances's speech_to_text.py (Google Cloud STT + Gemini) into a ROS2
node that publishes the extracted target object and action.

Publishes (latched / transient_local QoS):
    /target_object (std_msgs/String) - the object to find (e.g. "banana")
    /user_command  (std_msgs/String) - the action verb   (e.g. "get")

Environment:
    GOOGLE_APPLICATION_CREDENTIALS must be set for Google Cloud STT
    GOOGLE_API_KEY (or genai configured) for Gemini

Usage:
    ros2 run tidybot_bringup voice_command.py
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from std_msgs.msg import String

# Import Frances's speech functions
from speech_to_text import record_audio, transcribe_audio

import google.generativeai as genai


def extract_command(transcript: str) -> tuple:
    """Use Gemini to extract (action, object) from a transcript.

    Returns:
        (action, target_object) e.g. ("get", "banana")
    """
    if not transcript.strip():
        return ('', '')

    model = genai.GenerativeModel("gemini-2.0-flash")
    prompt = (
        'Extract the action verb and the target object from this voice command. '
        'Return ONLY a JSON object with keys "action" and "object", nothing else. '
        'Example: {"action": "get", "object": "banana"}\n\n'
        f'Voice command: "{transcript}"'
    )
    response = model.generate_content(prompt)
    text = response.text.strip()

    # Strip markdown code fences if present
    if text.startswith('```'):
        text = text.split('\n', 1)[-1].rsplit('```', 1)[0].strip()

    try:
        data = json.loads(text)
        action = data.get('action', '').strip().lower()
        obj = data.get('object', '').strip().lower()
        return (action, obj)
    except (json.JSONDecodeError, AttributeError):
        return ('', '')


class VoiceCommandNode(Node):
    """Records audio, transcribes, extracts command, and publishes."""

    def __init__(self):
        super().__init__('voice_command')

        # Latched publishers so late subscribers still get the message
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(String, '/target_object', latched_qos)
        self.command_pub = self.create_publisher(String, '/user_command', latched_qos)

        self.get_logger().info('Voice Command Node ready.')
        self.get_logger().info('Press Enter in the terminal to record a command.')

        # Start listening loop after a short delay
        self.create_timer(1.0, self._prompt_and_record)
        self._recording = False

    def _prompt_and_record(self):
        """Wait for user to press Enter, then record + process."""
        if self._recording:
            return
        self._recording = True

        try:
            input('\n>>> Press Enter to record a voice command (5s)... ')
        except EOFError:
            # Non-interactive terminal — just record immediately
            pass

        wav_file = '/tmp/voice_command.wav'
        self.get_logger().info('Recording 5 seconds of audio...')
        record_audio(wav_file, duration=5, sample_rate=16000)

        self.get_logger().info('Transcribing...')
        transcript = transcribe_audio(wav_file)
        self.get_logger().info(f'Transcript: "{transcript}"')

        if not transcript.strip():
            self.get_logger().warn('Empty transcript — try again.')
            self._recording = False
            return

        self.get_logger().info('Extracting command via Gemini...')
        action, target_object = extract_command(transcript)
        self.get_logger().info(
            f'Extracted: action="{action}", object="{target_object}"')

        if target_object:
            msg = String()
            msg.data = target_object
            self.target_pub.publish(msg)
            self.get_logger().info(f'Published /target_object: {target_object}')

        if action:
            msg = String()
            msg.data = action
            self.command_pub.publish(msg)
            self.get_logger().info(f'Published /user_command: {action}')

        if not target_object:
            self.get_logger().warn(
                'Could not extract target object — try again.')

        self._recording = False


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
