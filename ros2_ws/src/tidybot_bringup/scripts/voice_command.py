#!/usr/bin/env python3
"""
Voice Command Node

Wraps Frances's speech_to_text.py (Google Cloud STT + Gemini) into a ROS2
node that publishes the extracted target object and action. Now supports
multi-step task extraction so one sentence can enqueue several tasks.

Publishes (latched / transient_local QoS):
    /target_object        (std_msgs/String) - object for the current step
    /user_command         (std_msgs/String) - action verb for the current step
    /target_location      (std_msgs/String) - optional destination/place
    /multi_step_commands  (std_msgs/String) - JSON list of all parsed steps

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


def extract_commands(transcript: str) -> list:
    """Use Gemini to extract a list of commands (action/object/location)."""
    if not transcript.strip():
        return []

    model = genai.GenerativeModel("gemini-2.0-flash")
    prompt = (
        "You extract sequential robot commands from a single voice instruction.\n"
        "Return ONLY valid JSON of the form:\n"
        '{\"commands\": [\n'
        '  {\"action\": \"get\", \"object\": \"banana\"},\n'
        '  {\"action\": \"place\", \"object\": \"banana\", \"location\": \"bowl\"}\n'
        "]}\n"
        "Rules:\n"
        "  - Split the transcript into ordered imperative verbs.\n"
        "  - action/object/location values must be lowercase strings.\n"
        "  - Always include an \"object\" key; leave it empty if unknown.\n"
        "  - Include \"location\" only when a destination/place is stated.\n"
        "  - Resolve pronouns like \"it\" to the last mentioned object.\n"
        "Voice command:\n"
        f"\"{transcript}\""
    )
    response = model.generate_content(prompt)
    text = (response.text or "").strip()

    # Strip markdown code fences if present
    if text.startswith('```'):
        text = text.split('\n', 1)[-1].rsplit('```', 1)[0].strip()

    try:
        data = json.loads(text)
        if isinstance(data, dict) and isinstance(data.get('commands'), list):
            raw_cmds = data['commands']
        elif isinstance(data, list):
            raw_cmds = data
        else:
            return []
    except json.JSONDecodeError:
        return []

    commands = []
    last_object = ''
    for raw in raw_cmds:
        if not isinstance(raw, dict):
            continue
        action = str(raw.get('action', '')).strip().lower()
        obj = str(raw.get('object', '')).strip().lower()
        location = str(raw.get('location', '')).strip().lower()

        if not action:
            continue
        if not obj:
            obj = last_object

        if obj:
            last_object = obj

        cmd = {'action': action}
        if obj:
            cmd['object'] = obj
        else:
            cmd['object'] = ''
        if location:
            cmd['location'] = location

        commands.append(cmd)

    return commands


class VoiceCommandNode(Node):
    """Records audio, transcribes, extracts command, and publishes."""

    def __init__(self):
        super().__init__('voice_command')

        # Latched publishers so late subscribers still get the message
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.target_pub = self.create_publisher(String, '/target_object', latched_qos)
        self.command_pub = self.create_publisher(String, '/user_command', latched_qos)
        self.location_pub = self.create_publisher(String, '/target_location', latched_qos)
        self.multi_command_pub = self.create_publisher(String, '/multi_step_commands', latched_qos)

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

        self.get_logger().info('Extracting command(s) via Gemini...')
        commands = extract_commands(transcript)

        if not commands:
            self.get_logger().warn('Could not extract any commands — try again.')
            self._recording = False
            return

        self.get_logger().info(f'Extracted {len(commands)} command(s).')

        # Publish the full sequence as JSON for downstream planners that can parse it.
        sequence_msg = String()
        sequence_msg.data = json.dumps({'commands': commands})
        self.multi_command_pub.publish(sequence_msg)
        self.get_logger().info(f'Published /multi_step_commands: {sequence_msg.data}')

        for idx, command in enumerate(commands, start=1):
            action = command.get('action', '').strip().lower()
            target_object = command.get('object', '').strip().lower()
            location = command.get('location', '').strip().lower()

            self.get_logger().info(
                f'Step {idx}: action="{action}" object="{target_object}"'
                + (f' location="{location}"' if location else '')
            )

            if action:
                msg = String()
                msg.data = action
                self.command_pub.publish(msg)
                self.get_logger().info(f'Published /user_command: {action}')

            if target_object:
                msg = String()
                msg.data = target_object
                self.target_pub.publish(msg)
                self.get_logger().info(f'Published /target_object: {target_object}')
            else:
                self.get_logger().warn(
                    f'Step {idx}: Missing target object — downstream node may skip it.')

            if location:
                msg = String()
                msg.data = location
                self.location_pub.publish(msg)
                self.get_logger().info(f'Published /target_location: {location}')

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
