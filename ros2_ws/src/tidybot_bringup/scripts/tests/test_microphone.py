#!/usr/bin/env python3
"""
Test script for the microphone recording service.

Records audio for a specified duration and saves it to a WAV file.

Prerequisites:
    - Microphone node running:
        ros2 launch tidybot_bringup real.launch.py
      or standalone:
        ros2 run tidybot_control microphone_node

Usage:
    python3 test_microphone.py                  # Record 3 seconds (default)
    python3 test_microphone.py --duration 5     # Record 5 seconds
    python3 test_microphone.py -o my_clip.wav   # Custom output filename
"""

import argparse
import os
import time
import wave
import numpy as np

import rclpy
from rclpy.node import Node
from tidybot_msgs.srv import AudioRecord


class MicrophoneTest(Node):
    def __init__(self):
        super().__init__('test_microphone')
        self.client = self.create_client(AudioRecord, '/microphone/record')
        self.get_logger().info('Waiting for /microphone/record service...')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available! Is the microphone node running?')
            raise RuntimeError('Service not available')
        self.get_logger().info('Service connected.')

    def call_service(self, start: bool) -> AudioRecord.Response:
        req = AudioRecord.Request()
        req.start = start
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if future.result() is None:
            raise RuntimeError('Service call failed')
        return future.result()

    def record(self, duration: float) -> AudioRecord.Response:
        # Start
        resp = self.call_service(start=True)
        if not resp.success:
            self.get_logger().error(f'Start failed: {resp.message}')
            raise RuntimeError(resp.message)
        self.get_logger().info(f'Recording for {duration:.1f} seconds...')

        time.sleep(duration)

        # Stop
        resp = self.call_service(start=False)
        if not resp.success:
            self.get_logger().error(f'Stop failed: {resp.message}')
            raise RuntimeError(resp.message)
        self.get_logger().info(
            f'Got {len(resp.audio_data)} samples, '
            f'{resp.duration:.2f}s @ {resp.sample_rate} Hz'
        )
        return resp


def save_wav(filename: str, audio_data: list, sample_rate: int):
    """Save float32 audio data to a 16-bit WAV file."""
    audio = np.array(audio_data, dtype=np.float32)
    # Clamp and convert to int16
    audio = np.clip(audio, -1.0, 1.0)
    int16_data = (audio * 32767).astype(np.int16)

    with wave.open(filename, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)  # 16-bit
        wf.setframerate(sample_rate)
        wf.writeframes(int16_data.tobytes())


def main():
    parser = argparse.ArgumentParser(description='Test microphone recording')
    parser.add_argument('--duration', '-d', type=float, default=3.0,
                        help='Recording duration in seconds (default: 3)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output WAV path (default: recordings/recording.wav)')
    args = parser.parse_args()

    # Default to recordings/ directory (gitignored)
    if args.output is None:
        rec_dir = os.path.join(os.getcwd(), 'recordings')
        os.makedirs(rec_dir, exist_ok=True)
        args.output = os.path.join(rec_dir, 'recording.wav')

    rclpy.init()
    node = MicrophoneTest()

    try:
        resp = node.record(args.duration)
        if resp.audio_data:
            save_wav(args.output, resp.audio_data, resp.sample_rate)
            print(f'\nSaved to {args.output}')
            print(f'  Duration: {resp.duration:.2f}s')
            print(f'  Sample rate: {resp.sample_rate} Hz')
            print(f'  Samples: {len(resp.audio_data)}')
            print(f'\nPlay with:  aplay {args.output}')
        else:
            print('No audio data received.')
    except Exception as e:
        print(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
