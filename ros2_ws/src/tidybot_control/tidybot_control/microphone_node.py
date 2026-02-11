#!/usr/bin/env python3
"""
Microphone Node for TidyBot2.

Provides a service-based audio recording interface.
A remote client can start/stop recording and receive the audio data in the response.

Service:
    /microphone/record (tidybot_msgs/srv/AudioRecord)
        - start=true:  begin recording, responds immediately
        - start=false: stop recording, responds with audio data

Parameters:
    - sample_rate (int, default 16000): Audio sample rate in Hz
    - channels (int, default 1): Number of audio channels (1=mono)
    - device_index (int, default -1): ALSA device index (-1=auto-detect)

Usage:
    ros2 run tidybot_control microphone_node

    # Start recording:
    ros2 service call /microphone/record tidybot_msgs/srv/AudioRecord "{start: true}"

    # Stop and get audio:
    ros2 service call /microphone/record tidybot_msgs/srv/AudioRecord "{start: false}"
"""

import threading
import numpy as np

import rclpy
from rclpy.node import Node
from tidybot_msgs.srv import AudioRecord

try:
    import sounddevice as sd
    HAS_SOUNDDEVICE = True
except ImportError:
    HAS_SOUNDDEVICE = False


class MicrophoneNode(Node):
    """ROS2 node for microphone recording via service interface."""

    def __init__(self):
        super().__init__('microphone')

        if not HAS_SOUNDDEVICE:
            self.get_logger().error(
                'sounddevice not installed! Install with: pip install sounddevice'
            )
            raise RuntimeError('sounddevice not available')

        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('device_index', -1)

        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        device_index = self.get_parameter('device_index').value
        self.device_index = None if device_index < 0 else device_index

        # Recording state
        self.is_recording = False
        self.audio_buffer = []
        self.lock = threading.Lock()
        self.stream = None

        # Service
        self.srv = self.create_service(
            AudioRecord, '/microphone/record', self._record_callback
        )

        self.get_logger().info('Microphone node ready')
        self.get_logger().info(f'  Sample rate: {self.sample_rate} Hz')
        self.get_logger().info(f'  Channels: {self.channels}')
        self.get_logger().info(f'  Device: {self.device_index or "auto"}')

    def _audio_input_callback(self, indata, frames, time_info, status):
        """Called by sounddevice for each audio block."""
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        with self.lock:
            if self.is_recording:
                self.audio_buffer.append(indata[:, 0].copy())

    def _start_recording(self):
        """Start audio recording."""
        with self.lock:
            self.audio_buffer = []
            self.is_recording = True

        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            device=self.device_index,
            dtype='float32',
            callback=self._audio_input_callback,
        )
        self.stream.start()
        self.get_logger().info('Recording started')

    def _stop_recording(self):
        """Stop recording and return audio data."""
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
            self.stream = None

        with self.lock:
            self.is_recording = False
            if self.audio_buffer:
                audio = np.concatenate(self.audio_buffer)
            else:
                audio = np.array([], dtype=np.float32)
            self.audio_buffer = []

        duration = len(audio) / self.sample_rate if len(audio) > 0 else 0.0
        self.get_logger().info(f'Recording stopped: {duration:.2f}s, {len(audio)} samples')
        return audio, duration

    def _record_callback(self, request, response):
        """Handle AudioRecord service calls."""
        if request.start:
            # Start recording
            if self.is_recording:
                response.success = False
                response.message = 'Already recording'
            else:
                try:
                    self._start_recording()
                    response.success = True
                    response.message = 'Recording started'
                except Exception as e:
                    response.success = False
                    response.message = f'Failed to start recording: {str(e)}'
                    self.get_logger().error(response.message)

            response.audio_data = []
            response.sample_rate = self.sample_rate
            response.duration = 0.0
        else:
            # Stop recording and return audio
            if not self.is_recording:
                response.success = False
                response.message = 'Not recording'
                response.audio_data = []
                response.sample_rate = self.sample_rate
                response.duration = 0.0
            else:
                try:
                    audio, duration = self._stop_recording()
                    response.success = True
                    response.message = f'Recorded {duration:.2f}s'
                    response.audio_data = audio.tolist()
                    response.sample_rate = self.sample_rate
                    response.duration = duration
                except Exception as e:
                    response.success = False
                    response.message = f'Failed to stop recording: {str(e)}'
                    response.audio_data = []
                    response.sample_rate = self.sample_rate
                    response.duration = 0.0
                    self.get_logger().error(response.message)

        return response

    def destroy_node(self):
        """Clean up audio stream on shutdown."""
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicrophoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
