#!/usr/bin/env python3
"""
Simple Audio Capture Node for ROS2

This node captures audio from a microphone using GStreamer and publishes it to ROS2 topics.
It handles both PulseAudio and ALSA sources automatically.

Author: Auto-generated
Date: October 30, 2025
License: BSD
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from audio_common_msgs.msg import AudioData, AudioDataStamped, AudioInfo

import threading
import os


class AudioCaptureNode(Node):
    """
    ROS2 Node for capturing audio from microphone and publishing to topics.

    This node uses GStreamer to capture audio and publishes:
    - /audio: Raw audio data (AudioData)
    - /audio_stamped: Timestamped audio data (AudioDataStamped)
    - /audio_info: Audio configuration info (AudioInfo)

    Parameters:
    -----------
    device : str
        Audio device to use (e.g., 'hw:0,0', 'plughw:0,0', or empty for default)
    source_type : str
        GStreamer source element ('pulsesrc' or 'alsasrc'). Default: 'pulsesrc'
    format : str
        Audio format ('wave' or 'mp3'). Default: 'wave'
    sample_format : str
        Sample format (e.g., 'S16LE', 'S16BE'). Default: 'S16LE'
    sample_rate : int
        Sample rate in Hz. Default: 16000
    channels : int
        Number of audio channels. Default: 1 (mono)
    depth : int
        Bit depth. Default: 16
    chunk_size : int
        Size of audio chunks to publish. Default: 4096
    """

    def __init__(self):
        super().__init__('simple_audio_capture')

        # Initialize GStreamer
        Gst.init(None)

        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()

        # Log configuration
        self._log_configuration()

        # Create publishers
        self._create_publishers()

        # Initialize GStreamer pipeline
        self._init_gstreamer_pipeline()

        # Create timer for periodic audio_info publishing
        self.info_timer = self.create_timer(5.0, self._publish_audio_info)

        # Publish initial audio info
        self._publish_audio_info()

        # Start GStreamer pipeline in a separate thread
        self.gst_thread = threading.Thread(target=self._run_gstreamer, daemon=True)
        self.gst_thread.start()

        self.get_logger().info('Audio Capture Node initialized successfully')

    def _declare_parameters(self):
        """Declare all ROS2 parameters with default values."""
        self.declare_parameter('device', '')
        self.declare_parameter('source_type', 'pulsesrc')
        self.declare_parameter('format', 'wave')
        self.declare_parameter('sample_format', 'S16LE')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('depth', 16)
        self.declare_parameter('chunk_size', 4096)
        self.declare_parameter('audio_topic', 'audio')

    def _get_parameters(self):
        """Retrieve parameter values from ROS2 parameter server."""
        self.device = self.get_parameter('device').value
        # Normalize source_type (strip whitespace) and provide default if empty
        raw_source = self.get_parameter('source_type').value
        if isinstance(raw_source, str):
            self.source_type = raw_source.strip()
        else:
            self.source_type = str(raw_source)
        if not self.source_type:
            # Default to pulsesrc for most desktop systems
            self.get_logger().warning('source_type parameter empty, defaulting to "pulsesrc"')
            self.source_type = 'pulsesrc'
        self.format = self.get_parameter('format').value
        self.sample_format = self.get_parameter('sample_format').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.depth = self.get_parameter('depth').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.audio_topic = self.get_parameter('audio_topic').value

    def _log_configuration(self):
        """Log the current configuration."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Audio Capture Configuration:')
        self.get_logger().info(f'  Source Type:    {self.source_type}')
        self.get_logger().info(f'  Device:         {self.device if self.device else "default"}')
        self.get_logger().info(f'  Format:         {self.format}')
        self.get_logger().info(f'  Sample Format:  {self.sample_format}')
        self.get_logger().info(f'  Sample Rate:    {self.sample_rate} Hz')
        self.get_logger().info(f'  Channels:       {self.channels}')
        self.get_logger().info(f'  Bit Depth:      {self.depth}')
        self.get_logger().info(f'  Chunk Size:     {self.chunk_size}')
        self.get_logger().info(f'  Audio Topic:    /{self.audio_topic}')
        self.get_logger().info('=' * 60)

    def _create_publishers(self):
        """Create ROS2 publishers for audio data and info."""
        # Standard QoS for audio data
        audio_qos = QoSProfile(depth=10)

        # Transient local QoS for audio info (latched topic)
        info_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Create publishers
        self.audio_pub = self.create_publisher(AudioData, self.audio_topic, audio_qos)
        self.audio_stamped_pub = self.create_publisher(
            AudioDataStamped, f'{self.audio_topic}_stamped', audio_qos
        )
        self.audio_info_pub = self.create_publisher(AudioInfo, 'audio_info', info_qos)

        self.get_logger().info('Publishers created successfully')

    def _init_gstreamer_pipeline(self):
        """Initialize the GStreamer pipeline for audio capture."""
        try:
            # Extra diagnostics to help when elements/plugins are missing
            self._log_gst_diagnostics()

            # Create pipeline
            self.pipeline = Gst.Pipeline.new('audio_capture_pipeline')

            # Create source element
            # Normalize source_type again (safety) and attempt sensible fallbacks
            self.source_type = (self.source_type or '').strip()
            factory = Gst.ElementFactory.find(self.source_type)
            self.get_logger().debug(f'ElementFactory.find({self.source_type}) -> {factory}')

            # If configured source_type isn't available, try common fallbacks
            tried = []
            if not factory:
                tried.append(self.source_type)
                for alt in ('pulsesrc', 'alsasrc'):
                    if alt == self.source_type:
                        continue
                    falt = Gst.ElementFactory.find(alt)
                    self.get_logger().debug(f'ElementFactory.find({alt}) -> {falt}')
                    if falt:
                        self.get_logger().warning(
                            f'Configured source_type "{self.source_type}" not found; using fallback "{alt}"'
                        )
                        self.source_type = alt
                        factory = falt
                        break

            self.source = Gst.ElementFactory.make(self.source_type, 'source')
            if not self.source:
                # Provide extra diagnostics in the error
                available = self._list_audio_factories(limit=30)
                raise RuntimeError(
                    f'Failed to create element "{self.source_type}". '\
                    f'ElementFactory.find returned: {factory}. '\
                    f'Tried: {tried}. Nearby audio-related factories (sample): {available}'
                )

            # Set device if specified (only for alsasrc)
            if self.device and self.source_type == 'alsasrc':
                self.source.set_property('device', self.device)
                self.get_logger().info(f'Set ALSA device to: {self.device}')
            elif self.device and self.source_type == 'pulsesrc':
                # For PulseAudio, device is set differently
                self.source.set_property('device', self.device)
                self.get_logger().info(f'Set PulseAudio device to: {self.device}')

            # Create appsink to receive audio data
            self.sink = Gst.ElementFactory.make('appsink', 'sink')
            if not self.sink:
                raise RuntimeError('Failed to create appsink element. '\
                                   'Check that the gst-plugins-base package is installed and visible to this process.')

            # Configure appsink
            self.sink.set_property('emit-signals', True)
            self.sink.set_property('max-buffers', 100)
            self.sink.set_property('drop', False)
            self.sink.set_property('sync', False)

            # Set caps for raw audio
            caps_string = (
                f'audio/x-raw,'
                f'format={self.sample_format},'
                f'channels={self.channels},'
                f'rate={self.sample_rate},'
                f'layout=interleaved'
            )
            caps = Gst.Caps.from_string(caps_string)
            self.sink.set_property('caps', caps)

            # Connect callback for new samples
            self.sink.connect('new-sample', self._on_new_sample)

            # Add elements to pipeline
            self.pipeline.add(self.source)
            self.pipeline.add(self.sink)

            # Link elements
            if not self.source.link(self.sink):
                raise RuntimeError('Failed to link source to sink')

            # Create GLib main loop
            self.loop = GLib.MainLoop()

            # Set up bus for error handling
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect('message::error', self._on_error)
            bus.connect('message::eos', self._on_eos)
            bus.connect('message::warning', self._on_warning)

            self.get_logger().info('GStreamer pipeline initialized successfully')

        except Exception as e:
            # Log full exception plus some registry context for diagnosis
            self.get_logger().error(f'Failed to initialize GStreamer pipeline: {e}')
            try:
                reg = Gst.Registry.get()
                self.get_logger().debug(f'GStreamer registry: {reg}')
                # List a few audio-related factories to aid debugging
                audio_list = self._list_audio_factories(limit=50)
                self.get_logger().debug(f'Audio-related factories (sample): {audio_list}')
            except Exception:
                self.get_logger().debug('Could not query Gst.Registry for extra diagnostics')
            raise

    def _log_gst_diagnostics(self):
        """Log environment and registry diagnostics useful when plugins or elements are missing."""
        try:
            # Environment variables that commonly affect GStreamer/PA visibility
            env_keys = ['GST_PLUGIN_PATH', 'GI_TYPELIB_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH', 'XDG_RUNTIME_DIR', 'PULSE_SERVER']
            for k in env_keys:
                self.get_logger().debug(f'{k}={os.environ.get(k, "")!s}')

            # Current user and working dir
            self.get_logger().debug(f'USER={os.environ.get("USER")} UID={os.getuid()} CWD={os.getcwd()}')

            # Quick check if requested source element exists in this process
            try:
                found = Gst.ElementFactory.find(self.source_type)
                self.get_logger().debug(f'Gst.ElementFactory.find({self.source_type}) -> {found}')
            except Exception as e:
                self.get_logger().debug(f'Error calling Gst.ElementFactory.find: {e}')

        except Exception as e:
            # Keep diagnostics best-effort
            self.get_logger().debug(f'Error while logging Gst diagnostics: {e}')

    def _list_audio_factories(self, limit: int = 20):
        """Return a short list of audio-related element factory names (best-effort)."""
        try:
            reg = Gst.Registry.get()
            feats = reg.get_feature_list(Gst.ElementFactory)
            results = []
            for f in feats:
                try:
                    name = f.get_name()
                    klass = f.get_klass() or ''
                    if 'Audio' in klass or 'audio' in klass.lower() or 'Source' in klass or 'Sink' in klass:
                        results.append(f'{name}({klass})')
                        if len(results) >= limit:
                            break
                except Exception:
                    continue
            return results
        except Exception:
            return ['<could not enumerate factories>']

    def _run_gstreamer(self):
        """Run the GStreamer pipeline (called in separate thread)."""
        try:
            # Start playing
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error('Unable to set pipeline to PLAYING state')
                return

            self.get_logger().info('GStreamer pipeline started')

            # Run main loop
            self.loop.run()

        except Exception as e:
            self.get_logger().error(f'Error in GStreamer thread: {e}')
        finally:
            self.pipeline.set_state(Gst.State.NULL)

    def _on_new_sample(self, sink):
        """
        Callback when new audio sample is available.

        This method is called by GStreamer when a new audio buffer is ready.
        It extracts the audio data and publishes it to ROS2 topics.
        """
        try:
            # Pull sample from appsink
            sample = sink.emit('pull-sample')
            if not sample:
                return Gst.FlowReturn.OK

            # Get buffer from sample
            buffer = sample.get_buffer()
            if not buffer:
                return Gst.FlowReturn.OK

            # Extract audio data
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.OK

            # Create AudioData message
            audio_msg = AudioData()
            audio_msg.data = list(map_info.data)

            # Create AudioDataStamped message
            stamped_msg = AudioDataStamped()
            stamped_msg.header.stamp = self.get_clock().now().to_msg()
            stamped_msg.header.frame_id = 'microphone'
            stamped_msg.audio = audio_msg

            # Publish messages
            self.audio_pub.publish(audio_msg)
            self.audio_stamped_pub.publish(stamped_msg)

            # Unmap buffer
            buffer.unmap(map_info)

            return Gst.FlowReturn.OK

        except Exception as e:
            self.get_logger().error(f'Error processing audio sample: {e}')
            return Gst.FlowReturn.ERROR

    def _publish_audio_info(self):
        """Publish audio configuration information."""
        try:
            info_msg = AudioInfo()
            info_msg.channels = self.channels
            info_msg.sample_rate = self.sample_rate
            info_msg.sample_format = self.sample_format
            info_msg.bitrate = 0  # Not applicable for raw audio
            info_msg.coding_format = self.format

            self.audio_info_pub.publish(info_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing audio info: {e}')

    def _on_error(self, bus, message):
        """Handle GStreamer error messages."""
        err, debug = message.parse_error()
        self.get_logger().error(f'GStreamer Error: {err.message}')
        self.get_logger().debug(f'Debug info: {debug}')
        self.loop.quit()

    def _on_eos(self, bus, message):
        """Handle end-of-stream messages."""
        self.get_logger().info('End of stream reached')
        self.loop.quit()

    def _on_warning(self, bus, message):
        """Handle GStreamer warning messages."""
        warn, debug = message.parse_warning()
        self.get_logger().warning(f'GStreamer Warning: {warn.message}')
        self.get_logger().debug(f'Debug info: {debug}')

    def destroy_node(self):
        """Clean up resources when node is destroyed."""
        self.get_logger().info('Shutting down audio capture node...')

        # Stop GStreamer pipeline
        if hasattr(self, 'loop') and self.loop:
            self.loop.quit()

        if hasattr(self, 'pipeline') and self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)

        # Wait for GStreamer thread to finish
        if hasattr(self, 'gst_thread') and self.gst_thread.is_alive():
            self.gst_thread.join(timeout=2.0)

        super().destroy_node()


def main(args=None):
    """Main entry point for the audio capture node."""
    rclpy.init(args=args)

    try:
        node = AudioCaptureNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
