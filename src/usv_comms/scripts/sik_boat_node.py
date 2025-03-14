#!/usr/bin/env python3

from sik_base_node import SiKBaseNode
import rclpy
import struct
import threading
import time

class SiKBoatNode(SiKBaseNode):
    def __init__(self):
        # For the boat, we pass node_type "boat"
        super().__init__('sik_boat_node', 'boat')
        
        # Separate dictionaries for our publishers (for topics we receive) and subscribers (for topics we send)
        self.topic_publishers = {}
        self.topic_subscribers = {}
        
        # Initialize communication interfaces
        self.setup_publishers()
        self.setup_subscribers()
        self.setup_timer()
        
        # Initialize read thread for incoming radio data
        self._running = True
        self._read_thread = threading.Thread(target=self._read_serial)
        self._read_thread.start()
        
        # Initialize statistics
        self.packet_stats = {
            'total_received': 0,
            'valid_packets': 0,
            'invalid_packets': 0,
            'bytes_received': 0,
            'last_packet_time': None,
            'packet_rates': {}
        }
        
        # Diagnostic timer (publishes every 5 seconds)
        self.diagnostic_timer = self.create_timer(5.0, self._publish_diagnostics)
        
    def setup_radio(self):
        """Setup SiK radio for boat"""
        self.radio = self._init_radio(
            self.config['radio_config']['port']['boat'],
            self.config['radio_config']['baud_rate']
        )
        
    def setup_publishers(self):
        """Create publishers for topics we receive (from station) with the /usv_comms prefix"""
        for topic in self.receiving_topics:
            topic_name = topic['config']['topic']
            msg_type = self.get_message_type(topic['config']['msg_type'])
            # Add the /usv_comms prefix to the published topic name
            pub_topic = f"{topic_name}"
            self.topic_publishers[topic['name']] = self.create_publisher(
                msg_type,
                pub_topic,
                10
            )
            
    def setup_subscribers(self):
        """Create subscribers for topics we send (from boat sensors)"""
        for topic in self.sending_topics:
            topic_name = topic['config']['topic']
            msg_type = self.get_message_type(topic['config']['msg_type'])
            self.topic_subscribers[topic['name']] = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, t=topic['name']: self.topic_callback(msg, t),
                10
            )
            
    def setup_timer(self):
        """Setup timer for periodic transmissions"""
        timer_period = self.config['radio_config']['update_rate']
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def _publish_diagnostics(self):
        """Publish communication statistics"""
        now = self.get_clock().now()
        self.get_logger().info("\nBoat Communication Statistics:")
        self.get_logger().info(f"Total packets received: {self.packet_stats['total_received']}")
        self.get_logger().info(f"Valid packets: {self.packet_stats['valid_packets']}")
        self.get_logger().info(f"Invalid packets: {self.packet_stats['invalid_packets']}")
        if self.packet_stats['last_packet_time']:
            time_since_last = (now - self.packet_stats['last_packet_time']).nanoseconds / 1e9
            self.get_logger().info(f"Time since last packet: {time_since_last:.2f} seconds")

    def _update_packet_stats(self, topic_name: str, is_valid: bool):
        """Update packet statistics"""
        now = self.get_clock().now()
        self.packet_stats['total_received'] += 1
        if is_valid:
            self.packet_stats['valid_packets'] += 1
        else:
            self.packet_stats['invalid_packets'] += 1
        self.packet_stats['last_packet_time'] = now

    def topic_callback(self, msg, topic_name: str):
        """Callback for topics we send: process and store data for transmission"""
        try:
            if topic_name == 'objects':
                self.topic_data[topic_name] = self.process_object_list(msg)
            else:
                self.topic_data[topic_name] = self.process_standard_message(msg, topic_name)
        except Exception as e:
            self.get_logger().error(f"Error in topic callback: {str(e)}")

    def process_object_list(self, msg) -> list:
        """Process object list message into transmittable data"""
        obj_data = []
        max_objects = self.config['topics']['objects']['max_objects']
        for obj in msg.obj_list[:max_objects]:
            obj_data.extend([obj.x, obj.y, float(obj.color), obj.type])
        return obj_data
        
    def process_standard_message(self, msg, topic_name: str) -> list:
        """Process standard message into transmittable data (for outgoing data)"""
        try:
            config = self.config['topics']
            for key in topic_name.split('.'):
                config = config[key]
            fields = config['fields']
            return [float(getattr(msg, field)) for field in fields]
        except Exception as e:
            self.get_logger().error(f"Error processing message: {str(e)}")
            raise

    def handle_standard_message(self, topic_name: str, data: list) -> None:
        """
        Process an incoming standard message (received via radio) and publish it.
        This method looks up the corresponding topic configuration from the receiving topics.
        """
        try:
            config = None
            for topic in self.receiving_topics:
                if topic['name'] == topic_name:
                    config = topic['config']
                    break
            if config is None:
                raise ValueError(f"Configuration not found for topic '{topic_name}'")
            fields = config['fields']
            if len(data) != len(fields):
                raise ValueError(f"Field count mismatch. Expected {len(fields)}, got {len(data)}")
            msg_type = self.get_message_type(config['msg_type'])
            msg = msg_type()
            for field, value in zip(fields, data):
                setattr(msg, field, value)
            self.topic_publishers[topic_name].publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error handling standard message: {e}")
            raise

    def _read_serial(self):
        """Read thread for incoming messages"""
        buffer = bytearray()
        MAX_BUFFER_SIZE = 1024
        
        while self._running:
            if self.radio.in_waiting:
                chunk = self.radio.read(min(64, self.radio.in_waiting))
                buffer.extend(chunk)
                
                while len(buffer) > 4:  # Minimum packet size
                    start_idx = buffer.find(b'\xFE')
                    if start_idx == -1:
                        buffer.clear()
                        break
                        
                    if start_idx > 0:
                        buffer = buffer[start_idx:]
                        
                    end_idx = buffer.find(b'\xFD', 1)
                    if end_idx == -1:
                        if len(buffer) > MAX_BUFFER_SIZE:
                            buffer.clear()
                        break
                        
                    packet = buffer[:end_idx + 1]
                    buffer = buffer[end_idx + 1:]
                    
                    if not self._validate_packet(packet):
                        self._update_packet_stats("unknown", False)
                        continue
                        
                    try:
                        msg_id, data = self.parse_message_chunk(packet)
                        topic_names = [t['name'] for t in self.receiving_topics]
                        
                        if msg_id >= len(topic_names):
                            raise ValueError(f"Invalid message ID: {msg_id}")
                            
                        topic_name = topic_names[msg_id]
                        
                        if topic_name == 'objects':
                            self.handle_object_list(data)
                        else:
                            self.handle_standard_message(topic_name, data)
                            
                        self._update_packet_stats(topic_name, True)
                    except Exception as e:
                        self.get_logger().error(f"Error processing packet: {e}")
                        self._update_packet_stats("unknown", False)
                        
            time.sleep(0.001)

    def timer_callback(self):
        """Periodic transmission of outgoing messages"""
        for topic_name, data in self.topic_data.items():
            if data is not None:
                try:
                    topic_names = [t['name'] for t in self.sending_topics]
                    msg_id = topic_names.index(topic_name)
                    chunk = self.create_message_chunk(msg_id, data)
                    self.radio.write(chunk)
                    self.radio.flush()
                except Exception as e:
                    self.get_logger().error(f"Failed to send data for {topic_name}: {e}")

    def destroy_node(self):
        """Cleanup when shutting down"""
        self._running = False
        if self._read_thread.is_alive():
            self._read_thread.join()
        if hasattr(self, 'radio') and self.radio.is_open:
            self.radio.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SiKBoatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
