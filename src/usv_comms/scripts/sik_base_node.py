#!/usr/bin/env python3

import os
import yaml
import struct
import serial
from typing import Dict, List, Any
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64, Int8
from usv_interfaces.msg import Waypoint, ObjectList, Object

class SiKBaseNode(Node):
    MESSAGE_TYPES = {
        'geometry_msgs/msg/Pose2D': Pose2D,
        'geometry_msgs/msg/Vector3': Vector3,
        'std_msgs/msg/Float64': Float64,
        'std_msgs/msg/Int8': Int8,
        'usv_interfaces/msg/Waypoint': Waypoint,
        'usv_interfaces/msg/ObjectList': ObjectList,
        'usv_interfaces/msg/Object': Object
    }

    def __init__(self, node_name: str, node_type: str):
        """Initialize base node"""
        super().__init__(node_name)
        
        # Initialize instance variables
        self.node_type = node_type
        self.config = self.load_config()
        self.get_logger().info("Loading configuration...")
        
        # Determine topics based on direction:
        # For sending topics (outgoing data) we want those where 'direction' starts with "<node_type>_to_"
        # For receiving topics (incoming data) we want those where 'direction' ends with "_to_<node_type>"
        self.sending_topics = self.get_topics_by_direction(f"{node_type}_to_", use_startswith=True)
        self.receiving_topics = self.get_topics_by_direction(f"_to_{node_type}", use_startswith=False)
        
        self.get_logger().info(f"Sending topics: {[t['name'] for t in self.sending_topics]}")
        self.get_logger().info(f"Receiving topics: {[t['name'] for t in self.receiving_topics]}")
        
        # Initialize topic data storage for sending topics
        self.topic_data = {topic['name']: None for topic in self.sending_topics}
        
        # Initialize radio (each derived node must implement setup_radio)
        self.setup_radio()

    def load_config(self) -> Dict:
        """Load configuration from YAML file"""
        config_path = os.path.join(
            get_package_share_directory('usv_comms'),
            'config',
            'topics_config.yaml'
        )
            
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found at {config_path}")
            raise FileNotFoundError(f"Config file not found at {config_path}")
            
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            self.get_logger().info(f"Loaded config from {config_path}")
            return config

    def get_topics_by_direction(self, direction_pattern: str, use_startswith: bool) -> List[Dict]:
        """
        Get topics based on the 'direction' field.
        If use_startswith is True, returns topics whose 'direction' starts with direction_pattern.
        Otherwise, returns topics whose 'direction' ends with direction_pattern.
        """
        topics = []
        
        def process_topic_config(name, config):
            if isinstance(config, dict):
                if 'direction' in config and config.get('enabled', False):
                    direction = config['direction']
                    if use_startswith:
                        if direction.startswith(direction_pattern):
                            topics.append({
                                'name': name,
                                'config': config
                            })
                    else:
                        if direction.endswith(direction_pattern):
                            topics.append({
                                'name': name,
                                'config': config
                            })
        
        for topic_name, config in self.config['topics'].items():
            if isinstance(config, dict):
                if 'direction' in config:
                    process_topic_config(topic_name, config)
                else:
                    for subtopic_name, subtopic_config in config.items():
                        if isinstance(subtopic_config, dict) and 'direction' in subtopic_config:
                            process_topic_config(f"{topic_name}.{subtopic_name}", subtopic_config)
        
        return topics

    def _init_radio(self, port: str, baud_rate: int) -> serial.Serial:
        """Initialize SiK radio with error checking"""
        try:
            radio = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=1,
                write_timeout=1
            )
            self.get_logger().info(f"Radio opened on {port}")
            return radio
        except serial.SerialException as e:
            self.get_logger().error(f"Port {port} is not accessible: {e}")
            self.get_logger().error("Try: sudo chmod 666 " + port)
            raise
        except Exception as e:
            self.get_logger().error(f"Error initializing radio: {str(e)}")
            raise

    def setup_radio(self):
        """Abstract method to be implemented by child classes"""
        raise NotImplementedError

    def get_message_type(self, type_str: str):
        """Convert string message type to actual message type class"""
        return self.MESSAGE_TYPES[type_str]

    def pack_data(self, data: List[Any]) -> bytes:
        """Pack data into bytes"""
        try:
            result = bytearray()
            type_markers = []
            
            for item in data:
                if isinstance(item, (int, float)):
                    type_markers.append(0)
                    result.extend(struct.pack('!f', float(item)))
                elif isinstance(item, str):
                    type_markers.append(1)
                    encoded = item.encode('utf-8')
                    result.extend(struct.pack('!H', len(encoded)) + encoded)
            
            header = struct.pack('!H', len(type_markers))
            header += struct.pack(f'!{len(type_markers)}B', *type_markers)
            
            return header + result
            
        except Exception as e:
            self.get_logger().error(f"Error in pack_data: {str(e)}")
            raise

    def unpack_data(self, data: bytes) -> List[Any]:
        """Unpack data from bytes"""
        try:
            if len(data) < 2:
                raise ValueError(f"Data too short: {len(data)} bytes")
                
            num_items = struct.unpack('!H', data[:2])[0]
            if len(data) < 2 + num_items:
                raise ValueError(f"Data too short for markers: {len(data)} bytes")
                
            markers = struct.unpack(f'!{num_items}B', data[2:2+num_items])
            result = []
            pos = 2 + num_items
            
            for marker in markers:
                if marker == 0:  # Float
                    if pos + 4 > len(data):
                        raise ValueError(f"Truncated float at {pos}")
                    value = struct.unpack('!f', data[pos:pos+4])[0]
                    result.append(value)
                    pos += 4
                elif marker == 1:  # String
                    if pos + 2 > len(data):
                        raise ValueError(f"Truncated string length at {pos}")
                    str_len = struct.unpack('!H', data[pos:pos+2])[0]
                    pos += 2
                    if pos + str_len > len(data):
                        raise ValueError(f"Truncated string at {pos}")
                    string = data[pos:pos+str_len].decode('utf-8')
                    result.append(string)
                    pos += str_len
                else:
                    raise ValueError(f"Invalid marker: {marker}")
                    
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error unpacking data: {e}")
            raise

    def create_message_chunk(self, msg_id: int, data: List[Any]) -> bytes:
        """Create a message chunk for transmission"""
        try:
            data_bytes = self.pack_data(data)
            packet_length = len(data_bytes)
            header = struct.pack('!BH', msg_id, packet_length)
            return b'\xFE' + header + data_bytes + b'\xFD'
        except Exception as e:
            self.get_logger().error(f"Error creating message chunk: {e}")
            raise

    def parse_message_chunk(self, chunk: bytes) -> tuple:
        """Parse a message chunk into msg_id and data"""
        try:
            if len(chunk) < 5:
                raise ValueError("Chunk too short")
            if chunk[0] != 0xFE or chunk[-1] != 0xFD:
                raise ValueError("Invalid markers")
            msg_id = chunk[1]
            payload_length = struct.unpack('!H', chunk[2:4])[0]
            data_chunk = chunk[4:-1]
            if len(data_chunk) != payload_length:
                raise ValueError(f"Length mismatch: {len(data_chunk)} != {payload_length}")
            return msg_id, self.unpack_data(data_chunk)
        except Exception as e:
            self.get_logger().error(f"Error parsing chunk: {e}")
            raise

    def _validate_packet(self, packet: bytes) -> bool:
        """Validate packet structure"""
        try:
            if len(packet) < 5:
                return False
            if packet[0] != 0xFE or packet[-1] != 0xFD:
                return False
            payload_length = struct.unpack('!H', packet[2:4])[0]
            return len(packet) - 5 == payload_length
        except Exception:
            return False
