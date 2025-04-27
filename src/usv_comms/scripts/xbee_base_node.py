#!/usr/bin/env python3

import os
import yaml
import struct
from typing import Dict, List, Any
from digi.xbee.devices import XBeeDevice, RemoteXBeeDevice
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose2D, Vector3
from std_msgs.msg import Float64, Int8
from usv_interfaces.msg import Waypoint, ObjectList, Object

class XBeeBaseNode(Node):
    MESSAGE_TYPES = {
        'geometry_msgs/msg/Pose2D': Pose2D,
        'geometry_msgs/msg/Vector3': Vector3,
        'std_msgs/msg/Float64': Float64,
        'std_msgs/msg/Int8': Int8,
        'usv_interfaces/msg/Waypoint': Waypoint,
        'usv_interfaces/msg/ObjectList': ObjectList,
        'usv_interfaces/msg/Object': Object
    }

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.config = self.load_config()
        self.get_logger().info("Loading configuration...")
        enabled_topics = self.get_enabled_topics()
        if not enabled_topics:
            self.get_logger().error("No enabled topics found in config!")
            self.get_logger().error("Config content: " + str(self.config))
        else:
            self.get_logger().info(f"Found enabled topics: {[t['name'] for t in enabled_topics]}")
        # Initialize topic_data with enabled topics
        self.topic_data = {topic['name']: None for topic in enabled_topics}
        self.setup_xbee()
        
    def load_config(self) -> Dict:
        """Load configuration from YAML file"""
        # First try package share directory
        config_path = os.path.join(
        get_package_share_directory('usv_comms'),
        'config',
        'topics_xbee_config.yaml'
        )
            
        if not os.path.exists(config_path):
            self.get_logger().error(f"Config file not found at {config_path}")
            raise FileNotFoundError(f"Config file not found at {config_path}")
            
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            self.get_logger().info(f"Loaded config from {config_path}")
            return config
            
    def _init_xbee_device(self, port: str, baud_rate: int) -> XBeeDevice:
        """Initialize XBee device with error checking"""
        import serial
        from digi.xbee.exception import XBeeException
        
        # First check if port is accessible
        try:
            ser = serial.Serial(port)
            ser.close()
        except serial.SerialException as e:
            self.get_logger().error(f"Port {port} is not accessible: {e}")
            self.get_logger().error("Try: sudo chmod 666 " + port)
            raise
            
        # Now try to initialize XBee
        try:
            device = XBeeDevice(port, baud_rate)
            device.open()
            self.get_logger().info(f"XBee device opened on {port}")
            
            if device.is_open():
                self.get_logger().info(f"Device info: {device.get_node_id()}")
                self.get_logger().info(f"Device address: {device.get_64bit_addr()}")
                return device
            else:
                raise Exception("Device could not be opened")
        except Exception as e:
            self.get_logger().error(f"Error initializing XBee: {str(e)}")
            self.get_logger().error(f"Please check if {port} is correct and accessible")
            raise
        except Exception as e:
            self.get_logger().error(f"Error initializing XBee: {str(e)}")
            self.get_logger().error(f"Please check if {port} is correct and accessible")
            raise
            
    def setup_xbee(self):
        """Abstract method to be implemented by child classes"""
        raise NotImplementedError
        
    def pack_string(self, s: str) -> bytes:
        """Pack a string into bytes, prefixed with its length"""
        encoded = s.encode('utf-8')
        return struct.pack('!H', len(encoded)) + encoded
        
    def unpack_string(self, data: bytes) -> tuple[str, bytes]:
        """Unpack a string from bytes, return string and remaining bytes"""
        str_len = struct.unpack('!H', data[:2])[0]
        string = data[2:2+str_len].decode('utf-8')
        return string, data[2+str_len:]
        
    def pack_data(self, data: List[Any]) -> bytes:
        """Pack data into bytes, handling both numbers and strings"""
        result = b''
        type_markers = []  # 0 for float, 1 for string
        
        for item in data:
            if isinstance(item, (int, float)):
                type_markers.append(0)
                result += struct.pack('!f', float(item))
            elif isinstance(item, str):
                type_markers.append(1)
                result += self.pack_string(item)
                
        # Prepend type markers
        markers_bytes = struct.pack(f'!{len(type_markers)}B', *type_markers)
        return struct.pack('!H', len(type_markers)) + markers_bytes + result
        
    def unpack_data(self, data: bytes) -> List[Any]:
        """Unpack mixed data (numbers and strings) from bytes"""
        # Read number of items
        num_items = struct.unpack('!H', data[:2])[0]
        
        # Read type markers
        markers = struct.unpack(f'!{num_items}B', data[2:2+num_items])
        
        # Process data according to markers
        result = []
        current_pos = 2 + num_items
        
        for marker in markers:
            if marker == 0:  # Float
                result.append(struct.unpack('!f', data[current_pos:current_pos+4])[0])
                current_pos += 4
            elif marker == 1:  # String
                string, remaining = self.unpack_string(data[current_pos:])
                result.append(string)
                current_pos += 2 + len(string.encode('utf-8'))
                
        return result
        
    def get_message_type(self, type_str: str):
        """Convert string message type to actual message type class"""
        return self.MESSAGE_TYPES[type_str]
        
    def get_enabled_topics(self) -> List[Dict]:
        """Get list of enabled topics from config"""
        enabled_topics = []
        for topic_group, config in self.config['topics'].items():
            if isinstance(config, dict):
                if config.get('enabled', False):
                    enabled_topics.append({
                        'name': topic_group,
                        'config': config
                    })
        return enabled_topics
        
    def create_message_chunk(self, msg_id: int, data: List[Any]) -> bytes:
        """Create a message chunk for transmission"""
        data_bytes = self.pack_data(data)
        # Header format: msg_id (1 byte), total_message_size (2 bytes)
        header = struct.pack('!BH', msg_id, len(data_bytes))
        return header + data_bytes

    def parse_message_chunk(self, chunk: bytes) -> tuple:
        """Parse a message chunk into msg_id and data"""
        try:
            # Parse header
            msg_id, msg_size = struct.unpack('!BH', chunk[:3])
            
            # Parse data
            data = self.unpack_data(chunk[3:])
            return msg_id, data
        except struct.error as e:
            self.get_logger().error(f"Failed to parse message header: {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"Error parsing message: {e}")
            raise
