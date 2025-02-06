#!/usr/bin/env python3

from xbee_base_node import XBeeBaseNode
import rclpy
import struct

class XBeeBoatNode(XBeeBaseNode):
    subscribers = {}

    def __init__(self):
        super().__init__('xbee_boat_node')
        self.setup_subscribers()
        self.setup_timer()
        
    def setup_xbee(self):
        """Setup XBee device for boat"""
        self.device = self._init_xbee_device(
            self.config['xbee_config']['port']['boat'],
            self.config['xbee_config']['baud_rate']
        )
        
        xbee_network = self.device.get_network()
        xbee_network.start_discovery_process()
        
        self.get_logger().info("Searching for remote device...")
        while xbee_network.is_discovery_running():
            pass
            
        self.remote_device = xbee_network.discover_device(
            self.config['xbee_config']['remote_ids']['station']
        )
        
        if self.remote_device is None:
            self.get_logger().error("Could not find remote device")
            raise Exception("Remote device not found")
            
        self.get_logger().info(f"Found remote device: {self.remote_device.get_node_id()}")
        
    def setup_subscribers(self):
        """Create subscribers based on enabled topics in config"""
        self.subscribers = {}
        
        for topic in self.get_enabled_topics():
            topic_name = topic['config']['topic']
            msg_type = self.get_message_type(topic['config']['msg_type'])
            
            self.subscribers[topic['name']] = self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, t=topic['name']: self.topic_callback(msg, t),
                10
            )
            self.topic_data[topic['name']] = None
            
    def setup_timer(self):
        """Setup timer for periodic transmissions"""
        timer_period = self.config['xbee_config']['update_rate']
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def process_object_list(self, msg) -> list:
        """Process object list message into transmittable data"""
        obj_data = []
        max_objects = self.config['topics']['objects']['max_objects']
        for obj in msg.obj_list[:max_objects]:
            obj_data.extend([obj.x, obj.y, float(obj.color)])
        return obj_data
        
    def process_standard_message(self, msg, topic_name: str) -> list:
        """Process standard message into transmittable data"""
        fields = self.config['topics'][topic_name]['fields']
        return [float(getattr(msg, field)) for field in fields]
            
    def topic_callback(self, msg, topic_name: str):
        """Generic callback for all topics"""
        try:
            if topic_name == 'objects':
                self.topic_data[topic_name] = self.process_object_list(msg)
            else:
                self.topic_data[topic_name] = self.process_standard_message(msg, topic_name)
        except Exception as e:
            self.get_logger().error(f"Error processing message for {topic_name}: {e}")
            
    def timer_callback(self):
        """Periodic transmission of data"""
        for topic_name, data in self.topic_data.items():
            if data is not None:
                try:
                    msg_id = list(self.topic_data.keys()).index(topic_name)
                    chunk = self.create_message_chunk(msg_id, data)
                    self.device.send_data(self.remote_device, chunk)
                except Exception as e:
                    self.get_logger().error(f"Failed to send data for {topic_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = XBeeBoatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
