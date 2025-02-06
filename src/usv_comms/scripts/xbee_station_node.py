#!/usr/bin/env python3

from xbee_base_node import XBeeBaseNode
import rclpy
import struct

class XBeeStationNode(XBeeBaseNode):
    publishers = {}

    def __init__(self):
        super().__init__('xbee_station_node')
        self.setup_publishers()
        self.msg_handlers = self.setup_message_handlers()

    def setup_xbee(self):
        """Setup XBee device for station"""
        self.device = self._init_xbee_device(
            self.config['xbee_config']['port']['station'],
            self.config['xbee_config']['baud_rate']
        )

        xbee_network = self.device.get_network()
        xbee_network.start_discovery_process()

        self.get_logger().info("Searching for remote device...")
        while xbee_network.is_discovery_running():
            pass

        remote_device = xbee_network.discover_device(
            self.config['xbee_config']['remote_ids']['boat']
        )

        if remote_device is None:
            self.get_logger().error("Could not find remote device")
            raise Exception("Remote device not found")

        self.get_logger().info(f"Found remote device: {remote_device.get_node_id()}")
        self.device.add_data_received_callback(self.data_received_callback)

    def setup_publishers(self):
        """Create publishers based on enabled topics in config"""
        self.publishers = {}
        enabled_topics = self.get_enabled_topics()

        # First ensure topic_data is initialized
        for topic in enabled_topics:
            self.topic_data[topic['name']] = None

            # Then create publishers
            for topic in enabled_topics:
                topic_name = topic['config']['topic']
                msg_type = self.get_message_type(topic['config']['msg_type'])

                # Create publisher with modified topic name
                pub_topic = f"/usv_comms{topic_name}"
                self.publishers[topic['name']] = self.create_publisher(
                    msg_type,
                    pub_topic,
                    10
                )

        self.get_logger().info(f"Initialized publishers for topics: {list(self.topic_data.keys())}")
        if not self.topic_data:
            self.get_logger().error("No topics were initialized! Check your config file.")

    def setup_message_handlers(self):
        """Create message handlers for each topic type"""
        handlers = {}
        for topic in self.get_enabled_topics():
            if topic['name'] == 'objects':
                handlers[topic['name']] = self.handle_object_list
            else:
                handlers[topic['name']] = self.handle_standard_message
        return handlers

    def handle_object_list(self, data: list) -> None:
        """Handle object list messages"""
        msg = self.get_message_type('usv_interfaces/msg/ObjectList')()
        max_objects = self.config['topics']['objects']['max_objects']

        for i in range(0, len(data), 3):
            if i/3 >= max_objects:
                break

            obj = self.get_message_type('usv_interfaces/msg/Object')()
            obj.x = data[i]
            obj.y = data[i + 1]
            obj.color = int(data[i + 2])
            obj.type = "XBEE STD"
            msg.obj_list.append(obj)

        self.publishers['objects'].publish(msg)

    def handle_standard_message(self, topic_name: str, data: list) -> None:
        """Handle standard messages (non-object list)"""
        msg_type = self.get_message_type(self.config['topics'][topic_name]['msg_type'])
        msg = msg_type()

        for field, value in zip(self.config['topics'][topic_name]['fields'], data):
            setattr(msg, field, value)

        self.publishers[topic_name].publish(msg)

    def data_received_callback(self, xbee_message):
        """Handle received XBee messages"""
        try:
            # Parse message chunk
            msg_id, data = self.parse_message_chunk(xbee_message.data)

            # Get topic name safely
            topic_names = list(self.topic_data.keys())
            if msg_id >= len(topic_names):
                raise ValueError(f"Invalid message ID: {msg_id}. Max ID: {len(topic_names)-1}")
            topic_name = topic_names[msg_id]

            # Get handler
            handler = self.msg_handlers.get(topic_name)
            if handler is None:
                raise ValueError(f"No handler found for topic: {topic_name}")

            # Handle message
            self.get_logger().debug(f"Received message for topic {topic_name}: {data}")
            if topic_name == 'objects':
                handler(data)
            else:
                handler(topic_name, data)

        except (struct.error, ValueError) as e:
            self.get_logger().error(f"Error processing message: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {str(e)}")
            self.get_logger().error(f"Message data: {xbee_message.data}")

def main(args=None):
    rclpy.init(args=args)
    node = XBeeStationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
