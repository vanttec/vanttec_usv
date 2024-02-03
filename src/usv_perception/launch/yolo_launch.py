from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='usv_perception',
            executable='yolo',
            name='yolov8_node',
            output='screen',
            parameters=[
                {'engine_path': 'param_value1'},
                {'classes_path': 'param_value2'},
                {'input_topic': '/video'},
                {'output_topic': 'param_value2'},
            ],
        ),
        Node(
            package='usv_perception',
            executable='video.py',
        ),
    ])

