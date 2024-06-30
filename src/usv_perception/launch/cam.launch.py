from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package= 'usv_perception',
            executable='beeblebrox',
            name='beeblebrox',
            output='screen',
            parameters=[
                {'objects_yolo_topic': '/objects'},
                {'video_topic': '/bebblebrox/video'},
                {'yolo_sub_topic': '/yolo/detections'},
                # {'shapes_sub_topic': '/shapes/detections'},
                {'frame_interval': 100}, # run every 50 ms
            ]
        ),
    ])

