from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='beeblebrox',
            executable='beeblebrox',
            name='beeblebrox',
            output='screen',
            parameters=[
                {'object_topic': '/bebblebrox/objects'},
                {'video_topic': '/bebblebrox/video'},
                {'yolo_sub_topic': '/yolo/detections'},
                {'shapes_sub_topic': '/shapes/detections'},
                {'frame_interval': 50}, # run every 50 ms
            ]
        ),

        Node(
            package='beeblebrox',
            executable='shapes',
            name='shapes',
            output='screen',
            parameters=[]
        ),

        Node(
            package='beeblebrox',
            executable='yolo',
            name='yolo',
            output='screen',
            parameters=[
                {'engine_path': '/home/vanttec/vanttec_usv/vtec_agx_v2.engine'},
                {'video_topic': '/bebblebrox/video'},
                {'output_topic': '/yolo/detections'},
                {'threshold': 0.8},
            ]
        ),

    ])

