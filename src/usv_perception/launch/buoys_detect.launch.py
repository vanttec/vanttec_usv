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

        # Node(
        #     package='usv_perception',
        #     executable='shapes.py',
        #     name='shapes',
        #     output='screen',
        #     parameters=[]
        # ),

        Node(
            package='usv_perception',
            executable='yolo',
            name='yolo',
            output='screen',
            parameters=[
                {'engine_path': '/home/vanttec/vanttec_usv/vtec_agx_v2.engine'},
                {'video_topic': '/bebblebrox/video'},
                {'output_topic': '/yolo/detections'},
                {'threshold': 0.4},
            ],
            #arguments=['--ros-args', '--log-level', 'DEBUG']
        ),

    ])

