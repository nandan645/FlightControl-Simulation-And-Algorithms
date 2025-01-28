
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Tello driver node
        Node(
            package='tello',
            executable='tello',
            output='screen',
            namespace='/',
            name='tello',
            parameters=[
                {'connect_timeout': 10.0},
                {'tello_ip': '192.168.10.1'},
                {'tf_base': 'map'},
                {'tf_drone': 'drone'}
            ],
            remappings=[
                ('/image_raw', '/camera')
            ],
            respawn=True
        ),

        #Tello control node
        # Node(
        #     package='tello_control',
        #     executable='tello_control',
        #     namespace='/',
        #     name='control',
        #     output='screen',
        #     respawn=False
        # ),

        # RQT topic debug tool
        # Node(
        #     package='rqt_gui',
        #     executable='rqt_gui',
        #     output='screen',
        #     namespace='/',
        #     name='rqt',
        #     respawn=False
        # ),

        # Static TF publisher
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     namespace='/',
        #     name='tf',
        #     arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'drone'],
        #     respawn=True
        # ),

    ]


    return LaunchDescription(nodes)