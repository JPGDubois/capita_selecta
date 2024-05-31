import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('navigate'),
        'config',
        'params.yaml'
        )
        
    nav_pose_server=Node(
        package = 'navigate',
        name = 'nav_pose_server',
        executable = 'nav_pose_server',
        parameters = [config],
        remappings=[
            ('/input/odom', '/ground_truth/odom'),
            ('/output/cmd_vel', '/diff_cont/cmd_vel')
        ]
    )

    nav_pose_client=Node(
        package = 'navigate',
        name = 'nav_pose_client',
        executable = 'nav_pose_client',
        # remappings=[
        #     ('/input/goal_pose', '/move_base_simple/goal')
        # ]
    )

    
    return LaunchDescription([
        nav_pose_server,
        #nav_pose_client
    ])