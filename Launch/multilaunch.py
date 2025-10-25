from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multinodal_lawnbot',
            # namespace='turtlesim1',
            executable='imgPub',
            # name='sim'
        ),
        Node(
            package='multinodal_lawnbot',
            # namespace='turtlesim2',
            executable='graySub',
            # name='sim'
        ),
        Node(
            package='multinodal_lawnbot',
            executable='cannyEdge',
            # name='mimic',
            # remappings=[
            #     ('/input/pose', '/turtlesim1/turtle1/pose'),
            #     ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            # ]
        )
    ])