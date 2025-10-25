from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            # namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        ExecuteProcess(
            cmd=['gnome-terminal','--','bash','-ic','ros2 run turtlesim turtle_teleop_key; exec bash'], output='screen'
        )
        # name='sim'
        # ),
        # Node(
        #     package='turtlesim',
        #     executable='turtle_teleop_key',
        #     name='teleop'
        #     # remappings=[
        #     #     ('/input/pose', '/turtlesim1/turtle1/pose'),
        #     #     ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        #     # ]
        # )
    ])