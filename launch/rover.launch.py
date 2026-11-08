from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    poslogger = Node(
        package='fp_rival_dor',
        executable='poslogger',
        name='poslogger',
        output='screen'
    )

    driving = Node(
        package='fp_rival_dor',
        executable='driving_node',
        name='driving_node',
        output='screen'
    )

    navigation = Node(
        package='fp_rival_dor',
        executable='navigation_node',
        name='navigation_node',
        output='screen'
    )

    waypoint_controller = Node(
        package='fp_rival_dor',
        executable='waypoint_controller',
        name='waypoint_controller',
        output='screen'
    )

    return LaunchDescription([
        poslogger,
        TimerAction(period=2.0, actions=[driving]),
        TimerAction(period=4.0, actions=[navigation]),
        TimerAction(period=6.0, actions=[waypoint_controller])
    ])