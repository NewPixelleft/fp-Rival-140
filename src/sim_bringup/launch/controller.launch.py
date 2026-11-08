import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    workspace = Path(
        get_package_prefix('sim_bringup')
    ).parents[1]

    python = sys.executable

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sim_bringup'),
                'launch',
                'dummy.launch.py'
            ])
        )
    )

    def python_node(name, script):
        return ExecuteProcess(
            cmd=[
                'gnome-terminal',
                '--tab',
                f'--title={name}',
                '--',
                'bash',
                '-c',
                f'cd "{workspace}" && '
                f'exec "{python}" -u '
                f'"{workspace / "pyscripts" / script}"'
            ],
            name=name,
            output='screen'
        )

    # ROS nodes
    poslogger = python_node(
        'PosLogger',
        'poslogger.py'
    )

    driving = python_node(
        'DrivingNode',
        'navigation/driving_node.py'
    )

    navigation = python_node(
        'NavigationNode',
        'navigation/navigation_node.py'
    )

    aruco_detector = python_node(
        'ArucoDetector',
        'arucodetector.py'
    )

    waypoint_controller = python_node(
        'WaypointController',
        'waypointController.py'
    )

    return LaunchDescription([

        # Start Gazebo first
        gazebo,

        # Start localization
        TimerAction(
            period=3.0,
            actions=[poslogger]
        ),

        # Start driving controller
        TimerAction(
            period=5.0,
            actions=[driving]
        ),

        # Start navigation
        TimerAction(
            period=7.0,
            actions=[navigation]
        ),

        # Start ArUco detector
        TimerAction(
            period=9.0,
            actions=[aruco_detector]
        ),

        # Start interactive waypoint controller last
        TimerAction(
            period=11.0,
            actions=[waypoint_controller]
        )

    ])
