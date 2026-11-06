"""HW3 full run: perception (8 tags) + motor_control + hw3_slam.

ros2 launch robot_control hw3_launch.py [square_laps:=3] [octagon_laps:=3] [dry_run:=true]
    [square_size:=1.5] [reset_between_phases:=true] [phase_pause_s:=30]
    [floor:=study_wood2_hw2_ext] [tag_z_sign:=1]
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PASSED = {      # launch argument -> default, passed to hw3_slam as a parameter
    'square_size': '1.5',
    'square_laps': '3',
    'octagon_laps': '3',
    'reset_between_phases': 'true',
    'phase_pause_s': '30.0',
    'dry_run': 'false',
    'floor': 'study_wood2_hw2_ext',
    'tag_z_sign': '1',
}


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in PASSED.items()]
    share = FindPackageShare('robot_control')
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [share, 'launch', 'hw2_perception_launch.py'])),
        launch_arguments={'apriltag_config': PathJoinSubstitution(
            [share, 'config', 'hw3', 'apriltag.yaml'])}.items())
    motor_node = Node(
        package='robot_control', executable='motor_control', name='motor_controller_node',
        output='screen', emulate_tty=True)
    slam_node = Node(
        package='robot_control', executable='hw3_slam', name='hw3_slam',
        output='screen', emulate_tty=True,
        parameters=[{k: LaunchConfiguration(k) for k in PASSED}])
    # Bring everything down when the run ends or a node dies.
    shutdown_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[Shutdown(reason=f'{name} exited')]))
        for node, name in ((slam_node, 'hw3_slam'), (motor_node, 'motor_control'))
    ]
    return LaunchDescription([*args, perception, motor_node, slam_node, *shutdown_handlers])
