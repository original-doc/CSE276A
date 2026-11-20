"""HW4 run: perception (12 tags) + motor_control + hw4_nav.

ros2 launch robot_control hw4_launch.py planner:=safe|fast [dry_run:=true]
    [floor:=study_wood2_hw2_ext] [tag_z_sign:=1] [wp_pause_s:=1.0]
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

PASSED = {      # launch argument -> default, passed to hw4_nav as a parameter
    'planner': '',
    'dry_run': 'false',
    'floor': 'study_wood2_hw2_ext',
    'tag_z_sign': '1',
    'wp_pause_s': '1.0',
}


def generate_launch_description():
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in PASSED.items()]
    share = FindPackageShare('robot_control')
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [share, 'launch', 'hw2_perception_launch.py'])),
        launch_arguments={'apriltag_config': PathJoinSubstitution(
            [share, 'config', 'hw4', 'apriltag.yaml'])}.items())
    motor_node = Node(
        package='robot_control', executable='motor_control', name='motor_controller_node',
        output='screen', emulate_tty=True)
    nav_node = Node(
        package='robot_control', executable='hw4_nav', name='hw4_nav',
        output='screen', emulate_tty=True,
        parameters=[{k: LaunchConfiguration(k) for k in PASSED}])
    # Bring everything down when the run ends or a node dies.
    shutdown_handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=node, on_exit=[Shutdown(reason=f'{name} exited')]))
        for node, name in ((nav_node, 'hw4_nav'), (motor_node, 'motor_control'))
    ]
    return LaunchDescription([*args, perception, motor_node, nav_node, *shutdown_handlers])
