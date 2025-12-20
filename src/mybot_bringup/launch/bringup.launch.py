from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('mybot_bringup')

    xacro_path = os.path.join(pkg, 'urdf', 'mybot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_path).toxml()

    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controllers_yaml],
        output='screen'
    )

    # Spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    diff_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml]
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        jsb_spawner,
        diff_spawner,
        ekf_node,
    ])
