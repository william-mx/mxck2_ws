import os
import yaml
import subprocess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Load config paths
    pkg_dir = get_package_share_directory('vehicle_control')
    vesc_config = os.path.join(pkg_dir, 'config', 'vesc_config.yaml')
    control_config = os.path.join(pkg_dir, 'config', 'control_config.yaml')

    # Load control_type from config
    with open(control_config, 'r') as f:
        config = yaml.safe_load(f)
    control_type = config.get('/**', {}).get('ros__parameters', {}).get('control_type', 'rc')

    # Define all nodes
    vesc = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[vesc_config],
        respawn=True,
        respawn_delay=20.0
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '-b', '921600', '--dev', '/dev/stm32_nucleo'],
        respawn=True,
        respawn_delay=20.0
    )

    rc2joy = Node(
        package='vehicle_control',
        executable='rc_to_joy',
        name='rc_to_joy',
        parameters=[control_config]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.05,
            'autorepeat_rate': 50.0
        }]
    )

    joy2ackermann = Node(
        package='vehicle_control',
        executable='joy_to_ackermann',
        name='joy_to_ackermann',
        parameters=[control_config]
    )

    ackermann2vesc = Node(
        package='vehicle_control',
        executable='ackermann_to_vesc',
        name='ackermann_to_vesc',
        parameters=[control_config],
        output='screen'
    )

    # Query running nodes
    result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)
    running_nodes = [n.lstrip('/') for n in result.stdout.strip().split('\n')] if result.stdout else []

    # Always add common nodes if not running
    for node in [vesc, joy2ackermann, ackermann2vesc]:
        if node.name not in running_nodes:
            ld.add_action(node)

    # Conditionally add input nodes
    if control_type == 'rc':
        if micro_ros_agent.name not in running_nodes:
            ld.add_action(micro_ros_agent)
        if rc2joy.name not in running_nodes:
            ld.add_action(rc2joy)
    else:
        if joy_node.name not in running_nodes:
            ld.add_action(joy_node)

    return ld
