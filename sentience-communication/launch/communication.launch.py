from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Start the sensor_interface node
    communication_node = Node(
        package='sentience-communication',
        executable='embed_communication_node',
        name='embed_communication_node',
        output='screen'
    )

    ld.add_action(communication_node)
    return ld