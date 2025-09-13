from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    turtlesim = Node(
        package='turtlesim',
        executable='turtlesim_node',
    )
    my_node = Node(
        package='turtle_pkg',
        executable='turtle_chase',
    )
    
    ld.add_action(turtlesim)
    ld.add_action(my_node)
    return ld