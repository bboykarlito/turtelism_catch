from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    
    spawner_node = Node(
        package="turtelism_catch_them_all",
        executable="turtle_spawner",
        parameters=[
            {"spawn_frequency": 0.5}
        ]
    )
    
    controller_node = Node(
        package="turtelism_catch_them_all",
        executable="turtlesim_controller"
    )
    
    ld.add_action(turtlesim_node)
    ld.add_action(spawner_node)
    ld.add_action(controller_node)
    return ld