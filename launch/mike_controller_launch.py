import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(get_package_share_directory('mike_controller'), 'config', 'config.yaml')

    node1 = Node(package='turtlesim',
                       executable='turtlesim_node',
                       name="turtlesim",
                       )
    
    node2 = Node(package='mike_controller',
                          executable='odometry',
                            name="odometry",
                          )
        
    node3 = Node(package='mike_controller',
                       executable='path_generator',
                       name="path_generator",
                       )
    
    node4 = Node(package='mike_controller',
                       executable='odometry_turtle',
                       name="odometry_turtle",
                       )
    
    node5 = Node(package='mike_controller',
                       executable='main_controller',
                       name="main_controller",
                       parameters=[config]
                       )
    

    
    
    
    l_d = LaunchDescription([node1, node2, node3, node4, node5])

    return l_d