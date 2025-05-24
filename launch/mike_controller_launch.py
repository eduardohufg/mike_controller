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
    
    node6 = Node(package='mike_controller',
                       executable='encoder',
                       name="encoder",
                       )
    
    node7 = Node(package='mike_controller',
                        executable='odometry_calculated_turtle',
                        name="odometry_calculated_turtle",
                        )
    
    node8 = Node(package='mike_controller',
                        executable='trayectory',
                        name="trayectory",
                        )
    
    node9 = Node(package='mike_controller',
                        executable='error_metrics',
                        name="error_metrics",
                        parameters=[config]
                        )
    
    node10 = Node(package='mike_controller',
                        executable='itae',
                        name="itae",
                        parameters=[config]
                        )
    

    
    
    
    l_d = LaunchDescription([node1,node2, node6, node7, node8, node5, node10])

    return l_d