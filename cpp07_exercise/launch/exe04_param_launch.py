from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # turtlesim node
    t = Node(package="turtlesim", executable="turtlesim_node")
    # param node
    param = Node(package="cpp07_exercise", executable="exe04_param")
    
    ld = LaunchDescription([t, param])

    return ld