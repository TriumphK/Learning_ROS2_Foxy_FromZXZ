from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # turtlesim node
    t1 = Node(package="turtlesim", executable="turtlesim_node")
    # server node
    server = Node(package="cpp07_exercise", executable="exe02_server")

    ld = LaunchDescription([t1, server])

    return ld