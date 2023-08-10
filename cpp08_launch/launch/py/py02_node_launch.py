import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    turtle1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        exec_name="my_label",
        respawn=True,
        remappings=[("/turtle1/cmd_vel", "cmd_vel")]
        # foxy no ros_arguments # ros_arguments=["--remap", " __ns:=/t1"]
    )

    turtle2 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="haha",
        # parameters=[{"background_r": 255, "background_g": 0, "background_b": 0}]
        parameters=[os.path.join(get_package_share_directory("cpp08_launch"), "config", "haha.yaml")]
    )

    ld1 = LaunchDescription([turtle1])
    ld2 = LaunchDescription([turtle2])

    return ld1