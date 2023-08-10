from launch import LaunchDescription
from launch_ros.actions import Node
# 参数声明与获取
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bg_r = DeclareLaunchArgument(name="background_r", default_value="255")
    bg_g = DeclareLaunchArgument(name="background_g", default_value="255")
    bg_b = DeclareLaunchArgument(name="background_b", default_value="255")

    t1 = Node(
        package="turtlesim",
        executable="turtlesim_node",
        parameters=[{"background_r": LaunchConfiguration("background_r"),
                     "background_g": LaunchConfiguration("background_g"),
                     "background_b": LaunchConfiguration("background_b")}]
    )


    ld = LaunchDescription([bg_r, bg_g, bg_b, t1])

    return ld