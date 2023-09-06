from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # param
    escort_back = DeclareLaunchArgument(name="escort_back", default_value="escort_back")
    # turtle
    turtle = Node(package="turtlesim", executable="turtlesim_node")

    # spawn
    spawn = Node(package="cpp11_tf_exercise", executable="exe01_spawn", name="spawan_back", 
                 parameters=[{"x":  2.0}, {"y": 2.5},
                     {"turtle_name": LaunchConfiguration("escort_back")}
                 ]
    )

    # broadcast two tf
    broadcaster1_world = Node(package="cpp11_tf_exercise", executable="exe02_tf_broadcaster", name="turtle1_world")
    broadcaster2_world = Node(package="cpp11_tf_exercise", executable="exe02_tf_broadcaster", name="turtle2_world", parameters=[{"turtle": LaunchConfiguration("escort_back")}])
    back_goal = Node(package="tf2_ros", executable="static_transform_publisher", name="back_turtle1", arguments=["-2", "0", "0", "0", "0", "0", "turtle1", "back_goal"])

    # escort
    escort = Node(package="cpp11_tf_exercise", executable="exe03_tf_listener", parameters=[{"father_frame": LaunchConfiguration("escort_back")}, {"child_frame": "back_goal"}])

    # 按理应该加入 OnProcessExit 先spawn、broad再lookuptrans
    ld = LaunchDescription([escort_back, turtle, spawn, broadcaster1_world, broadcaster2_world, back_goal, escort])

    return ld