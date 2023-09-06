from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # param
    t2 = DeclareLaunchArgument(name="t2_name", default_value="t2")
    # turtle
    turtle = Node(package="turtlesim", executable="turtlesim_node")

    # spawn
    spawn = Node(package="cpp11_tf_exercise", executable="exe01_spawn", parameters=[{"turtle_name": LaunchConfiguration("t2_name")}])

    # broadcast two tf
    broadcaster1 = Node(package="cpp11_tf_exercise", executable="exe02_tf_broadcaster", name="broad1")
    broadcaster2 = Node(package="cpp11_tf_exercise", executable="exe02_tf_broadcaster", name="broad2", parameters=[{"turtle": LaunchConfiguration("t2_name")}])

    # follow
    follow = Node(package="cpp11_tf_exercise", executable="exe03_tf_listener", parameters=[{"father_frame": LaunchConfiguration("t2_name")}, {"child_frame": "turtle1"}])

    # 按理应该加入 OnProcessExit 先spawn、broad再lookuptrans
    ld = LaunchDescription([t2, turtle, spawn, broadcaster1, broadcaster2, follow])

    return ld