from launch import LaunchDescription
from launch_ros.actions import Node
# 终端指令的相关类
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    t1 = Node(package="turtlesim", executable="turtlesim_node")

    #cmd
    cmd_test = ExecuteProcess(
        # cmd=["ros2 topic echo /turtle1/pose"],
        # cmd=["ros2 topic", "echo", "/turtle1/pose"],
        cmd=[FindExecutable(name="ros2"), "topic", "echo", "/turtle1/pose"],
        output="both",
        shell=True
    )

    ld = LaunchDescription([t1, cmd_test])

    return ld