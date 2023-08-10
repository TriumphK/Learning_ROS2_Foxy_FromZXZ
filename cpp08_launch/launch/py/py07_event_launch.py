from launch import LaunchDescription
from launch_ros.actions import Node
# 终端指令的相关类
from launch.substitutions import FindExecutable
# 事件相关
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo

def generate_launch_description():
    turtle = Node(package="turtlesim", executable="turtlesim_node")

    spawn = ExecuteProcess(
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': 8.0, 'y': 2.0}\""],
        output="both",
        shell=True
    )

    event_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=turtle,
            on_start=spawn
        )
    )

    event_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turtle,
            on_exit=LogInfo(msg="--- Exit. ---")
        )
    )

    ld = LaunchDescription([turtle, event_start, event_exit])

    return ld