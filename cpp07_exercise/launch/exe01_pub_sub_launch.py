from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

""" 
    使用python形式的launch文件可以实现按序执行对应的节点!!!
    ---这应该是推荐python形式launch编写的主要原因(执行顺序)
        --通过注册事件完成 RegisterEventHandler
    ---另外xml形式的launch文件与ROS1的launch编写更为接近
"""

def generate_launch_description():
    # two turtles
    t1 = Node(package="turtlesim", executable="turtlesim_node", namespace="t1")
    t2 = Node(package="turtlesim", executable="turtlesim_node", namespace="t2")

    rotate = ExecuteProcess(
        cmd=["ros2 action send_goal /t2/turtle1/rotate_absolute turtlesim/action/RotateAbsolute \"{'theta': 3.14}\""],
        output="both",
        shell=True
    )

    pub_sub = Node(package="cpp07_exercise", executable="exe01_pub_sub")
    rotate_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit( # 触发动作
            target_action=rotate, # 目标节点
            on_exit=pub_sub # 触发执行的事件
        )
    )

    ld = LaunchDescription([t1, t2, rotate, rotate_exit_event])

    return ld