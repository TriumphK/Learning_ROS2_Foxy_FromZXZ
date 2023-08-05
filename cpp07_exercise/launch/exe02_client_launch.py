from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # inital
    x = 6
    y = 9
    theta = 0.0
    name = "t2"
    
    # spawn a new turtle
    # ros2 service call /spawn turtlesim/srv/Spawn "{'x': 3, 'y': 4, 'theta': 1.57, 'name': 't3'}"
    spawn = ExecuteProcess(
        cmd=["ros2 service call /spawn turtlesim/srv/Spawn \"{'x': " + str(x) 
             + ", 'y': " + str(y) 
             + ", 'theta': " + str(theta) 
             + ", 'name': '" + name + "'}\""],
        output="both",
        shell=True
    )
    # client node
    client = Node(package="cpp07_exercise",
                  executable="exe02_client",
                  arguments=[str(x), str(y), str(theta)]
                  # ros2 run cpp07_exercise exe02_client 6 9 0.0 --ros-args
    )
    
    ld = LaunchDescription([spawn, client])

    return ld