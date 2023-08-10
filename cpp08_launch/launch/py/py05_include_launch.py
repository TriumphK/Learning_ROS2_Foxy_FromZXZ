import os
from launch import LaunchDescription
from launch_ros.actions import Node
# 文件包含相关
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# 获取功能包下share目录路径
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    include = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(get_package_share_directory("cpp08_launch"), "launch/py", "py04_args_launch.py")
        ),
        launch_arguments=[
            ("background_r", "80"),
            ("background_g", "80"),
            ("background_b", "80")
        ]
    )

    ld = LaunchDescription([include])

    return ld