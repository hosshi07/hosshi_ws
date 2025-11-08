import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ウェイポイントファイルがある 'happy_params' パッケージのパスを取得
    params_pkg_dir = get_package_share_directory('happy_params')
    map_file_env = os.getenv("HAPPY_MAP", "arc25")
    waypoint_file = os.path.join(params_pkg_dir, 'location', f'{map_file_env}.yaml')

    return LaunchDescription([
        Node(
            package='happy_mobility', # smach_planner.py があるパッケージ
            executable='smach_planner', # setup.py で登録した名前
            name='smach_planner_node',
            output='screen',
            # waypoint_fileのパスをパラメータとして渡す
            parameters=[{'waypoint_file': waypoint_file}] 
        )
    ])