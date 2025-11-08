import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # happy_params ではなく、ウェイポイントファイルがあるパッケージを指定
    # (もし happy_params にあるならそのままでOK)
    params_pkg_dir = get_package_share_directory('happy_params') # ★ウェイポイントファイルがあるパッケージ
    map_file_env = os.getenv("HAPPY_MAP", "arc25")
    waypoint_file = os.path.join(params_pkg_dir, 'location', f'{map_file_env}.yaml') # ★.yanlなら修正

    declare_target_location_arg = DeclareLaunchArgument(
        'target_location',
        default_value='default_location', 
        description='Name of the location to navigate to'
    )

    return LaunchDescription([
        declare_target_location_arg,
        
        Node(
            # ★★★ ここからが最重要 ★★★
            package='happy_mobility',         # 'happy_params' から変更
            executable='navigation',          # 'location_navigator' から変更
            name='location_navigator_node',   # この名前は自由
            output='screen',
            # waypoint_fileのパスを渡すパラメータ
            parameters=[{'waypoint_file': waypoint_file}], 
            # ロケーション名をコマンドライン引数として渡す
            arguments=[LaunchConfiguration('target_location')] 
        )
    ])