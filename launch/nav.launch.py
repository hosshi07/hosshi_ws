import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    my_package_path = get_package_share_directory('happy_params')

    # 使用するマップファイルのパスを指定
    # nav.launch.py 側
    map_file_env = os.getenv("HAPPY_MAP", "arc25")
    map_file_path = os.path.join(my_package_path, 'maps', f"{map_file_env}.yaml")

    # Nav2のパラメータファイルのパスを指定
    nav2_params_path = os.path.join(my_package_path, 'param', 'nav2_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    # Nav2が出力する/cmd_velを、ロボットが聞いている/rover_twistに中継するノード
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='cmd_vel_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['/cmd_vel', '/rover_twist']
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=map_file_path),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_path,
                'autostart': 'true'
            }.items()
        ),
        cmd_vel_relay
    ])