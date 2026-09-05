import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    hik_camera_launch = os.path.join(
        get_package_share_directory('hik_camera'),
        'launch',
        'hik_camera.launch.py'
    )

    # web_tuner 入口脚本
    web_tuner_script = os.path.join(
        get_package_share_directory('yq_dart_aim'),
        'web_tuner',
        'server.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='true',
            description='Set true to start foxglove bridge'
        ),
        DeclareLaunchArgument(
            'web_tuner',
            default_value='true',
            description='Set true to start web tuning dashboard on port 8080'
        ),
        DeclareLaunchArgument(
            'web_tuner_port',
            default_value='8080',
            description='Web tuner HTTP port'
        ),
        DeclareLaunchArgument(
            'crop_width',
            default_value='1280',
            description='Center crop width in pixels'
        ),
        DeclareLaunchArgument(
            'crop_height',
            default_value='720',
            description='Center crop height in pixels'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hik_camera_launch)
        ),

        Node(
            package='yq_dart_aim',
            executable='dart_aim_node',
            name='dart_aim_node',
            parameters=[
                '/root/dart_ws_on/src/yq_dart_aim/config/params.yaml',
                {
                    'crop_width': ParameterValue(LaunchConfiguration('crop_width'), value_type=int),
                    'crop_height': ParameterValue(LaunchConfiguration('crop_height'), value_type=int),
                }
            ],
            output='screen'
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('debug'))
        ),

        # Web 调参系统（可选，web_tuner:=true 启动）
        ExecuteProcess(
            cmd=['python3', web_tuner_script,
                 '--port', LaunchConfiguration('web_tuner_port')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('web_tuner'))
        ),
    ])
