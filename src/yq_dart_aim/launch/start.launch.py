import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    # 用安装后的 share 路径，不再写死 /root/dart_ws_on（RDK X5 与 x86 共用）
    params_file = os.path.join(
        get_package_share_directory('yq_dart_aim'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='true',
            description='Set true to start foxglove bridge'
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
        DeclareLaunchArgument(
            'infer_backend',
            default_value='auto',
            description='推理后端: auto / CPU / BPU / OpenVINO / DNN（RDK X5 用 BPU，x86 用 auto 或 OpenVINO）'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hik_camera_launch),
            # 图像话题用 sensor data QoS（best_effort）：1280x720 每帧 2.76MB，
            # reliable 传输会被接收端 ACK 流控拖死（实测发布 31fps 只送出 2fps）。
            # dart_aim 的订阅端用的就是 SensorDataQoS，两者兼容。
            launch_arguments={'use_sensor_data_qos': 'true'}.items()
        ),

        Node(
            package='yq_dart_aim',
            executable='dart_aim_node',
            name='dart_aim_node',
            parameters=[
                params_file,
                {
                    'infer_backend': LaunchConfiguration('infer_backend'),
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

        # 相机节点已直接发布 /image_raw/jpeg（约 100KB/帧），
        # 不再需要 image_transport republish 把 2.76MB 的 raw 转 JPEG。
    ])
