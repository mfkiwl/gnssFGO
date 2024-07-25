import os
import launch.launch_description_sources
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    logger = LaunchConfiguration("log_level")
    share_dir = get_package_share_directory('online_fgo')

    config_common_path = LaunchConfiguration('config_common_path')

    default_config_dataset = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/learning_gp',
        'dataset.yaml'
    )

    default_config_common = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/learning_gp',
        'common.yaml'
    )

    default_config_integrator = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/learning_gp',
        'integrator.yaml'
    )

    default_config_optimizer = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/learning_gp',
        'optimizer.yaml'
    )

    declare_config_common_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_common,
        description='CommonParameters')

    declare_config_integrtor_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_integrator,
        description='IntegratorParameters')

    declare_config_optimizer_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_optimizer,
        description='OptimizerParameters')

    declare_config_dataset_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_dataset,
        description='DatasetParameters')

    default_config_sensor_parameters = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/learning_gp',
        'sensor_parameters.yaml'
    )
    declare_config_sensor_parameters_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_sensor_parameters,
        description='SensorParameters')

    online_fgo_node = Node(
        package='online_fgo',
        executable='learning_gp_node',
        name="learning_gp",
        namespace="offline_process",
        output='screen',
        emulate_tty=True,
        prefix=['xterm -sl 10000 -hold -e '],
        # arguments=['--ros-args', '--log-level', logger],
        parameters=[
            config_common_path,
            default_config_common,
            default_config_integrator,
            default_config_optimizer,
            default_config_dataset,
            default_config_sensor_parameters,
            {

            }
            # Overriding
            # {
            # }
        ],
        remappings=[
        ]
    )

    ### MAPVIZ
    mapviz_dir = get_package_share_directory("mapviz")
    mapviz_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(mapviz_dir + '/launch/mapviz.boreas.launch.py'))

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value=["debug"],
        description="Logging level"))
    ld.add_action(declare_config_common_path_cmd)
    ld.add_action(declare_config_integrtor_path_cmd)
    ld.add_action(declare_config_optimizer_path_cmd)
    ld.add_action(declare_config_dataset_path_cmd)
    ld.add_action(declare_config_sensor_parameters_path_cmd)
    ld.add_action(online_fgo_node)
    ld.add_action(mapviz_launch)

    return ld
