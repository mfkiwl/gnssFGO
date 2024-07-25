import os
import launch.launch_description_sources
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    config_common_path = LaunchConfiguration('config_common_path')
    ### GNSS PREPROCESSING
    default_config_gnss_preprocessing = os.path.join(
        get_package_share_directory('irt_gnss_preprocessing'),
        'config',
        'gnss_preprocessing.yaml'
    )

    declare_config_gnss_preprocessing_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_gnss_preprocessing,
        description='GNSS_Parameters')

    node_gnss_preprocessing = Node(
        package='irt_gnss_preprocessing',
        executable='node_gnss_preprocessing',
        namespace="irt_gnss_preprocessing",
        name="irt_gnss_preprocessing",
        output='screen',
        emulate_tty=True,
        prefix=['xterm -sl 10000 -hold -e '],
        #arguments=['--ros-args', '--log-level', logger],
        parameters=[
            config_common_path,  # Common parameter
            default_config_gnss_preprocessing
        ],
        remappings=[
            ('/user_estimation', '/deutschland/user_estimation'),
            ('/rtcmL1E1', '/sapos/rtcmL1E1')
        ]
    )

    ### FGO
    default_config_common = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/deutschland_tc',
        'common.yaml'
    )

    default_config_integrator = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/deutschland_tc',
        'integrator.yaml'
    )

    default_config_optimizer = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/deutschland_tc',
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

    default_config_sensor_parameters = os.path.join(
        get_package_share_directory('online_fgo'),
        'config/deutschland_tc',
        'sensor_parameters.yaml'
    )
    declare_config_sensor_parameters_path_cmd = DeclareLaunchArgument(
        'config_common_path',
        default_value=default_config_sensor_parameters,
        description='SensorParameters')

    node_online_fgo = Node(
        package='online_fgo',
        executable='online_fgo_node',
        name="online_fgo",
        namespace="deutschland",
        output='screen',
        emulate_tty=True,
        # prefix=['gdb -ex run --args'],
        # arguments=['--ros-args', '--log-level', logger],
        parameters=[
            config_common_path,
            default_config_common,
            default_config_integrator,
            default_config_optimizer,
            default_config_sensor_parameters,
            {

            }
            # Overriding
            # {
            # }
        ]  # ,
        # remapping=[
        #
        # ]
    )

    ### LIOSAM
    liosam_dir = get_package_share_directory('lio_sam')
    liosam_parameter_file = LaunchConfiguration('params_file')
    rviz_config_file = os.path.join(liosam_dir, 'config', 'rviz2.rviz')

    liosam_params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            liosam_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    node_liosam_tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        parameters=[liosam_parameter_file],
        output='screen'
    )

    node_liosam_projection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[liosam_parameter_file],
        output='screen'
    )

    node_liosam_feature = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[liosam_parameter_file],
        prefix=['xterm -sl 10000 -hold -e '],
        output='screen'
    )

    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ### ROSBAG
    BAG_PATH = "/mnt/DataSmall/DELoco/fgo/nodelay/DUS"
    START_OFFSET = "1"
    try:
        if(EnvironmentVariable('BAG_PATH')):
            BAG_PATH = EnvironmentVariable('BAG_PATH')
        if(EnvironmentVariable('START_OFFSET')):
            START_OFFSET = EnvironmentVariable('START_OFFSET')
    except:
        pass
    print("Current bag path: ", BAG_PATH)
    print("Current start offset: ", START_OFFSET)

    exec_play_rosbag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            BAG_PATH,
            "--clock",
            "--start-offset",
            START_OFFSET,
        ],
        output="screen",
        prefix=['xterm -sl 10000 -hold -e '],
    )

    ### MAPVIZ
    mapviz_dir = get_package_share_directory("mapviz")
    mapviz_launch = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(mapviz_dir + '/launch/mapviz.launch.py'))

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value=["debug"],
        description="Logging level"))
    ld.add_action(declare_config_common_path_cmd)
    ld.add_action(declare_config_integrtor_path_cmd)
    ld.add_action(declare_config_optimizer_path_cmd)
    ld.add_action(declare_config_sensor_parameters_path_cmd)
    ld.add_action(declare_config_gnss_preprocessing_path_cmd)
    ld.add_action(node_gnss_preprocessing)
    ld.add_action(node_online_fgo)
    ld.add_action(mapviz_launch)
    ld.add_action(exec_play_rosbag)
    ld.add_action(liosam_params_declare)
    ld.add_action(node_liosam_tf2)
    ld.add_action(node_liosam_projection)
    ld.add_action(node_liosam_feature)
    ld.add_action(node_rviz2)

    return ld
