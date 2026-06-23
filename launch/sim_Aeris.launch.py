import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim = get_package_share_directory('autonomous_driving')
    # pkg_lio_sam = get_package_share_directory('lio_sam')

    # Gazebo findet die paketinternen Modelle (model://sonoma_raceway,
    # model://prius_hybrid) über diesen Resource-Pfad. Macht das Package
    # selbst-enthalten – keine Abhängigkeit vom Fuel-Cache des Users.
    models_path = os.path.join(pkg_sim, 'models')
    set_gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', models_path
    )

    world_file = os.path.join(pkg_sim, 'worlds', 'Sonoma_world.sdf')
    gui_config = os.path.join(pkg_sim, 'config', 'gazebo_gui.config')
    # lio_sam_params = os.path.join(pkg_lio_sam, 'config', 'Parameter', 'params_Aeris.yaml')
    sim_params = os.path.join(pkg_sim, 'config', 'params_sim.yaml')
    rviz_config = os.path.join(pkg_sim, 'config', 'rviz2_sim_Aeris.rviz')
    xacro_path = os.path.join(pkg_sim, 'config', 'robot_sim.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_path]),
        value_type=str
    )

    # ------------------------------------------------------------------
    # Gazebo (Ignition) – Sonoma-Welt
    # ------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_file} --gui-config {gui_config}',
        }.items(),
    )

    # ------------------------------------------------------------------
    # ROS–Gazebo Bridge
    # /clock          – Simulationszeit
    # /ouster/points  – LiDAR-Punktwolke (Ouster OS1-128)
    # /imu/data       – IMU-Daten
    # ------------------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/ouster/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/prius_hybrid/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        parameters=[{'use_sim_time': True}],
        ros_arguments=[
            '-p', 'qos_overrides./imu/data.publisher.depth:=200',
            '-p', 'qos_overrides./imu/data.publisher.history:=keep_last',
            '-p', 'qos_overrides./imu/data.publisher.reliability:=reliable',
        ],
        output='screen',
    )



    # ------------------------------------------------------------------
    # Ouster Timestamp Relay
    # Gazebo gpu_lidar liefert kein 't'-Feld (per-Punkt-Zeitstempel).
    # Ohne dieses Feld setzt imageProjection timeScanEnd = timeScanCur,
    # was deskewInfo() dazu bringt, jeden Scan zu verwerfen.
    # Dieser Node berechnet 't' synthetisch aus dem Azimutwinkel.
    # ------------------------------------------------------------------
    ouster_timestamp_relay = Node(
        package='autonomous_driving',
        executable='ouster_timestamp_relay',
        name='ouster_timestamp_relay',
        parameters=[{
            'use_sim_time': True,
            'scan_frequency': 10.0,
            'num_columns': 1024,
            'input_topic': '/ouster/points',
            'output_topic': '/ouster/points_timestamped',
        }],
        output='screen',
    )

    # ------------------------------------------------------------------
    # Transforms & Robot State
    # ------------------------------------------------------------------
    tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Gazebo publiziert Sensor-Daten mit vollständig qualifizierten Frame-Namen
    # (Schema: {model}/{link}/{sensor}). Diese Aliase verbinden den Gazebo-
    # Scoped-Frame mit dem ROS-TF-Baum, damit LIO-SAM Transforms auflösen kann.
    tf_lidar_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_lidar_frame_alias',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'ouster_lidar_link',
            'prius_hybrid/ouster_lidar_link/ouster_os1_128',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    tf_imu_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_frame_alias',
        arguments=[
            '0', '0', '0', '0', '0', '0',
            'imu_link',
            'prius_hybrid/imu_link/imu_sensor',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )

    # ------------------------------------------------------------------
    # LIO-SAM SLAM
    # params_Aeris.yaml   – Basis-Konfiguration (Sensor-Typ, LOAM-Parameter …)
    # params_sim.yaml     – Simulation-Overrides (Topics, Frames, Extrinsik)
    # ------------------------------------------------------------------
    # lio_sam_params_combined = [lio_sam_params, sim_params]

    lio_sam_imu_preintegration = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[sim_params, {'use_sim_time': True}],
        output='screen',
    )
    lio_sam_image_projection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[sim_params, {'use_sim_time': True}],
        output='screen',
    )
    lio_sam_feature_extraction = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[sim_params, {'use_sim_time': True}],
        output='screen',
    )
    lio_sam_map_optimization = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[sim_params, {'use_sim_time': True}],
        output='screen',
    )

    # ------------------------------------------------------------------
    # RViz
    # ------------------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        bridge,
        ouster_timestamp_relay,
        tf_map_odom,
        tf_lidar_alias,
        tf_imu_alias,
        robot_state_publisher,
        lio_sam_imu_preintegration,
        lio_sam_image_projection,
        lio_sam_feature_extraction,
        lio_sam_map_optimization,
        rviz,
    ])
