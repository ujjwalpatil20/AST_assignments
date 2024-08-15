#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    world = os.path.join(
        get_package_share_directory('robile_gazebo'),
        'worlds',
        'empty_scene.world'
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robile_description"),
                    "gazebo",
                    "gazebo_robile_laserscanner_camera.xacro"
                ]
            ),
            " ",
            "platform_config:=4_wheel_config",
            " ",
            "movable_joints:=False",
        ]
    )

    def spawn_robot(robot):
        namespace = robot['name']
        x_pose = robot['x_pose']
        y_pose = robot['y_pose']
        z_pose = robot['z_pose']
        
        return [
            # Node to spawn the robot in Gazebo
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', f'/{namespace}/robot_description',
                           '-entity', namespace,
                           '-x', str(x_pose),
                           '-y', str(y_pose),
                           '-z', str(z_pose),
                           '-robot_namespace', namespace
                           ],
                output='screen'
            ),
            
            # Node to publish robot state information
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                namespace=namespace,
                remappings=[
                    ('/tf', '/' + namespace + '/tf'),
                    ('/tf_static', '/' + namespace + '/tf_static')
                ],
                parameters=[{
                    'robot_description': ParameterValue(robot_description_content, value_type=str),
                    'frame_prefix': namespace + '/',
                    'use_sim_time': use_sim_time
                }]
            ),

            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0", "0", "0", "0", "0",
                            "0", "base_footprint", "base_link"],
                namespace=namespace,
            ),
            LogInfo(msg=f"Spawning robot {namespace} at ({x_pose}, {y_pose})")
        ]

    def generate_robot_descriptions(robot_count):
        robots = []
        for i in range(robot_count):
            robots.append({
                'name': f'robile_{i}',
                'x_pose': 2.0 * i,  # each subsequent robot is 2 meters apart on the x-axis
                'y_pose': 0.0,
                'z_pose': 0.01
            })
        return robots

    robots = generate_robot_descriptions(3)
    spawn_robots_cmds = []

    for robot in robots:
        spawn_robots_cmds.extend(spawn_robot(robot))

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    rviz_cmd = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    nodes = [
        declare_use_sim_time_argument,
        declare_use_rviz_cmd,
        rviz_cmd,
        gzserver_cmd,
        gzclient_cmd,
        *spawn_robots_cmds
    ]

    return LaunchDescription(nodes)
