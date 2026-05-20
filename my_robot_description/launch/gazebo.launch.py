import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'my_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── 1. Gazebo ──────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # ── 2. Robot State Publisher ───────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # ── 3. Spawn robot ─────────────────────────────────────────
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # ── 4. Bridge ROS2 <-> Gazebo ──────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Bridge ROS2 /cmd_vel to Gazebo /model/my_robot/cmd_vel
            '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        ],
        output='screen'
    )

    # ── 5. Controller Spawners ─────────────────────────────────
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/controller_manager'
        ],
        output='screen'
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '-c', '/controller_manager'
        ],
        output='screen'
    )

    # Remap /cmd_vel to Gazebo internal topic
    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/cmd_vel', '/model/my_robot/cmd_vel'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        bridge,
        cmd_vel_relay,
        TimerAction(period=5.0, actions=[joint_state_broadcaster]),
        TimerAction(period=6.0, actions=[diff_drive_controller]),
    ])