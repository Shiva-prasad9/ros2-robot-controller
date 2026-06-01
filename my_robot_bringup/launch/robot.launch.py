import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription,
                            DeclareLaunchArgument,
                            TimerAction,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # ── Package paths ──────────────────────────────────────
    desc_pkg  = get_package_share_directory('my_robot_description')
    bringup_pkg = get_package_share_directory('my_robot_bringup')

    xacro_file = os.path.join(
        desc_pkg, 'urdf', 'my_robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    # ── Launch Arguments ───────────────────────────────────
    # Allows overriding controller at launch time:
    # ros2 launch my_robot_bringup robot.launch.py
    #   controller:=impedance
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid',
        description='Controller to use: pid or impedance'
    )

    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Launch Gazebo with GUI (true) or headless (false)'
    )

    controller    = LaunchConfiguration('controller')
    use_gui       = LaunchConfiguration('use_gui')

    # ── 1. Gazebo (headless by default) ───────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': f'-r -s {os.path.join(desc_pkg, "worlds", "my_robot.world.sdf")}'}.items()
    )

    # ── 2. Robot State Publisher ──────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # ── 3. Spawn robot into Gazebo ────────────────────────
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

    # ── 4. ROS-Gazebo Bridge ──────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/my_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # ── 5. PID Controller ─────────────────────────────────
    pid_controller = Node(
        package='my_robot_controllers',
        executable='pid_controller',
        name='pid_controller',
        parameters=[{
            'kp': 1.0,
            'ki': 0.1,
            'kd': 0.05
        }],
        output='screen'
    )

    # ── 6. Impedance Controller ───────────────────────────
    impedance_controller = Node(
        package='my_robot_controllers',
        executable='impedance_controller',
        name='impedance_controller',
        parameters=[{
            'k': 1.0,
            'b': 0.8,
            'm': 0.1
        }],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        controller_arg,
        use_gui_arg,

        # Core simulation
        LogInfo(msg="Starting Gazebo simulation..."),
        gazebo,
        rsp,

        # Spawn robot after 2 seconds
        TimerAction(period=2.0, actions=[
            LogInfo(msg="Spawning robot into Gazebo..."),
            spawn,
        ]),

        # Start bridge after 3 seconds
        TimerAction(period=3.0, actions=[
            LogInfo(msg="Starting ROS-Gazebo bridge..."),
            bridge,
        ]),

        # Start controllers after 4 seconds
        TimerAction(period=4.0, actions=[
            LogInfo(msg="Starting controllers..."),
            pid_controller,
            impedance_controller,
        ]),
    ])