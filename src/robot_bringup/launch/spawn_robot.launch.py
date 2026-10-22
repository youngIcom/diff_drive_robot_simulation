import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    world = LaunchConfiguration('world').perform(context)
    model = LaunchConfiguration('model')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    possible_roots = [
        os.getcwd(),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')),
        '/home/yesaya/Documents/Kuliah/Semester_5/Arsitektur_Sistem_Robot_Cerdas/UTS_fix'
    ]
    workspace_root = os.getcwd()
    for root in possible_roots:
        if os.path.isdir(os.path.join(root, 'models')) and os.path.isdir(os.path.join(root, 'world')):
            workspace_root = root
            break

    # Determine Gazebo and RViz behavior based on mode
    # - 'gazebo': Gazebo Sim with GUI, no RViz (default)
    # - 'full_visualize': Gazebo Sim with GUI AND RViz2
    # - 'rviz': Headless Gazebo Sim (-s) and RViz2
    if mode == 'rviz':
        gz_args_val = ['-s -r ', world]
        run_rviz = True
    elif mode == 'full_visualize':
        gz_args_val = ['-r ', world]
        run_rviz = True
    else:  # default 'gazebo'
        gz_args_val = ['-r ', world]
        run_rviz = False

    # Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': gz_args_val}.items()
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'diff_drive_robot',
            '-file', model,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ],
        output='screen'
    )

    # ROS-GZ Bridge node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Velocity Command (ROS 2 -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Wheel Odometry (Gazebo -> ROS 2)
            '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Ground Truth Odometry (Gazebo -> ROS 2, Absolute Pose)
            '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # IMU Data (Gazebo -> ROS 2)
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Joint States (Gazebo -> ROS 2)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Camera Image (Gazebo -> ROS 2)
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen'
    )

    # C++ Vision Node: Buoy Detection & Gate Centering
    buoy_centering_node = Node(
        package='robot_bringup',
        executable='buoy_centering',
        name='buoy_centering',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'cmd_vel_topic': '/cmd_vel',
            'odom_topic': '/ground_truth/odom',
            'publish_cmd_vel': False,
            'kp_angular': 1.5,
            'max_angular_speed': 1.0,
            'nominal_linear_speed': 0.25
        }],
        output='screen'
    )

    # C++ Vision Node: Yellow & Blue Obstacle Capture
    obstacle_detector_node = Node(
        package='robot_bringup',
        executable='obstacle_detector_capture',
        name='obstacle_detector_capture',
        parameters=[{
            'image_topic': '/camera/image_raw',
            'odom_topic': '/ground_truth/odom',
            'save_dir': 'saved_capcture',
            'capture_cooldown_sec': 2.0,
            'min_contour_area': 400.0,
            'require_gate_10': True
        }],
        output='screen'
    )

    nodes = [
        gz_sim,
        spawn_robot,
        bridge,
        buoy_centering_node,
        obstacle_detector_node
    ]

    # RViz and Path Visualizer Nodes
    if run_rviz:
        # Path Visualizer (Publishes /visual/path, /visual/heading arrow, and TF)
        script_path = os.path.join(workspace_root, 'src', 'robot_bringup', 'scripts', 'path_visualizer.py')
        installed_bin = os.path.join(workspace_root, 'install', 'robot_bringup', 'lib', 'robot_bringup', 'path_visualizer')
        if os.path.exists(installed_bin):
            path_vis_node = Node(
                package='robot_bringup',
                executable='path_visualizer',
                name='path_visualizer',
                output='screen'
            )
        else:
            path_vis_node = ExecuteProcess(
                cmd=['python3', script_path],
                output='screen'
            )
        nodes.append(path_vis_node)

        # RViz2 Node
        rviz_config_path = os.path.join(workspace_root, 'src', 'robot_bringup', 'rviz', 'robot_view.rviz')
        if not os.path.exists(rviz_config_path):
            rviz_config_path = os.path.join(workspace_root, 'install', 'robot_bringup', 'share', 'robot_bringup', 'rviz', 'robot_view.rviz')

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
        nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    possible_roots = [
        os.getcwd(),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')),
        '/home/yesaya/Documents/Kuliah/Semester_5/Arsitektur_Sistem_Robot_Cerdas/UTS_fix'
    ]

    workspace_root = os.getcwd()
    for root in possible_roots:
        if os.path.isdir(os.path.join(root, 'models')) and os.path.isdir(os.path.join(root, 'world')):
            workspace_root = root
            break

    models_dir = os.path.join(workspace_root, 'models')
    world_dir = os.path.join(workspace_root, 'world')
    default_world_path = os.path.join(world_dir, 'field.sdf')
    default_model_path = os.path.join(models_dir, 'diff_drive_robot', 'model.sdf')

    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_resource_path = f"{models_dir}:{world_dir}:{existing_resource_path}".strip(':')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_resource_path
    )

    set_gz_file_path = SetEnvironmentVariable(
        name='GZ_FILE_PATH',
        value=new_resource_path
    )

    # Launch Arguments
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='gazebo',
        description='Visualization mode: gazebo (default), full_visualize, or rviz'
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world SDF file'
    )
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Full path to robot model SDF file'
    )
    x_arg = DeclareLaunchArgument('x', default_value='1.80', description='Initial X position')
    y_arg = DeclareLaunchArgument('y', default_value='-2.24', description='Initial Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.05', description='Initial Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.5708', description='Initial Yaw orientation (rad)')

    return LaunchDescription([
        set_gz_resource_path,
        set_gz_file_path,
        mode_arg,
        world_arg,
        model_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        OpaqueFunction(function=launch_setup)
    ])
