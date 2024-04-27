import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro


def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = 'iodine_description'
    file_subpath = 'urdf/iodine.urdf.xacro'
    pkg_path = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    coursework2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('coursework2'), '/launch', '/sim_bringup.launch.py']),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items(),
    )

    node_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(pkg_path, 'urdf', 'iodine.urdf.xacro'),
            '-name', 'iodine',
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )


    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                             arguments=['-topic', '/robot_description',
                                        '-z', '0.5'],
                             output='screen')


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]  
    )

    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock' + '@rosgraph_msgs/msg/Clock' + '[' + 'ignition.msgs.Clock',
            '/model/iodine/cmd_vel' + '@geometry_msgs/msg/Twist' + '@' + 'ignition.msgs.Twist',
            '/model/iodine/odometry' + '@nav_msgs/msg/Odometry' + '[' + 'ignition.msgs.Odometry',
            '/model/iodine/scan' + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
            '/model/iodine/imu' + '@sensor_msgs/msg/Imu' + '[' + 'ignition.msgs.IMU',
            '/model/iodine/depth' + '@sensor_msgs/msg/Image' + '[' + 'ignition.msgs.Image',
            '/world/empty/model/iodine/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
        ],
        parameters=[{'qos_overrides./iodine.subscriber.reliability': 'reliable'}],
        remappings=[
            ('/model/iodine/cmd_vel', '/iodine/cmd_vel'),
            ('/model/iodine/odometry', '/odom_raw'),
            ('/model/iodine/scan', '/scan'),
            ('/model/iodine/imu', '/imu_raw'),
            ('/model/iodine/depth', '/depth'),
            ('/world/empty/model/iodine/joint_state', '/joint_states')
        ],
        output='screen'
    )
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',
        output='screen',
        remappings=[('/cmd_vel', '/teleop_cmd_vel')],
    )

    twist_mux_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/iodine/cmd_vel')]
    )



    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(coursework2_launch)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(node_spawn_robot)
    ld.add_action(teleop_twist_keyboard_node)
    ld.add_action(twist_mux)
    return ld