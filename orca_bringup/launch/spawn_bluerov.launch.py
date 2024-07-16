from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    urdf_launch_package = FindPackageShare('orca_bringup')
    ld.add_action(DeclareLaunchArgument(
        'ardusub',
        default_value='False',
        description='Launch ArduSUB with SIM_JSON?'
    ))


    ld.add_action(DeclareLaunchArgument(
        'bag',
        default_value='False',
        description='Bag interesting topics?',
    ))

    ld.add_action(DeclareLaunchArgument(
        'base',
        default_value='True',
        description='Launch base controller?',
    ))

    ld.add_action(DeclareLaunchArgument(
        'gzclient',
        default_value='True',
        description='Launch Gazebo UI?'
    ))

    ld.add_action(DeclareLaunchArgument(
        'mavros',
        default_value='True',
        description='Launch mavros?',
    ))

    ld.add_action(DeclareLaunchArgument(
        'nav',
        default_value='True',
        description='Launch navigation?',
    ))

    ld.add_action(DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch rviz?',
    ))

    ld.add_action(DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Launch SLAM?',
    ))

    ld.add_action(DeclareLaunchArgument(name='jsp_gui', default_value='false', choices=['true', 'false'],
                                        description='Flag to enable joint_state_publisher_gui'))

    default_rviz_config_path = PathJoinSubstitution([urdf_launch_package, 'cfg', 'urdf.rviz'])
    ld.add_action(DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
                                        description='Absolute path to rviz config file'))

    # need to manually pass configuration in because of https://github.com/ros2/launch/issues/313
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': 'orca_description',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'bluerov2.urdf.xacro'])}.items()
    ))

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('jsp_gui'))
    ))

    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('jsp_gui'))
    ))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    ))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('orca_bringup'), 'launch', 'bringup_urdf.py']),
        launch_arguments={
            'base': LaunchConfiguration('base'),
            'mavros': LaunchConfiguration('mavros'),
            'mavros_params_file': PathJoinSubstitution(
                [FindPackageShare('orca_bringup'), 'params', 'sim_mavros_params.yaml']),
            'nav': LaunchConfiguration('nav'),
            'orca_params_file': PathJoinSubstitution([FindPackageShare('orca_bringup'), 'params', 'sim_orca_params.yaml']),
            'slam': LaunchConfiguration('slam'),
        }.items(),
    ))
    return ld
