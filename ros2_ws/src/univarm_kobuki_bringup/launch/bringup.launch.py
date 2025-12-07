from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    this_dir = os.path.dirname(__file__)
    urdf_path = os.path.join(this_dir, '..', 'urdf', 'kobuki.urdf.xacro')
    controllers = os.path.join(this_dir, '..', 'config', 'controllers.yaml')

    ld = LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false')
    ])

    robot_desc = open(urdf_path).read()

    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    ))

    ld.add_action(Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_desc}, controllers],
        output='screen'
    ))

    ld.add_action(Node(
        package='univarm_kobuki_bringup',
        executable='path_follower',
        name='path_follower',
        output='screen'
    ))

    if str(use_rviz.perform({})) == 'true':
        ld.add_action(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ))

    return ld
