from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    this_dir = os.path.dirname(__file__)
    urdf_path = os.path.join(this_dir, '..', 'urdf', 'helix_figure3.urdf.xacro')
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': open(urdf_path).read()}],
             output='screen'),
    ])
