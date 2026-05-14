from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_path = PathJoinSubstitution([
        FindPackageShare('whycode'),
        'config',
        'helhest.yaml'
    ])

    node = Node(
        package='whycode',
        executable='whycode_node',
        name='whycode_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file_path],
    )

    return LaunchDescription([node])
