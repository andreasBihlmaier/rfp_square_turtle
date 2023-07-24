from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution, LaunchConfiguration


def generate_launch_description():
    side_length_launch_arg = DeclareLaunchArgument(
        'side_length', default_value=TextSubstitution(text='1.0')
    )
    linear_velocity_launch_arg = DeclareLaunchArgument(
        'linear_velocity', default_value=TextSubstitution(text='1.0')
    )
    angular_velocity_launch_arg = DeclareLaunchArgument(
        'angular_velocity', default_value=TextSubstitution(text='0.523')
        # 30 deg/s
    )
    square_turtle_node = Node(
        package='rfp_square_turtle',
        executable='square_turtle',
        parameters=[{
            'side_length': LaunchConfiguration('side_length'),
            'linear_velocity': LaunchConfiguration('linear_velocity'),
            'angular_velocity': LaunchConfiguration('angular_velocity'),
        }],
        remappings=[
            ('cmd_vel', '/turtle1/cmd_vel'),
            ('pose', '/turtle1/pose'),
        ]
    )
    return LaunchDescription([
        side_length_launch_arg,
        linear_velocity_launch_arg,
        angular_velocity_launch_arg,
        square_turtle_node,
    ])
