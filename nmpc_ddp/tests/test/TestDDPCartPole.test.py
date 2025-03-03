import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, PushRosNamespace, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

def generate_launch_description():
    # Declare launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('no_exit', default_value='false', description='Flag to control exit behavior'),
        DeclareLaunchArgument('time_limit', default_value='500.0', description='Time limit'),

        # Test node
        Node(
            package='nmpc_ddp',
            executable='TestDDPCartPole',
            name='test_ddp_cart_pole',
            output='screen',
            parameters=[{
                'no_exit': LaunchConfiguration('no_exit'),
                'control': {
                    'horizon_dt': 0.01,
                    'horizon_duration': 2.0,
                    'mpc_dt': 0.004,
                    'sim_dt': 0.002
                },
                'param': {
                    'cart_mass': 1.0,
                    'pole_mass': 0.5,
                    'pole_length': 2.0
                },
                'cost': {
                    'running_x': [0.1, 1.0, 0.01, 0.1],
                    'running_u': [0.01],
                    'terminal_x': [0.1, 1.0, 0.01, 0.1]
                }
            }],
            remappings=[],
            arguments=[['--time-limit', LaunchConfiguration('time_limit')]],
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', [ThisLaunchFileDir(), '/tests/rviz/TestDDPCartPole.rviz']],
        ),

        # RQT node
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            output='screen',
            arguments=['--perspective-file', [ThisLaunchFileDir(), '/tests/rqt/TestDDPCartPole.perspective']],
        ),
    ])
