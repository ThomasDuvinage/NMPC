import unittest
import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers
import pytest
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


# This function specifies the processes to be run for our test
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch the test processes for testing the FMPC Cart-Pole."""
    
    # Define the LaunchDescription with processes to be tested
    return launch.LaunchDescription([
        # Declare launch arguments first
        launch.actions.DeclareLaunchArgument('no_exit', default_value='false', description='Flag to control exit behavior'),
        launch.actions.DeclareLaunchArgument('time_limit', default_value='500.0', description='Time limit'),

        # Test node (similar to the ROS 1 <test> tag but with Node in ROS 2)
        Node(
            package='nmpc_fmpc',
            executable='TestFmpcCartPole',
            name='test_fmpc_cart_pole',
            output='screen',
            parameters=[{
                'no_exit': LaunchConfiguration('no_exit'),
                'control': {
                    'horizon_dt': 0.01,
                    'horizon_duration': 3.0,
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
            arguments=[['--time-limit', LaunchConfiguration('time_limit')]],
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', [ThisLaunchFileDir(), '/tests/rviz/TestFmpcCartPole.rviz']],
        ),

        # RQT node
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='rqt_gui',
            output='screen',
            arguments=['--perspective-file', [ThisLaunchFileDir(), '/tests/rqt/TestFmpcCartPole.perspective']],
        ),

        # Ready to test action (waiting for the processes to be ready)
        launch_testing.actions.ReadyToTest()
    ])


# This is our test fixture. Each method is a test case.
# These run alongside the processes specified in generate_test_description()
class TestFmpcCartPoleProcess(unittest.TestCase):

    def test_read_stdout(self, proc_output):
        """Check if 'hello_world' or 'FMPC Cart Pole' appears in the stdout."""
        # 'proc_output' is an object added automatically by the launch_testing framework.
        # It captures the outputs of the processes launched in generate_test_description()
        proc_output.assertWaitFor('FMPC Cart Pole', timeout=10, stream='stdout')

    def test_check_node_exit(self, proc_info):
        """Ensure that the processes exit normally."""
        launch_testing.asserts.assertExitCodes(proc_info)


# These tests are run after the processes in generate_test_description() have shutdown.
@launch_testing.post_shutdown_test()
class TestFmpcCartPoleShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check if the processes exited normally."""
        launch_testing.asserts.assertExitCodes(proc_info)
