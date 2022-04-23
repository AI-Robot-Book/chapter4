import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            prefix='xterm -e', package='happy_lidar', executable='happy_lidar_node', output='screen')
            ])

