import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            prefix='xterm -e', package='happy_move', executable='happy_move_node', output='screen')
            ])

