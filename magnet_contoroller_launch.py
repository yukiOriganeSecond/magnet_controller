
import launch
import launch_ros.actions
import launch.event_handlers

def generate_launch_description():
    controller = launch_ros.actions.Node(
        package='magnet_controller',
        namespace='magnet_controller',
        executable='controller',
        output='screen'
    )
    gui = launch_ros.actions.Node(
        package='magnet_controller',
        namespace='magnet_controller',
        executable='gui',
        output='screen',
    )
    return launch.LaunchDescription([
        controller,
        gui,
        launch.actions.RegisterEventHandler(    # if gui is closed, cntroller will also closed
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=gui,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])