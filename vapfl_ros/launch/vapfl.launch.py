import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg


def generate_launch_description():
    pkg_share = get_package_share_directory('vapfl')
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to parameter file for VAPFL node'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure and activate the lifecycle node'
    )

    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    vapfl_node = LifecycleNode(
        package='vapfl',
        executable='vapfl_node',
        name='vapfl_node',
        namespace='',
        output='screen',
        parameters=[params_file],
    )

    # Event to trigger configure when node starts if autostart is true
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node == vapfl_node,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    # Event handler to trigger activate once configured
    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=vapfl_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node == vapfl_node,
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        params_file_arg,
        autostart_arg,
        vapfl_node,
        configure_event,
        activate_event_handler,
    ])
