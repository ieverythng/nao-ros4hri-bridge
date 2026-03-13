from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import TimerAction
from launch.events import matches_action
from launch_pal import get_pal_configuration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = "nao_orchestrator"
    node_name = "nao_orchestrator"
    ld = LaunchDescription()
    config = get_pal_configuration(pkg=pkg, node=node_name, ld=ld)

    node = LifecycleNode(
        package=pkg,
        executable="run_app",
        namespace="",
        name=node_name,
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=config["arguments"],
        output="both",
        emulate_tty=True,
    )

    ld.add_action(node)
    ld.add_action(
        TimerAction(
            period=1.0,
            actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )
    ld.add_action(
        TimerAction(
            period=2.0,
            actions=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return ld
