from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_pal import get_pal_configuration
from launch_ros.actions import LifecycleNode


def _lifecycle_bootstrap_script(node_name: str, timeout_sec: int = 30) -> str:
    normalized_name = f'/{str(node_name).lstrip("/")}'
    return f"""
node_name="{normalized_name}"
deadline=$((SECONDS + {max(1, int(timeout_sec))}))
while true; do
  state="$(ros2 lifecycle get "$node_name" 2>/dev/null | awk '{{print $1}}')"
  case "$state" in
    active)
      exit 0
      ;;
    inactive)
      ros2 lifecycle set "$node_name" activate >/dev/null 2>&1 || true
      ;;
    unconfigured)
      ros2 lifecycle set "$node_name" configure >/dev/null 2>&1 || true
      ;;
    finalized|errorprocessing)
      echo "lifecycle bootstrap failed for $node_name: state=$state" >&2
      exit 1
      ;;
  esac
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "lifecycle bootstrap timed out for $node_name (last_state=${{state:-unknown}})" >&2
    exit 1
  fi
  sleep 0.2
done
""".strip()


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
        ExecuteProcess(
            cmd=["bash", "-lc", _lifecycle_bootstrap_script(node_name)],
            output="screen",
        )
    )

    return ld
