import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _make_lifecycle_bundle(
    *,
    package_name,
    executable,
    node_name,
    condition,
    extra_parameters=None,
):
    config_path = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "00-defaults.yml"]
    )
    parameters = [config_path]
    if extra_parameters:
        parameters.extend(extra_parameters)

    node = LifecycleNode(
        package=package_name,
        executable=executable,
        namespace="",
        name=node_name,
        parameters=parameters,
        output="both",
        emulate_tty=True,
        condition=condition,
    )
    bootstrap = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            _lifecycle_bootstrap_script(node_name),
        ],
        output="screen",
        condition=condition,
    )
    return [node, bootstrap]


def _not_launching_chatbot_llm_condition():
    return IfCondition(
        PythonExpression(
            [
                '"',
                LaunchConfiguration("start_dialogue_manager"),
                '" == "true" and "',
                LaunchConfiguration("start_chatbot_llm"),
                '" != "true"',
            ]
        )
    )


def _standalone_naoqi_driver_condition():
    return IfCondition(
        PythonExpression(
            [
                '"',
                LaunchConfiguration("start_naoqi_driver"),
                '" == "true" and "',
                LaunchConfiguration("start_nao_robot"),
                '" != "true"',
            ]
        )
    )


def _lifecycle_bootstrap_script(node_name: str, timeout_sec: int = 30) -> str:
    normalized_name = f"/{str(node_name).lstrip('/')}"
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


def _service_wait_script(service_name: str, timeout_sec: int = 30) -> str:
    normalized_name = f"/{str(service_name).lstrip('/')}"
    return f"""
service_name="{normalized_name}"
deadline=$((SECONDS + {max(1, int(timeout_sec))}))
while true; do
  if ros2 service type "$service_name" >/dev/null 2>&1; then
    exit 0
  fi
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "service wait timed out for $service_name" >&2
    exit 1
  fi
  sleep 0.2
done
""".strip()


def _prefer_first_non_empty(*names: str):
    if not names:
        raise ValueError("At least one launch argument name is required")

    expression: list[str] = []
    for index, name in enumerate(names):
        expression.extend(
            [
                '"',
                LaunchConfiguration(name),
                '"',
            ]
        )
        if index < len(names) - 1:
            expression.extend(
                [
                    ' if "',
                    LaunchConfiguration(name),
                    '" != "" else ',
                ]
            )
    return PythonExpression(expression)


def _optional_launch_description(
    context,
    *,
    package_name: str,
    launch_file_name: str,
    launch_arg_name: str,
    required_packages=None,
    launch_arguments=None,
    display_name=None,
):
    if LaunchConfiguration(launch_arg_name).perform(context).lower() != "true":
        return []

    missing_packages = []
    for required_package in required_packages or []:
        try:
            get_package_share_directory(required_package)
        except PackageNotFoundError:
            missing_packages.append(required_package)

    if missing_packages:
        return [
            LogInfo(
                msg=(
                    f"{display_name or package_name} launch skipped because the following upstream "
                    f"packages are missing: {', '.join(missing_packages)}"
                )
            )
        ]

    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f"{package_name} is not installed in this workspace; "
                    f"skipping optional launch '{launch_file_name}'."
                )
            )
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, "launch", launch_file_name)
            ),
            launch_arguments=(launch_arguments or {}).items(),
        )
    ]


def _optional_object_detection_launch(context):
    if LaunchConfiguration("start_object_detection").perform(context).lower() != "true":
        return []

    backend = LaunchConfiguration("object_detection_backend").perform(context).strip().lower()
    if backend in ("yolo_ros", "yolo"):
        return _optional_launch_description(
            context,
            package_name="yolo_bringup",
            launch_file_name="yolo.launch.py",
            launch_arg_name="start_object_detection",
            display_name="yolo_ros",
            launch_arguments={
                "namespace": LaunchConfiguration("object_detection_namespace"),
                "model": LaunchConfiguration("object_detection_model"),
                "device": LaunchConfiguration("object_detection_device"),
                "threshold": LaunchConfiguration("object_detection_threshold"),
                "input_image_topic": LaunchConfiguration("object_detection_input_image_topic"),
                "image_reliability": LaunchConfiguration("object_detection_image_reliability"),
                "use_tracking": "True",
                "use_debug": "True",
            },
            required_packages=["yolo_bringup", "yolo_ros"],
        )

    if backend in ("emorobcare_cv", "emorobot", "emorobcare"):
        try:
            get_package_share_directory("emorobcare_cv_object_detection")
        except PackageNotFoundError:
            return [
                LogInfo(
                    msg=(
                        "emorobcare object detection launch skipped because package "
                        "'emorobcare_cv_object_detection' is not installed in this environment."
                    )
                )
            ]
        return [
            LogInfo(
                msg=(
                    "Launching emorobcare_cv_object_detection. Its package-local config.yaml "
                    "still controls options such as draw_image and human_radar."
                )
            ),
            Node(
                package="emorobcare_cv_object_detection",
                executable="object_detector_node",
                output="screen",
                emulate_tty=True,
                remappings=[
                    ("/camera/image_raw", LaunchConfiguration("object_detection_input_image_topic")),
                ],
            ),
        ]

    return [
        LogInfo(
            msg=(
                "Unsupported object_detection_backend='%s'. Supported backends are "
                "emorobcare_cv and yolo_ros."
            )
            % backend
        )
    ]


def _optional_rviz_launch(
    context,
    *,
    launch_arg_name: str,
    package_name: str,
    config_relative_path: str,
    display_name=None,
):
    if LaunchConfiguration(launch_arg_name).perform(context).lower() != "true":
        return []

    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f"{display_name or package_name} launch skipped because package "
                    f"'{package_name}' is not installed in this environment."
                )
            )
        ]

    rviz_config = os.path.join(package_share, config_relative_path)
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
        LogInfo(msg=f"RViz started with config: {rviz_config}"),
    ]


def generate_launch_description():
    start_naoqi_driver_arg = DeclareLaunchArgument(
        "start_naoqi_driver",
        default_value="false",
        description="Optionally launch naoqi_driver alongside the migrated ROS4HRI stack.",
    )
    nao_ip_arg = DeclareLaunchArgument(
        "nao_ip",
        default_value="172.26.112.62",
        description="NAO robot IP passed to replay motion nodes and naoqi_driver.",
    )
    nao_port_arg = DeclareLaunchArgument(
        "nao_port",
        default_value="9559",
        description="NAOqi port passed to replay motion nodes and naoqi_driver.",
    )
    network_interface_arg = DeclareLaunchArgument(
        "network_interface",
        default_value="eth0",
        description="Network interface used by naoqi_driver when enabled.",
    )
    qi_listen_url_arg = DeclareLaunchArgument(
        "qi_listen_url",
        default_value="tcp://0.0.0.0:0",
        description="QI listen URL used by naoqi_driver when enabled.",
    )
    start_chatbot_llm_arg = DeclareLaunchArgument(
        "start_chatbot_llm",
        default_value="true",
        description="Launch the upstream-aligned chatbot_llm backend.",
    )
    start_nao_robot_arg = DeclareLaunchArgument(
        "start_nao_robot",
        default_value="false",
        description=(
            "Launch the packaged nao_robot bring-up (naoqi_driver + NAO camera "
            "face detection) for real-robot validation."
        ),
    )
    start_nao_robot_hri_visualization_arg = DeclareLaunchArgument(
        "start_nao_robot_hri_visualization",
        default_value="true",
        description=(
            "Launch hri_visualization together with nao_robot for robot-camera "
            "overlay topics and diagnostics."
        ),
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Launch rviz2 using the packaged nao_robot robot-camera config.",
    )
    start_object_detection_arg = DeclareLaunchArgument(
        "start_object_detection",
        default_value="false",
        description="Optionally launch the configured external object detector backend.",
    )
    object_detection_backend_arg = DeclareLaunchArgument(
        "object_detection_backend",
        default_value="emorobcare_cv",
        description="External detector backend to launch: emorobcare_cv or yolo_ros.",
    )
    start_scene_grounding_arg = DeclareLaunchArgument(
        "start_scene_grounding",
        default_value="false",
        description="Launch the local object-to-KnowledgeCore grounding node.",
    )
    object_detection_namespace_arg = DeclareLaunchArgument(
        "object_detection_namespace",
        default_value="yolo",
        description="Namespace used for the external detector stack.",
    )
    object_detection_model_arg = DeclareLaunchArgument(
        "object_detection_model",
        default_value="yolov8n.pt",
        description="Detector model name or path forwarded to yolo_ros.",
    )
    object_detection_device_arg = DeclareLaunchArgument(
        "object_detection_device",
        default_value="cpu",
        description="Detector device forwarded to yolo_ros, for example cpu or cuda:0.",
    )
    object_detection_threshold_arg = DeclareLaunchArgument(
        "object_detection_threshold",
        default_value="0.35",
        description="Detector threshold forwarded to yolo_ros and mirrored into scene grounding defaults.",
    )
    object_detection_input_image_topic_arg = DeclareLaunchArgument(
        "object_detection_input_image_topic",
        default_value="/nao_robot/camera/front/image_raw",
        description="RGB image topic remapped into the external detector stack.",
    )
    object_detection_image_reliability_arg = DeclareLaunchArgument(
        "object_detection_image_reliability",
        default_value="2",
        description="Detector image QoS reliability, where 2 means Best Effort.",
    )
    scene_grounding_detector_topic_arg = DeclareLaunchArgument(
        "scene_grounding_detector_topic",
        default_value="/detected_objects",
        description="Detection topic consumed by nao_scene_grounding.",
    )
    scene_grounding_summary_topic_arg = DeclareLaunchArgument(
        "scene_grounding_summary_topic",
        default_value="/scene/summary",
        description="JSON summary topic published by nao_scene_grounding.",
    )
    scene_grounding_allowed_labels_arg = DeclareLaunchArgument(
        "scene_grounding_allowed_labels",
        default_value="bottle,cup,book,cell phone,backpack,remote,laptop,keyboard,mouse,chair,blueberry,corn,pear,tomato,zucchini",
        description="Comma-separated detector labels to ground into KnowledgeCore.",
    )
    scene_grounding_knowledge_lifespan_sec_arg = DeclareLaunchArgument(
        "scene_grounding_knowledge_lifespan_sec",
        default_value="4.0",
        description="KnowledgeCore lifespan used for transient grounded object facts.",
    )
    scene_grounding_knowledge_refresh_interval_sec_arg = DeclareLaunchArgument(
        "scene_grounding_knowledge_refresh_interval_sec",
        default_value="1.0",
        description="Minimum interval between grounding refreshes for the same tracked object.",
    )
    start_knowledge_core_arg = DeclareLaunchArgument(
        "start_knowledge_core",
        default_value="true",
        description="Optionally launch KnowledgeCore for chatbot_llm grounding when it is installed in the environment.",
    )
    start_dialogue_manager_arg = DeclareLaunchArgument(
        "start_dialogue_manager",
        default_value="true",
        description="Launch the upstream dialogue_manager lifecycle node.",
    )
    start_interaction_sim_arg = DeclareLaunchArgument(
        "start_interaction_sim",
        default_value="false",
        description="Optionally launch the official interaction_sim support launch for simulator testing.",
    )
    start_interaction_sim_perception_arg = DeclareLaunchArgument(
        "start_interaction_sim_perception",
        default_value="true",
        description="Launch the interaction_sim webcam/person/emotion perception components.",
    )
    start_interaction_sim_tools_arg = DeclareLaunchArgument(
        "start_interaction_sim_tools",
        default_value="true",
        description="Launch interaction_sim support tools such as rosbridge and ui_server.",
    )
    start_interaction_sim_ui_arg = DeclareLaunchArgument(
        "start_interaction_sim_ui",
        default_value="false",
        description="Start ui_server together with interaction_sim support tools.",
    )
    interaction_sim_gscam_config_arg = DeclareLaunchArgument(
        "interaction_sim_gscam_config",
        default_value="v4l2src device=/dev/video0 ! video/x-raw,framerate=30/1 ! videoconvert",
        description="GStreamer pipeline used by gscam when interaction_sim support is enabled.",
    )
    start_nao_orchestrator_arg = DeclareLaunchArgument(
        "start_nao_orchestrator",
        default_value="true",
        description="Launch the NAO orchestrator scaffold.",
    )
    start_nao_say_skill_arg = DeclareLaunchArgument(
        "start_nao_say_skill",
        default_value="true",
        description="Launch the dedicated NAO say skill.",
    )
    start_nao_replay_motion_arg = DeclareLaunchArgument(
        "start_nao_replay_motion",
        default_value="true",
        description="Launch replay_motion and retained head-motion servers.",
    )
    start_nao_look_at_arg = DeclareLaunchArgument(
        "start_nao_look_at",
        default_value="true",
        description="Launch the scaffolded look_at skill.",
    )
    start_rqt_console_arg = DeclareLaunchArgument(
        "start_rqt_console",
        default_value="true",
        description="Launch a single remapped rqt shell; when interaction_sim is enabled it loads the official simulator perspective.",
    )
    start_rqt_chat_arg = DeclareLaunchArgument(
        "start_rqt_chat",
        default_value="false",
        description="Optionally launch a separate rqt_chat window remapped onto the debug TTS action when the simulator perspective is not in use.",
    )
    start_robot_speech_debug_arg = DeclareLaunchArgument(
        "start_robot_speech_debug",
        default_value="true",
        description="Launch a logger that mirrors robot speech into ROS logs.",
    )
    posture_command_topic_arg = DeclareLaunchArgument(
        "posture_command_topic",
        default_value="/chatbot/posture_command",
        description="Temporary posture bridge topic used during migration.",
    )
    debug_tts_action_name_arg = DeclareLaunchArgument(
        "debug_tts_action_name",
        default_value="/debug/say",
        description="Debug-only TTS action used for rqt_chat and operator monitoring.",
    )
    dialogue_manager_chatbot_arg = DeclareLaunchArgument(
        "dialogue_manager_chatbot",
        default_value="chatbot_llm",
        description="Dialogue-manager chatbot backend prefix.",
    )
    dialogue_manager_enable_default_chat_arg = DeclareLaunchArgument(
        "dialogue_manager_enable_default_chat",
        default_value="true",
        description="Start a default dialogue so user speech routes to chatbot_llm immediately.",
    )
    dialogue_manager_default_chat_role_arg = DeclareLaunchArgument(
        "dialogue_manager_default_chat_role",
        default_value="__default__",
        description="Role used for the default dialogue session.",
    )
    dialogue_manager_default_chat_configuration_arg = DeclareLaunchArgument(
        "dialogue_manager_default_chat_configuration",
        default_value="",
        description="Optional JSON configuration passed to the default dialogue session.",
    )
    chatbot_model_arg = DeclareLaunchArgument(
        "chatbot_model",
        default_value="llama3.2:1b",
        description="Model used by chatbot_llm for response generation.",
    )
    ollama_model_arg = DeclareLaunchArgument(
        "ollama_model",
        default_value="",
        description="Backward-compatible alias for chatbot_model.",
    )
    chatbot_intent_model_arg = DeclareLaunchArgument(
        "chatbot_intent_model",
        default_value="",
        description="Optional dedicated model used by chatbot_llm for intent extraction.",
    )
    ollama_intent_model_arg = DeclareLaunchArgument(
        "ollama_intent_model",
        default_value="",
        description="Backward-compatible alias for chatbot_intent_model.",
    )
    chatbot_server_url_arg = DeclareLaunchArgument(
        "chatbot_server_url",
        default_value="http://localhost:11434/api/chat",
        description="Backend HTTP endpoint used by chatbot_llm.",
    )

    chatbot_llm_bundle = _make_lifecycle_bundle(
        package_name="chatbot_llm",
        executable="start_node",
        node_name="chatbot_llm",
        condition=IfCondition(LaunchConfiguration("start_chatbot_llm")),
        extra_parameters=[
            {
                "model": ParameterValue(
                    _prefer_first_non_empty("ollama_model", "chatbot_model"),
                    value_type=str,
                )
            },
            {
                "intent_model": ParameterValue(
                    _prefer_first_non_empty(
                        "ollama_intent_model",
                        "chatbot_intent_model",
                        "ollama_model",
                        "chatbot_model",
                    ),
                    value_type=str,
                )
            },
            {
                "server_url": ParameterValue(
                    LaunchConfiguration("chatbot_server_url"),
                    value_type=str,
                )
            },
        ],
    )

    dialogue_manager_bundle = _make_lifecycle_bundle(
        package_name="dialogue_manager",
        executable="start_manager",
        node_name="dialogue_manager",
        condition=IfCondition(LaunchConfiguration("start_dialogue_manager")),
        extra_parameters=[
            {
                "chatbot": ParameterValue(
                    LaunchConfiguration("dialogue_manager_chatbot"),
                    value_type=str,
                )
            },
            {
                "enable_default_chat": ParameterValue(
                    LaunchConfiguration("dialogue_manager_enable_default_chat"),
                    value_type=bool,
                )
            },
            {
                "default_chat_role": ParameterValue(
                    LaunchConfiguration("dialogue_manager_default_chat_role"),
                    value_type=str,
                )
            },
            {
                "default_chat_configuration": ParameterValue(
                    LaunchConfiguration("dialogue_manager_default_chat_configuration"),
                    value_type=str,
                )
            },
        ],
    )

    nao_orchestrator_bundle = _make_lifecycle_bundle(
        package_name="nao_orchestrator",
        executable="run_app",
        node_name="nao_orchestrator",
        condition=IfCondition(LaunchConfiguration("start_nao_orchestrator")),
        extra_parameters=[
            {
                "posture_command_topic": ParameterValue(
                    LaunchConfiguration("posture_command_topic"),
                    value_type=str,
                )
            }
        ],
    )

    nao_say_skill_bundle = _make_lifecycle_bundle(
        package_name="nao_say_skill",
        executable="start_skill",
        node_name="nao_say_skill",
        condition=IfCondition(LaunchConfiguration("start_nao_say_skill")),
        extra_parameters=[
            {
                "debug_tts_action_name": ParameterValue(
                    LaunchConfiguration("debug_tts_action_name"),
                    value_type=str,
                )
            }
        ],
    )

    nao_replay_motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nao_replay_motion"),
                    "launch",
                    "nao_replay_motion.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_nao_replay_motion")),
        launch_arguments={
            "nao_ip": LaunchConfiguration("nao_ip"),
            "nao_port": LaunchConfiguration("nao_port"),
            "posture_command_topic": LaunchConfiguration("posture_command_topic"),
        }.items(),
    )

    naoqi_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("naoqi_driver"), "launch", "naoqi_driver.launch.py"]
            )
        ),
        condition=_standalone_naoqi_driver_condition(),
        launch_arguments={
            "nao_ip": LaunchConfiguration("nao_ip"),
            "nao_port": LaunchConfiguration("nao_port"),
            "network_interface": LaunchConfiguration("network_interface"),
            "qi_listen_url": LaunchConfiguration("qi_listen_url"),
        }.items(),
    )
    nao_robot_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_naoqi_driver"),
                    '" == "true" and "',
                    LaunchConfiguration("start_nao_robot"),
                    '" == "true"',
                ]
            )
        ),
        msg=(
            "start_naoqi_driver and start_nao_robot were both requested. "
            "Skipping the standalone naoqi_driver launch because nao_robot "
            "already includes it."
        ),
    )
    robot_perception_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_nao_robot"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim_perception"),
                    '" == "true"',
                ]
            )
        ),
        msg=(
            "start_nao_robot and interaction_sim perception are both enabled. "
            "This can launch overlapping perception nodes. Prefer "
            "start_nao_robot:=true start_interaction_sim:=true "
            "start_interaction_sim_perception:=false when validating the real "
            "robot camera with simulator tools only."
        ),
    )
    robot_tools_only_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_nao_robot"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim_perception"),
                    '" != "true" and "',
                    LaunchConfiguration("start_interaction_sim_tools"),
                    '" == "true"',
                ]
            )
        ),
        msg=(
            "start_nao_robot is enabled together with interaction_sim tools-only mode. "
            "This is the intended path for combining the real robot camera/TF with "
            "simulator-side operator tools such as rosbridge, rqt_human_radar, and UI helpers."
        ),
    )

    nao_look_at_bundle = _make_lifecycle_bundle(
        package_name="nao_look_at",
        executable="start_skill",
        node_name="nao_look_at",
        condition=IfCondition(LaunchConfiguration("start_nao_look_at")),
    )
    nao_scene_grounding_node = Node(
        package="nao_scene_grounding",
        executable="start_node",
        name="nao_scene_grounding",
        output="screen",
        emulate_tty=True,
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("nao_scene_grounding"), "config", "00-defaults.yml"]
            ),
            {
                "detector_backend": ParameterValue(
                    LaunchConfiguration("object_detection_backend"),
                    value_type=str,
                )
            },
            {
                "detector_topic": ParameterValue(
                    LaunchConfiguration("scene_grounding_detector_topic"),
                    value_type=str,
                )
            },
            {
                "summary_topic": ParameterValue(
                    LaunchConfiguration("scene_grounding_summary_topic"),
                    value_type=str,
                )
            },
            {
                "min_detection_score": ParameterValue(
                    LaunchConfiguration("object_detection_threshold"),
                    value_type=float,
                )
            },
            {
                "allowed_labels": ParameterValue(
                    LaunchConfiguration("scene_grounding_allowed_labels"),
                    value_type=str,
                )
            },
            {
                "knowledge_enabled": ParameterValue(
                    LaunchConfiguration("start_knowledge_core"),
                    value_type=bool,
                )
            },
            {
                "knowledge_lifespan_sec": ParameterValue(
                    LaunchConfiguration("scene_grounding_knowledge_lifespan_sec"),
                    value_type=float,
                )
            },
            {
                "knowledge_refresh_interval_sec": ParameterValue(
                    LaunchConfiguration("scene_grounding_knowledge_refresh_interval_sec"),
                    value_type=float,
                )
            },
            {
                "local_stale_after_sec": ParameterValue(
                    LaunchConfiguration("scene_grounding_knowledge_lifespan_sec"),
                    value_type=float,
                )
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_scene_grounding")),
    )

    rqt_console = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_rqt_console"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" != "true"',
                ]
            )
        ),
        cmd=[
            "bash",
            "-lc",
            [
                "if ! command -v rqt >/dev/null 2>&1; then "
                "echo 'rqt is not installed in this environment'; "
                "elif [ -z \"${DISPLAY:-}\" ] && [ -z \"${WAYLAND_DISPLAY:-}\" ]; then "
                "echo 'rqt launch skipped: DISPLAY/WAYLAND_DISPLAY is not set'; "
                "else "
                "exec rqt --clear-config --ros-args -r /tts_engine/tts:=",
                LaunchConfiguration("debug_tts_action_name"),
                "; "
                "fi",
            ],
        ],
        output="screen",
    )
    interaction_sim_rqt = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_rqt_console"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" == "true"',
                ]
            )
        ),
        cmd=[
            "bash",
            "-lc",
            [
                "if ! command -v rqt >/dev/null 2>&1; then "
                "echo 'rqt is not installed in this environment'; "
                "elif ! ros2 pkg prefix interaction_sim >/dev/null 2>&1; then "
                "echo 'interaction_sim is not installed in this environment'; "
                "elif [ -z \"${DISPLAY:-}\" ] && [ -z \"${WAYLAND_DISPLAY:-}\" ]; then "
                "echo 'rqt launch skipped: DISPLAY/WAYLAND_DISPLAY is not set'; "
                "else "
                "perspective=\"$(ros2 pkg prefix interaction_sim)/share/interaction_sim/config/simulator.perspective\"; "
                "exec rqt --clear-config --perspective-file \"$perspective\" --ros-args -r /tts_engine/tts:=",
                LaunchConfiguration("debug_tts_action_name"),
                "; "
                "fi",
            ],
        ],
        output="screen",
    )
    interaction_sim_rqt_chat_note = LogInfo(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_rqt_chat"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" == "true"',
                ]
            )
        ),
        msg=(
            "start_rqt_chat was requested together with start_interaction_sim. "
            "Skipping the separate rqt_chat window because the interaction_sim "
            "perspective already loads rqt_chat on the debug TTS action."
        ),
    )
    rqt_chat = ExecuteProcess(
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_rqt_chat"),
                    '" == "true" and "',
                    LaunchConfiguration("start_interaction_sim"),
                    '" != "true"',
                ]
            )
        ),
        cmd=[
            "bash",
            "-lc",
            [
            "if ! command -v rqt >/dev/null 2>&1; then "
            "echo 'rqt is not installed in this environment'; "
            "elif ! python3 -c 'import importlib.util,sys; "
            "sys.exit(0 if importlib.util.find_spec(\"rqt_chat\") else 1)' >/dev/null 2>&1; then "
            "echo 'rqt_chat is not installed in this environment'; "
            "elif [ -z \"${DISPLAY:-}\" ] && [ -z \"${WAYLAND_DISPLAY:-}\" ]; then "
            "echo 'rqt_chat launch skipped: DISPLAY/WAYLAND_DISPLAY is not set'; "
            "else "
                "exec rqt --clear-config --standalone rqt_chat.chat.ChatPlugin --ros-args "
                "-r /tts_engine/tts:=",
                LaunchConfiguration("debug_tts_action_name"),
                "; "
                "fi",
            ],
        ],
        output="screen",
    )
    robot_speech_debug = Node(
        package="nao_chatbot",
        executable="robot_speech_debug",
        name="robot_speech_debug",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_robot_speech_debug")),
    )

    dialogue_manager_node = dialogue_manager_bundle[0]
    dialogue_manager_bootstrap = dialogue_manager_bundle[1]
    chatbot_llm_bootstrap_immediate = ExecuteProcess(
        cmd=chatbot_llm_bundle[1].cmd,
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_chatbot_llm"),
                    '" == "true" and "',
                    LaunchConfiguration("start_knowledge_core"),
                    '" != "true"',
                ]
            )
        ),
    )
    knowledge_core_query_wait = ExecuteProcess(
        cmd=["bash", "-lc", _service_wait_script("/kb/query")],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_chatbot_llm"),
                    '" == "true" and "',
                    LaunchConfiguration("start_knowledge_core"),
                    '" == "true"',
                ]
            )
        ),
    )
    chatbot_llm_bootstrap_after_knowledge = ExecuteProcess(
        cmd=chatbot_llm_bundle[1].cmd,
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_chatbot_llm"),
                    '" == "true" and "',
                    LaunchConfiguration("start_knowledge_core"),
                    '" == "true"',
                ]
            )
        ),
    )

    dialogue_manager_bootstrap_immediate = ExecuteProcess(
        cmd=dialogue_manager_bootstrap.cmd,
        output="screen",
        condition=_not_launching_chatbot_llm_condition(),
    )
    chatbot_llm_bootstrap_after_knowledge_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=knowledge_core_query_wait,
            on_exit=[chatbot_llm_bootstrap_after_knowledge],
        )
    )
    dialogue_manager_bootstrap_after_chatbot_immediate = RegisterEventHandler(
        OnProcessExit(
            target_action=chatbot_llm_bootstrap_immediate,
            on_exit=[dialogue_manager_bootstrap],
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_dialogue_manager"),
                    '" == "true" and "',
                    LaunchConfiguration("start_chatbot_llm"),
                    '" == "true" and "',
                    LaunchConfiguration("start_knowledge_core"),
                    '" != "true"',
                ]
            )
        ),
    )
    dialogue_manager_bootstrap_after_chatbot_delayed = RegisterEventHandler(
        OnProcessExit(
            target_action=chatbot_llm_bootstrap_after_knowledge,
            on_exit=[dialogue_manager_bootstrap],
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    '"',
                    LaunchConfiguration("start_dialogue_manager"),
                    '" == "true" and "',
                    LaunchConfiguration("start_chatbot_llm"),
                    '" == "true" and "',
                    LaunchConfiguration("start_knowledge_core"),
                    '" == "true"',
                ]
            )
        ),
    )

    return LaunchDescription(
        [
            start_naoqi_driver_arg,
            start_nao_robot_arg,
            start_nao_robot_hri_visualization_arg,
            start_rviz_arg,
            start_object_detection_arg,
            object_detection_backend_arg,
            start_scene_grounding_arg,
            start_chatbot_llm_arg,
            start_knowledge_core_arg,
            start_dialogue_manager_arg,
            start_interaction_sim_arg,
            start_interaction_sim_perception_arg,
            start_interaction_sim_tools_arg,
            start_interaction_sim_ui_arg,
            start_nao_orchestrator_arg,
            start_nao_say_skill_arg,
            start_nao_replay_motion_arg,
            start_nao_look_at_arg,
            start_rqt_console_arg,
            start_rqt_chat_arg,
            start_robot_speech_debug_arg,
            interaction_sim_gscam_config_arg,
            nao_ip_arg,
            nao_port_arg,
            network_interface_arg,
            qi_listen_url_arg,
            posture_command_topic_arg,
            debug_tts_action_name_arg,
            dialogue_manager_chatbot_arg,
            dialogue_manager_enable_default_chat_arg,
            dialogue_manager_default_chat_role_arg,
            dialogue_manager_default_chat_configuration_arg,
            chatbot_model_arg,
            ollama_model_arg,
            chatbot_intent_model_arg,
            ollama_intent_model_arg,
            chatbot_server_url_arg,
            object_detection_namespace_arg,
            object_detection_model_arg,
            object_detection_device_arg,
            object_detection_threshold_arg,
            object_detection_input_image_topic_arg,
            object_detection_image_reliability_arg,
            scene_grounding_detector_topic_arg,
            scene_grounding_summary_topic_arg,
            scene_grounding_allowed_labels_arg,
            scene_grounding_knowledge_lifespan_sec_arg,
            scene_grounding_knowledge_refresh_interval_sec_arg,
            naoqi_driver_launch,
            nao_robot_note,
            robot_perception_note,
            robot_tools_only_note,
            rqt_console,
            interaction_sim_rqt,
            interaction_sim_rqt_chat_note,
            rqt_chat,
            robot_speech_debug,
            OpaqueFunction(
                function=_optional_launch_description,
                kwargs={
                    "package_name": "knowledge_core",
                    "launch_file_name": "knowledge_core.launch.py",
                    "launch_arg_name": "start_knowledge_core",
                },
            ),
            OpaqueFunction(
                function=_optional_launch_description,
                kwargs={
                    "package_name": "nao_robot",
                    "launch_file_name": "nao_robot.launch.py",
                    "launch_arg_name": "start_nao_robot",
                    "display_name": "nao_robot",
                    "launch_arguments": {
                        "nao_ip": LaunchConfiguration("nao_ip"),
                        "nao_port": LaunchConfiguration("nao_port"),
                    },
                    "required_packages": ["nao_robot"],
                },
            ),
            OpaqueFunction(
                function=_optional_launch_description,
                kwargs={
                    "package_name": "hri_person_manager",
                    "launch_file_name": "person_manager.launch.py",
                    "launch_arg_name": "start_nao_robot",
                    "display_name": "hri_person_manager",
                    "launch_arguments": {
                        "reference_frame": "CameraTop_optical_frame",
                        "robot_reference_frame": "base_link",
                    },
                    "required_packages": ["hri_person_manager"],
                },
            ),
            OpaqueFunction(
                function=_optional_launch_description,
                kwargs={
                    "package_name": "hri_visualization",
                    "launch_file_name": "hri_visualization.launch.py",
                    "launch_arg_name": "start_nao_robot_hri_visualization",
                    "display_name": "hri_visualization",
                    "required_packages": ["hri_visualization"],
                },
            ),
            OpaqueFunction(
                function=_optional_rviz_launch,
                kwargs={
                    "launch_arg_name": "start_rviz",
                    "package_name": "nao_chatbot",
                    "config_relative_path": os.path.join("config", "nao_robot_safe.rviz"),
                    "display_name": "nao_chatbot rviz",
                },
            ),
            OpaqueFunction(
                function=_optional_object_detection_launch,
            ),
            OpaqueFunction(
                function=_optional_launch_description,
                kwargs={
                    "package_name": "nao_chatbot",
                    "launch_file_name": "nao_chatbot_interaction_sim.launch.py",
                    "launch_arg_name": "start_interaction_sim",
                    "display_name": "interaction_sim",
                    "launch_arguments": {
                        "debug_tts_action_name": LaunchConfiguration(
                            "debug_tts_action_name"
                        ),
                        "interaction_sim_gscam_config": LaunchConfiguration(
                            "interaction_sim_gscam_config"
                        ),
                        "start_interaction_sim_perception": LaunchConfiguration(
                            "start_interaction_sim_perception"
                        ),
                        "start_interaction_sim_tools": LaunchConfiguration(
                            "start_interaction_sim_tools"
                        ),
                        "start_interaction_sim_ui": LaunchConfiguration(
                            "start_interaction_sim_ui"
                        ),
                    },
                    "required_packages": ["interaction_sim"],
                },
            ),
            chatbot_llm_bundle[0],
            chatbot_llm_bootstrap_immediate,
            knowledge_core_query_wait,
            chatbot_llm_bootstrap_after_knowledge_ready,
            dialogue_manager_node,
            dialogue_manager_bootstrap_immediate,
            dialogue_manager_bootstrap_after_chatbot_immediate,
            dialogue_manager_bootstrap_after_chatbot_delayed,
            *nao_orchestrator_bundle,
            *nao_say_skill_bundle,
            nao_replay_motion_launch,
            *nao_look_at_bundle,
            nao_scene_grounding_node,
        ]
    )
