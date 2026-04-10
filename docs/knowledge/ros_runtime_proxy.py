"""Generated ROS interaction proxy for GitNexus.

This file makes ROS runtime seams explicit so GitNexus can index
node/topic/service/action relationships that are otherwise hidden
behind ROS APIs and launch indirection.
"""

def ros_topic_audio():
    """ROS topic endpoint /audio."""
    return "/audio"

def ros_topic_audio_voice_detected():
    """ROS topic endpoint /audio/voice_detected."""
    return "/audio/voice_detected"

def ros_topic_audio_info():
    """ROS topic endpoint /audio_info."""
    return "/audio_info"

def ros_topic_audio_stamped():
    """ROS topic endpoint /audio_stamped."""
    return "/audio_stamped"

def ros_topic_chatbot_intent():
    """ROS topic endpoint /chatbot/intent."""
    return "/chatbot/intent"

def ros_topic_chatbot_posture_command():
    """ROS topic endpoint /chatbot/posture_command."""
    return "/chatbot/posture_command"

def ros_service_chatbot_llm_dialogue_interaction():
    """ROS service endpoint /chatbot_llm/dialogue_interaction."""
    return "/chatbot_llm/dialogue_interaction"

def ros_service_chatbot_llm_get_supported_locales():
    """ROS service endpoint /chatbot_llm/get_supported_locales."""
    return "/chatbot_llm/get_supported_locales"

def ros_action_chatbot_llm_set_default_locale():
    """ROS action endpoint /chatbot_llm/set_default_locale."""
    return "/chatbot_llm/set_default_locale"

def ros_action_chatbot_llm_start_dialogue():
    """ROS action endpoint /chatbot_llm/start_dialogue."""
    return "/chatbot_llm/start_dialogue"

def ros_topic_debug_nao_say_speech():
    """ROS topic endpoint /debug/nao_say/speech."""
    return "/debug/nao_say/speech"

def ros_action_debug_say():
    """ROS action endpoint /debug/say."""
    return "/debug/say"

def ros_topic_detected_objects():
    """ROS topic endpoint /detected_objects."""
    return "/detected_objects"

def ros_topic_diagnostics():
    """ROS topic endpoint /diagnostics."""
    return "/diagnostics"

def ros_topic_dialogue_manager_closed_captions():
    """ROS topic endpoint /dialogue_manager/closed_captions."""
    return "/dialogue_manager/closed_captions"

def ros_topic_dialogue_manager_currently_waiting_for_chatbot_response():
    """ROS topic endpoint /dialogue_manager/currently_waiting_for_chatbot_response."""
    return "/dialogue_manager/currently_waiting_for_chatbot_response"

def ros_topic_dialogue_manager_robot_speech():
    """ROS topic endpoint /dialogue_manager/robot_speech."""
    return "/dialogue_manager/robot_speech"

def ros_topic_enriched_snapshot_topic():
    """ROS topic endpoint /enriched_snapshot_topic."""
    return "/enriched_snapshot_topic"

def ros_topic_enriched_text_topic():
    """ROS topic endpoint /enriched_text_topic."""
    return "/enriched_text_topic"

def ros_topic_humans_voices_any_speech():
    """ROS topic endpoint /humans/voices/*/speech."""
    return "/humans/voices/*/speech"

def ros_topic_humans_voices_anonymous_speaker_audio():
    """ROS topic endpoint /humans/voices/anonymous_speaker/audio."""
    return "/humans/voices/anonymous_speaker/audio"

def ros_topic_humans_voices_anonymous_speaker_is_speaking():
    """ROS topic endpoint /humans/voices/anonymous_speaker/is_speaking."""
    return "/humans/voices/anonymous_speaker/is_speaking"

def ros_topic_humans_voices_tracked():
    """ROS topic endpoint /humans/voices/tracked."""
    return "/humans/voices/tracked"

def ros_topic_intent_topic():
    """ROS topic endpoint /intent_topic."""
    return "/intent_topic"

def ros_topic_intents():
    """ROS topic endpoint /intents."""
    return "/intents"

def ros_topic_joint_angles():
    """ROS topic endpoint /joint_angles."""
    return "/joint_angles"

def ros_topic_joint_states():
    """ROS topic endpoint /joint_states."""
    return "/joint_states"

def ros_service_kb_about():
    """ROS service endpoint /kb/about."""
    return "/kb/about"

def ros_topic_kb_active_concepts():
    """ROS topic endpoint /kb/active_concepts."""
    return "/kb/active_concepts"

def ros_topic_kb_add_fact():
    """ROS topic endpoint /kb/add_fact."""
    return "/kb/add_fact"

def ros_service_kb_details():
    """ROS service endpoint /kb/details."""
    return "/kb/details"

def ros_service_kb_events():
    """ROS service endpoint /kb/events."""
    return "/kb/events"

def ros_service_kb_label():
    """ROS service endpoint /kb/label."""
    return "/kb/label"

def ros_service_kb_lookup():
    """ROS service endpoint /kb/lookup."""
    return "/kb/lookup"

def ros_service_kb_manage():
    """ROS service endpoint /kb/manage."""
    return "/kb/manage"

def ros_service_kb_query():
    """ROS service endpoint /kb/query."""
    return "/kb/query"

def ros_topic_kb_remove_fact():
    """ROS topic endpoint /kb/remove_fact."""
    return "/kb/remove_fact"

def ros_service_kb_revise():
    """ROS service endpoint /kb/revise."""
    return "/kb/revise"

def ros_service_kb_sparql():
    """ROS service endpoint /kb/sparql."""
    return "/kb/sparql"

def ros_action_nao_say():
    """ROS action endpoint /nao/say."""
    return "/nao/say"

def ros_topic_nao_scene_grounding_summary():
    """ROS topic endpoint /nao_scene_grounding/summary."""
    return "/nao_scene_grounding/summary"

def ros_topic_planner_execution_feedback():
    """ROS topic endpoint /planner/execution_feedback."""
    return "/planner/execution_feedback"

def ros_topic_planner_dialogue_act_topic():
    """ROS topic endpoint /planner_dialogue_act_topic."""
    return "/planner_dialogue_act_topic"

def ros_topic_planner_feedback_topic():
    """ROS topic endpoint /planner_feedback_topic."""
    return "/planner_feedback_topic"

def ros_topic_planner_request_topic():
    """ROS topic endpoint /planner_request_topic."""
    return "/planner_request_topic"

def ros_topic_robot_speaking():
    """ROS topic endpoint /robot_speaking."""
    return "/robot_speaking"

def ros_action_skill_ask():
    """ROS action endpoint /skill/ask."""
    return "/skill/ask"

def ros_action_skill_chat():
    """ROS action endpoint /skill/chat."""
    return "/skill/chat"

def ros_action_skill_do_head_motion():
    """ROS action endpoint /skill/do_head_motion."""
    return "/skill/do_head_motion"

def ros_action_skill_do_led_effect():
    """ROS action endpoint /skill/do_led_effect."""
    return "/skill/do_led_effect"

def ros_action_skill_do_posture():
    """ROS action endpoint /skill/do_posture."""
    return "/skill/do_posture"

def ros_action_skill_execute_cartesian_trajectory():
    """ROS action endpoint /skill/execute_cartesian_trajectory."""
    return "/skill/execute_cartesian_trajectory"

def ros_action_skill_execute_joint_trajectory():
    """ROS action endpoint /skill/execute_joint_trajectory."""
    return "/skill/execute_joint_trajectory"

def ros_action_skill_look_at():
    """ROS action endpoint /skill/look_at."""
    return "/skill/look_at"

def ros_action_skill_replay_motion():
    """ROS action endpoint /skill/replay_motion."""
    return "/skill/replay_motion"

def ros_action_skill_say():
    """ROS action endpoint /skill/say."""
    return "/skill/say"

def ros_topic_skill_set_expression():
    """ROS topic endpoint /skill/set_expression."""
    return "/skill/set_expression"

def ros_topic_speech():
    """ROS topic endpoint /speech."""
    return "/speech"

def ros_action_tts_engine_tts():
    """ROS action endpoint /tts_engine/tts."""
    return "/tts_engine/tts"

def ros_contract_asr_vosk():
    """Interface contracts exported by asr_vosk."""
    return None

def ros_node_asr_vosk():
    """Runtime ROS proxy for package/node asr_vosk."""
    ros_topic_diagnostics()
    ros_topic_humans_voices_anonymous_speaker_audio()
    ros_topic_humans_voices_anonymous_speaker_is_speaking()
    ros_topic_humans_voices_tracked()
    ros_topic_audio_voice_detected()
    ros_topic_robot_speaking()

def ros_contract_chatbot_llm():
    """Interface contracts exported by chatbot_llm."""
    return None

def ros_node_chatbot_llm():
    """Runtime ROS proxy for package/node chatbot_llm."""
    ros_topic_diagnostics()
    ros_service_chatbot_llm_dialogue_interaction()
    ros_service_chatbot_llm_get_supported_locales()
    ros_action_chatbot_llm_set_default_locale()
    ros_action_chatbot_llm_start_dialogue()

def ros_contract_communication_skills():
    """Interface contracts exported by communication_skills."""
    ros_action_skill_ask()
    ros_action_skill_chat()
    ros_action_skill_say()

def ros_node_communication_skills():
    """Runtime ROS proxy for package/node communication_skills."""
    return None

def ros_contract_dialogue_manager():
    """Interface contracts exported by dialogue_manager."""
    return None

def ros_node_dialogue_manager():
    """Runtime ROS proxy for package/node dialogue_manager."""
    ros_topic_diagnostics()
    ros_topic_dialogue_manager_closed_captions()
    ros_topic_dialogue_manager_currently_waiting_for_chatbot_response()
    ros_topic_dialogue_manager_robot_speech()
    ros_topic_intents()
    ros_topic_humans_voices_any_speech()
    ros_topic_humans_voices_tracked()
    ros_service_chatbot_llm_dialogue_interaction()
    ros_action_chatbot_llm_start_dialogue()
    ros_action_skill_ask()
    ros_action_skill_chat()
    ros_action_skill_say()

def ros_contract_interaction_skills():
    """Interface contracts exported by interaction_skills."""
    ros_action_skill_do_led_effect()
    ros_action_skill_look_at()
    ros_topic_skill_set_expression()

def ros_node_interaction_skills():
    """Runtime ROS proxy for package/node interaction_skills."""
    return None

def ros_contract_kb_skills():
    """Interface contracts exported by kb_skills."""
    ros_service_kb_query()
    ros_service_kb_revise()

def ros_node_kb_skills():
    """Runtime ROS proxy for package/node kb_skills."""
    ros_service_kb_query()
    ros_service_kb_revise()

def ros_contract_knowledge_core():
    """Interface contracts exported by knowledge_core."""
    return None

def ros_node_knowledge_core():
    """Runtime ROS proxy for package/node knowledge_core."""
    ros_topic_diagnostics()
    ros_topic_kb_active_concepts()
    ros_topic_kb_active_concepts()
    ros_topic_kb_add_fact()
    ros_topic_kb_remove_fact()
    ros_service_kb_about()
    ros_service_kb_details()
    ros_service_kb_events()
    ros_service_kb_label()
    ros_service_kb_lookup()
    ros_service_kb_manage()
    ros_service_kb_query()
    ros_service_kb_revise()
    ros_service_kb_sparql()

def ros_contract_motions_skills():
    """Interface contracts exported by motions_skills."""
    ros_action_skill_execute_cartesian_trajectory()
    ros_action_skill_execute_joint_trajectory()

def ros_node_motions_skills():
    """Runtime ROS proxy for package/node motions_skills."""
    return None

def ros_contract_nao_chatbot():
    """Interface contracts exported by nao_chatbot."""
    return None

def ros_node_nao_chatbot():
    """Runtime ROS proxy for package/node nao_chatbot."""
    ros_topic_debug_nao_say_speech()
    ros_topic_dialogue_manager_closed_captions()

def ros_contract_nao_look_at():
    """Interface contracts exported by nao_look_at."""
    return None

def ros_node_nao_look_at():
    """Runtime ROS proxy for package/node nao_look_at."""
    ros_topic_diagnostics()
    ros_topic_joint_angles()
    ros_action_skill_look_at()

def ros_contract_nao_orchestrator():
    """Interface contracts exported by nao_orchestrator."""
    return None

def ros_node_nao_orchestrator():
    """Runtime ROS proxy for package/node nao_orchestrator."""
    ros_topic_chatbot_posture_command()
    ros_topic_diagnostics()
    ros_topic_joint_angles()
    ros_topic_planner_execution_feedback()
    ros_topic_chatbot_intent()
    ros_topic_intents()
    ros_action_nao_say()
    ros_action_skill_do_head_motion()
    ros_action_skill_look_at()
    ros_action_skill_replay_motion()

def ros_contract_nao_replay_motion():
    """Interface contracts exported by nao_replay_motion."""
    return None

def ros_node_nao_replay_motion():
    """Runtime ROS proxy for package/node nao_replay_motion."""
    ros_topic_chatbot_posture_command()
    ros_topic_joint_angles()
    ros_topic_joint_states()
    ros_action_skill_do_head_motion()
    ros_action_skill_do_posture()
    ros_action_skill_replay_motion()

def ros_contract_nao_say_skill():
    """Interface contracts exported by nao_say_skill."""
    ros_action_nao_say()

def ros_node_nao_say_skill():
    """Runtime ROS proxy for package/node nao_say_skill."""
    ros_topic_debug_nao_say_speech()
    ros_topic_diagnostics()
    ros_topic_speech()
    ros_action_debug_say()
    ros_action_nao_say()
    ros_action_tts_engine_tts()

def ros_contract_nao_scene_grounding():
    """Interface contracts exported by nao_scene_grounding."""
    return None

def ros_node_nao_scene_grounding():
    """Runtime ROS proxy for package/node nao_scene_grounding."""
    ros_topic_nao_scene_grounding_summary()
    ros_topic_detected_objects()

def ros_contract_nao_skills():
    """Interface contracts exported by nao_skills."""
    ros_action_skill_do_head_motion()
    ros_action_skill_do_posture()
    ros_action_skill_replay_motion()

def ros_node_nao_skills():
    """Runtime ROS proxy for package/node nao_skills."""
    return None

def ros_contract_planner_llm():
    """Interface contracts exported by planner_llm."""
    return None

def ros_node_planner_llm():
    """Runtime ROS proxy for package/node planner_llm."""
    ros_topic_intent_topic()
    ros_topic_planner_dialogue_act_topic()
    ros_topic_enriched_snapshot_topic()
    ros_topic_enriched_text_topic()
    ros_topic_planner_feedback_topic()
    ros_topic_planner_request_topic()

def ros_contract_simple_audio_capture():
    """Interface contracts exported by simple_audio_capture."""
    return None

def ros_node_simple_audio_capture():
    """Runtime ROS proxy for package/node simple_audio_capture."""
    ros_topic_audio()
    ros_topic_audio_info()
    ros_topic_audio_stamped()

def ros_flow_service_chatbot_llm_dialogue_interaction():
    """Client/server flow for /chatbot_llm/dialogue_interaction."""
    ros_node_chatbot_llm()
    ros_service_chatbot_llm_dialogue_interaction()
    ros_node_dialogue_manager()

def ros_flow_action_chatbot_llm_start_dialogue():
    """Action server/client flow for /chatbot_llm/start_dialogue."""
    ros_node_chatbot_llm()
    ros_action_chatbot_llm_start_dialogue()
    ros_node_dialogue_manager()

def ros_flow_topic_debug_nao_say_speech():
    """Publisher/subscriber flow for /debug/nao_say/speech."""
    ros_node_nao_say_skill()
    ros_topic_debug_nao_say_speech()
    ros_node_nao_chatbot()

def ros_flow_topic_dialogue_manager_closed_captions():
    """Publisher/subscriber flow for /dialogue_manager/closed_captions."""
    ros_node_dialogue_manager()
    ros_topic_dialogue_manager_closed_captions()
    ros_node_nao_chatbot()

def ros_flow_topic_humans_voices_tracked():
    """Publisher/subscriber flow for /humans/voices/tracked."""
    ros_node_asr_vosk()
    ros_topic_humans_voices_tracked()
    ros_node_dialogue_manager()

def ros_flow_topic_intents():
    """Publisher/subscriber flow for /intents."""
    ros_node_dialogue_manager()
    ros_topic_intents()
    ros_node_nao_orchestrator()

def ros_flow_topic_kb_active_concepts():
    """Publisher/subscriber flow for /kb/active_concepts."""
    ros_node_knowledge_core()
    ros_topic_kb_active_concepts()
    ros_node_knowledge_core()

def ros_flow_service_kb_query():
    """Client/server flow for /kb/query."""
    ros_node_knowledge_core()
    ros_service_kb_query()
    ros_node_kb_skills()

def ros_flow_service_kb_revise():
    """Client/server flow for /kb/revise."""
    ros_node_knowledge_core()
    ros_service_kb_revise()
    ros_node_kb_skills()

def ros_flow_action_nao_say():
    """Action server/client flow for /nao/say."""
    ros_node_nao_say_skill()
    ros_action_nao_say()
    ros_node_nao_orchestrator()

def ros_flow_action_skill_do_head_motion():
    """Action server/client flow for /skill/do_head_motion."""
    ros_node_nao_replay_motion()
    ros_action_skill_do_head_motion()
    ros_node_nao_orchestrator()

def ros_flow_action_skill_look_at():
    """Action server/client flow for /skill/look_at."""
    ros_node_nao_look_at()
    ros_action_skill_look_at()
    ros_node_nao_orchestrator()

def ros_flow_action_skill_replay_motion():
    """Action server/client flow for /skill/replay_motion."""
    ros_node_nao_replay_motion()
    ros_action_skill_replay_motion()
    ros_node_nao_orchestrator()
