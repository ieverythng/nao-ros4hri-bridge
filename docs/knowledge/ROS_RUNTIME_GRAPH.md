# ROS Runtime Graph

This file is generated from the repository source to make ROS runtime
publish/subscribe, service, and action seams explicit for humans and for
GitNexus indexing.

Generated at: 2026-04-10T16:35:40.392349+00:00

## Packages

| Package | Publishes | Subscribes | Service Clients | Service Servers | Action Clients | Action Servers | Contracts |
| --- | --- | --- | --- | --- | --- | --- | --- |
| asr_vosk | /diagnostics<br>/humans/voices/anonymous_speaker/audio<br>/humans/voices/anonymous_speaker/is_speaking<br>/humans/voices/tracked | /audio/voice_detected<br>/robot_speaking | - | - | - | - | - |
| chatbot_llm | /diagnostics | - | - | /chatbot_llm/dialogue_interaction<br>/chatbot_llm/get_supported_locales | - | /chatbot_llm/set_default_locale<br>/chatbot_llm/start_dialogue | - |
| communication_skills | - | - | - | - | - | - | /skill/ask<br>/skill/chat<br>/skill/say |
| dialogue_manager | /diagnostics<br>/dialogue_manager/closed_captions<br>/dialogue_manager/currently_waiting_for_chatbot_response<br>/dialogue_manager/robot_speech<br>/intents | /humans/voices/*/speech<br>/humans/voices/tracked | /chatbot_llm/dialogue_interaction | - | /chatbot_llm/start_dialogue | /skill/ask<br>/skill/chat<br>/skill/say | - |
| interaction_skills | - | - | - | - | - | - | /skill/do_led_effect<br>/skill/look_at<br>/skill/set_expression |
| kb_skills | - | - | /kb/query<br>/kb/revise | - | - | - | /kb/query<br>/kb/revise |
| knowledge_core | /diagnostics<br>/kb/active_concepts | /kb/active_concepts<br>/kb/add_fact<br>/kb/remove_fact | - | /kb/about<br>/kb/details<br>/kb/events<br>/kb/label<br>/kb/lookup<br>/kb/manage<br>/kb/query<br>/kb/revise<br>/kb/sparql | - | - | - |
| motions_skills | - | - | - | - | - | - | /skill/execute_cartesian_trajectory<br>/skill/execute_joint_trajectory |
| nao_chatbot | - | /debug/nao_say/speech<br>/dialogue_manager/closed_captions | - | - | - | - | - |
| nao_look_at | /diagnostics<br>/joint_angles | - | - | - | - | /skill/look_at | - |
| nao_orchestrator | /chatbot/posture_command<br>/diagnostics<br>/joint_angles<br>/planner/execution_feedback | /chatbot/intent<br>/intents | - | - | /nao/say<br>/skill/do_head_motion<br>/skill/look_at<br>/skill/replay_motion | - | - |
| nao_replay_motion | /chatbot/posture_command<br>/joint_angles | /joint_states | - | - | - | /skill/do_head_motion<br>/skill/do_posture<br>/skill/replay_motion | - |
| nao_say_skill | /debug/nao_say/speech<br>/diagnostics<br>/speech | - | - | - | /debug/say | /nao/say<br>/tts_engine/tts | /nao/say |
| nao_scene_grounding | /nao_scene_grounding/summary | /detected_objects | - | - | - | - | - |
| nao_skills | - | - | - | - | - | - | /skill/do_head_motion<br>/skill/do_posture<br>/skill/replay_motion |
| planner_llm | /intent_topic<br>/planner_dialogue_act_topic | /enriched_snapshot_topic<br>/enriched_text_topic<br>/planner_feedback_topic<br>/planner_request_topic | - | - | - | - | - |
| simple_audio_capture | /audio<br>/audio_info<br>/audio_stamped | - | - | - | - | - | - |

## Shared Runtime Endpoints

### `/audio`

- Publishers: simple_audio_capture

### `/audio/voice_detected`

- Subscribers: asr_vosk

### `/audio_info`

- Publishers: simple_audio_capture

### `/audio_stamped`

- Publishers: simple_audio_capture

### `/chatbot/intent`

- Subscribers: nao_orchestrator

### `/chatbot/posture_command`

- Publishers: nao_orchestrator, nao_replay_motion

### `/chatbot_llm/dialogue_interaction`

- Service Servers: chatbot_llm
- Service Clients: dialogue_manager

### `/chatbot_llm/get_supported_locales`

- Service Servers: chatbot_llm

### `/chatbot_llm/set_default_locale`

- Action Servers: chatbot_llm

### `/chatbot_llm/start_dialogue`

- Action Servers: chatbot_llm
- Action Clients: dialogue_manager

### `/debug/nao_say/speech`

- Publishers: nao_say_skill
- Subscribers: nao_chatbot

### `/debug/say`

- Action Clients: nao_say_skill

### `/detected_objects`

- Subscribers: nao_scene_grounding

### `/diagnostics`

- Publishers: asr_vosk, chatbot_llm, dialogue_manager, knowledge_core, nao_look_at, nao_orchestrator, nao_say_skill

### `/dialogue_manager/closed_captions`

- Publishers: dialogue_manager
- Subscribers: nao_chatbot

### `/dialogue_manager/currently_waiting_for_chatbot_response`

- Publishers: dialogue_manager

### `/dialogue_manager/robot_speech`

- Publishers: dialogue_manager

### `/enriched_snapshot_topic`

- Subscribers: planner_llm

### `/enriched_text_topic`

- Subscribers: planner_llm

### `/humans/voices/*/speech`

- Subscribers: dialogue_manager

### `/humans/voices/anonymous_speaker/audio`

- Publishers: asr_vosk

### `/humans/voices/anonymous_speaker/is_speaking`

- Publishers: asr_vosk

### `/humans/voices/tracked`

- Publishers: asr_vosk
- Subscribers: dialogue_manager

### `/intent_topic`

- Publishers: planner_llm

### `/intents`

- Publishers: dialogue_manager
- Subscribers: nao_orchestrator

### `/joint_angles`

- Publishers: nao_look_at, nao_orchestrator, nao_replay_motion

### `/joint_states`

- Subscribers: nao_replay_motion

### `/kb/about`

- Service Servers: knowledge_core

### `/kb/active_concepts`

- Publishers: knowledge_core
- Subscribers: knowledge_core

### `/kb/add_fact`

- Subscribers: knowledge_core

### `/kb/details`

- Service Servers: knowledge_core

### `/kb/events`

- Service Servers: knowledge_core

### `/kb/label`

- Service Servers: knowledge_core

### `/kb/lookup`

- Service Servers: knowledge_core

### `/kb/manage`

- Service Servers: knowledge_core

### `/kb/query`

- Service Servers: knowledge_core
- Service Clients: kb_skills
- Contracts: kb_skills

### `/kb/remove_fact`

- Subscribers: knowledge_core

### `/kb/revise`

- Service Servers: knowledge_core
- Service Clients: kb_skills
- Contracts: kb_skills

### `/kb/sparql`

- Service Servers: knowledge_core

### `/nao/say`

- Action Servers: nao_say_skill
- Action Clients: nao_orchestrator
- Contracts: nao_say_skill

### `/nao_scene_grounding/summary`

- Publishers: nao_scene_grounding

### `/planner/execution_feedback`

- Publishers: nao_orchestrator

### `/planner_dialogue_act_topic`

- Publishers: planner_llm

### `/planner_feedback_topic`

- Subscribers: planner_llm

### `/planner_request_topic`

- Subscribers: planner_llm

### `/robot_speaking`

- Subscribers: asr_vosk

### `/skill/ask`

- Action Servers: dialogue_manager
- Contracts: communication_skills

### `/skill/chat`

- Action Servers: dialogue_manager
- Contracts: communication_skills

### `/skill/do_head_motion`

- Action Servers: nao_replay_motion
- Action Clients: nao_orchestrator
- Contracts: nao_skills

### `/skill/do_led_effect`

- Contracts: interaction_skills

### `/skill/do_posture`

- Action Servers: nao_replay_motion
- Contracts: nao_skills

### `/skill/execute_cartesian_trajectory`

- Contracts: motions_skills

### `/skill/execute_joint_trajectory`

- Contracts: motions_skills

### `/skill/look_at`

- Action Servers: nao_look_at
- Action Clients: nao_orchestrator
- Contracts: interaction_skills

### `/skill/replay_motion`

- Action Servers: nao_replay_motion
- Action Clients: nao_orchestrator
- Contracts: nao_skills

### `/skill/say`

- Action Servers: dialogue_manager
- Contracts: communication_skills

### `/skill/set_expression`

- Contracts: interaction_skills

### `/speech`

- Publishers: nao_say_skill

### `/tts_engine/tts`

- Action Servers: nao_say_skill
