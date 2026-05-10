# GPT-5.5 Handoff: Container Runtime, Planning, and Scan Skill

Generated: 2026-05-10 15:06:16 CEST (2026-05-10 13:06:16 UTC)

Repo: /home/juanbeck/nao-ros4hri-bridge
Container: nao_ros2
Latest ROS log dir: /root/.ros/log/2026-05-10-12-43-42-497688-juanbeck-Zenbook-UX425QA-UM425QA-2144
Latest launch log: /root/.ros/log/2026-05-10-12-43-42-497688-juanbeck-Zenbook-UX425QA-UM425QA-2144/launch.log (2415 lines)

## Executive Summary

- The latest container run on 2026-05-10 shows stack boot success and chatbot preflight success against vLLM (http://10.7.138.215:8004).
- Planner-mode path is active (planner requests forwarded, scan steps executed).
- Scan execution is active, but person-target scan wording is still inconsistent:
  - Human evidence exists in KnowledgeCore logs (multiple rdf:type Human lines).
  - Scan completion utterances can still report object-only summaries or no-people outcomes.
- A planned-step failure appears in this run: look_at_target action server unavailable.

## Recently Fixed Issues (Code/Workflow)

1. Speech debug reliability fix: explicit lifecycle configure/activate wiring for nao_say_skill in launch stack.
2. Simulator laptop TTS toggle: sim_use_laptop_tts:=true to route utterances to debug TTS in sim.
3. Pre-commit stability: launch smoke now skips untracked launch files in hook context.
4. Scan person-awareness patch is implemented in local repo branch (not yet reflected in this running container snapshot).

## Current Endpoint / Runtime Snapshot

### vLLM reachability (host)

```
{"object":"list","data":[{"id":"QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ","object":"model","created":1778418671,"owned_by":"vllm","root":"QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ","parent":null,"max_model_len":15000,"permission":[{"id":"modelperm-880b148bf4a1c25e","object":"model_permission","created":1778418671,"allow_create_engine":false,"allow_sampling":true,"allow_logprobs":true,"allow_search_indices":false,"allow_view":true,"allow_fine_tuning":false,"organization":"*","group":null,"is_blocking":false}]}]}
HTTP_STATUS:200
```

### vLLM reachability (container)

```
{"object":"list","data":[{"id":"QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ","object":"model","created":1778418672,"owned_by":"vllm","root":"QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ","parent":null,"max_model_len":15000,"permission":[{"id":"modelperm-9f6610bb2a56f325","object":"model_permission","created":1778418672,"allow_create_engine":false,"allow_sampling":true,"allow_logprobs":true,"allow_search_indices":false,"allow_view":true,"allow_fine_tuning":false,"organization":"*","group":null,"is_blocking":false}]}]}
HTTP_STATUS:200
```

### Runtime nodes

```
/chatbot_llm
/demo_rosout_filter
/dialogue_manager
/emotion_hri_listener
/gscam_publisher
/head_motion_skill_server
/hri_emotion_recognizer
/hri_emotion_recognizer/add_analyzer_node
/hri_face_detect_yunet
/hri_face_detect_yunet/add_analyzer_node
/hri_listener
/hri_person_manager
/hri_person_manager/add_analyzer_node
/hri_visualization
/hri_visualization/add_analyzer_node
/kb/knowledge_core
/knowledge_core/add_analyzer_node
/launch_ros_2144
/nao_look_at
/nao_orchestrator
/nao_posture_bridge
/nao_say_skill
/nao_scene_grounding
/naoqi_driver/rosapi
/naoqi_driver/rosbridge_websocket
/object_detector_node
/planner_llm
/replay_motion_skill_server
/robot_speech_debug
/rqt_gui_cpp_node_2161
/rqt_gui_py_node_2161
/static_transform_publisher_sAQlh3xoHRJE4rh8
/static_transform_publisher_wpSnjWE0kIEtFaxg
/transform_listener_impl_57e81c77f600
```

### Key state/params

chatbot_llm lifecycle state:

```
active [3]
```

planner_llm params:

```
String value is: http://10.7.138.215:8004
String value is: openai_compatible
```

## Filtered Evidence Excerpts

### A) Stack, preflight, planner/chatbot

```
3:1778417022.6871521 [INFO] [launch.user]: [STACK] nao_chatbot launch | chatbot_model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ planner_model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ planner_mode=true chatbot_url=http://10.7.138.215:8004/v1/chat/completions planner_url=http://10.7.138.215:8004 planner_gate=true scan=enabled
4:1778417022.6873305 [INFO] [launch.user]: [STACK] enabled nodes | chatbot_llm=true dialogue_manager=true planner_llm=true nao_orchestrator=true scene_grounding=true object_detection=true
5:1778417022.6873965 [INFO] [launch.user]: [LLM PREFLIGHT] launch policy | chatbot_required=true planner_required=true chatbot_timeout=60.0 planner_timeout=60.0
6:1778417022.6874409 [INFO] [launch.user]: [STACK] lifecycle sequencing | chatbot_llm configures before dialogue_manager; planner_llm and executor seams start independently
33:1778417023.0018840 [INFO] [launch.user]: interaction_sim perception + tools layer enabled. This keeps simulator utilities separate from chatbot_llm, dialogue_manager, and knowledge_core.
85:1778417027.8186345 [start_node-22] [0m[INFO] [1778417027.814135810] [chatbot_llm]: Chatbot backend created, awaiting lifecycle configuration.[0m
113:1778417028.2842200 [start_node-22] [0m[INFO] [1778417028.278837214] [chatbot_llm]: [LLM PREFLIGHT] chatbot starting | models=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ timeout=60.0s required=True attempts=3 realistic=True[0m
114:1778417029.2025340 [start_node-22] [0m[INFO] [1778417029.193339257] [chatbot_llm]: [LLM PREFLIGHT] chatbot model ready | model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ attempt=1/3[0m
115:1778417029.3346720 [start_node-22] [0m[INFO] [1778417029.330953427] [chatbot_llm]: [STACK READY] chatbot_llm configured | server_url=http://10.7.138.215:8004/v1/chat/completions model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ intent_model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ intent_mode=llm_with_rules_fallback skill_catalog=11 planner_mode=True planner_topic=/nao_orchestrator/planner_request[0m
116:1778417029.6254685 [start_node-22] [0m[INFO] [1778417029.578332454] [chatbot_llm]: Available LLM models: QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ[0m
117:1778417030.2552340 [start_node-22] [0m[INFO] [1778417030.254777022] [chatbot_llm]: chatbot_llm is active and serving ~/start_dialogue and ~/dialogue_interaction[0m
120:1778417030.5876470 [start_manager-23] [0m[INFO] [1778417030.587258005] [dialogue_manager]: [CHATBOT] Created action client: chatbot_llm/start_dialogue[0m
121:1778417030.5925329 [start_manager-23] [0m[INFO] [1778417030.592194234] [dialogue_manager]: [CHATBOT] Created service client: chatbot_llm/dialogue_interaction[0m
131:1778417031.2058432 [INFO] [launch.user]: [STACK READY] dialogue path active | chatbot_llm=true planner_llm=true planner_mode=true dialogue_manager=/dialogue_manager
132:1778417031.2567875 [start_node-22] [0m[INFO] [1778417031.187271631] [chatbot_llm]: Started dialogue role=__default__ id=3444e73b[0m
156:1778417069.6751742 [start_node-22] [0m[INFO] [1778417069.674756985] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:1 input=Hey Pop![0m
161:1778417069.7454698 [start_node-22] [0m[INFO] [1778417069.744900804] [chatbot_llm]: [turn:__default__:1] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
162:1778417069.7510767 [start_node-22] [0m[INFO] [1778417069.746816479] [chatbot_llm]: [turn:__default__:1] TURN_START | user="Hey Pop!"[0m
163:1778417069.7520683 [start_node-22] [0m[INFO] [1778417069.749442869] [chatbot_llm]: [turn:__default__:1] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=0 timeout=75.0s[0m
164:1778417070.1822665 [start_node-22] [0m[INFO] [1778417070.181895295] [chatbot_llm]: [turn:__default__:1] ROUTE_RESOLVED | route=dialogue intent=greet source=llm_response_route confidence=0.92[0m
165:1778417070.1834493 [start_node-22] [0m[INFO] [1778417070.183177980] [chatbot_llm]: [turn:__default__:1] TURN_DONE | planner-mode response complete[0m
179:1778417087.4193215 [start_node-22] [0m[INFO] [1778417087.418852501] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:2 input=Can you scan the room for me?[0m
184:1778417087.4967444 [start_node-22] [0m[INFO] [1778417087.494900010] [chatbot_llm]: [turn:__default__:2] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
185:1778417087.4994721 [start_node-22] [0m[INFO] [1778417087.498920509] [chatbot_llm]: [turn:__default__:2] TURN_START | user="Can you scan the room for me?"[0m
186:1778417087.5057337 [start_node-22] [0m[INFO] [1778417087.500994358] [chatbot_llm]: [turn:__default__:2] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=2 timeout=60.0s[0m
187:1778417088.1783886 [start_node-22] [0m[INFO] [1778417088.177619004] [chatbot_llm]: [turn:__default__:2] ROUTE_RESOLVED | route=execution intent=- source=llm_response_route confidence=0.85[0m
188:1778417088.1806040 [start_node-22] [0m[INFO] [1778417088.179880225] [chatbot_llm]: [turn:__default__:2] TURN_DONE | planner-mode response complete[0m
189:1778417088.1937819 [start_node-22] [0m[INFO] [1778417088.191892391] [chatbot_llm]: [turn:__default__:2] PLANNER_REQUEST | published planner request on /nao_orchestrator/planner_request goal_id=goal_default___2 kind=new_goal[0m
216:1778417095.6939147 [start_node-22] [0m[INFO] [1778417095.693699931] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=__system__ turn=__default__:3 input=The robot has finished executing a user-requested task. Reply to the ...[0m
221:1778417095.7496676 [start_node-22] [0m[INFO] [1778417095.748941512] [chatbot_llm]: [turn:__default__:3] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
222:1778417095.7518375 [start_node-22] [0m[INFO] [1778417095.751419415] [chatbot_llm]: [turn:__default__:3] TURN_START | user="The robot has finished executing a user-requested task. Reply to the ..."[0m
223:1778417095.7537789 [start_node-22] [0m[INFO] [1778417095.753293966] [chatbot_llm]: [turn:__default__:3] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=5 timeout=60.0s[0m
224:1778417096.1847832 [start_node-22] [0m[INFO] [1778417096.184196279] [chatbot_llm]: [turn:__default__:3] ROUTE_RESOLVED | route=execution intent=- source=llm_response_inferred_route confidence=0.00[0m
225:1778417096.1860545 [start_node-22] [0m[INFO] [1778417096.185283556] [chatbot_llm]: [turn:__default__:3] TURN_DONE | planner-mode response complete[0m
1018:1778417703.9448035 [start_node-22] [0m[INFO] [1778417703.942834645] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:4 input=What can you see right now?[0m
1023:1778417704.1051288 [start_node-22] [0m[INFO] [1778417704.104436311] [chatbot_llm]: [turn:__default__:4] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1024:1778417704.1089718 [start_node-22] [0m[INFO] [1778417704.108543974] [chatbot_llm]: [turn:__default__:4] TURN_START | user="What can you see right now?"[0m
1025:1778417704.1124871 [start_node-22] [0m[INFO] [1778417704.111665530] [chatbot_llm]: [turn:__default__:4] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=7 timeout=60.0s[0m
1026:1778417704.5007694 [start_node-22] [0m[INFO] [1778417704.499475542] [chatbot_llm]: [turn:__default__:4] ROUTE_RESOLVED | route=dialogue intent=- source=llm_response_route confidence=0.00[0m
1027:1778417704.5040090 [start_node-22] [0m[INFO] [1778417704.502610302] [chatbot_llm]: [turn:__default__:4] TURN_DONE | planner-mode response complete[0m
1056:1778417710.4730814 [start_node-22] [0m[INFO] [1778417710.472578538] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:5 input=Whats the id?[0m
1061:1778417710.5551414 [start_node-22] [0m[INFO] [1778417710.553631060] [chatbot_llm]: [turn:__default__:5] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1062:1778417710.5577614 [start_node-22] [0m[INFO] [1778417710.555922078] [chatbot_llm]: [turn:__default__:5] TURN_START | user="Whats the id?"[0m
1063:1778417710.5596418 [start_node-22] [0m[INFO] [1778417710.557539538] [chatbot_llm]: [turn:__default__:5] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=9 timeout=60.0s[0m
1064:1778417710.9257514 [start_node-22] [0m[INFO] [1778417710.924803432] [chatbot_llm]: [turn:__default__:5] ROUTE_RESOLVED | route=dialogue intent=- source=llm_response_route confidence=0.00[0m
1065:1778417710.9352283 [start_node-22] [0m[INFO] [1778417710.928824480] [chatbot_llm]: [turn:__default__:5] TURN_DONE | planner-mode response complete[0m
1080:1778417734.9336765 [start_node-22] [0m[INFO] [1778417734.929415420] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:6 input=Perfect, can you do a scan and tell me if you see the same person?[0m
1085:1778417734.9557333 [start_node-22] [0m[INFO] [1778417734.955306736] [chatbot_llm]: [turn:__default__:6] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1086:1778417734.9578321 [start_node-22] [0m[INFO] [1778417734.956467860] [chatbot_llm]: [turn:__default__:6] TURN_START | user="Perfect, can you do a scan and tell me if you see the same person?"[0m
1087:1778417734.9588342 [start_node-22] [0m[INFO] [1778417734.957743941] [chatbot_llm]: [turn:__default__:6] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=11 timeout=60.0s[0m
1088:1778417735.3733766 [start_node-22] [0m[INFO] [1778417735.372733688] [chatbot_llm]: [turn:__default__:6] ROUTE_RESOLVED | route=execution intent=- source=llm_response_inferred_route confidence=0.00[0m
1089:1778417735.3757441 [start_node-22] [0m[INFO] [1778417735.374403560] [chatbot_llm]: [turn:__default__:6] TURN_DONE | planner-mode response complete[0m
1090:1778417735.3960633 [start_node-22] [0m[INFO] [1778417735.395659758] [chatbot_llm]: [turn:__default__:6] PLANNER_REQUEST | published planner request on /nao_orchestrator/planner_request goal_id=goal_default___6 kind=new_goal[0m
1131:1778417742.8354568 [start_node-22] [0m[INFO] [1778417742.834047666] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=__system__ turn=__default__:7 input=The robot has finished executing a user-requested task. Reply to the ...[0m
1136:1778417742.8699589 [start_node-22] [0m[INFO] [1778417742.869592138] [chatbot_llm]: [turn:__default__:7] KB_SNAPSHOT | loaded 293 chars from /kb/query[0m
1137:1778417742.8713076 [start_node-22] [0m[INFO] [1778417742.871005352] [chatbot_llm]: [turn:__default__:7] TURN_START | user="The robot has finished executing a user-requested task. Reply to the ..."[0m
1138:1778417742.8724980 [start_node-22] [0m[INFO] [1778417742.872284494] [chatbot_llm]: [turn:__default__:7] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=15 timeout=60.0s[0m
1143:1778417743.2575481 [start_node-22] [0m[INFO] [1778417743.256988910] [chatbot_llm]: [turn:__default__:7] ROUTE_RESOLVED | route=execution intent=- source=llm_response_inferred_route confidence=0.00[0m
1144:1778417743.2595031 [start_node-22] [0m[INFO] [1778417743.258407655] [chatbot_llm]: [turn:__default__:7] TURN_DONE | planner-mode response complete[0m
1329:1778417841.4780416 [start_node-22] [0m[INFO] [1778417841.477381938] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:8 input=DId you see any people in the last scan?[0m
1334:1778417841.5169866 [start_node-22] [0m[INFO] [1778417841.516561295] [chatbot_llm]: [turn:__default__:8] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1335:1778417841.5182283 [start_node-22] [0m[INFO] [1778417841.517635688] [chatbot_llm]: [turn:__default__:8] TURN_START | user="DId you see any people in the last scan?"[0m
1336:1778417841.5189979 [start_node-22] [0m[INFO] [1778417841.518769825] [chatbot_llm]: [turn:__default__:8] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=17 timeout=60.0s[0m
1337:1778417842.0537539 [start_node-22] [0m[INFO] [1778417842.052652160] [chatbot_llm]: [turn:__default__:8] ROUTE_RESOLVED | route=execution intent=- source=llm_response_inferred_route confidence=0.00[0m
1338:1778417842.0589337 [start_node-22] [0m[INFO] [1778417842.056527622] [chatbot_llm]: [turn:__default__:8] TURN_DONE | planner-mode response complete[0m
1339:1778417842.0683453 [start_node-22] [0m[INFO] [1778417842.062222275] [chatbot_llm]: [turn:__default__:8] PLANNER_REQUEST | published planner request on /nao_orchestrator/planner_request goal_id=goal_default___8 kind=new_goal[0m
1353:1778417844.7426324 [run_app-24] [33m[WARN] [1778417844.741233689] [nao_orchestrator]: Planned intent step failed | intent=raw_user_input source=planner_llm plan_id=plan_1778417843439 step={'id': 'step_3', 'type': 'skill', 'name': 'look_at', 'args': {'target_frame': 'head_center'}, 'requires': [], 'on_failure': 'fail', 'retry_budget': 0} reason=look_at_target action server unavailable[0m
```

### B) Scan/planner outcomes

```
3:1778417022.6871521 [INFO] [launch.user]: [STACK] nao_chatbot launch | chatbot_model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ planner_model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ planner_mode=true chatbot_url=http://10.7.138.215:8004/v1/chat/completions planner_url=http://10.7.138.215:8004 planner_gate=true scan=enabled
166:1778417070.2031207 [start_manager-23] [0m[INFO] [1778417070.200804132] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "Hello! How can I help you today?"[0m
167:1778417070.2114127 [start_manager-23] [0m[INFO] [1778417070.210345032] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
176:1778417087.4074688 [start_manager-23] [0m[INFO] [1778417087.406772604] [dialogue_manager]: [SPEECH INPUT] voice_id="anonymous_speaker": "Can you scan the room for me?" (locale=, confidence=0.00)[0m
178:1778417087.4165046 [start_manager-23] [0m[INFO] [1778417087.414753949] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="anonymous_speaker", text="Can you scan the room for me?"[0m
179:1778417087.4193215 [start_node-22] [0m[INFO] [1778417087.418852501] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:2 input=Can you scan the room for me?[0m
185:1778417087.4994721 [start_node-22] [0m[INFO] [1778417087.498920509] [chatbot_llm]: [turn:__default__:2] TURN_START | user="Can you scan the room for me?"[0m
190:1778417088.2018723 [start_manager-23] [0m[INFO] [1778417088.200914729] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "Sure, I will look around and report what is visible."[0m
191:1778417088.2109225 [start_manager-23] [0m[INFO] [1778417088.203720428] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
206:1778417095.6785271 [run_app-24] [0m[INFO] [1778417095.677467053] [nao_orchestrator]: ORCH SCAN | target=scene target_kind=scene result_mode=success[0m
226:1778417096.1894927 [start_manager-23] [0m[INFO] [1778417096.189096949] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "I looked around and can report the current scene summary."[0m
227:1778417096.1905572 [start_manager-23] [0m[INFO] [1778417096.190132461] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1028:1778417704.5177279 [start_manager-23] [0m[INFO] [1778417704.517314739] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "I can see an anonymous person currently."[0m
1029:1778417704.5195785 [start_manager-23] [0m[INFO] [1778417704.519064308] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1066:1778417710.9424760 [start_manager-23] [0m[INFO] [1778417710.941846924] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "The ID of the person currently visible is anonymous_speaker."[0m
1067:1778417710.9451756 [start_manager-23] [0m[INFO] [1778417710.943916117] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1077:1778417734.9189126 [start_manager-23] [0m[INFO] [1778417734.918202556] [dialogue_manager]: [SPEECH INPUT] voice_id="anonymous_speaker": "Perfect, can you do a scan and tell me if you see the same person?" (locale=, confidence=0.00)[0m
1079:1778417734.9324028 [start_manager-23] [0m[INFO] [1778417734.924027745] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="anonymous_speaker", text="Perfect, can you do a scan and tell me if you see the same person?"[0m
1080:1778417734.9336765 [start_node-22] [0m[INFO] [1778417734.929415420] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:6 input=Perfect, can you do a scan and tell me if you see the same person?[0m
1086:1778417734.9578321 [start_node-22] [0m[INFO] [1778417734.956467860] [chatbot_llm]: [turn:__default__:6] TURN_START | user="Perfect, can you do a scan and tell me if you see the same person?"[0m
1091:1778417735.3999803 [start_manager-23] [0m[INFO] [1778417735.399620852] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "Sure, I will look around and report what I can see."[0m
1092:1778417735.4011049 [start_manager-23] [0m[INFO] [1778417735.400880081] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1122:1778417742.7870121 [run_app-24] [0m[INFO] [1778417742.786509992] [nao_orchestrator]: ORCH SCAN | target=person target_kind=person result_mode=success[0m
1127:1778417742.8254411 [start_manager-23] Original user request: Perfect, can you do a scan and tell me if you see the same person?
1145:1778417743.2623968 [start_manager-23] [0m[INFO] [1778417743.262068910] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "I can see two blueberries in the current scene."[0m
1146:1778417743.2635612 [start_manager-23] [0m[INFO] [1778417743.263312926] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1147:1778417743.2645888 [start_manager-23] [0m[INFO] [1778417743.264301901] [dialogue_manager]: [TTS] Speaking text (priority=128): "I can see two blueberries in the current scene."[0m
1148:1778417743.2680762 [start_skill-26] [0m[INFO] [1778417743.267790320] [nao_say_skill]: [turn:I can see two blueberries in the] TTS_ACCEPTED | goal accepted[0m
1149:1778417743.2700043 [start_skill-26] [0m[INFO] [1778417743.269757620] [nao_say_skill]: [turn:I can see two blueberries in the] SAY_START | text_len=47 lang=en-US volume=1.00[0m
1150:1778417743.2719982 [start_skill-26] [0m[INFO] [1778417743.271153171] [nao_say_skill]: [turn:I can see two blueberries in the] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=47[0m
1151:1778417743.2735555 [start_skill-26] [0m[INFO] [1778417743.273404397] [nao_say_skill]: [turn:I can see two blueberries in the] DEBUG_TTS_FORWARD | action="/debug/say" text_len=47[0m
1152:1778417743.2748914 [start_skill-26] [33m[WARN] [1778417743.274767327] [nao_say_skill]: [turn:I can see two blueberries in the] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1153:1778417743.2765212 [start_skill-26] [0m[INFO] [1778417743.276342888] [nao_say_skill]: [turn:I can see two blueberries in the] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=47 subscribers=0[0m
1154:1778417743.2860651 [start_skill-26] [0m[INFO] [1778417743.285690396] [nao_say_skill]: [turn:I can see two blueberries in the] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1326:1778417841.4654408 [start_manager-23] [0m[INFO] [1778417841.464558670] [dialogue_manager]: [SPEECH INPUT] voice_id="anonymous_speaker": "DId you see any people in the last scan?" (locale=, confidence=0.00)[0m
1328:1778417841.4732385 [start_manager-23] [0m[INFO] [1778417841.470728154] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="anonymous_speaker", text="DId you see any people in the last scan?"[0m
1329:1778417841.4780416 [start_node-22] [0m[INFO] [1778417841.477381938] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:8 input=DId you see any people in the last scan?[0m
1335:1778417841.5182283 [start_node-22] [0m[INFO] [1778417841.517635688] [chatbot_llm]: [turn:__default__:8] TURN_START | user="DId you see any people in the last scan?"[0m
1341:1778417842.0739424 [start_manager-23] [0m[INFO] [1778417842.072077789] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "No, I did not see any people in the last scan."[0m
1342:1778417842.0767751 [start_manager-23] [0m[INFO] [1778417842.074858097] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1343:1778417842.0775642 [start_manager-23] [0m[INFO] [1778417842.076488006] [dialogue_manager]: [TTS] Speaking text (priority=128): "No, I did not see any people in the last scan."[0m
1344:1778417842.0840290 [start_skill-26] [0m[INFO] [1778417842.083637874] [nao_say_skill]: [turn:No, I did not see any people in] TTS_ACCEPTED | goal accepted[0m
1345:1778417842.0883355 [start_skill-26] [0m[INFO] [1778417842.087575404] [nao_say_skill]: [turn:No, I did not see any people in] SAY_START | text_len=46 lang=en-US volume=1.00[0m
1346:1778417842.0932231 [start_skill-26] [0m[INFO] [1778417842.092927841] [nao_say_skill]: [turn:No, I did not see any people in] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=46[0m
1347:1778417842.0976820 [start_skill-26] [0m[INFO] [1778417842.094606384] [nao_say_skill]: [turn:No, I did not see any people in] DEBUG_TTS_FORWARD | action="/debug/say" text_len=46[0m
1348:1778417842.0979106 [start_skill-26] [33m[WARN] [1778417842.096731905] [nao_say_skill]: [turn:No, I did not see any people in] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1349:1778417842.1015990 [start_skill-26] [0m[INFO] [1778417842.098376363] [nao_say_skill]: [turn:No, I did not see any people in] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=46 subscribers=0[0m
1350:1778417842.1059709 [start_skill-26] [0m[INFO] [1778417842.105504239] [nao_say_skill]: [turn:No, I did not see any people in] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1351:1778417843.4596641 [run_app-24] [0m[INFO] [1778417843.458587227] [nao_orchestrator]: ORCH SCAN | target=scene target_kind=scene result_mode=success[0m
1353:1778417844.7426324 [run_app-24] [33m[WARN] [1778417844.741233689] [nao_orchestrator]: Planned intent step failed | intent=raw_user_input source=planner_llm plan_id=plan_1778417843439 step={'id': 'step_3', 'type': 'skill', 'name': 'look_at', 'args': {'target_frame': 'head_center'}, 'requires': [], 'on_failure': 'fail', 'retry_budget': 0} reason=look_at_target action server unavailable[0m
1354:1778417844.7512512 [start_manager-23] [33m[WARN] [1778417844.749524071] [dialogue_manager]: [PLANNER ACT] explain_failure suppressed for TTS goal_id=goal_default___8 reason=look_at_target action server unavailable text_hint=look_at_target action server unavailable[0m
```

### C) Speech debug / TTS

```
169:1778417070.2445648 [start_skill-26] [0m[INFO] [1778417070.244015289] [nao_say_skill]: [turn:Hello! How can I help you today?] TTS_ACCEPTED | goal accepted[0m
171:1778417070.2553883 [start_skill-26] [0m[INFO] [1778417070.255130560] [nao_say_skill]: [turn:Hello! How can I help you today?] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=32[0m
173:1778417070.2604129 [start_skill-26] [33m[WARN] [1778417070.259879739] [nao_say_skill]: [turn:Hello! How can I help you today?] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
175:1778417070.2672508 [start_skill-26] [0m[INFO] [1778417070.266897981] [nao_say_skill]: [turn:Hello! How can I help you today?] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
194:1778417088.2202942 [start_skill-26] [0m[INFO] [1778417088.219911745] [nao_say_skill]: [turn:Sure, I will look around and rep] TTS_ACCEPTED | goal accepted[0m
196:1778417088.2256136 [start_skill-26] [0m[INFO] [1778417088.224871376] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=52[0m
198:1778417088.2278214 [start_skill-26] [33m[WARN] [1778417088.227511653] [nao_say_skill]: [turn:Sure, I will look around and rep] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
200:1778417088.2757733 [start_skill-26] [0m[INFO] [1778417088.274056331] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
229:1778417096.1937699 [start_skill-26] [0m[INFO] [1778417096.193618810] [nao_say_skill]: [turn:I looked around and can report t] TTS_ACCEPTED | goal accepted[0m
231:1778417096.1985216 [start_skill-26] [0m[INFO] [1778417096.197831425] [nao_say_skill]: [turn:I looked around and can report t] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=57[0m
233:1778417096.2003837 [start_skill-26] [33m[WARN] [1778417096.200124062] [nao_say_skill]: [turn:I looked around and can report t] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
235:1778417096.2364602 [start_skill-26] [0m[INFO] [1778417096.235134727] [nao_say_skill]: [turn:I looked around and can report t] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1031:1778417704.6421146 [start_skill-26] [0m[INFO] [1778417704.641040426] [nao_say_skill]: [turn:I can see an anonymous person cu] TTS_ACCEPTED | goal accepted[0m
1033:1778417704.6959786 [start_skill-26] [0m[INFO] [1778417704.694520566] [nao_say_skill]: [turn:I can see an anonymous person cu] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=40[0m
1035:1778417704.7285349 [start_skill-26] [33m[WARN] [1778417704.728120166] [nao_say_skill]: [turn:I can see an anonymous person cu] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1037:1778417704.7965844 [start_skill-26] [0m[INFO] [1778417704.796105091] [nao_say_skill]: [turn:I can see an anonymous person cu] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1069:1778417710.9524248 [start_skill-26] [0m[INFO] [1778417710.950626656] [nao_say_skill]: [turn:The ID of the person currently v] TTS_ACCEPTED | goal accepted[0m
1071:1778417710.9581616 [start_skill-26] [0m[INFO] [1778417710.955984299] [nao_say_skill]: [turn:The ID of the person currently v] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=60[0m
1073:1778417710.9613636 [start_skill-26] [33m[WARN] [1778417710.961110801] [nao_say_skill]: [turn:The ID of the person currently v] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1075:1778417710.9718721 [start_skill-26] [0m[INFO] [1778417710.971250412] [nao_say_skill]: [turn:The ID of the person currently v] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1094:1778417735.4050620 [start_skill-26] [0m[INFO] [1778417735.404505774] [nao_say_skill]: [turn:Sure, I will look around and rep] TTS_ACCEPTED | goal accepted[0m
1096:1778417735.4084246 [start_skill-26] [0m[INFO] [1778417735.408211748] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=51[0m
1098:1778417735.4107480 [start_skill-26] [33m[WARN] [1778417735.410641670] [nao_say_skill]: [turn:Sure, I will look around and rep] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1100:1778417735.4443271 [start_skill-26] [0m[INFO] [1778417735.443522853] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1148:1778417743.2680762 [start_skill-26] [0m[INFO] [1778417743.267790320] [nao_say_skill]: [turn:I can see two blueberries in the] TTS_ACCEPTED | goal accepted[0m
1150:1778417743.2719982 [start_skill-26] [0m[INFO] [1778417743.271153171] [nao_say_skill]: [turn:I can see two blueberries in the] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=47[0m
1152:1778417743.2748914 [start_skill-26] [33m[WARN] [1778417743.274767327] [nao_say_skill]: [turn:I can see two blueberries in the] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1154:1778417743.2860651 [start_skill-26] [0m[INFO] [1778417743.285690396] [nao_say_skill]: [turn:I can see two blueberries in the] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1344:1778417842.0840290 [start_skill-26] [0m[INFO] [1778417842.083637874] [nao_say_skill]: [turn:No, I did not see any people in] TTS_ACCEPTED | goal accepted[0m
1346:1778417842.0932231 [start_skill-26] [0m[INFO] [1778417842.092927841] [nao_say_skill]: [turn:No, I did not see any people in] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=46[0m
1348:1778417842.0979106 [start_skill-26] [33m[WARN] [1778417842.096731905] [nao_say_skill]: [turn:No, I did not see any people in] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1350:1778417842.1059709 [start_skill-26] [0m[INFO] [1778417842.105504239] [nao_say_skill]: [turn:No, I did not see any people in] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
```

### D) Human evidence excerpts

```
139:1778417036.3247180 [knowledge_core-5] 	- anonymous_person_daeba rdf:type Human[0m
142:1778417036.6674154 [knowledge_core-5] 	- myself sees anonymous_person_daeba[0m
146:1778417036.9357853 [knowledge_core-5] 	- anonymous_person_daeba sees myself[0m
238:1778417163.1188693 [knowledge_core-5] 	- anonymous_person_daeba rdf:type Human[0m
241:1778417163.7250426 [knowledge_core-5] 	- myself sees anonymous_person_daeba[0m
244:1778417164.0011632 [knowledge_core-5] 	- anonymous_person_aibac rdf:type Human[0m
247:1778417164.1645081 [knowledge_core-5] 	- anonymous_person_aibac sees myself[0m
250:1778417164.4631674 [knowledge_core-5] 	- myself sees anonymous_person_aibac[0m
253:1778417164.7842839 [knowledge_core-5] 	- anonymous_person_aibac sees myself[0m
256:1778417165.0769172 [knowledge_core-5] 	- anonymous_person_aibac rdf:type Human[0m
259:1778417165.3335202 [knowledge_core-5] 	- myself sees anonymous_person_aibac[0m
265:1778417178.3449283 [knowledge_core-5] 	- anonymous_person_hajcf rdf:type Human[0m
268:1778417178.5958271 [knowledge_core-5] 	- myself sees anonymous_person_hajcf[0m
271:1778417178.8852015 [knowledge_core-5] 	- anonymous_person_hajcf sees myself[0m
274:1778417179.4040310 [knowledge_core-5] 	- anonymous_person_hajcf sees myself[0m
277:1778417179.7406495 [knowledge_core-5] 	- anonymous_person_hajcf sees myself[0m
280:1778417180.4021442 [knowledge_core-5] 	- anonymous_person_hajcf rdf:type Human[0m
283:1778417180.6388836 [knowledge_core-5] 	- myself sees anonymous_person_hajcf[0m
286:1778417182.1384943 [knowledge_core-5] 	- anonymous_person_cgfbi rdf:type Human[0m
289:1778417182.4209745 [knowledge_core-5] 	- myself sees anonymous_person_cgfbi[0m
292:1778417182.7179646 [knowledge_core-5] 	- anonymous_person_cgfbi sees myself[0m
295:1778417183.1370032 [knowledge_core-5] 	- anonymous_person_cgfbi rdf:type Human[0m
298:1778417183.6099408 [knowledge_core-5] 	- myself sees anonymous_person_cgfbi[0m
301:1778417183.9839480 [knowledge_core-5] 	- anonymous_person_hjdbd rdf:type Human[0m
304:1778417184.2664163 [knowledge_core-5] 	- myself sees anonymous_person_hjdbd[0m
307:1778417184.5249274 [knowledge_core-5] 	- anonymous_person_hjdbd sees myself[0m
310:1778417185.1437893 [knowledge_core-5] 	- anonymous_person_hjdbd rdf:type Human[0m
313:1778417185.4173951 [knowledge_core-5] 	- myself sees anonymous_person_hjdbd[0m
316:1778417185.9502211 [knowledge_core-5] 	- anonymous_person_fadfa rdf:type Human[0m
319:1778417186.2460768 [knowledge_core-5] 	- myself sees anonymous_person_fadfa[0m
322:1778417186.5143094 [knowledge_core-5] 	- anonymous_person_fadfa sees myself[0m
325:1778417188.0559130 [knowledge_core-5] 	- anonymous_person_fadfa rdf:type Human[0m
328:1778417188.3519678 [knowledge_core-5] 	- myself sees anonymous_person_fadfa[0m
331:1778417189.3646984 [knowledge_core-5] 	- anonymous_person_gihgb rdf:type Human[0m
334:1778417189.8324721 [knowledge_core-5] 	- myself sees anonymous_person_gihgb[0m
337:1778417190.1008146 [knowledge_core-5] 	- anonymous_person_gihgb sees myself[0m
340:1778417190.5003958 [knowledge_core-5] 	- anonymous_person_gihgb rdf:type Human[0m
343:1778417190.7220790 [knowledge_core-5] 	- myself sees anonymous_person_gihgb[0m
349:1778417217.8860898 [knowledge_core-5] 	- anonymous_person_ejdba rdf:type Human[0m
353:1778417218.1695657 [knowledge_core-5] 	- myself sees anonymous_person_ejdba[0m
356:1778417219.2355678 [knowledge_core-5] 	- anonymous_person_ejdba rdf:type Human[0m
359:1778417219.4528637 [knowledge_core-5] 	- myself sees anonymous_person_ejdba[0m
362:1778417222.8898330 [knowledge_core-5] 	- anonymous_person_gighj rdf:type Human[0m
365:1778417223.1770890 [knowledge_core-5] 	- myself sees anonymous_person_gighj[0m
368:1778417223.6477134 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
371:1778417223.8620100 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
374:1778417224.1005847 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
378:1778417225.2390807 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
381:1778417225.9164622 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
384:1778417226.4628222 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
387:1778417227.0620210 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
391:1778417229.3976150 [knowledge_core-5] 	- anonymous_person_gighj rdf:type Human[0m
394:1778417229.6500964 [knowledge_core-5] 	- myself sees anonymous_person_gighj[0m
412:1778417322.9596457 [knowledge_core-5] 	- anonymous_person_afefe rdf:type Human[0m
415:1778417323.2394164 [knowledge_core-5] 	- myself sees anonymous_person_afefe[0m
418:1778417323.7078383 [knowledge_core-5] 	- anonymous_person_afefe sees myself[0m
421:1778417335.8656094 [knowledge_core-5] 	- anonymous_person_afefe sees myself[0m
424:1778417336.1811364 [knowledge_core-5] 	- anonymous_person_afefe sees myself[0m
427:1778417336.9258523 [knowledge_core-5] 	- anonymous_person_afefe rdf:type Human[0m
430:1778417337.2214062 [knowledge_core-5] 	- myself sees anonymous_person_afefe[0m
433:1778417337.9243748 [knowledge_core-5] 	- anonymous_person_hfccj rdf:type Human[0m
436:1778417338.2339911 [knowledge_core-5] 	- myself sees anonymous_person_hfccj[0m
439:1778417338.4956710 [knowledge_core-5] 	- anonymous_person_hfccj sees myself[0m
442:1778417342.1500051 [knowledge_core-5] 	- anonymous_person_hfccj rdf:type Human[0m
445:1778417342.4349079 [knowledge_core-5] 	- myself sees anonymous_person_hfccj[0m
452:1778417369.4761932 [knowledge_core-5] 	- anonymous_person_eicdd rdf:type Human[0m
455:1778417369.7409396 [knowledge_core-5] 	- myself sees anonymous_person_eicdd[0m
458:1778417370.0032604 [knowledge_core-5] 	- anonymous_person_eicdd sees myself[0m
461:1778417372.9467223 [knowledge_core-5] 	- anonymous_person_eicdd sees myself[0m
464:1778417373.3824298 [knowledge_core-5] 	- anonymous_person_eicdd sees myself[0m
475:1778417398.8812404 [knowledge_core-5] 	- anonymous_person_eicdd rdf:type Human[0m
478:1778417399.2072029 [knowledge_core-5] 	- myself sees anonymous_person_eicdd[0m
481:1778417399.7515604 [knowledge_core-5] 	- anonymous_person_ahhbi rdf:type Human[0m
484:1778417400.0514309 [knowledge_core-5] 	- myself sees anonymous_person_ahhbi[0m
487:1778417400.3433197 [knowledge_core-5] 	- anonymous_person_ahhbi sees myself[0m
490:1778417400.8598063 [knowledge_core-5] 	- anonymous_person_ahhbi rdf:type Human[0m
493:1778417401.4279068 [knowledge_core-5] 	- myself sees anonymous_person_ahhbi[0m
528:1778417419.1293139 [knowledge_core-5] 	- anonymous_person_ifchh rdf:type Human[0m
531:1778417419.4659066 [knowledge_core-5] 	- myself sees anonymous_person_ifchh[0m
534:1778417419.9705331 [knowledge_core-5] 	- anonymous_person_ifchh sees myself[0m
537:1778417463.6845465 [knowledge_core-5] 	- anonymous_person_ifchh rdf:type Human[0m
540:1778417464.1689825 [knowledge_core-5] 	- myself sees anonymous_person_ifchh[0m
544:1778417470.4291604 [knowledge_core-5] 	- anonymous_person_gacga rdf:type Human[0m
547:1778417470.7074711 [knowledge_core-5] 	- myself sees anonymous_person_gacga[0m
550:1778417470.9962504 [knowledge_core-5] 	- anonymous_person_gacga sees myself[0m
554:1778417473.5621595 [knowledge_core-5] 	- anonymous_person_gacga rdf:type Human[0m
557:1778417474.0413039 [knowledge_core-5] 	- myself sees anonymous_person_gacga[0m
560:1778417479.4671743 [knowledge_core-5] 	- anonymous_person_jfabi rdf:type Human[0m
563:1778417479.7524006 [knowledge_core-5] 	- myself sees anonymous_person_jfabi[0m
567:1778417480.0557024 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
570:1778417487.2370341 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
573:1778417487.6920741 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
576:1778417487.9739678 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
579:1778417490.1649766 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
582:1778417490.9423521 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
585:1778417491.3216364 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
588:1778417495.4539032 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
591:1778417495.6986957 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
594:1778417500.5474775 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
597:1778417500.8514407 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
600:1778417501.9400282 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
603:1778417502.2106571 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
606:1778417504.0596228 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
609:1778417504.3009682 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
612:1778417504.6220891 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
615:1778417504.8968902 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
618:1778417519.9381247 [knowledge_core-5] 	- anonymous_person_jfabi rdf:type Human[0m
621:1778417520.2426057 [knowledge_core-5] 	- myself sees anonymous_person_jfabi[0m
624:1778417520.8789101 [knowledge_core-5] 	- anonymous_person_jcedc rdf:type Human[0m
627:1778417521.1842818 [knowledge_core-5] 	- myself sees anonymous_person_jcedc[0m
630:1778417521.4811313 [knowledge_core-5] 	- anonymous_person_jcedc sees myself[0m
673:1778417583.5041304 [knowledge_core-5] 	- anonymous_person_jcedc rdf:type Human[0m
676:1778417583.7566528 [knowledge_core-5] 	- myself sees anonymous_person_jcedc[0m
679:1778417584.7605717 [knowledge_core-5] 	- anonymous_person_eache rdf:type Human[0m
682:1778417585.0538833 [knowledge_core-5] 	- myself sees anonymous_person_eache[0m
685:1778417585.5595238 [knowledge_core-5] 	- anonymous_person_eache sees myself[0m
688:1778417585.8569636 [knowledge_core-5] 	- anonymous_person_eache rdf:type Human[0m
691:1778417586.1364641 [knowledge_core-5] 	- myself sees anonymous_person_eache[0m
695:1778417586.6814768 [knowledge_core-5] 	- anonymous_person_jbehb rdf:type Human[0m
698:1778417586.9963515 [knowledge_core-5] 	- myself sees anonymous_person_jbehb[0m
701:1778417587.2666187 [knowledge_core-5] 	- anonymous_person_jbehb sees myself[0m
705:1778417602.8842762 [knowledge_core-5] 	- anonymous_person_jbehb rdf:type Human[0m
708:1778417603.1725261 [knowledge_core-5] 	- myself sees anonymous_person_jbehb[0m
713:1778417611.9898655 [knowledge_core-5] 	- anonymous_person_hfeia rdf:type Human[0m
716:1778417612.2766011 [knowledge_core-5] 	- myself sees anonymous_person_hfeia[0m
719:1778417612.7449522 [knowledge_core-5] 	- anonymous_person_hfeia sees myself[0m
722:1778417614.6354024 [knowledge_core-5] 	- anonymous_person_hfeia sees myself[0m
725:1778417615.9786081 [knowledge_core-5] 	- anonymous_person_hfeia rdf:type Human[0m
729:1778417616.2565567 [knowledge_core-5] 	- myself sees anonymous_person_hfeia[0m
732:1778417617.1406116 [knowledge_core-5] 	- anonymous_person_icghb rdf:type Human[0m
735:1778417617.6658697 [knowledge_core-5] 	- myself sees anonymous_person_icghb[0m
738:1778417617.9609354 [knowledge_core-5] 	- anonymous_person_icghb sees myself[0m
741:1778417618.2416084 [knowledge_core-5] 	- anonymous_person_icghb rdf:type Human[0m
744:1778417618.4605346 [knowledge_core-5] 	- myself sees anonymous_person_icghb[0m
747:1778417618.9702404 [knowledge_core-5] 	- anonymous_person_ecaab rdf:type Human[0m
750:1778417619.2591605 [knowledge_core-5] 	- myself sees anonymous_person_ecaab[0m
753:1778417619.8062773 [knowledge_core-5] 	- anonymous_person_ecaab sees myself[0m
756:1778417627.1480060 [knowledge_core-5] 	- anonymous_person_ecaab rdf:type Human[0m
759:1778417627.4683101 [knowledge_core-5] 	- myself sees anonymous_person_ecaab[0m
762:1778417629.6018326 [knowledge_core-5] 	- anonymous_person_bgfha rdf:type Human[0m
765:1778417629.8854928 [knowledge_core-5] 	- myself sees anonymous_person_bgfha[0m
768:1778417630.4710348 [knowledge_core-5] 	- anonymous_person_bgfha sees myself[0m
771:1778417630.6756229 [knowledge_core-5] 	- anonymous_person_bgfha rdf:type Human[0m
774:1778417630.9854653 [knowledge_core-5] 	- anonymous_person_egcid rdf:type Human[0m
777:1778417631.1834488 [knowledge_core-5] 	- myself sees anonymous_person_bgfha[0m
780:1778417631.5216565 [knowledge_core-5] 	- myself sees anonymous_person_egcid[0m
783:1778417631.8387606 [knowledge_core-5] 	- anonymous_person_egcid rdf:type Human[0m
786:1778417632.3289132 [knowledge_core-5] 	- myself sees anonymous_person_egcid[0m
790:1778417638.2360475 [knowledge_core-5] 	- anonymous_person_cdccc rdf:type Human[0m
793:1778417638.5066929 [knowledge_core-5] 	- myself sees anonymous_person_cdccc[0m
796:1778417638.7712479 [knowledge_core-5] 	- anonymous_person_cdccc sees myself[0m
807:1778417645.7265277 [knowledge_core-5] 	- anonymous_person_cdccc rdf:type Human[0m
810:1778417645.9964452 [knowledge_core-5] 	- myself sees anonymous_person_cdccc[0m
813:1778417649.0183103 [knowledge_core-5] 	- anonymous_person_cdaad rdf:type Human[0m
816:1778417649.3010483 [knowledge_core-5] 	- myself sees anonymous_person_cdaad[0m
819:1778417649.6110959 [knowledge_core-5] 	- anonymous_person_cdaad sees myself[0m
850:1778417660.4162426 [knowledge_core-5] 	- anonymous_person_cdaad rdf:type Human[0m
853:1778417660.6973903 [knowledge_core-5] 	- myself sees anonymous_person_cdaad[0m
859:1778417663.3888228 [knowledge_core-5] 	- anonymous_person_effcb rdf:type Human[0m
862:1778417663.6751866 [knowledge_core-5] 	- myself sees anonymous_person_effcb[0m
865:1778417664.2551641 [knowledge_core-5] 	- anonymous_person_effcb sees myself[0m
868:1778417665.1598899 [knowledge_core-5] 	- anonymous_person_effcb rdf:type Human[0m
871:1778417665.4795995 [knowledge_core-5] 	- myself sees anonymous_person_effcb[0m
874:1778417665.8897209 [knowledge_core-5] 	- anonymous_person_iiaeg rdf:type Human[0m
877:1778417666.1822305 [knowledge_core-5] 	- myself sees anonymous_person_iiaeg[0m
880:1778417666.7342651 [knowledge_core-5] 	- anonymous_person_iiaeg sees myself[0m
913:1778417673.3878026 [knowledge_core-5] 	- anonymous_person_iiaeg rdf:type Human[0m
916:1778417673.7106414 [knowledge_core-5] 	- myself sees anonymous_person_iiaeg[0m
924:1778417674.4518366 [knowledge_core-5] 	- anonymous_person_aabjj rdf:type Human[0m
927:1778417674.7278712 [knowledge_core-5] 	- myself sees anonymous_person_aabjj[0m
930:1778417675.2220490 [knowledge_core-5] 	- anonymous_person_aabjj sees myself[0m
953:1778417679.2648954 [knowledge_core-5] 	- anonymous_person_aabjj rdf:type Human[0m
956:1778417679.6108377 [knowledge_core-5] 	- anonymous_person_jaidh rdf:type Human[0m
959:1778417679.8237495 [knowledge_core-5] 	- myself sees anonymous_person_aabjj[0m
962:1778417680.0981860 [knowledge_core-5] 	- myself sees anonymous_person_jaidh[0m
970:1778417680.6226161 [knowledge_core-5] 	- anonymous_person_jaidh sees myself[0m
983:1778417683.0459013 [knowledge_core-5] 	- anonymous_person_jaidh rdf:type Human[0m
986:1778417683.3388314 [knowledge_core-5] 	- myself sees anonymous_person_jaidh[0m
989:1778417684.1860707 [knowledge_core-5] 	- anonymous_person_jdifh rdf:type Human[0m
992:1778417684.4482083 [knowledge_core-5] 	- myself sees anonymous_person_jdifh[0m
995:1778417684.9879274 [knowledge_core-5] 	- anonymous_person_jdifh sees myself[0m
1001:1778417692.3766918 [knowledge_core-5] 	- anonymous_person_jdifh rdf:type Human[0m
1004:1778417692.6615543 [knowledge_core-5] 	- myself sees anonymous_person_jdifh[0m
1007:1778417694.3069735 [knowledge_core-5] 	- anonymous_person_ehcfa rdf:type Human[0m
1010:1778417694.8225157 [knowledge_core-5] 	- myself sees anonymous_person_ehcfa[0m
1013:1778417695.0853393 [knowledge_core-5] 	- anonymous_person_ehcfa sees myself[0m
1039:1778417707.6837285 [knowledge_core-5] 	- anonymous_person_ehcfa rdf:type Human[0m
1042:1778417707.9851394 [knowledge_core-5] 	- anonymous_person_fffig rdf:type Human[0m
1045:1778417708.1882308 [knowledge_core-5] 	- myself sees anonymous_person_ehcfa[0m
1048:1778417708.6135938 [knowledge_core-5] 	- myself sees anonymous_person_fffig[0m
1051:1778417708.9024129 [knowledge_core-5] 	- anonymous_person_fffig sees myself[0m
1198:1778417790.1944666 [knowledge_core-5] 	- anonymous_person_fffig rdf:type Human[0m
1201:1778417790.4470675 [knowledge_core-5] 	- myself sees anonymous_person_fffig[0m
1204:1778417792.8800001 [knowledge_core-5] 	- anonymous_person_fidfb rdf:type Human[0m
1207:1778417793.1921191 [knowledge_core-5] 	- myself sees anonymous_person_fidfb[0m
1210:1778417793.4723518 [knowledge_core-5] 	- anonymous_person_fidfb sees myself[0m
1228:1778417798.6783323 [knowledge_core-5] 	- anonymous_person_fidfb rdf:type Human[0m
1231:1778417798.9508877 [knowledge_core-5] 	- myself sees anonymous_person_fidfb[0m
1234:1778417799.5396140 [knowledge_core-5] 	- anonymous_person_jeigi rdf:type Human[0m
1237:1778417800.0877681 [knowledge_core-5] 	- myself sees anonymous_person_jeigi[0m
1240:1778417800.3910365 [knowledge_core-5] 	- anonymous_person_jeigi sees myself[0m
1309:1778417837.0421984 [knowledge_core-5] 	- anonymous_person_jeigi rdf:type Human[0m
1312:1778417837.3330133 [knowledge_core-5] 	- myself sees anonymous_person_jeigi[0m
1315:1778417837.5964403 [knowledge_core-5] 	- anonymous_person_jajdc rdf:type Human[0m
1318:1778417837.8957138 [knowledge_core-5] 	- myself sees anonymous_person_jajdc[0m
1321:1778417838.4378748 [knowledge_core-5] 	- anonymous_person_jajdc sees myself[0m
1412:1778417857.3941677 [knowledge_core-5] 	- anonymous_person_jajdc rdf:type Human[0m
1415:1778417857.6921728 [knowledge_core-5] 	- myself sees anonymous_person_jajdc[0m
1418:1778417858.1838212 [knowledge_core-5] 	- anonymous_person_ibica rdf:type Human[0m
1429:1778417858.7707367 [knowledge_core-5] 	- myself sees anonymous_person_ibica[0m
1432:1778417859.0203080 [knowledge_core-5] 	- anonymous_person_ibica sees myself[0m
1545:1778417876.8469357 [knowledge_core-5] 	- anonymous_person_ibica rdf:type Human[0m
1548:1778417877.0804863 [knowledge_core-5] 	- myself sees anonymous_person_ibica[0m
1551:1778417877.3500156 [knowledge_core-5] 	- anonymous_person_jbbii rdf:type Human[0m
1559:1778417877.7331622 [knowledge_core-5] 	- myself sees anonymous_person_jbbii[0m
1562:1778417878.2021854 [knowledge_core-5] 	- anonymous_person_jbbii sees myself[0m
1743:1778417935.1277258 [knowledge_core-5] 	- anonymous_person_jbbii rdf:type Human[0m
1747:1778417935.3439162 [knowledge_core-5] 	- myself sees anonymous_person_jbbii[0m
1750:1778417935.9707706 [knowledge_core-5] 	- anonymous_person_gcaji rdf:type Human[0m
1753:1778417936.2069404 [knowledge_core-5] 	- myself sees anonymous_person_gcaji[0m
```

## Full Latest launch.log (Raw)

Source: /root/.ros/log/2026-05-10-12-43-42-497688-juanbeck-Zenbook-UX425QA-UM425QA-2144/launch.log

```
1778417022.5146177 [INFO] [launch]: All log files can be found below /root/.ros/log/2026-05-10-12-43-42-497688-juanbeck-Zenbook-UX425QA-UM425QA-2144
1778417022.5147865 [INFO] [launch]: Default logging verbosity is set to INFO
1778417022.6871521 [INFO] [launch.user]: [STACK] nao_chatbot launch | chatbot_model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ planner_model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ planner_mode=true chatbot_url=http://10.7.138.215:8004/v1/chat/completions planner_url=http://10.7.138.215:8004 planner_gate=true scan=enabled
1778417022.6873305 [INFO] [launch.user]: [STACK] enabled nodes | chatbot_llm=true dialogue_manager=true planner_llm=true nao_orchestrator=true scene_grounding=true object_detection=true
1778417022.6873965 [INFO] [launch.user]: [LLM PREFLIGHT] launch policy | chatbot_required=true planner_required=true chatbot_timeout=60.0 planner_timeout=60.0
1778417022.6874409 [INFO] [launch.user]: [STACK] lifecycle sequencing | chatbot_llm configures before dialogue_manager; planner_llm and executor seams start independently
1778417022.6912990 [INFO] [launch.user]: start_naoqi_driver is enabled together with interaction_sim perception. This keeps simulator perception on /camera/image_raw while the real robot camera stays on /camera/front/image_raw. Prefer start_interaction_sim_perception:=false for robot-camera validation, or override object_detection_input_image_topic and hri_visualization_image_topic to /camera/front/image_raw.
1778417022.7317264 [INFO] [launch.user]: [33mWARNING: user configuration path /root/.pal/config does not exist. User overrides will not be available.[0m
1778417022.7318697 [INFO] [launch.user]: [1;36mLoaded configuration for <knowledge_core>:[0m
- User overrides (from higher to lower precedence, listing the used presets indented):
	- (none)
- System configuration (from higher to lower precedence, listing the used presets indented):
	- /opt/ros/jazzy/share/knowledge_core/config/00-defaults.yaml
1778417022.7319324 [INFO] [launch.user]: [36mParameters[0m (if "overridable", can be overridden with <full.name>:=<value>):
- default_kb[33m [overridable][0m: ontology://oro/oro.owl

1778417022.7336187 [INFO] [launch.user]: [33mWARNING: user configuration path /root/.pal/config does not exist. User overrides will not be available.[0m
1778417022.7337241 [INFO] [launch.user]: [31mERROR: configuration files found, but node knowledge_viewer has no entry![0m
I looked into the following configuration files: ['/opt/ros/jazzy/share/knowledge_core/config/00-defaults.yaml']
 Returning empty parameters/remappings/arguments
1778417022.7365839 [INFO] [launch.user]: Launching emorobcare_cv_object_detection. Keep its package-local config.yaml aligned with this demo path: use_knowledge_base=false, use_human_radar=false, and draw_image=true when debug overlays are needed.
1778417022.9059484 [INFO] [launch.user]: [33mWARNING: user configuration path /root/.pal/config does not exist. User overrides will not be available.[0m
1778417022.9062290 [INFO] [launch.user]: [1;36mLoaded configuration for <hri_visualization>:[0m
- User overrides (from higher to lower precedence, listing the used presets indented):
	- (none)
- System configuration (from higher to lower precedence, listing the used presets indented):
	- /opt/ros/jazzy/share/hri_visualization/config/00-defaults.yml
1778417022.9063413 [INFO] [launch.user]: [36mParameters[0m (if "overridable", can be overridden with <full.name>:=<value>):
- compressed_input[33m [overridable][0m: True
- compressed_output[33m [overridable][0m: True
- funny_names[33m [overridable][0m: False

1778417023.0018840 [INFO] [launch.user]: interaction_sim perception + tools layer enabled. This keeps simulator utilities separate from chatbot_llm, dialogue_manager, and knowledge_core.
1778417023.0020299 [INFO] [launch.user]: The nao_chatbot sim profile loads a debug-ready rqt perspective with /debug/object_detection prewired into the spare image view.
1778417023.0021372 [INFO] [launch.user]: interaction_sim expressive_face is disabled.
1778417023.0022261 [INFO] [launch.user]: interaction_sim HRI perception log profile is quiet.
1778417023.1782396 [INFO] [robot_speech_debug-3]: process started with pid [2162]
1778417023.1789072 [INFO] [ros2-4]: process started with pid [2163]
1778417023.1793516 [INFO] [knowledge_core-5]: process started with pid [2164]
1778417023.1881890 [INFO] [knowledge_viewer-6]: process started with pid [2165]
1778417023.2038543 [INFO] [add_analyzer-7]: process started with pid [2167]
1778417023.2042882 [INFO] [object_detector_node-8]: process started with pid [2168]
1778417023.2046142 [INFO] [hri_person_manager-9]: process started with pid [2169]
1778417023.2049327 [INFO] [add_analyzer-10]: process started with pid [2170]
1778417023.2052326 [INFO] [hri_face_detect_yunet-11]: process started with pid [2172]
1778417023.2054904 [INFO] [add_analyzer-12]: process started with pid [2173]
1778417023.2095761 [INFO] [hri_emotion_recognizer-13]: process started with pid [2174]
1778417023.2099710 [INFO] [add_analyzer-14]: process started with pid [2176]
1778417023.2138090 [INFO] [add_analyzer-17]: process started with pid [2179]
1778417023.2141502 [INFO] [start_node-22]: process started with pid [2184]
1778417023.2235746 [INFO] [start_manager-23]: process started with pid [2185]
1778417023.2239001 [INFO] [run_app-24]: process started with pid [2186]
1778417023.2242577 [INFO] [start_skill-26]: process started with pid [2188]
1778417023.2245529 [INFO] [replay_motion_skill_server_node-28]: process started with pid [2192]
1778417023.2248051 [INFO] [head_motion_skill_server_node-29]: process started with pid [2194]
1778417023.2250545 [INFO] [nao_posture_bridge_node-30]: process started with pid [2197]
1778417023.2254982 [INFO] [start_skill-31]: process started with pid [2200]
1778417023.2289007 [INFO] [start_node-33]: process started with pid [2211]
1778417023.3025672 [INFO] [naoqi_driver_node-1]: process started with pid [2160]
1778417023.3032017 [INFO] [bash-2]: process started with pid [2161]
1778417023.3038237 [INFO] [gscam_node-15]: process started with pid [2177]
1778417023.3086188 [INFO] [visualization-16]: process started with pid [2178]
1778417023.3099935 [INFO] [static_transform_publisher-18]: process started with pid [2180]
1778417023.3113873 [INFO] [static_transform_publisher-19]: process started with pid [2181]
1778417023.3116016 [INFO] [rosbridge_websocket-20]: process started with pid [2182]
1778417023.3117783 [INFO] [rosapi_node-21]: process started with pid [2183]
1778417023.3154752 [INFO] [bash-25]: process started with pid [2187]
1778417023.3157589 [INFO] [bash-27]: process started with pid [2191]
1778417023.3197799 [INFO] [bash-32]: process started with pid [2201]
1778417023.5361564 [knowledge_viewer-6] knowledge_viewer is running. Open http://localhost:8010 in a webbrowser to start exploring the knowledge base.
1778417024.5538402 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:44,546] Using RDFlib 6.1.1[0m
1778417024.5552185 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:44,554] reasonable OWL2 RL reasoner available. Running with reasoning enabled.[0m
1778417024.6050768 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:44,599] Materialisation performed by reasoner in 14.1ms[0m
1778417024.6121819 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:44,600] Initializing the ROS node...[0m
1778417025.4234433 [start_skill-26] [0m[INFO] [1778417025.421270066] [nao_say_skill]: nao_say_skill created; waiting for lifecycle configure[0m
1778417025.4728649 [start_manager-23] [0m[INFO] [1778417025.469635960] [dialogue_manager]: Dialogue Manager node created, awaiting configuration.[0m
1778417026.1574881 [INFO] [start_node-34]: process started with pid [2642]
1778417026.4869633 [knowledge_core-5] [0m[INFO] [1778417026.483621294] [kb.knowledge_core]: Default knowledge base file: ontology://oro/oro.owl[0m
1778417026.5111094 [knowledge_core-5] [0m[INFO] [1778417026.504583892] [kb.knowledge_core]: Loading default knowledge base [ontology://oro/oro.owl] from /opt/ros/jazzy/share/oro/ontologies/oro.owl[0m
1778417026.5216246 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:46,507] Adding default ontology </opt/ros/jazzy/share/oro/ontologies/oro.owl>[0m
1778417026.5305128 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:46,508] Loading file </opt/ros/jazzy/share/oro/ontologies/oro.owl> in model <default>[0m
1778417026.9985180 [run_app-24] [0m[INFO] [1778417026.998072732] [nao_orchestrator]: nao_orchestrator created; waiting for lifecycle configure[0m
1778417027.5241792 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:47,523] Materialisation performed by reasoner in 181.8ms[0m
1778417027.6074836 [start_skill-31] [0m[INFO] [1778417027.607122160] [nao_look_at]: nao_look_at created; waiting for lifecycle configure[0m
1778417027.8186345 [start_node-22] [0m[INFO] [1778417027.814135810] [chatbot_llm]: Chatbot backend created, awaiting lifecycle configuration.[0m
1778417027.9779742 [knowledge_core-5] [0m[INFO] [1778417027.976714361] [kb.knowledge_core]:
1778417027.9786057 [knowledge_core-5] KnowledgeCore
1778417027.9795902 [knowledge_core-5] =============
1778417027.9806094 [knowledge_core-5]
1778417027.9819713 [knowledge_core-5] Knowledge base started.
1778417027.9826512 [knowledge_core-5]
1778417027.9846187 [knowledge_core-5] Available topics:
1778417027.9849353 [knowledge_core-5] - /kb/add_fact [std_msgs/String]
1778417027.9852951 [knowledge_core-5] - /kb/remove_fact [std_msgs/String]
1778417027.9855778 [knowledge_core-5] - /kb/active_concepts [kb_msgs/ActiveConcepts]
1778417027.9857666 [knowledge_core-5] - /kb/events/<id> [std_msgs/String] for
1778417027.9859471 [knowledge_core-5]   each subscribed event
1778417027.9861248 [knowledge_core-5]
1778417027.9862981 [knowledge_core-5] Available services:
1778417027.9864852 [knowledge_core-5] - /kb/manage [kb_msgs/Manage]
1778417027.9881113 [knowledge_core-5] - /kb/revise [kb_msgs/Revise]
1778417027.9883730 [knowledge_core-5] - /kb/query [kb_msgs/Query]
1778417027.9893277 [knowledge_core-5] - /kb/about [kb_msgs/About]
1778417027.9905324 [knowledge_core-5] - /kb/label [kb_msgs/About]
1778417027.9907582 [knowledge_core-5] - /kb/details [kb_msgs/About]
1778417027.9911788 [knowledge_core-5] - /kb/lookup [kb_msgs/Lookup]
1778417027.9956355 [knowledge_core-5] - /kb/sparql [kb_msgs/Sparql]
1778417027.9958386 [knowledge_core-5] - /kb/events [kb_msgs/KbEvent]
1778417027.9959912 [knowledge_core-5]
1778417027.9961271 [knowledge_core-5] [0m
1778417027.9971290 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:47,981] Starting to serve at port 6969...[0m
1778417028.0003927 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:47,981] KnowledgeCore ready[0m
1778417028.2842200 [start_node-22] [0m[INFO] [1778417028.278837214] [chatbot_llm]: [LLM PREFLIGHT] chatbot starting | models=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ timeout=60.0s required=True attempts=3 realistic=True[0m
1778417029.2025340 [start_node-22] [0m[INFO] [1778417029.193339257] [chatbot_llm]: [LLM PREFLIGHT] chatbot model ready | model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ attempt=1/3[0m
1778417029.3346720 [start_node-22] [0m[INFO] [1778417029.330953427] [chatbot_llm]: [STACK READY] chatbot_llm configured | server_url=http://10.7.138.215:8004/v1/chat/completions model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ intent_model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ intent_mode=llm_with_rules_fallback skill_catalog=11 planner_mode=True planner_topic=/nao_orchestrator/planner_request[0m
1778417029.6254685 [start_node-22] [0m[INFO] [1778417029.578332454] [chatbot_llm]: Available LLM models: QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ[0m
1778417030.2552340 [start_node-22] [0m[INFO] [1778417030.254777022] [chatbot_llm]: chatbot_llm is active and serving ~/start_dialogue and ~/dialogue_interaction[0m
1778417030.3240435 [start_manager-23] [0m[INFO] [1778417030.321185806] [dialogue_manager]: Configuring Dialogue Manager...[0m
1778417030.5569439 [start_manager-23] [0m[INFO] [1778417030.556405487] [dialogue_manager]: [TTS] Created action client: tts_engine/tts[0m
1778417030.5876470 [start_manager-23] [0m[INFO] [1778417030.587258005] [dialogue_manager]: [CHATBOT] Created action client: chatbot_llm/start_dialogue[0m
1778417030.5925329 [start_manager-23] [0m[INFO] [1778417030.592194234] [dialogue_manager]: [CHATBOT] Created service client: chatbot_llm/dialogue_interaction[0m
1778417030.6968110 [start_manager-23] [0m[INFO] [1778417030.696487674] [dialogue_manager]: [SKILLS] Created /skill/chat action server[0m
1778417030.7869327 [start_manager-23] [0m[INFO] [1778417030.786133064] [dialogue_manager]: [SKILLS] Created /skill/ask action server[0m
1778417030.8962474 [start_manager-23] [0m[INFO] [1778417030.895043396] [dialogue_manager]: [SKILLS] Created /skill/say action server[0m
1778417030.9341588 [start_manager-23] [0m[INFO] [1778417030.897858632] [dialogue_manager]: Dialogue Manager configured.[0m
1778417031.0525582 [start_manager-23] [0m[INFO] [1778417031.051995504] [dialogue_manager]: Activating Dialogue Manager...[0m
1778417031.0902567 [start_manager-23] [0m[INFO] [1778417031.089888441] [dialogue_manager]: [SPEECH] Subscribed to /humans/voices/tracked[0m
1778417031.1086397 [start_manager-23] [0m[INFO] [1778417031.108280262] [dialogue_manager]: [ACTIVATE] Planner dialogue acts subscribed on "/planner/dialogue_act"[0m
1778417031.1340954 [start_manager-23] [0m[INFO] [1778417031.125324273] [dialogue_manager]: [DEFAULT CHAT] Starting with role="__default__", configuration=""[0m
1778417031.1669424 [start_manager-23] [0m[INFO] [1778417031.142799482] [dialogue_manager]: Dialogue Manager activated.[0m
1778417031.2058432 [INFO] [launch.user]: [STACK READY] dialogue path active | chatbot_llm=true planner_llm=true planner_mode=true dialogue_manager=/dialogue_manager
1778417031.2567875 [start_node-22] [0m[INFO] [1778417031.187271631] [chatbot_llm]: Started dialogue role=__default__ id=3444e73b[0m
1778417031.2629590 [start_manager-23] [0m[INFO] [1778417031.201799971] [dialogue_manager]: [DEFAULT CHAT] Started successfully, internal_id=973c0f1e-a654-42da-b888-6b3e1078600d, chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189[0m
1778417032.1042693 [start_skill-31] /opt/ros/jazzy/lib/python3.12/site-packages/rclpy/service.py:78: RuntimeWarning: failed to send response (timeout): client will not receive response, at ./src/rmw_response.cpp:153, at ./src/rcl/service.c:400
1778417032.1046050 [start_skill-31]   self.__service.service_send_response(response, header.request_id)
1778417035.9156938 [start_skill-26] [0m[INFO] [1778417035.915262345] [nao_say_skill]: nao_say_skill configured | say:/nao/say tts:/tts_engine/tts backend:(speech_topic_fallback) debug_tts:/debug/say speech:/speech debug:/debug/nao_say/speech[0m
1778417035.9167931 [start_skill-26] [33m[WARN] [1778417035.916611927] [nao_say_skill]: nao_say_skill is running without a downstream TTS backend; robot speech depends on an active subscriber to "/speech"[0m
1778417036.3237131 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:56,322] Updating ['default'] with:
1778417036.3247180 [knowledge_core-5] 	- anonymous_person_daeba rdf:type Human[0m
1778417036.4429402 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:56,442] Materialisation performed by reasoner in 117.5ms[0m
1778417036.6671724 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:56,666] Updating ['default'] with:
1778417036.6674154 [knowledge_core-5] 	- myself sees anonymous_person_daeba[0m
1778417036.7844751 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:56,783] Materialisation performed by reasoner in 116.9ms[0m
1778417036.8749993 [run_app-24] [0m[INFO] [1778417036.874608942] [nao_orchestrator]: nao_orchestrator configured | intents:/intents legacy:/chatbot/intent say:/nao/say replay:/skill/replay_motion head:/skill/do_head_motion look:/skill/look_at planner_gate:/nao_orchestrator/planner_request->/planner/request[0m
1778417036.9354258 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:56,934] Updating ['default'] with:
1778417036.9357853 [knowledge_core-5] 	- anonymous_person_daeba sees myself[0m
1778417037.2802935 [knowledge_core-5] [37m[INFO] [2026-05-10 12:43:57,279] Materialisation performed by reasoner in 344.7ms[0m
1778417045.3314862 [run_app-24] [0m[INFO] [1778417045.330722693] [nao_orchestrator]: nao_orchestrator active[0m
1778417046.2343514 [start_skill-26] [0m[INFO] [1778417046.233151355] [nao_say_skill]: nao_say_skill active[0m
1778417050.0555301 [INFO] [bash-25]: process has finished cleanly [pid 2187]
1778417051.1638031 [INFO] [bash-27]: process has finished cleanly [pid 2191]
1778417069.1701324 [start_manager-23] [0m[INFO] [1778417069.169614664] [dialogue_manager]: [SPEECH] Subscribing to /humans/voices/anonymous_speaker/speech[0m
1778417069.6676443 [start_manager-23] [0m[INFO] [1778417069.667104732] [dialogue_manager]: [SPEECH INPUT] voice_id="anonymous_speaker": "Hey Pop!" (locale=, confidence=0.00)[0m
1778417069.6700690 [start_manager-23] [0m[INFO] [1778417069.669849035] [dialogue_manager]: [SPEECH INPUT] Using default dialogue 973c0f1e-a654-42da-b888-6b3e1078600d[0m
1778417069.6712985 [start_manager-23] [0m[INFO] [1778417069.671082183] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="anonymous_speaker", text="Hey Pop!"[0m
1778417069.6751742 [start_node-22] [0m[INFO] [1778417069.674756985] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:1 input=Hey Pop![0m
1778417069.7216365 [knowledge_core-5] [37m[INFO] [2026-05-10 12:44:29,719] Searching ['?entity', '?type'] in models {'default'} matching:
1778417069.7222016 [knowledge_core-5] 	- myself sees ?entity
1778417069.7224741 [knowledge_core-5] 	- ?entity rdf:type ?type[0m
1778417069.7351029 [knowledge_core-5] [37m[INFO] [2026-05-10 12:44:29,733] Found: [{'entity': 'anonymous_person_daeba', 'type': 'cyc:SolidTangibleThing'}, {'entity': 'anonymous_person_daeba', 'type': 'Human'}, {'entity': 'anonymous_person_daeba', 'type': 'foaf:Person'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:SpatialThing-Localized'}, {'entity': 'anonymous_person_daeba', 'type': 'Agent'}, {'entity': 'anonymous_person_daeba', 'type': 'foaf:Agent'}, {'entity': 'anonymous_person_daeba', 'type': 'owl:Thing'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:PartiallyTangible'}, {'entity': 'anonymous_person_daeba', 'type': 'EmbodiedAgent'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:SpatialThing'}, {'entity': 'anonymous_person_daeba', 'type': 'Location'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:EnduringThing-Localized'}][0m
1778417069.7454698 [start_node-22] [0m[INFO] [1778417069.744900804] [chatbot_llm]: [turn:__default__:1] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1778417069.7510767 [start_node-22] [0m[INFO] [1778417069.746816479] [chatbot_llm]: [turn:__default__:1] TURN_START | user="Hey Pop!"[0m
1778417069.7520683 [start_node-22] [0m[INFO] [1778417069.749442869] [chatbot_llm]: [turn:__default__:1] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=0 timeout=75.0s[0m
1778417070.1822665 [start_node-22] [0m[INFO] [1778417070.181895295] [chatbot_llm]: [turn:__default__:1] ROUTE_RESOLVED | route=dialogue intent=greet source=llm_response_route confidence=0.92[0m
1778417070.1834493 [start_node-22] [0m[INFO] [1778417070.183177980] [chatbot_llm]: [turn:__default__:1] TURN_DONE | planner-mode response complete[0m
1778417070.2031207 [start_manager-23] [0m[INFO] [1778417070.200804132] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "Hello! How can I help you today?"[0m
1778417070.2114127 [start_manager-23] [0m[INFO] [1778417070.210345032] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1778417070.2162893 [start_manager-23] [0m[INFO] [1778417070.215507889] [dialogue_manager]: [TTS] Speaking text (priority=128): "Hello! How can I help you today?"[0m
1778417070.2445648 [start_skill-26] [0m[INFO] [1778417070.244015289] [nao_say_skill]: [turn:Hello! How can I help you today?] TTS_ACCEPTED | goal accepted[0m
1778417070.2535043 [start_skill-26] [0m[INFO] [1778417070.252937921] [nao_say_skill]: [turn:Hello! How can I help you today?] SAY_START | text_len=32 lang=en-US volume=1.00[0m
1778417070.2553883 [start_skill-26] [0m[INFO] [1778417070.255130560] [nao_say_skill]: [turn:Hello! How can I help you today?] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=32[0m
1778417070.2570827 [start_skill-26] [0m[INFO] [1778417070.256725275] [nao_say_skill]: [turn:Hello! How can I help you today?] DEBUG_TTS_FORWARD | action="/debug/say" text_len=32[0m
1778417070.2604129 [start_skill-26] [33m[WARN] [1778417070.259879739] [nao_say_skill]: [turn:Hello! How can I help you today?] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1778417070.2617147 [start_skill-26] [0m[INFO] [1778417070.260975662] [nao_say_skill]: [turn:Hello! How can I help you today?] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=32 subscribers=0[0m
1778417070.2672508 [start_skill-26] [0m[INFO] [1778417070.266897981] [nao_say_skill]: [turn:Hello! How can I help you today?] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1778417087.4074688 [start_manager-23] [0m[INFO] [1778417087.406772604] [dialogue_manager]: [SPEECH INPUT] voice_id="anonymous_speaker": "Can you scan the room for me?" (locale=, confidence=0.00)[0m
1778417087.4126143 [start_manager-23] [0m[INFO] [1778417087.412063444] [dialogue_manager]: [SPEECH INPUT] Using default dialogue 973c0f1e-a654-42da-b888-6b3e1078600d[0m
1778417087.4165046 [start_manager-23] [0m[INFO] [1778417087.414753949] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="anonymous_speaker", text="Can you scan the room for me?"[0m
1778417087.4193215 [start_node-22] [0m[INFO] [1778417087.418852501] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:2 input=Can you scan the room for me?[0m
1778417087.4761252 [knowledge_core-5] [37m[INFO] [2026-05-10 12:44:47,472] Searching ['?entity', '?type'] in models {'default'} matching:
1778417087.4770124 [knowledge_core-5] 	- myself sees ?entity
1778417087.4797714 [knowledge_core-5] 	- ?entity rdf:type ?type[0m
1778417087.4899685 [knowledge_core-5] [37m[INFO] [2026-05-10 12:44:47,488] Found: [{'entity': 'anonymous_person_daeba', 'type': 'cyc:SolidTangibleThing'}, {'entity': 'anonymous_person_daeba', 'type': 'Human'}, {'entity': 'anonymous_person_daeba', 'type': 'foaf:Person'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:SpatialThing-Localized'}, {'entity': 'anonymous_person_daeba', 'type': 'Agent'}, {'entity': 'anonymous_person_daeba', 'type': 'foaf:Agent'}, {'entity': 'anonymous_person_daeba', 'type': 'owl:Thing'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:PartiallyTangible'}, {'entity': 'anonymous_person_daeba', 'type': 'EmbodiedAgent'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:SpatialThing'}, {'entity': 'anonymous_person_daeba', 'type': 'Location'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:EnduringThing-Localized'}][0m
1778417087.4967444 [start_node-22] [0m[INFO] [1778417087.494900010] [chatbot_llm]: [turn:__default__:2] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1778417087.4994721 [start_node-22] [0m[INFO] [1778417087.498920509] [chatbot_llm]: [turn:__default__:2] TURN_START | user="Can you scan the room for me?"[0m
1778417087.5057337 [start_node-22] [0m[INFO] [1778417087.500994358] [chatbot_llm]: [turn:__default__:2] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=2 timeout=60.0s[0m
1778417088.1783886 [start_node-22] [0m[INFO] [1778417088.177619004] [chatbot_llm]: [turn:__default__:2] ROUTE_RESOLVED | route=execution intent=- source=llm_response_route confidence=0.85[0m
1778417088.1806040 [start_node-22] [0m[INFO] [1778417088.179880225] [chatbot_llm]: [turn:__default__:2] TURN_DONE | planner-mode response complete[0m
1778417088.1937819 [start_node-22] [0m[INFO] [1778417088.191892391] [chatbot_llm]: [turn:__default__:2] PLANNER_REQUEST | published planner request on /nao_orchestrator/planner_request goal_id=goal_default___2 kind=new_goal[0m
1778417088.2018723 [start_manager-23] [0m[INFO] [1778417088.200914729] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "Sure, I will look around and report what is visible."[0m
1778417088.2109225 [start_manager-23] [0m[INFO] [1778417088.203720428] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1778417088.2112794 [start_manager-23] [0m[INFO] [1778417088.209645478] [dialogue_manager]: [TTS] Speaking text (priority=128): "Sure, I will look around and report what is visible."[0m
1778417088.2127767 [run_app-24] [0m[INFO] [1778417088.211831981] [nao_orchestrator]: Planner gate forwarded request | goal_id=goal_default___2 kind=new_goal active_goal=goal_default___2 topic=/planner/request[0m
1778417088.2202942 [start_skill-26] [0m[INFO] [1778417088.219911745] [nao_say_skill]: [turn:Sure, I will look around and rep] TTS_ACCEPTED | goal accepted[0m
1778417088.2247360 [start_skill-26] [0m[INFO] [1778417088.223631263] [nao_say_skill]: [turn:Sure, I will look around and rep] SAY_START | text_len=52 lang=en-US volume=1.00[0m
1778417088.2256136 [start_skill-26] [0m[INFO] [1778417088.224871376] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=52[0m
1778417088.2263174 [start_skill-26] [0m[INFO] [1778417088.226130869] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_TTS_FORWARD | action="/debug/say" text_len=52[0m
1778417088.2278214 [start_skill-26] [33m[WARN] [1778417088.227511653] [nao_say_skill]: [turn:Sure, I will look around and rep] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1778417088.2298031 [start_skill-26] [0m[INFO] [1778417088.229429524] [nao_say_skill]: [turn:Sure, I will look around and rep] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=52 subscribers=0[0m
1778417088.2757733 [start_skill-26] [0m[INFO] [1778417088.274056331] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1778417091.5851448 [run_app-24] [0m[INFO] [1778417091.584510245] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=0.000 pitch=0.000 relative=False[0m
1778417092.6037776 [run_app-24] [0m[INFO] [1778417092.603254459] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=0.450 pitch=0.000 relative=False[0m
1778417093.6267502 [run_app-24] [0m[INFO] [1778417093.626264151] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=-0.450 pitch=0.000 relative=False[0m
1778417094.6524012 [run_app-24] [0m[INFO] [1778417094.651981930] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=0.000 pitch=-0.200 relative=False[0m
1778417095.6760361 [run_app-24] [0m[INFO] [1778417095.675591961] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=0.000 pitch=0.200 relative=False[0m
1778417095.6785271 [run_app-24] [0m[INFO] [1778417095.677467053] [nao_orchestrator]: ORCH SCAN | target=scene target_kind=scene result_mode=success[0m
1778417095.6901617 [start_manager-23] [0m[INFO] [1778417095.689834109] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="__system__", text="The robot has finished executing a user-requested task.
1778417095.6903431 [start_manager-23] Reply to the human with one short, natural sentence about the completed task.
1778417095.6904457 [start_manager-23] Do not propose new actions, mention planner internals, or repeat the initial acknowledgement.
1778417095.6905258 [start_manager-23] Use only the execution result and suggested factual content as facts; if they do not answer whether a requested person, object, or target was found, say that no confirmed result was available instead of guessing.
1778417095.6905971 [start_manager-23] Original user request: look around and report what is visible
1778417095.6906610 [start_manager-23] Execution result: I looked around and can report the current scene summary.
1778417095.6907530 [start_manager-23] Suggested factual content: I looked around and can report the current scene summary.
1778417095.6908267 [start_manager-23] Normalized intents: inspect_scene"[0m
1778417095.6921201 [start_manager-23] [0m[INFO] [1778417095.691076491] [dialogue_manager]: [PLANNER ACT] Requested chatbot wording for goal_id=goal_default___2[0m
1778417095.6939147 [start_node-22] [0m[INFO] [1778417095.693699931] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=__system__ turn=__default__:3 input=The robot has finished executing a user-requested task. Reply to the ...[0m
1778417095.7315059 [knowledge_core-5] [37m[INFO] [2026-05-10 12:44:55,730] Searching ['?entity', '?type'] in models {'default'} matching:
1778417095.7319102 [knowledge_core-5] 	- myself sees ?entity
1778417095.7321594 [knowledge_core-5] 	- ?entity rdf:type ?type[0m
1778417095.7427592 [knowledge_core-5] [37m[INFO] [2026-05-10 12:44:55,741] Found: [{'entity': 'anonymous_person_daeba', 'type': 'cyc:SolidTangibleThing'}, {'entity': 'anonymous_person_daeba', 'type': 'Human'}, {'entity': 'anonymous_person_daeba', 'type': 'foaf:Person'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:SpatialThing-Localized'}, {'entity': 'anonymous_person_daeba', 'type': 'Agent'}, {'entity': 'anonymous_person_daeba', 'type': 'foaf:Agent'}, {'entity': 'anonymous_person_daeba', 'type': 'owl:Thing'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:PartiallyTangible'}, {'entity': 'anonymous_person_daeba', 'type': 'EmbodiedAgent'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:SpatialThing'}, {'entity': 'anonymous_person_daeba', 'type': 'Location'}, {'entity': 'anonymous_person_daeba', 'type': 'cyc:EnduringThing-Localized'}][0m
1778417095.7496676 [start_node-22] [0m[INFO] [1778417095.748941512] [chatbot_llm]: [turn:__default__:3] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1778417095.7518375 [start_node-22] [0m[INFO] [1778417095.751419415] [chatbot_llm]: [turn:__default__:3] TURN_START | user="The robot has finished executing a user-requested task. Reply to the ..."[0m
1778417095.7537789 [start_node-22] [0m[INFO] [1778417095.753293966] [chatbot_llm]: [turn:__default__:3] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=5 timeout=60.0s[0m
1778417096.1847832 [start_node-22] [0m[INFO] [1778417096.184196279] [chatbot_llm]: [turn:__default__:3] ROUTE_RESOLVED | route=execution intent=- source=llm_response_inferred_route confidence=0.00[0m
1778417096.1860545 [start_node-22] [0m[INFO] [1778417096.185283556] [chatbot_llm]: [turn:__default__:3] TURN_DONE | planner-mode response complete[0m
1778417096.1894927 [start_manager-23] [0m[INFO] [1778417096.189096949] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "I looked around and can report the current scene summary."[0m
1778417096.1905572 [start_manager-23] [0m[INFO] [1778417096.190132461] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1778417096.1911383 [start_manager-23] [0m[INFO] [1778417096.191026693] [dialogue_manager]: [TTS] Speaking text (priority=128): "I looked around and can report the current scene summary."[0m
1778417096.1937699 [start_skill-26] [0m[INFO] [1778417096.193618810] [nao_say_skill]: [turn:I looked around and can report t] TTS_ACCEPTED | goal accepted[0m
1778417096.1972308 [start_skill-26] [0m[INFO] [1778417096.196919234] [nao_say_skill]: [turn:I looked around and can report t] SAY_START | text_len=57 lang=en-US volume=1.00[0m
1778417096.1985216 [start_skill-26] [0m[INFO] [1778417096.197831425] [nao_say_skill]: [turn:I looked around and can report t] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=57[0m
1778417096.1992354 [start_skill-26] [0m[INFO] [1778417096.198867589] [nao_say_skill]: [turn:I looked around and can report t] DEBUG_TTS_FORWARD | action="/debug/say" text_len=57[0m
1778417096.2003837 [start_skill-26] [33m[WARN] [1778417096.200124062] [nao_say_skill]: [turn:I looked around and can report t] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1778417096.2012455 [start_skill-26] [0m[INFO] [1778417096.201013035] [nao_say_skill]: [turn:I looked around and can report t] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=57 subscribers=0[0m
1778417096.2364602 [start_skill-26] [0m[INFO] [1778417096.235134727] [nao_say_skill]: [turn:I looked around and can report t] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1778417160.2437146 [ERROR] [naoqi_driver_node-1]: process has died [pid 2160, exit code -6, cmd '/opt/ros/jazzy/lib/naoqi_driver/naoqi_driver_node --ros-args --params-file /tmp/launch_params_3p4tp7ub'].
1778417163.1136911 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:03,112] Deleting from ['default']:
1778417163.1188693 [knowledge_core-5] 	- anonymous_person_daeba rdf:type Human[0m
1778417163.5327752 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:03,530] Materialisation performed by reasoner in 414.7ms[0m
1778417163.6986701 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:03,696] Deleting from ['default']:
1778417163.7250426 [knowledge_core-5] 	- myself sees anonymous_person_daeba[0m
1778417163.8435142 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:03,842] Materialisation performed by reasoner in 145.0ms[0m
1778417164.0007505 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:03,999] Updating ['default'] with:
1778417164.0011632 [knowledge_core-5] 	- anonymous_person_aibac rdf:type Human[0m
1778417164.0825396 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:04,079] Materialisation performed by reasoner in 79.6ms[0m
1778417164.1639755 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:04,160] Deleting from ['default']:
1778417164.1645081 [knowledge_core-5] 	- anonymous_person_aibac sees myself[0m
1778417164.3158886 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:04,314] Materialisation performed by reasoner in 153.4ms[0m
1778417164.4629433 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:04,462] Updating ['default'] with:
1778417164.4631674 [knowledge_core-5] 	- myself sees anonymous_person_aibac[0m
1778417164.6071358 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:04,606] Materialisation performed by reasoner in 143.7ms[0m
1778417164.7837689 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:04,774] Updating ['default'] with:
1778417164.7842839 [knowledge_core-5] 	- anonymous_person_aibac sees myself[0m
1778417164.9291072 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:04,928] Materialisation performed by reasoner in 150.2ms[0m
1778417165.0760362 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:05,073] Deleting from ['default']:
1778417165.0769172 [knowledge_core-5] 	- anonymous_person_aibac rdf:type Human[0m
1778417165.1869881 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:05,185] Materialisation performed by reasoner in 111.3ms[0m
1778417165.3330822 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:05,331] Deleting from ['default']:
1778417165.3335202 [knowledge_core-5] 	- myself sees anonymous_person_aibac[0m
1778417165.6281481 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:05,626] Materialisation performed by reasoner in 294.5ms[0m
1778417168.4759295 [hri_face_detect_yunet-11] [33m[WARN] [1778417168.475266372] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 139.8sec[0m
1778417173.0800350 [hri_face_detect_yunet-11] [33m[WARN] [1778417173.079499211] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.7sec[0m
1778417177.9128845 [hri_face_detect_yunet-11] [33m[WARN] [1778417177.912326521] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.8sec[0m
1778417178.3447328 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:18,344] Updating ['default'] with:
1778417178.3449283 [knowledge_core-5] 	- anonymous_person_hajcf rdf:type Human[0m
1778417178.4442565 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:18,443] Materialisation performed by reasoner in 99.1ms[0m
1778417178.5953021 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:18,593] Updating ['default'] with:
1778417178.5958271 [knowledge_core-5] 	- myself sees anonymous_person_hajcf[0m
1778417178.7260525 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:18,725] Materialisation performed by reasoner in 131.4ms[0m
1778417178.8848000 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:18,883] Updating ['default'] with:
1778417178.8852015 [knowledge_core-5] 	- anonymous_person_hajcf sees myself[0m
1778417179.0088482 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:19,008] Materialisation performed by reasoner in 124.2ms[0m
1778417179.4032376 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:19,401] Deleting from ['default']:
1778417179.4040310 [knowledge_core-5] 	- anonymous_person_hajcf sees myself[0m
1778417179.5228953 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:19,522] Materialisation performed by reasoner in 120.4ms[0m
1778417179.7403905 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:19,739] Updating ['default'] with:
1778417179.7406495 [knowledge_core-5] 	- anonymous_person_hajcf sees myself[0m
1778417180.0775673 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:20,076] Materialisation performed by reasoner in 336.5ms[0m
1778417180.3999600 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:20,387] Deleting from ['default']:
1778417180.4021442 [knowledge_core-5] 	- anonymous_person_hajcf rdf:type Human[0m
1778417180.4949462 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:20,494] Materialisation performed by reasoner in 104.2ms[0m
1778417180.6387162 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:20,638] Deleting from ['default']:
1778417180.6388836 [knowledge_core-5] 	- myself sees anonymous_person_hajcf[0m
1778417180.7354093 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:20,734] Materialisation performed by reasoner in 96.2ms[0m
1778417182.1381407 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:22,137] Updating ['default'] with:
1778417182.1384943 [knowledge_core-5] 	- anonymous_person_cgfbi rdf:type Human[0m
1778417182.2669706 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:22,266] Materialisation performed by reasoner in 128.8ms[0m
1778417182.4195390 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:22,418] Updating ['default'] with:
1778417182.4209745 [knowledge_core-5] 	- myself sees anonymous_person_cgfbi[0m
1778417182.5439017 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:22,543] Materialisation performed by reasoner in 124.3ms[0m
1778417182.7174253 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:22,713] Updating ['default'] with:
1778417182.7179646 [knowledge_core-5] 	- anonymous_person_cgfbi sees myself[0m
1778417182.8426244 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:22,841] Materialisation performed by reasoner in 126.9ms[0m
1778417183.1364133 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:23,135] Deleting from ['default']:
1778417183.1370032 [knowledge_core-5] 	- anonymous_person_cgfbi rdf:type Human[0m
1778417183.4537549 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:23,452] Materialisation performed by reasoner in 317.1ms[0m
1778417183.6090255 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:23,604] Deleting from ['default']:
1778417183.6099408 [knowledge_core-5] 	- myself sees anonymous_person_cgfbi[0m
1778417183.7876992 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:23,786] Materialisation performed by reasoner in 180.9ms[0m
1778417183.9837742 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:23,983] Updating ['default'] with:
1778417183.9839480 [knowledge_core-5] 	- anonymous_person_hjdbd rdf:type Human[0m
1778417184.1169443 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:24,114] Materialisation performed by reasoner in 131.4ms[0m
1778417184.2661831 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:24,265] Updating ['default'] with:
1778417184.2664163 [knowledge_core-5] 	- myself sees anonymous_person_hjdbd[0m
1778417184.3732939 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:24,372] Materialisation performed by reasoner in 106.6ms[0m
1778417184.5235844 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:24,522] Updating ['default'] with:
1778417184.5249274 [knowledge_core-5] 	- anonymous_person_hjdbd sees myself[0m
1778417184.6196527 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:24,617] Materialisation performed by reasoner in 93.8ms[0m
1778417185.1432674 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:25,141] Deleting from ['default']:
1778417185.1437893 [knowledge_core-5] 	- anonymous_person_hjdbd rdf:type Human[0m
1778417185.2654331 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:25,264] Materialisation performed by reasoner in 122.5ms[0m
1778417185.4158378 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:25,411] Deleting from ['default']:
1778417185.4173951 [knowledge_core-5] 	- myself sees anonymous_person_hjdbd[0m
1778417185.7154453 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:25,714] Materialisation performed by reasoner in 300.9ms[0m
1778417185.9497793 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:25,948] Updating ['default'] with:
1778417185.9502211 [knowledge_core-5] 	- anonymous_person_fadfa rdf:type Human[0m
1778417186.0734649 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:26,072] Materialisation performed by reasoner in 123.5ms[0m
1778417186.2440333 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:26,242] Updating ['default'] with:
1778417186.2460768 [knowledge_core-5] 	- myself sees anonymous_person_fadfa[0m
1778417186.3693719 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:26,368] Materialisation performed by reasoner in 125.2ms[0m
1778417186.5141292 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:26,513] Updating ['default'] with:
1778417186.5143094 [knowledge_core-5] 	- anonymous_person_fadfa sees myself[0m
1778417186.6313975 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:26,630] Materialisation performed by reasoner in 116.9ms[0m
1778417188.0544477 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:28,049] Deleting from ['default']:
1778417188.0559130 [knowledge_core-5] 	- anonymous_person_fadfa rdf:type Human[0m
1778417188.1921692 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:28,191] Materialisation performed by reasoner in 140.3ms[0m
1778417188.3509855 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:28,347] Deleting from ['default']:
1778417188.3519678 [knowledge_core-5] 	- myself sees anonymous_person_fadfa[0m
1778417188.4430237 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:28,442] Materialisation performed by reasoner in 93.7ms[0m
1778417189.3641038 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:29,362] Updating ['default'] with:
1778417189.3646984 [knowledge_core-5] 	- anonymous_person_gihgb rdf:type Human[0m
1778417189.6851234 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:29,683] Materialisation performed by reasoner in 321.0ms[0m
1778417189.8320553 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:29,830] Updating ['default'] with:
1778417189.8324721 [knowledge_core-5] 	- myself sees anonymous_person_gihgb[0m
1778417189.9479017 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:29,947] Materialisation performed by reasoner in 116.2ms[0m
1778417190.1004958 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:30,099] Updating ['default'] with:
1778417190.1008146 [knowledge_core-5] 	- anonymous_person_gihgb sees myself[0m
1778417190.2495124 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:30,248] Materialisation performed by reasoner in 148.8ms[0m
1778417190.4998596 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:30,493] Deleting from ['default']:
1778417190.5003958 [knowledge_core-5] 	- anonymous_person_gihgb rdf:type Human[0m
1778417190.5789113 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:30,578] Materialisation performed by reasoner in 84.1ms[0m
1778417190.7217331 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:30,720] Deleting from ['default']:
1778417190.7220790 [knowledge_core-5] 	- myself sees anonymous_person_gihgb[0m
1778417190.8295641 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:30,829] Materialisation performed by reasoner in 107.7ms[0m
1778417194.9160693 [hri_face_detect_yunet-11] [33m[WARN] [1778417194.915636778] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 17.0sec[0m
1778417199.3951292 [hri_face_detect_yunet-11] [33m[WARN] [1778417199.394603199] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.5sec[0m
1778417203.8807073 [hri_face_detect_yunet-11] [33m[WARN] [1778417203.879972167] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.5sec[0m
1778417217.8859057 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:57,885] Updating ['default'] with:
1778417217.8860898 [knowledge_core-5] 	- anonymous_person_ejdba rdf:type Human[0m
1778417218.0212483 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:58,019] Materialisation performed by reasoner in 134.2ms[0m
1778417218.0566907 [hri_face_detect_yunet-11] [33m[WARN] [1778417218.056182634] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 14.2sec[0m
1778417218.1692317 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:58,168] Updating ['default'] with:
1778417218.1695657 [knowledge_core-5] 	- myself sees anonymous_person_ejdba[0m
1778417218.2780616 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:58,277] Materialisation performed by reasoner in 108.8ms[0m
1778417219.2348526 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:59,233] Deleting from ['default']:
1778417219.2355678 [knowledge_core-5] 	- anonymous_person_ejdba rdf:type Human[0m
1778417219.3058140 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:59,305] Materialisation performed by reasoner in 71.2ms[0m
1778417219.4522200 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:59,449] Deleting from ['default']:
1778417219.4528637 [knowledge_core-5] 	- myself sees anonymous_person_ejdba[0m
1778417219.5696151 [knowledge_core-5] [37m[INFO] [2026-05-10 12:46:59,568] Materialisation performed by reasoner in 118.2ms[0m
1778417222.8896053 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:02,888] Updating ['default'] with:
1778417222.8898330 [knowledge_core-5] 	- anonymous_person_gighj rdf:type Human[0m
1778417223.0299468 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:03,029] Materialisation performed by reasoner in 139.7ms[0m
1778417223.1766925 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:03,175] Updating ['default'] with:
1778417223.1770890 [knowledge_core-5] 	- myself sees anonymous_person_gighj[0m
1778417223.4974585 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:03,496] Materialisation performed by reasoner in 320.6ms[0m
1778417223.6472294 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:03,646] Updating ['default'] with:
1778417223.6477134 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
1778417223.7805052 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:03,779] Materialisation performed by reasoner in 132.9ms[0m
1778417223.8616033 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:03,860] Deleting from ['default']:
1778417223.8620100 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
1778417223.9566538 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:03,955] Materialisation performed by reasoner in 94.7ms[0m
1778417224.1003330 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:04,099] Updating ['default'] with:
1778417224.1005847 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
1778417224.2153666 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:04,214] Materialisation performed by reasoner in 114.9ms[0m
1778417224.2688570 [hri_face_detect_yunet-11] [33m[WARN] [1778417224.268074713] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.2sec[0m
1778417225.2388670 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:05,238] Deleting from ['default']:
1778417225.2390807 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
1778417225.3421991 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:05,341] Materialisation performed by reasoner in 103.1ms[0m
1778417225.9161274 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:05,915] Updating ['default'] with:
1778417225.9164622 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
1778417226.0195322 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:06,018] Materialisation performed by reasoner in 102.7ms[0m
1778417226.4626584 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:06,462] Deleting from ['default']:
1778417226.4628222 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
1778417226.7720940 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:06,771] Materialisation performed by reasoner in 309.0ms[0m
1778417227.0616434 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:07,060] Updating ['default'] with:
1778417227.0620210 [knowledge_core-5] 	- anonymous_person_gighj sees myself[0m
1778417227.1772327 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:07,176] Materialisation performed by reasoner in 115.6ms[0m
1778417229.0086141 [hri_face_detect_yunet-11] [33m[WARN] [1778417229.006528406] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.7sec[0m
1778417229.3973355 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:09,396] Deleting from ['default']:
1778417229.3976150 [knowledge_core-5] 	- anonymous_person_gighj rdf:type Human[0m
1778417229.5043197 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:09,503] Materialisation performed by reasoner in 106.2ms[0m
1778417229.6495209 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:09,647] Deleting from ['default']:
1778417229.6500964 [knowledge_core-5] 	- myself sees anonymous_person_gighj[0m
1778417229.7605243 [knowledge_core-5] [37m[INFO] [2026-05-10 12:47:09,759] Materialisation performed by reasoner in 111.2ms[0m
1778417233.4659956 [hri_face_detect_yunet-11] [33m[WARN] [1778417233.465575739] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.5sec[0m
1778417238.0300496 [hri_face_detect_yunet-11] [33m[WARN] [1778417238.029505999] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.6sec[0m
1778417244.3455961 [hri_face_detect_yunet-11] [33m[WARN] [1778417244.345092232] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.3sec[0m
1778417249.1253412 [hri_face_detect_yunet-11] [33m[WARN] [1778417249.116943580] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.8sec[0m
1778417253.9845343 [hri_face_detect_yunet-11] [33m[WARN] [1778417253.984167128] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.9sec[0m
1778417261.1329114 [hri_face_detect_yunet-11] [33m[WARN] [1778417261.132593150] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 7.1sec[0m
1778417265.6592469 [hri_face_detect_yunet-11] [33m[WARN] [1778417265.658774054] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.5sec[0m
1778417270.1639760 [hri_face_detect_yunet-11] [33m[WARN] [1778417270.163609348] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.5sec[0m
1778417280.5248353 [hri_face_detect_yunet-11] [33m[WARN] [1778417280.523959079] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 10.4sec[0m
1778417286.1450965 [hri_face_detect_yunet-11] [33m[WARN] [1778417286.144613842] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778417292.5153708 [hri_face_detect_yunet-11] [33m[WARN] [1778417292.514947742] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.4sec[0m
1778417296.8218567 [hri_face_detect_yunet-11] [33m[WARN] [1778417296.821333246] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.3sec[0m
1778417306.8271542 [hri_face_detect_yunet-11] [33m[WARN] [1778417306.826581805] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 10.0sec[0m
1778417311.8160245 [hri_face_detect_yunet-11] [33m[WARN] [1778417311.811309303] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.0sec[0m
1778417317.5894835 [hri_face_detect_yunet-11] [33m[WARN] [1778417317.589054119] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.8sec[0m
1778417322.9593701 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:42,958] Updating ['default'] with:
1778417322.9596457 [knowledge_core-5] 	- anonymous_person_afefe rdf:type Human[0m
1778417323.0918877 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:43,090] Materialisation performed by reasoner in 131.6ms[0m
1778417323.2391775 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:43,238] Updating ['default'] with:
1778417323.2394164 [knowledge_core-5] 	- myself sees anonymous_person_afefe[0m
1778417323.5186026 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:43,517] Materialisation performed by reasoner in 279.1ms[0m
1778417323.7075896 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:43,707] Updating ['default'] with:
1778417323.7078383 [knowledge_core-5] 	- anonymous_person_afefe sees myself[0m
1778417323.8344572 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:43,833] Materialisation performed by reasoner in 126.7ms[0m
1778417335.8643343 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:55,860] Deleting from ['default']:
1778417335.8656094 [knowledge_core-5] 	- anonymous_person_afefe sees myself[0m
1778417336.0193827 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:56,018] Materialisation performed by reasoner in 156.9ms[0m
1778417336.1790392 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:56,174] Updating ['default'] with:
1778417336.1811364 [knowledge_core-5] 	- anonymous_person_afefe sees myself[0m
1778417336.3180959 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:56,317] Materialisation performed by reasoner in 139.0ms[0m
1778417336.9245086 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:56,917] Deleting from ['default']:
1778417336.9258523 [knowledge_core-5] 	- anonymous_person_afefe rdf:type Human[0m
1778417337.0420868 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:57,041] Materialisation performed by reasoner in 121.5ms[0m
1778417337.2211156 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:57,220] Deleting from ['default']:
1778417337.2214062 [knowledge_core-5] 	- myself sees anonymous_person_afefe[0m
1778417337.5489905 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:57,548] Materialisation performed by reasoner in 327.7ms[0m
1778417337.9241178 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:57,922] Updating ['default'] with:
1778417337.9243748 [knowledge_core-5] 	- anonymous_person_hfccj rdf:type Human[0m
1778417338.0469856 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:58,046] Materialisation performed by reasoner in 122.8ms[0m
1778417338.2336643 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:58,233] Updating ['default'] with:
1778417338.2339911 [knowledge_core-5] 	- myself sees anonymous_person_hfccj[0m
1778417338.3478630 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:58,347] Materialisation performed by reasoner in 113.7ms[0m
1778417338.4953873 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:58,494] Updating ['default'] with:
1778417338.4956710 [knowledge_core-5] 	- anonymous_person_hfccj sees myself[0m
1778417338.6209431 [knowledge_core-5] [37m[INFO] [2026-05-10 12:48:58,619] Materialisation performed by reasoner in 124.9ms[0m
1778417342.1498094 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:02,143] Deleting from ['default']:
1778417342.1500051 [knowledge_core-5] 	- anonymous_person_hfccj rdf:type Human[0m
1778417342.2751732 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:02,273] Materialisation performed by reasoner in 126.3ms[0m
1778417342.4346714 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:02,434] Deleting from ['default']:
1778417342.4349079 [knowledge_core-5] 	- myself sees anonymous_person_hfccj[0m
1778417342.5389867 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:02,538] Materialisation performed by reasoner in 104.1ms[0m
1778417347.0306184 [hri_face_detect_yunet-11] [33m[WARN] [1778417347.029890612] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 29.4sec[0m
1778417353.9185772 [hri_face_detect_yunet-11] [33m[WARN] [1778417353.917484415] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.9sec[0m
1778417361.4600053 [hri_face_detect_yunet-11] [33m[WARN] [1778417361.459267807] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 7.5sec[0m
1778417365.8652554 [hri_face_detect_yunet-11] [33m[WARN] [1778417365.864611425] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.4sec[0m
1778417369.4759371 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:29,475] Updating ['default'] with:
1778417369.4761932 [knowledge_core-5] 	- anonymous_person_eicdd rdf:type Human[0m
1778417369.5908425 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:29,590] Materialisation performed by reasoner in 114.6ms[0m
1778417369.7406154 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:29,739] Updating ['default'] with:
1778417369.7409396 [knowledge_core-5] 	- myself sees anonymous_person_eicdd[0m
1778417369.8577793 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:29,856] Materialisation performed by reasoner in 116.8ms[0m
1778417370.0030146 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:30,002] Updating ['default'] with:
1778417370.0032604 [knowledge_core-5] 	- anonymous_person_eicdd sees myself[0m
1778417370.1454053 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:30,144] Materialisation performed by reasoner in 141.9ms[0m
1778417372.9454999 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:32,941] Deleting from ['default']:
1778417372.9467223 [knowledge_core-5] 	- anonymous_person_eicdd sees myself[0m
1778417373.2381170 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:33,237] Materialisation performed by reasoner in 294.6ms[0m
1778417373.3821716 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:33,381] Updating ['default'] with:
1778417373.3824298 [knowledge_core-5] 	- anonymous_person_eicdd sees myself[0m
1778417373.5001297 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:33,499] Materialisation performed by reasoner in 117.5ms[0m
1778417384.1447260 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:44,143] Updating ['default'] with:
1778417384.1450484 [knowledge_core-5] 	- detected_tomato_219_304 rdf:type Tomato
1778417384.1452208 [knowledge_core-5] 	- myself sees detected_tomato_219_304 (lifespan: 4.0sec)[0m
1778417384.1461287 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:44,144] This statement will expire on 10/05/2026, 12:49:48[0m
1778417384.2768264 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:44,275] Materialisation performed by reasoner in 130.8ms[0m
1778417389.0636528 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:49:49,062] Removing expired statement <detected_tomato_219_304 rdf:type Tomato> from <default> (expired on 2026-05-10T12:49:48+00:00)[0m
1778417389.0656972 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:49:49,062] Removing expired statement <myself sees detected_tomato_219_304> from <default> (expired on 2026-05-10T12:49:48+00:00)[0m
1778417389.2010520 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:49,200] Materialisation performed by reasoner in 137.2ms[0m
1778417398.8808620 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:58,879] Deleting from ['default']:
1778417398.8812404 [knowledge_core-5] 	- anonymous_person_eicdd rdf:type Human[0m
1778417399.0622544 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:59,061] Materialisation performed by reasoner in 181.1ms[0m
1778417399.2069843 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:59,206] Deleting from ['default']:
1778417399.2072029 [knowledge_core-5] 	- myself sees anonymous_person_eicdd[0m
1778417399.3136933 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:59,313] Materialisation performed by reasoner in 106.7ms[0m
1778417399.7510569 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:59,749] Updating ['default'] with:
1778417399.7515604 [knowledge_core-5] 	- anonymous_person_ahhbi rdf:type Human[0m
1778417399.8921099 [knowledge_core-5] [37m[INFO] [2026-05-10 12:49:59,891] Materialisation performed by reasoner in 141.1ms[0m
1778417400.0497537 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:00,048] Updating ['default'] with:
1778417400.0514309 [knowledge_core-5] 	- myself sees anonymous_person_ahhbi[0m
1778417400.1862571 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:00,185] Materialisation performed by reasoner in 136.2ms[0m
1778417400.3428564 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:00,340] Updating ['default'] with:
1778417400.3433197 [knowledge_core-5] 	- anonymous_person_ahhbi sees myself[0m
1778417400.4834936 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:00,482] Materialisation performed by reasoner in 141.3ms[0m
1778417400.8587286 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:00,852] Deleting from ['default']:
1778417400.8598063 [knowledge_core-5] 	- anonymous_person_ahhbi rdf:type Human[0m
1778417401.2841804 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:01,283] Materialisation performed by reasoner in 429.7ms[0m
1778417401.4276838 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:01,426] Deleting from ['default']:
1778417401.4279068 [knowledge_core-5] 	- myself sees anonymous_person_ahhbi[0m
1778417401.5271800 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:01,526] Materialisation performed by reasoner in 99.4ms[0m
1778417402.1606257 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:02,158] Updating ['default'] with:
1778417402.1620233 [knowledge_core-5] 	- myself sees detected_blueberry_437_173
1778417402.1623442 [knowledge_core-5] 	- detected_blueberry_437_173 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417402.1642447 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:02,159] This statement will expire on 10/05/2026, 12:50:06[0m
1778417402.2937493 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:02,292] Materialisation performed by reasoner in 132.2ms[0m
1778417404.5601466 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:04,558] Updating ['default'] with:
1778417404.5605259 [knowledge_core-5] 	- myself sees detected_blueberry_447_188
1778417404.5609150 [knowledge_core-5] 	- detected_blueberry_447_188 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417404.5623088 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:04,559] This statement will expire on 10/05/2026, 12:50:08[0m
1778417404.6300392 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:04,629] Materialisation performed by reasoner in 69.3ms[0m
1778417407.0322971 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:50:07,031] Removing expired statement <myself sees detected_blueberry_437_173> from <default> (expired on 2026-05-10T12:50:06+00:00)[0m
1778417407.0330033 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:50:07,031] Removing expired statement <detected_blueberry_437_173 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:50:06+00:00)[0m
1778417407.1738067 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:07,172] Materialisation performed by reasoner in 140.5ms[0m
1778417408.1731977 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:08,171] Updating ['default'] with:
1778417408.1736295 [knowledge_core-5] 	- myself sees detected_blueberry_444_196
1778417408.1738369 [knowledge_core-5] 	- detected_blueberry_444_196 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417408.1750927 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:08,172] This statement will expire on 10/05/2026, 12:50:12[0m
1778417408.2925854 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:08,291] Materialisation performed by reasoner in 118.6ms[0m
1778417408.3235228 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:50:08,323] Removing expired statement <myself sees detected_blueberry_447_188> from <default> (expired on 2026-05-10T12:50:08+00:00)[0m
1778417408.3241951 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:50:08,323] Removing expired statement <detected_blueberry_447_188 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:50:08+00:00)[0m
1778417408.6799142 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:08,679] Materialisation performed by reasoner in 355.9ms[0m
1778417410.3428867 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:10,341] Updating ['default'] with:
1778417410.3435452 [knowledge_core-5] 	- myself sees detected_blueberry_431_179
1778417410.3439553 [knowledge_core-5] 	- detected_blueberry_431_179 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417410.3457382 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:10,342] This statement will expire on 10/05/2026, 12:50:14[0m
1778417410.4699433 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:10,469] Materialisation performed by reasoner in 126.2ms[0m
1778417412.7930226 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:50:12,792] Removing expired statement <myself sees detected_blueberry_444_196> from <default> (expired on 2026-05-10T12:50:12+00:00)[0m
1778417412.8014433 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:50:12,792] Removing expired statement <detected_blueberry_444_196 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:50:12+00:00)[0m
1778417412.9054394 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:12,904] Materialisation performed by reasoner in 112.1ms[0m
1778417414.9986436 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:50:14,996] Removing expired statement <myself sees detected_blueberry_431_179> from <default> (expired on 2026-05-10T12:50:14+00:00)[0m
1778417414.9993033 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:50:14,996] Removing expired statement <detected_blueberry_431_179 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:50:14+00:00)[0m
1778417415.1243701 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:15,123] Materialisation performed by reasoner in 126.5ms[0m
1778417419.1291108 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:19,128] Updating ['default'] with:
1778417419.1293139 [knowledge_core-5] 	- anonymous_person_ifchh rdf:type Human[0m
1778417419.2830780 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:19,281] Materialisation performed by reasoner in 153.0ms[0m
1778417419.4646273 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:19,463] Updating ['default'] with:
1778417419.4659066 [knowledge_core-5] 	- myself sees anonymous_person_ifchh[0m
1778417419.8205986 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:19,820] Materialisation performed by reasoner in 356.4ms[0m
1778417419.9701958 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:19,969] Updating ['default'] with:
1778417419.9705331 [knowledge_core-5] 	- anonymous_person_ifchh sees myself[0m
1778417420.0994718 [knowledge_core-5] [37m[INFO] [2026-05-10 12:50:20,098] Materialisation performed by reasoner in 129.4ms[0m
1778417463.6840153 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:03,682] Deleting from ['default']:
1778417463.6845465 [knowledge_core-5] 	- anonymous_person_ifchh rdf:type Human[0m
1778417464.0088246 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:04,008] Materialisation performed by reasoner in 324.9ms[0m
1778417464.1670623 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:04,164] Deleting from ['default']:
1778417464.1689825 [knowledge_core-5] 	- myself sees anonymous_person_ifchh[0m
1778417464.3162596 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:04,315] Materialisation performed by reasoner in 150.3ms[0m
1778417466.9461429 [hri_face_detect_yunet-11] [33m[WARN] [1778417466.944905948] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 101.1sec[0m
1778417470.4288526 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:10,426] Updating ['default'] with:
1778417470.4291604 [knowledge_core-5] 	- anonymous_person_gacga rdf:type Human[0m
1778417470.5549376 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:10,554] Materialisation performed by reasoner in 127.5ms[0m
1778417470.7070408 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:10,705] Updating ['default'] with:
1778417470.7074711 [knowledge_core-5] 	- myself sees anonymous_person_gacga[0m
1778417470.8378675 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:10,837] Materialisation performed by reasoner in 130.8ms[0m
1778417470.9955108 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:10,993] Updating ['default'] with:
1778417470.9962504 [knowledge_core-5] 	- anonymous_person_gacga sees myself[0m
1778417471.1070313 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:11,105] Materialisation performed by reasoner in 111.2ms[0m
1778417471.4755309 [hri_face_detect_yunet-11] [33m[WARN] [1778417471.475049994] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.5sec[0m
1778417473.5598176 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:13,546] Deleting from ['default']:
1778417473.5621595 [knowledge_core-5] 	- anonymous_person_gacga rdf:type Human[0m
1778417473.8943961 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:13,893] Materialisation performed by reasoner in 347.1ms[0m
1778417474.0405645 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:14,038] Deleting from ['default']:
1778417474.0413039 [knowledge_core-5] 	- myself sees anonymous_person_gacga[0m
1778417474.1639552 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:14,163] Materialisation performed by reasoner in 123.6ms[0m
1778417479.4667237 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:19,465] Updating ['default'] with:
1778417479.4671743 [knowledge_core-5] 	- anonymous_person_jfabi rdf:type Human[0m
1778417479.5918081 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:19,591] Materialisation performed by reasoner in 125.2ms[0m
1778417479.7516539 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:19,750] Updating ['default'] with:
1778417479.7524006 [knowledge_core-5] 	- myself sees anonymous_person_jfabi[0m
1778417479.8988492 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:19,897] Materialisation performed by reasoner in 147.0ms[0m
1778417479.9633343 [hri_face_detect_yunet-11] [33m[WARN] [1778417479.962575792] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 8.5sec[0m
1778417480.0551860 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:20,053] Updating ['default'] with:
1778417480.0557024 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417480.1956139 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:20,194] Materialisation performed by reasoner in 140.5ms[0m
1778417487.2368398 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:27,236] Deleting from ['default']:
1778417487.2370341 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417487.5539415 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:27,545] Materialisation performed by reasoner in 309.0ms[0m
1778417487.6916509 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:27,690] Updating ['default'] with:
1778417487.6920741 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417487.8074434 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:27,806] Materialisation performed by reasoner in 115.5ms[0m
1778417487.9727294 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:27,970] Deleting from ['default']:
1778417487.9739678 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417488.1041842 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:28,095] Materialisation performed by reasoner in 124.6ms[0m
1778417490.1549149 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:30,150] Updating ['default'] with:
1778417490.1649766 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417490.2970927 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:30,296] Materialisation performed by reasoner in 144.4ms[0m
1778417490.9421139 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:30,941] Deleting from ['default']:
1778417490.9423521 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417491.0913899 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:31,090] Materialisation performed by reasoner in 148.4ms[0m
1778417491.3213708 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:31,320] Updating ['default'] with:
1778417491.3216364 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417491.4422250 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:31,437] Materialisation performed by reasoner in 116.4ms[0m
1778417495.4535029 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:35,452] Deleting from ['default']:
1778417495.4539032 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417495.5265052 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:35,525] Materialisation performed by reasoner in 73.1ms[0m
1778417495.6983883 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:35,697] Updating ['default'] with:
1778417495.6986957 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417495.8055460 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:35,804] Materialisation performed by reasoner in 106.9ms[0m
1778417500.5463519 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:40,542] Deleting from ['default']:
1778417500.5474775 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417500.6908958 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:40,688] Materialisation performed by reasoner in 144.9ms[0m
1778417500.8501363 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:40,846] Updating ['default'] with:
1778417500.8514407 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417500.9956791 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:40,994] Materialisation performed by reasoner in 147.0ms[0m
1778417501.9395244 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:41,938] Deleting from ['default']:
1778417501.9400282 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417502.0591092 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:42,058] Materialisation performed by reasoner in 119.6ms[0m
1778417502.2094612 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:42,206] Updating ['default'] with:
1778417502.2106571 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417502.5761976 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:42,575] Materialisation performed by reasoner in 368.5ms[0m
1778417504.0593719 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:44,058] Deleting from ['default']:
1778417504.0596228 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417504.1587236 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:44,156] Materialisation performed by reasoner in 97.3ms[0m
1778417504.3007121 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:44,300] Updating ['default'] with:
1778417504.3009682 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417504.4036145 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:44,403] Materialisation performed by reasoner in 102.8ms[0m
1778417504.6212475 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:44,619] Deleting from ['default']:
1778417504.6220891 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417504.7534921 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:44,752] Materialisation performed by reasoner in 132.7ms[0m
1778417504.8967266 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:44,896] Updating ['default'] with:
1778417504.8968902 [knowledge_core-5] 	- anonymous_person_jfabi sees myself[0m
1778417505.0564723 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:45,055] Materialisation performed by reasoner in 159.3ms[0m
1778417519.9378581 [knowledge_core-5] [37m[INFO] [2026-05-10 12:51:59,937] Deleting from ['default']:
1778417519.9381247 [knowledge_core-5] 	- anonymous_person_jfabi rdf:type Human[0m
1778417520.0996821 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:00,097] Materialisation performed by reasoner in 160.2ms[0m
1778417520.2419817 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:00,241] Deleting from ['default']:
1778417520.2426057 [knowledge_core-5] 	- myself sees anonymous_person_jfabi[0m
1778417520.4006248 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:00,399] Materialisation performed by reasoner in 157.8ms[0m
1778417520.8784363 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:00,871] Updating ['default'] with:
1778417520.8789101 [knowledge_core-5] 	- anonymous_person_jcedc rdf:type Human[0m
1778417521.0181978 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:01,017] Materialisation performed by reasoner in 144.5ms[0m
1778417521.1838913 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:01,181] Updating ['default'] with:
1778417521.1842818 [knowledge_core-5] 	- myself sees anonymous_person_jcedc[0m
1778417521.3273828 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:01,326] Materialisation performed by reasoner in 144.6ms[0m
1778417521.4807515 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:01,479] Updating ['default'] with:
1778417521.4811313 [knowledge_core-5] 	- anonymous_person_jcedc sees myself[0m
1778417521.6078968 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:01,606] Materialisation performed by reasoner in 127.0ms[0m
1778417527.9562612 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:07,954] Updating ['default'] with:
1778417527.9568920 [knowledge_core-5] 	- myself sees detected_tomato_231_311
1778417527.9571712 [knowledge_core-5] 	- detected_tomato_231_311 rdf:type Tomato (lifespan: 4.0sec)[0m
1778417527.9585726 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:07,955] This statement will expire on 10/05/2026, 12:52:11[0m
1778417528.3185287 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:08,317] Materialisation performed by reasoner in 360.6ms[0m
1778417531.4828570 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:52:11,482] Removing expired statement <myself sees detected_tomato_231_311> from <default> (expired on 2026-05-10T12:52:11+00:00)[0m
1778417531.4843147 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:52:11,482] Removing expired statement <detected_tomato_231_311 rdf:type Tomato> from <default> (expired on 2026-05-10T12:52:11+00:00)[0m
1778417531.6028674 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:11,602] Materialisation performed by reasoner in 119.8ms[0m
1778417538.1422083 [hri_face_detect_yunet-11] [33m[WARN] [1778417538.141849489] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 58.2sec[0m
1778417538.5837991 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:18,581] Updating ['default'] with:
1778417538.5851989 [knowledge_core-5] 	- myself sees detected_tomato_230_302
1778417538.5866551 [knowledge_core-5] 	- detected_tomato_230_302 rdf:type Tomato (lifespan: 4.0sec)[0m
1778417538.5876298 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:18,582] This statement will expire on 10/05/2026, 12:52:22[0m
1778417538.7215185 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:18,720] Materialisation performed by reasoner in 138.4ms[0m
1778417542.9562914 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:52:22,955] Removing expired statement <myself sees detected_tomato_230_302> from <default> (expired on 2026-05-10T12:52:22+00:00)[0m
1778417542.9571228 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:52:22,955] Removing expired statement <detected_tomato_230_302 rdf:type Tomato> from <default> (expired on 2026-05-10T12:52:22+00:00)[0m
1778417543.0701358 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:23,069] Materialisation performed by reasoner in 113.2ms[0m
1778417544.6782832 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:24,675] Updating ['default'] with:
1778417544.6808307 [knowledge_core-5] 	- detected_tomato_179_336 rdf:type Tomato
1778417544.6817729 [knowledge_core-5] 	- myself sees detected_tomato_179_336 (lifespan: 4.0sec)[0m
1778417544.6828442 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:24,676] This statement will expire on 10/05/2026, 12:52:28[0m
1778417545.0063694 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:25,005] Materialisation performed by reasoner in 328.2ms[0m
1778417545.8181007 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:25,817] Updating ['default'] with:
1778417545.8183117 [knowledge_core-5] 	- detected_tomato_179_336 rdf:type Tomato
1778417545.8184094 [knowledge_core-5] 	- myself sees detected_tomato_179_336 (lifespan: 4.0sec)[0m
1778417545.8189788 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:25,817] Updating expiry date to 10/05/2026, 12:52:29[0m
1778417545.9261661 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:25,925] Materialisation performed by reasoner in 107.2ms[0m
1778417549.2578425 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:52:29,256] Removing expired statement <detected_tomato_179_336 rdf:type Tomato> from <default> (expired on 2026-05-10T12:52:29+00:00)[0m
1778417549.2663257 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:52:29,257] Removing expired statement <myself sees detected_tomato_179_336> from <default> (expired on 2026-05-10T12:52:29+00:00)[0m
1778417549.3829629 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:29,382] Materialisation performed by reasoner in 124.8ms[0m
1778417551.3883924 [hri_face_detect_yunet-11] [33m[WARN] [1778417551.388025129] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 13.2sec[0m
1778417555.3757381 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:35,374] Updating ['default'] with:
1778417555.3762491 [knowledge_core-5] 	- detected_tomato_195_344 rdf:type Tomato
1778417555.3767183 [knowledge_core-5] 	- myself sees detected_tomato_195_344 (lifespan: 4.0sec)[0m
1778417555.3784225 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:35,374] This statement will expire on 10/05/2026, 12:52:39[0m
1778417555.5164454 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:35,515] Materialisation performed by reasoner in 140.2ms[0m
1778417555.7808046 [hri_face_detect_yunet-11] [33m[WARN] [1778417555.777588116] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.4sec[0m
1778417559.7167754 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:52:39,711] Removing expired statement <detected_tomato_195_344 rdf:type Tomato> from <default> (expired on 2026-05-10T12:52:39+00:00)[0m
1778417559.7176678 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:52:39,712] Removing expired statement <myself sees detected_tomato_195_344> from <default> (expired on 2026-05-10T12:52:39+00:00)[0m
1778417559.8835266 [knowledge_core-5] [37m[INFO] [2026-05-10 12:52:39,882] Materialisation performed by reasoner in 168.9ms[0m
1778417583.5038981 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:03,502] Deleting from ['default']:
1778417583.5041304 [knowledge_core-5] 	- anonymous_person_jcedc rdf:type Human[0m
1778417583.6111369 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:03,610] Materialisation performed by reasoner in 106.9ms[0m
1778417583.7562265 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:03,754] Deleting from ['default']:
1778417583.7566528 [knowledge_core-5] 	- myself sees anonymous_person_jcedc[0m
1778417583.8774929 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:03,876] Materialisation performed by reasoner in 121.1ms[0m
1778417584.7602184 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:04,759] Updating ['default'] with:
1778417584.7605717 [knowledge_core-5] 	- anonymous_person_eache rdf:type Human[0m
1778417584.8888254 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:04,888] Materialisation performed by reasoner in 128.5ms[0m
1778417585.0532677 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:05,051] Updating ['default'] with:
1778417585.0538833 [knowledge_core-5] 	- myself sees anonymous_person_eache[0m
1778417585.3958588 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:05,395] Materialisation performed by reasoner in 343.1ms[0m
1778417585.5588360 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:05,556] Updating ['default'] with:
1778417585.5595238 [knowledge_core-5] 	- anonymous_person_eache sees myself[0m
1778417585.7046580 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:05,703] Materialisation performed by reasoner in 145.5ms[0m
1778417585.8561027 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:05,853] Deleting from ['default']:
1778417585.8569636 [knowledge_core-5] 	- anonymous_person_eache rdf:type Human[0m
1778417585.9821692 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:05,981] Materialisation performed by reasoner in 126.5ms[0m
1778417586.1362729 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:06,135] Deleting from ['default']:
1778417586.1364641 [knowledge_core-5] 	- myself sees anonymous_person_eache[0m
1778417586.2447054 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:06,244] Materialisation performed by reasoner in 108.2ms[0m
1778417586.6785426 [hri_face_detect_yunet-11] [33m[WARN] [1778417586.678136409] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 30.9sec[0m
1778417586.6811328 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:06,680] Updating ['default'] with:
1778417586.6814768 [knowledge_core-5] 	- anonymous_person_jbehb rdf:type Human[0m
1778417586.8339303 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:06,832] Materialisation performed by reasoner in 152.2ms[0m
1778417586.9956534 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:06,992] Updating ['default'] with:
1778417586.9963515 [knowledge_core-5] 	- myself sees anonymous_person_jbehb[0m
1778417587.0834186 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:07,082] Materialisation performed by reasoner in 88.8ms[0m
1778417587.2662928 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:07,265] Updating ['default'] with:
1778417587.2666187 [knowledge_core-5] 	- anonymous_person_jbehb sees myself[0m
1778417587.6772857 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:07,675] Materialisation performed by reasoner in 409.5ms[0m
1778417591.2704215 [hri_face_detect_yunet-11] [33m[WARN] [1778417591.269435143] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.6sec[0m
1778417602.8835630 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:22,879] Deleting from ['default']:
1778417602.8842762 [knowledge_core-5] 	- anonymous_person_jbehb rdf:type Human[0m
1778417603.0230050 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:23,021] Materialisation performed by reasoner in 140.7ms[0m
1778417603.1712492 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:23,167] Deleting from ['default']:
1778417603.1725261 [knowledge_core-5] 	- myself sees anonymous_person_jbehb[0m
1778417603.2989781 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:23,298] Materialisation performed by reasoner in 129.1ms[0m
1778417606.2096343 [hri_face_detect_yunet-11] [33m[WARN] [1778417606.208985816] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 14.9sec[0m
1778417611.7190673 [hri_face_detect_yunet-11] [33m[WARN] [1778417611.717305154] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.5sec[0m
1778417611.9883010 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:31,986] Updating ['default'] with:
1778417611.9898655 [knowledge_core-5] 	- anonymous_person_hfeia rdf:type Human[0m
1778417612.1207891 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:32,120] Materialisation performed by reasoner in 132.4ms[0m
1778417612.2760913 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:32,274] Updating ['default'] with:
1778417612.2766011 [knowledge_core-5] 	- myself sees anonymous_person_hfeia[0m
1778417612.5992398 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:32,598] Materialisation performed by reasoner in 323.4ms[0m
1778417612.7447360 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:32,744] Updating ['default'] with:
1778417612.7449522 [knowledge_core-5] 	- anonymous_person_hfeia sees myself[0m
1778417612.8871841 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:32,885] Materialisation performed by reasoner in 141.6ms[0m
1778417614.6351156 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:34,634] Deleting from ['default']:
1778417614.6354024 [knowledge_core-5] 	- anonymous_person_hfeia sees myself[0m
1778417614.7480032 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:34,747] Materialisation performed by reasoner in 112.6ms[0m
1778417615.9780734 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:35,976] Deleting from ['default']:
1778417615.9786081 [knowledge_core-5] 	- anonymous_person_hfeia rdf:type Human[0m
1778417616.0865297 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:36,085] Materialisation performed by reasoner in 109.0ms[0m
1778417616.2469239 [hri_face_detect_yunet-11] [33m[WARN] [1778417616.246454445] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.5sec[0m
1778417616.2562485 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:36,255] Deleting from ['default']:
1778417616.2565567 [knowledge_core-5] 	- myself sees anonymous_person_hfeia[0m
1778417616.3385887 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:36,337] Materialisation performed by reasoner in 81.6ms[0m
1778417617.1397130 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:37,137] Updating ['default'] with:
1778417617.1406116 [knowledge_core-5] 	- anonymous_person_icghb rdf:type Human[0m
1778417617.2701607 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:37,269] Materialisation performed by reasoner in 130.3ms[0m
1778417617.6652400 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:37,664] Updating ['default'] with:
1778417617.6658697 [knowledge_core-5] 	- myself sees anonymous_person_icghb[0m
1778417617.7825878 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:37,781] Materialisation performed by reasoner in 117.1ms[0m
1778417617.9605558 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:37,949] Updating ['default'] with:
1778417617.9609354 [knowledge_core-5] 	- anonymous_person_icghb sees myself[0m
1778417618.0959008 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:38,094] Materialisation performed by reasoner in 143.9ms[0m
1778417618.2412224 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:38,240] Deleting from ['default']:
1778417618.2416084 [knowledge_core-5] 	- anonymous_person_icghb rdf:type Human[0m
1778417618.3173993 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:38,316] Materialisation performed by reasoner in 76.3ms[0m
1778417618.4603477 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:38,459] Deleting from ['default']:
1778417618.4605346 [knowledge_core-5] 	- myself sees anonymous_person_icghb[0m
1778417618.5960605 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:38,595] Materialisation performed by reasoner in 135.0ms[0m
1778417618.9699907 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:38,969] Updating ['default'] with:
1778417618.9702404 [knowledge_core-5] 	- anonymous_person_ecaab rdf:type Human[0m
1778417619.1105409 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:39,109] Materialisation performed by reasoner in 139.8ms[0m
1778417619.2589700 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:39,258] Updating ['default'] with:
1778417619.2591605 [knowledge_core-5] 	- myself sees anonymous_person_ecaab[0m
1778417619.6371439 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:39,636] Materialisation performed by reasoner in 377.8ms[0m
1778417619.8055675 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:39,803] Updating ['default'] with:
1778417619.8062773 [knowledge_core-5] 	- anonymous_person_ecaab sees myself[0m
1778417619.9321313 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:39,931] Materialisation performed by reasoner in 127.2ms[0m
1778417627.1417203 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:47,139] Deleting from ['default']:
1778417627.1480060 [knowledge_core-5] 	- anonymous_person_ecaab rdf:type Human[0m
1778417627.3216307 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:47,316] Materialisation performed by reasoner in 175.8ms[0m
1778417627.4680538 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:47,467] Deleting from ['default']:
1778417627.4683101 [knowledge_core-5] 	- myself sees anonymous_person_ecaab[0m
1778417627.5882905 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:47,587] Materialisation performed by reasoner in 120.3ms[0m
1778417629.6011884 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:49,599] Updating ['default'] with:
1778417629.6018326 [knowledge_core-5] 	- anonymous_person_bgfha rdf:type Human[0m
1778417629.7333453 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:49,732] Materialisation performed by reasoner in 131.9ms[0m
1778417629.8851697 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:49,883] Updating ['default'] with:
1778417629.8854928 [knowledge_core-5] 	- myself sees anonymous_person_bgfha[0m
1778417630.2984297 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:50,297] Materialisation performed by reasoner in 413.2ms[0m
1778417630.4705124 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:50,468] Updating ['default'] with:
1778417630.4710348 [knowledge_core-5] 	- anonymous_person_bgfha sees myself[0m
1778417630.6019144 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:50,600] Materialisation performed by reasoner in 130.6ms[0m
1778417630.6751053 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:50,673] Deleting from ['default']:
1778417630.6756229 [knowledge_core-5] 	- anonymous_person_bgfha rdf:type Human[0m
1778417630.8321416 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:50,831] Materialisation performed by reasoner in 157.0ms[0m
1778417630.9851005 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:50,982] Updating ['default'] with:
1778417630.9854653 [knowledge_core-5] 	- anonymous_person_egcid rdf:type Human[0m
1778417631.1101217 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:51,109] Materialisation performed by reasoner in 125.4ms[0m
1778417631.1830845 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:51,181] Deleting from ['default']:
1778417631.1834488 [knowledge_core-5] 	- myself sees anonymous_person_bgfha[0m
1778417631.3113127 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:51,310] Materialisation performed by reasoner in 128.3ms[0m
1778417631.5213964 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:51,482] Updating ['default'] with:
1778417631.5216565 [knowledge_core-5] 	- myself sees anonymous_person_egcid[0m
1778417631.6928251 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:51,691] Materialisation performed by reasoner in 208.1ms[0m
1778417631.8385785 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:51,837] Deleting from ['default']:
1778417631.8387606 [knowledge_core-5] 	- anonymous_person_egcid rdf:type Human[0m
1778417632.1827166 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:52,182] Materialisation performed by reasoner in 343.9ms[0m
1778417632.3285692 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:52,327] Deleting from ['default']:
1778417632.3289132 [knowledge_core-5] 	- myself sees anonymous_person_egcid[0m
1778417632.4435580 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:52,443] Materialisation performed by reasoner in 115.0ms[0m
1778417636.6568975 [hri_face_detect_yunet-11] [33m[WARN] [1778417636.654222005] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 20.4sec[0m
1778417638.2354980 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:58,233] Updating ['default'] with:
1778417638.2360475 [knowledge_core-5] 	- anonymous_person_cdccc rdf:type Human[0m
1778417638.3580761 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:58,357] Materialisation performed by reasoner in 122.9ms[0m
1778417638.5064249 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:58,505] Updating ['default'] with:
1778417638.5066929 [knowledge_core-5] 	- myself sees anonymous_person_cdccc[0m
1778417638.6156623 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:58,615] Materialisation performed by reasoner in 109.1ms[0m
1778417638.7709596 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:58,770] Updating ['default'] with:
1778417638.7712479 [knowledge_core-5] 	- anonymous_person_cdccc sees myself[0m
1778417638.8981991 [knowledge_core-5] [37m[INFO] [2026-05-10 12:53:58,897] Materialisation performed by reasoner in 127.1ms[0m
1778417640.2923388 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:00,291] Updating ['default'] with:
1778417640.2926745 [knowledge_core-5] 	- myself sees detected_blueberry_341_183
1778417640.2928891 [knowledge_core-5] 	- detected_blueberry_341_183 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417640.2942576 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:00,291] This statement will expire on 10/05/2026, 12:54:04[0m
1778417640.4235313 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:00,422] Materialisation performed by reasoner in 130.3ms[0m
1778417644.1128390 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:54:04,111] Removing expired statement <myself sees detected_blueberry_341_183> from <default> (expired on 2026-05-10T12:54:04+00:00)[0m
1778417644.1130600 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:54:04,111] Removing expired statement <detected_blueberry_341_183 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:54:04+00:00)[0m
1778417644.2669010 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:04,265] Materialisation performed by reasoner in 153.3ms[0m
1778417645.7240968 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:05,722] Deleting from ['default']:
1778417645.7265277 [knowledge_core-5] 	- anonymous_person_cdccc rdf:type Human[0m
1778417645.8505361 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:05,849] Materialisation performed by reasoner in 126.9ms[0m
1778417645.9949222 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:05,993] Deleting from ['default']:
1778417645.9964452 [knowledge_core-5] 	- myself sees anonymous_person_cdccc[0m
1778417646.1294060 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:06,128] Materialisation performed by reasoner in 133.6ms[0m
1778417649.0179737 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:09,017] Updating ['default'] with:
1778417649.0183103 [knowledge_core-5] 	- anonymous_person_cdaad rdf:type Human[0m
1778417649.1369007 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:09,136] Materialisation performed by reasoner in 118.6ms[0m
1778417649.3001640 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:09,298] Updating ['default'] with:
1778417649.3010483 [knowledge_core-5] 	- myself sees anonymous_person_cdaad[0m
1778417649.4486098 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:09,447] Materialisation performed by reasoner in 148.7ms[0m
1778417649.6092496 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:09,607] Updating ['default'] with:
1778417649.6110959 [knowledge_core-5] 	- anonymous_person_cdaad sees myself[0m
1778417650.0415387 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:10,040] Materialisation performed by reasoner in 432.5ms[0m
1778417650.8686035 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:10,865] Updating ['default'] with:
1778417650.8691618 [knowledge_core-5] 	- myself sees detected_blueberry_356_182
1778417650.8696034 [knowledge_core-5] 	- detected_blueberry_356_182 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417650.8707435 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:10,866] This statement will expire on 10/05/2026, 12:54:14[0m
1778417651.0057492 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:11,005] Materialisation performed by reasoner in 137.7ms[0m
1778417653.3410954 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:13,339] Updating ['default'] with:
1778417653.3416398 [knowledge_core-5] 	- myself sees detected_blueberry_341_184
1778417653.3421323 [knowledge_core-5] 	- detected_blueberry_341_184 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417653.3431761 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:13,340] This statement will expire on 10/05/2026, 12:54:17[0m
1778417653.4728751 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:13,472] Materialisation performed by reasoner in 130.8ms[0m
1778417654.4319255 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:14,428] Updating ['default'] with:
1778417654.4331560 [knowledge_core-5] 	- myself sees detected_blueberry_341_184
1778417654.4339755 [knowledge_core-5] 	- detected_blueberry_341_184 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417654.4381125 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:14,434] Updating expiry date to 10/05/2026, 12:54:18[0m
1778417654.5844593 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:14,583] Materialisation performed by reasoner in 147.7ms[0m
1778417654.6511497 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:54:14,648] Removing expired statement <myself sees detected_blueberry_356_182> from <default> (expired on 2026-05-10T12:54:14+00:00)[0m
1778417654.6516080 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:54:14,648] Removing expired statement <detected_blueberry_356_182 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:54:14+00:00)[0m
1778417654.7774529 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:14,776] Materialisation performed by reasoner in 127.8ms[0m
1778417655.4106021 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:15,407] Updating ['default'] with:
1778417655.4115629 [knowledge_core-5] 	- myself sees detected_blueberry_341_184
1778417655.4124589 [knowledge_core-5] 	- detected_blueberry_341_184 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417655.4156623 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:15,409] Updating expiry date to 10/05/2026, 12:54:19[0m
1778417655.8293920 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:15,828] Materialisation performed by reasoner in 417.0ms[0m
1778417656.9669039 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:16,964] Updating ['default'] with:
1778417656.9695146 [knowledge_core-5] 	- myself sees detected_blueberry_341_184
1778417656.9699426 [knowledge_core-5] 	- detected_blueberry_341_184 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417656.9718151 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:16,966] Updating expiry date to 10/05/2026, 12:54:20[0m
1778417657.1013331 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:17,100] Materialisation performed by reasoner in 133.4ms[0m
1778417660.4111838 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:20,398] Deleting from ['default']:
1778417660.4162426 [knowledge_core-5] 	- anonymous_person_cdaad rdf:type Human[0m
1778417660.5532041 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:20,552] Materialisation performed by reasoner in 153.2ms[0m
1778417660.6971498 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:20,696] Deleting from ['default']:
1778417660.6973903 [knowledge_core-5] 	- myself sees anonymous_person_cdaad[0m
1778417660.8077958 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:20,804] Materialisation performed by reasoner in 107.6ms[0m
1778417661.0592101 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:54:21,058] Removing expired statement <myself sees detected_blueberry_341_184> from <default> (expired on 2026-05-10T12:54:20+00:00)[0m
1778417661.0599389 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:54:21,058] Removing expired statement <detected_blueberry_341_184 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:54:20+00:00)[0m
1778417661.1823344 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:21,181] Materialisation performed by reasoner in 122.8ms[0m
1778417663.3882396 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:23,386] Updating ['default'] with:
1778417663.3888228 [knowledge_core-5] 	- anonymous_person_effcb rdf:type Human[0m
1778417663.5087450 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:23,507] Materialisation performed by reasoner in 120.8ms[0m
1778417663.6741087 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:23,672] Updating ['default'] with:
1778417663.6751866 [knowledge_core-5] 	- myself sees anonymous_person_effcb[0m
1778417664.0955815 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:24,091] Materialisation performed by reasoner in 418.3ms[0m
1778417664.2548907 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:24,254] Updating ['default'] with:
1778417664.2551641 [knowledge_core-5] 	- anonymous_person_effcb sees myself[0m
1778417664.3960750 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:24,395] Materialisation performed by reasoner in 140.7ms[0m
1778417665.1585736 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:25,155] Deleting from ['default']:
1778417665.1598899 [knowledge_core-5] 	- anonymous_person_effcb rdf:type Human[0m
1778417665.3345246 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:25,333] Materialisation performed by reasoner in 177.7ms[0m
1778417665.4793346 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:25,478] Deleting from ['default']:
1778417665.4795995 [knowledge_core-5] 	- myself sees anonymous_person_effcb[0m
1778417665.6352255 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:25,633] Materialisation performed by reasoner in 154.9ms[0m
1778417665.8895550 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:25,889] Updating ['default'] with:
1778417665.8897209 [knowledge_core-5] 	- anonymous_person_iiaeg rdf:type Human[0m
1778417666.0357029 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:26,034] Materialisation performed by reasoner in 145.2ms[0m
1778417666.1818497 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:26,181] Updating ['default'] with:
1778417666.1822305 [knowledge_core-5] 	- myself sees anonymous_person_iiaeg[0m
1778417666.5707285 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:26,569] Materialisation performed by reasoner in 387.8ms[0m
1778417666.7333229 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:26,731] Updating ['default'] with:
1778417666.7342651 [knowledge_core-5] 	- anonymous_person_iiaeg sees myself[0m
1778417666.8835740 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:26,882] Materialisation performed by reasoner in 150.6ms[0m
1778417667.0269756 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:27,025] Updating ['default'] with:
1778417667.0273757 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417667.0276337 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417667.0293744 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:27,026] This statement will expire on 10/05/2026, 12:54:31[0m
1778417667.1512675 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:27,149] Materialisation performed by reasoner in 122.5ms[0m
1778417667.9467783 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:27,945] Updating ['default'] with:
1778417667.9482760 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417667.9485769 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417667.9487762 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:27,946] Updating expiry date to 10/05/2026, 12:54:31[0m
1778417668.0852609 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:28,083] Materialisation performed by reasoner in 136.7ms[0m
1778417669.6586196 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:29,657] Updating ['default'] with:
1778417669.6613004 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417669.6641233 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417669.6643100 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:29,659] Updating expiry date to 10/05/2026, 12:54:33[0m
1778417669.7824812 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:29,781] Materialisation performed by reasoner in 121.5ms[0m
1778417670.7520113 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:30,750] Updating ['default'] with:
1778417670.7524531 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417670.7528527 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417670.7538760 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:30,751] Updating expiry date to 10/05/2026, 12:54:34[0m
1778417670.8963065 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:30,895] Materialisation performed by reasoner in 143.0ms[0m
1778417671.8596568 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:31,857] Updating ['default'] with:
1778417671.8603094 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417671.8637795 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417671.8646579 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:31,859] Updating expiry date to 10/05/2026, 12:54:35[0m
1778417672.2665627 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:32,264] Materialisation performed by reasoner in 404.7ms[0m
1778417672.9382093 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:32,937] Updating ['default'] with:
1778417672.9384458 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417672.9385092 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417672.9389665 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:32,938] Updating expiry date to 10/05/2026, 12:54:36[0m
1778417673.0834529 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:33,082] Materialisation performed by reasoner in 144.3ms[0m
1778417673.3874407 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:33,383] Deleting from ['default']:
1778417673.3878026 [knowledge_core-5] 	- anonymous_person_iiaeg rdf:type Human[0m
1778417673.5401161 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:33,539] Materialisation performed by reasoner in 153.8ms[0m
1778417673.7103324 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:33,709] Deleting from ['default']:
1778417673.7106414 [knowledge_core-5] 	- myself sees anonymous_person_iiaeg[0m
1778417673.8025541 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:33,801] Materialisation performed by reasoner in 91.8ms[0m
1778417674.0947602 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:34,092] Updating ['default'] with:
1778417674.0953224 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417674.0957427 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417674.0974560 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:34,093] Updating expiry date to 10/05/2026, 12:54:38[0m
1778417674.2301347 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:34,229] Materialisation performed by reasoner in 134.3ms[0m
1778417674.4514832 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:34,450] Updating ['default'] with:
1778417674.4518366 [knowledge_core-5] 	- anonymous_person_aabjj rdf:type Human[0m
1778417674.5654414 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:34,564] Materialisation performed by reasoner in 113.9ms[0m
1778417674.7271082 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:34,726] Updating ['default'] with:
1778417674.7278712 [knowledge_core-5] 	- myself sees anonymous_person_aabjj[0m
1778417675.0714386 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:35,070] Materialisation performed by reasoner in 344.5ms[0m
1778417675.2208641 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:35,219] Updating ['default'] with:
1778417675.2220490 [knowledge_core-5] 	- anonymous_person_aabjj sees myself[0m
1778417675.3452427 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:35,344] Materialisation performed by reasoner in 123.1ms[0m
1778417675.4990737 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:35,493] Updating ['default'] with:
1778417675.5019252 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417675.5024619 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417675.5051525 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:35,496] Updating expiry date to 10/05/2026, 12:54:39[0m
1778417675.6355815 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:35,634] Materialisation performed by reasoner in 136.7ms[0m
1778417676.5210557 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:36,517] Updating ['default'] with:
1778417676.5220032 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417676.5227129 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417676.5262356 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:36,519] Updating expiry date to 10/05/2026, 12:54:40[0m
1778417676.6667778 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:36,665] Materialisation performed by reasoner in 143.7ms[0m
1778417677.6155858 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:37,614] Updating ['default'] with:
1778417677.6158969 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417677.6160135 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417677.6166933 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:37,615] Updating expiry date to 10/05/2026, 12:54:41[0m
1778417677.7512076 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:37,750] Materialisation performed by reasoner in 135.0ms[0m
1778417678.7272243 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:38,726] Updating ['default'] with:
1778417678.7277262 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417678.7297916 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417678.7308738 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:38,726] Updating expiry date to 10/05/2026, 12:54:42[0m
1778417679.1012216 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:39,099] Materialisation performed by reasoner in 372.9ms[0m
1778417679.2646320 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:39,263] Deleting from ['default']:
1778417679.2648954 [knowledge_core-5] 	- anonymous_person_aabjj rdf:type Human[0m
1778417679.4476788 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:39,446] Materialisation performed by reasoner in 182.8ms[0m
1778417679.6101587 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:39,608] Updating ['default'] with:
1778417679.6108377 [knowledge_core-5] 	- anonymous_person_jaidh rdf:type Human[0m
1778417679.7508683 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:39,750] Materialisation performed by reasoner in 141.2ms[0m
1778417679.8234186 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:39,822] Deleting from ['default']:
1778417679.8237495 [knowledge_core-5] 	- myself sees anonymous_person_aabjj[0m
1778417679.9391873 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:39,938] Materialisation performed by reasoner in 115.8ms[0m
1778417680.0969403 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:40,093] Updating ['default'] with:
1778417680.0981860 [knowledge_core-5] 	- myself sees anonymous_person_jaidh[0m
1778417680.2486825 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:40,247] Materialisation performed by reasoner in 152.7ms[0m
1778417680.3305585 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:40,327] Updating ['default'] with:
1778417680.3308716 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417680.3311241 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417680.3312864 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:40,328] Updating expiry date to 10/05/2026, 12:54:44[0m
1778417680.4689040 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:40,467] Materialisation performed by reasoner in 138.6ms[0m
1778417680.6221461 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:40,620] Updating ['default'] with:
1778417680.6226161 [knowledge_core-5] 	- anonymous_person_jaidh sees myself[0m
1778417680.9702034 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:40,969] Materialisation performed by reasoner in 348.0ms[0m
1778417681.1901090 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:41,189] Updating ['default'] with:
1778417681.1903048 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417681.1904066 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417681.1909168 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:41,189] Updating expiry date to 10/05/2026, 12:54:45[0m
1778417681.3027313 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:41,302] Materialisation performed by reasoner in 112.2ms[0m
1778417682.2721708 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:42,271] Updating ['default'] with:
1778417682.2724221 [knowledge_core-5] 	- detected_blueberry_390_172 rdf:type Blueberry
1778417682.2725024 [knowledge_core-5] 	- myself sees detected_blueberry_390_172 (lifespan: 4.0sec)[0m
1778417682.2730241 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:42,271] Updating expiry date to 10/05/2026, 12:54:46[0m
1778417682.4440246 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:42,442] Materialisation performed by reasoner in 170.5ms[0m
1778417683.0455503 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:43,044] Deleting from ['default']:
1778417683.0459013 [knowledge_core-5] 	- anonymous_person_jaidh rdf:type Human[0m
1778417683.1926954 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:43,192] Materialisation performed by reasoner in 147.2ms[0m
1778417683.3383458 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:43,336] Deleting from ['default']:
1778417683.3388314 [knowledge_core-5] 	- myself sees anonymous_person_jaidh[0m
1778417683.4137661 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:43,412] Materialisation performed by reasoner in 75.7ms[0m
1778417684.1855178 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:44,184] Updating ['default'] with:
1778417684.1860707 [knowledge_core-5] 	- anonymous_person_jdifh rdf:type Human[0m
1778417684.2979343 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:44,297] Materialisation performed by reasoner in 112.2ms[0m
1778417684.4478209 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:44,445] Updating ['default'] with:
1778417684.4482083 [knowledge_core-5] 	- myself sees anonymous_person_jdifh[0m
1778417684.8038046 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:44,803] Materialisation performed by reasoner in 357.1ms[0m
1778417684.9876924 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:44,986] Updating ['default'] with:
1778417684.9879274 [knowledge_core-5] 	- anonymous_person_jdifh sees myself[0m
1778417685.1066375 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:45,106] Materialisation performed by reasoner in 118.8ms[0m
1778417686.9719875 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:54:46,970] Removing expired statement <detected_blueberry_390_172 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:54:46+00:00)[0m
1778417686.9727092 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:54:46,971] Removing expired statement <myself sees detected_blueberry_390_172> from <default> (expired on 2026-05-10T12:54:46+00:00)[0m
1778417687.0915313 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:47,090] Materialisation performed by reasoner in 119.4ms[0m
1778417692.3735337 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:52,371] Deleting from ['default']:
1778417692.3766918 [knowledge_core-5] 	- anonymous_person_jdifh rdf:type Human[0m
1778417692.5168090 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:52,516] Materialisation performed by reasoner in 143.4ms[0m
1778417692.6612189 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:52,660] Deleting from ['default']:
1778417692.6615543 [knowledge_core-5] 	- myself sees anonymous_person_jdifh[0m
1778417692.7840912 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:52,783] Materialisation performed by reasoner in 122.8ms[0m
1778417694.3066618 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:54,305] Updating ['default'] with:
1778417694.3069735 [knowledge_core-5] 	- anonymous_person_ehcfa rdf:type Human[0m
1778417694.6456008 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:54,645] Materialisation performed by reasoner in 339.0ms[0m
1778417694.8223054 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:54,821] Updating ['default'] with:
1778417694.8225157 [knowledge_core-5] 	- myself sees anonymous_person_ehcfa[0m
1778417694.9320698 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:54,931] Materialisation performed by reasoner in 109.5ms[0m
1778417695.0849557 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:55,083] Updating ['default'] with:
1778417695.0853393 [knowledge_core-5] 	- anonymous_person_ehcfa sees myself[0m
1778417695.2209113 [knowledge_core-5] [37m[INFO] [2026-05-10 12:54:55,220] Materialisation performed by reasoner in 136.3ms[0m
1778417703.8802273 [start_manager-23] [0m[INFO] [1778417703.877243424] [dialogue_manager]: [SPEECH INPUT] voice_id="anonymous_speaker": "What can you see right now?" (locale=, confidence=0.00)[0m
1778417703.8941040 [start_manager-23] [0m[INFO] [1778417703.893560254] [dialogue_manager]: [SPEECH INPUT] Using default dialogue 973c0f1e-a654-42da-b888-6b3e1078600d[0m
1778417703.8991585 [start_manager-23] [0m[INFO] [1778417703.898709444] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="anonymous_speaker", text="What can you see right now?"[0m
1778417703.9448035 [start_node-22] [0m[INFO] [1778417703.942834645] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:4 input=What can you see right now?[0m
1778417704.0715487 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:04,069] Searching ['?entity', '?type'] in models {'default'} matching:
1778417704.0722952 [knowledge_core-5] 	- myself sees ?entity
1778417704.0728009 [knowledge_core-5] 	- ?entity rdf:type ?type[0m
1778417704.0878310 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:04,086] Found: [{'entity': 'anonymous_person_ehcfa', 'type': 'cyc:SolidTangibleThing'}, {'entity': 'anonymous_person_ehcfa', 'type': 'Human'}, {'entity': 'anonymous_person_ehcfa', 'type': 'foaf:Person'}, {'entity': 'anonymous_person_ehcfa', 'type': 'cyc:SpatialThing-Localized'}, {'entity': 'anonymous_person_ehcfa', 'type': 'Agent'}, {'entity': 'anonymous_person_ehcfa', 'type': 'foaf:Agent'}, {'entity': 'anonymous_person_ehcfa', 'type': 'owl:Thing'}, {'entity': 'anonymous_person_ehcfa', 'type': 'cyc:PartiallyTangible'}, {'entity': 'anonymous_person_ehcfa', 'type': 'EmbodiedAgent'}, {'entity': 'anonymous_person_ehcfa', 'type': 'cyc:SpatialThing'}, {'entity': 'anonymous_person_ehcfa', 'type': 'Location'}, {'entity': 'anonymous_person_ehcfa', 'type': 'cyc:EnduringThing-Localized'}][0m
1778417704.1051288 [start_node-22] [0m[INFO] [1778417704.104436311] [chatbot_llm]: [turn:__default__:4] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1778417704.1089718 [start_node-22] [0m[INFO] [1778417704.108543974] [chatbot_llm]: [turn:__default__:4] TURN_START | user="What can you see right now?"[0m
1778417704.1124871 [start_node-22] [0m[INFO] [1778417704.111665530] [chatbot_llm]: [turn:__default__:4] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=7 timeout=60.0s[0m
1778417704.5007694 [start_node-22] [0m[INFO] [1778417704.499475542] [chatbot_llm]: [turn:__default__:4] ROUTE_RESOLVED | route=dialogue intent=- source=llm_response_route confidence=0.00[0m
1778417704.5040090 [start_node-22] [0m[INFO] [1778417704.502610302] [chatbot_llm]: [turn:__default__:4] TURN_DONE | planner-mode response complete[0m
1778417704.5177279 [start_manager-23] [0m[INFO] [1778417704.517314739] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "I can see an anonymous person currently."[0m
1778417704.5195785 [start_manager-23] [0m[INFO] [1778417704.519064308] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1778417704.5221312 [start_manager-23] [0m[INFO] [1778417704.521786159] [dialogue_manager]: [TTS] Speaking text (priority=128): "I can see an anonymous person currently."[0m
1778417704.6421146 [start_skill-26] [0m[INFO] [1778417704.641040426] [nao_say_skill]: [turn:I can see an anonymous person cu] TTS_ACCEPTED | goal accepted[0m
1778417704.6869340 [start_skill-26] [0m[INFO] [1778417704.686401840] [nao_say_skill]: [turn:I can see an anonymous person cu] SAY_START | text_len=40 lang=en-US volume=1.00[0m
1778417704.6959786 [start_skill-26] [0m[INFO] [1778417704.694520566] [nao_say_skill]: [turn:I can see an anonymous person cu] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=40[0m
1778417704.7225127 [start_skill-26] [0m[INFO] [1778417704.722101613] [nao_say_skill]: [turn:I can see an anonymous person cu] DEBUG_TTS_FORWARD | action="/debug/say" text_len=40[0m
1778417704.7285349 [start_skill-26] [33m[WARN] [1778417704.728120166] [nao_say_skill]: [turn:I can see an anonymous person cu] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1778417704.7310784 [start_skill-26] [0m[INFO] [1778417704.729552906] [nao_say_skill]: [turn:I can see an anonymous person cu] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=40 subscribers=0[0m
1778417704.7965844 [start_skill-26] [0m[INFO] [1778417704.796105091] [nao_say_skill]: [turn:I can see an anonymous person cu] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1778417707.6816399 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:07,674] Deleting from ['default']:
1778417707.6837285 [knowledge_core-5] 	- anonymous_person_ehcfa rdf:type Human[0m
1778417707.8350568 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:07,834] Materialisation performed by reasoner in 155.1ms[0m
1778417707.9847887 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:07,983] Updating ['default'] with:
1778417707.9851394 [knowledge_core-5] 	- anonymous_person_fffig rdf:type Human[0m
1778417708.1059382 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:08,105] Materialisation performed by reasoner in 120.9ms[0m
1778417708.1879308 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:08,187] Deleting from ['default']:
1778417708.1882308 [knowledge_core-5] 	- myself sees anonymous_person_ehcfa[0m
1778417708.4642210 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:08,463] Materialisation performed by reasoner in 276.3ms[0m
1778417708.6132796 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:08,612] Updating ['default'] with:
1778417708.6135938 [knowledge_core-5] 	- myself sees anonymous_person_fffig[0m
1778417708.7413943 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:08,740] Materialisation performed by reasoner in 128.0ms[0m
1778417708.9018385 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:08,899] Updating ['default'] with:
1778417708.9024129 [knowledge_core-5] 	- anonymous_person_fffig sees myself[0m
1778417709.0360508 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:09,035] Materialisation performed by reasoner in 134.8ms[0m
1778417710.4624114 [start_manager-23] [0m[INFO] [1778417710.461830721] [dialogue_manager]: [SPEECH INPUT] voice_id="anonymous_speaker": "Whats the id?" (locale=, confidence=0.00)[0m
1778417710.4641588 [start_manager-23] [0m[INFO] [1778417710.463580728] [dialogue_manager]: [SPEECH INPUT] Using default dialogue 973c0f1e-a654-42da-b888-6b3e1078600d[0m
1778417710.4657991 [start_manager-23] [0m[INFO] [1778417710.464940016] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="anonymous_speaker", text="Whats the id?"[0m
1778417710.4730814 [start_node-22] [0m[INFO] [1778417710.472578538] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:5 input=Whats the id?[0m
1778417710.5317724 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:10,528] Searching ['?entity', '?type'] in models {'default'} matching:
1778417710.5323989 [knowledge_core-5] 	- myself sees ?entity
1778417710.5327697 [knowledge_core-5] 	- ?entity rdf:type ?type[0m
1778417710.5464966 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:10,545] Found: [{'entity': 'anonymous_person_fffig', 'type': 'cyc:SolidTangibleThing'}, {'entity': 'anonymous_person_fffig', 'type': 'Human'}, {'entity': 'anonymous_person_fffig', 'type': 'foaf:Person'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:SpatialThing-Localized'}, {'entity': 'anonymous_person_fffig', 'type': 'Agent'}, {'entity': 'anonymous_person_fffig', 'type': 'foaf:Agent'}, {'entity': 'anonymous_person_fffig', 'type': 'owl:Thing'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:PartiallyTangible'}, {'entity': 'anonymous_person_fffig', 'type': 'EmbodiedAgent'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:SpatialThing'}, {'entity': 'anonymous_person_fffig', 'type': 'Location'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:EnduringThing-Localized'}][0m
1778417710.5551414 [start_node-22] [0m[INFO] [1778417710.553631060] [chatbot_llm]: [turn:__default__:5] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1778417710.5577614 [start_node-22] [0m[INFO] [1778417710.555922078] [chatbot_llm]: [turn:__default__:5] TURN_START | user="Whats the id?"[0m
1778417710.5596418 [start_node-22] [0m[INFO] [1778417710.557539538] [chatbot_llm]: [turn:__default__:5] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=9 timeout=60.0s[0m
1778417710.9257514 [start_node-22] [0m[INFO] [1778417710.924803432] [chatbot_llm]: [turn:__default__:5] ROUTE_RESOLVED | route=dialogue intent=- source=llm_response_route confidence=0.00[0m
1778417710.9352283 [start_node-22] [0m[INFO] [1778417710.928824480] [chatbot_llm]: [turn:__default__:5] TURN_DONE | planner-mode response complete[0m
1778417710.9424760 [start_manager-23] [0m[INFO] [1778417710.941846924] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "The ID of the person currently visible is anonymous_speaker."[0m
1778417710.9451756 [start_manager-23] [0m[INFO] [1778417710.943916117] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1778417710.9467561 [start_manager-23] [0m[INFO] [1778417710.945628133] [dialogue_manager]: [TTS] Speaking text (priority=128): "The ID of the person currently visible is anonymous_speaker."[0m
1778417710.9524248 [start_skill-26] [0m[INFO] [1778417710.950626656] [nao_say_skill]: [turn:The ID of the person currently v] TTS_ACCEPTED | goal accepted[0m
1778417710.9561946 [start_skill-26] [0m[INFO] [1778417710.954307509] [nao_say_skill]: [turn:The ID of the person currently v] SAY_START | text_len=60 lang=en-US volume=1.00[0m
1778417710.9581616 [start_skill-26] [0m[INFO] [1778417710.955984299] [nao_say_skill]: [turn:The ID of the person currently v] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=60[0m
1778417710.9593990 [start_skill-26] [0m[INFO] [1778417710.958893451] [nao_say_skill]: [turn:The ID of the person currently v] DEBUG_TTS_FORWARD | action="/debug/say" text_len=60[0m
1778417710.9613636 [start_skill-26] [33m[WARN] [1778417710.961110801] [nao_say_skill]: [turn:The ID of the person currently v] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1778417710.9631133 [start_skill-26] [0m[INFO] [1778417710.962590494] [nao_say_skill]: [turn:The ID of the person currently v] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=60 subscribers=0[0m
1778417710.9718721 [start_skill-26] [0m[INFO] [1778417710.971250412] [nao_say_skill]: [turn:The ID of the person currently v] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1778417732.4520862 [hri_face_detect_yunet-11] [33m[WARN] [1778417732.451655469] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 95.8sec[0m
1778417734.9189126 [start_manager-23] [0m[INFO] [1778417734.918202556] [dialogue_manager]: [SPEECH INPUT] voice_id="anonymous_speaker": "Perfect, can you do a scan and tell me if you see the same person?" (locale=, confidence=0.00)[0m
1778417734.9214265 [start_manager-23] [0m[INFO] [1778417734.920870475] [dialogue_manager]: [SPEECH INPUT] Using default dialogue 973c0f1e-a654-42da-b888-6b3e1078600d[0m
1778417734.9324028 [start_manager-23] [0m[INFO] [1778417734.924027745] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="anonymous_speaker", text="Perfect, can you do a scan and tell me if you see the same person?"[0m
1778417734.9336765 [start_node-22] [0m[INFO] [1778417734.929415420] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:6 input=Perfect, can you do a scan and tell me if you see the same person?[0m
1778417734.9470856 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:34,945] Searching ['?entity', '?type'] in models {'default'} matching:
1778417734.9473100 [knowledge_core-5] 	- myself sees ?entity
1778417734.9474409 [knowledge_core-5] 	- ?entity rdf:type ?type[0m
1778417734.9523466 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:34,951] Found: [{'entity': 'anonymous_person_fffig', 'type': 'cyc:SolidTangibleThing'}, {'entity': 'anonymous_person_fffig', 'type': 'Human'}, {'entity': 'anonymous_person_fffig', 'type': 'foaf:Person'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:SpatialThing-Localized'}, {'entity': 'anonymous_person_fffig', 'type': 'Agent'}, {'entity': 'anonymous_person_fffig', 'type': 'foaf:Agent'}, {'entity': 'anonymous_person_fffig', 'type': 'owl:Thing'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:PartiallyTangible'}, {'entity': 'anonymous_person_fffig', 'type': 'EmbodiedAgent'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:SpatialThing'}, {'entity': 'anonymous_person_fffig', 'type': 'Location'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:EnduringThing-Localized'}][0m
1778417734.9557333 [start_node-22] [0m[INFO] [1778417734.955306736] [chatbot_llm]: [turn:__default__:6] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1778417734.9578321 [start_node-22] [0m[INFO] [1778417734.956467860] [chatbot_llm]: [turn:__default__:6] TURN_START | user="Perfect, can you do a scan and tell me if you see the same person?"[0m
1778417734.9588342 [start_node-22] [0m[INFO] [1778417734.957743941] [chatbot_llm]: [turn:__default__:6] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=11 timeout=60.0s[0m
1778417735.3733766 [start_node-22] [0m[INFO] [1778417735.372733688] [chatbot_llm]: [turn:__default__:6] ROUTE_RESOLVED | route=execution intent=- source=llm_response_inferred_route confidence=0.00[0m
1778417735.3757441 [start_node-22] [0m[INFO] [1778417735.374403560] [chatbot_llm]: [turn:__default__:6] TURN_DONE | planner-mode response complete[0m
1778417735.3960633 [start_node-22] [0m[INFO] [1778417735.395659758] [chatbot_llm]: [turn:__default__:6] PLANNER_REQUEST | published planner request on /nao_orchestrator/planner_request goal_id=goal_default___6 kind=new_goal[0m
1778417735.3999803 [start_manager-23] [0m[INFO] [1778417735.399620852] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "Sure, I will look around and report what I can see."[0m
1778417735.4011049 [start_manager-23] [0m[INFO] [1778417735.400880081] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1778417735.4018800 [start_manager-23] [0m[INFO] [1778417735.401667012] [dialogue_manager]: [TTS] Speaking text (priority=128): "Sure, I will look around and report what I can see."[0m
1778417735.4050620 [start_skill-26] [0m[INFO] [1778417735.404505774] [nao_say_skill]: [turn:Sure, I will look around and rep] TTS_ACCEPTED | goal accepted[0m
1778417735.4063020 [start_skill-26] [0m[INFO] [1778417735.406151401] [nao_say_skill]: [turn:Sure, I will look around and rep] SAY_START | text_len=51 lang=en-US volume=1.00[0m
1778417735.4084246 [start_skill-26] [0m[INFO] [1778417735.408211748] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=51[0m
1778417735.4096336 [start_skill-26] [0m[INFO] [1778417735.409474323] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_TTS_FORWARD | action="/debug/say" text_len=51[0m
1778417735.4107480 [start_skill-26] [33m[WARN] [1778417735.410641670] [nao_say_skill]: [turn:Sure, I will look around and rep] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1778417735.4116116 [start_skill-26] [0m[INFO] [1778417735.411470550] [nao_say_skill]: [turn:Sure, I will look around and rep] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=51 subscribers=0[0m
1778417735.4443271 [start_skill-26] [0m[INFO] [1778417735.443522853] [nao_say_skill]: [turn:Sure, I will look around and rep] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1778417735.4544120 [run_app-24] [0m[INFO] [1778417735.453259554] [nao_orchestrator]: Planner gate forwarded request | goal_id=goal_default___6 kind=new_goal active_goal=goal_default___6 topic=/planner/request[0m
1778417737.2563224 [hri_face_detect_yunet-11] [33m[WARN] [1778417737.254132517] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.8sec[0m
1778417738.1342905 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:38,133] Updating ['default'] with:
1778417738.1347461 [knowledge_core-5] 	- myself sees detected_blueberry_344_223
1778417738.1352863 [knowledge_core-5] 	- detected_blueberry_344_223 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417738.1364989 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:38,133] This statement will expire on 10/05/2026, 12:55:42[0m
1778417738.2702901 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:38,269] Materialisation performed by reasoner in 135.4ms[0m
1778417738.6581187 [run_app-24] [0m[INFO] [1778417738.656417463] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=0.000 pitch=0.000 relative=False[0m
1778417739.6860013 [run_app-24] [0m[INFO] [1778417739.685634168] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=0.450 pitch=0.000 relative=False[0m
1778417740.7152841 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:40,712] Updating ['default'] with:
1778417740.7156651 [knowledge_core-5] 	- detected_blueberry_336_219 rdf:type Blueberry
1778417740.7158914 [knowledge_core-5] 	- myself sees detected_blueberry_336_219 (lifespan: 4.0sec)[0m
1778417740.7181158 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:40,713] This statement will expire on 10/05/2026, 12:55:44[0m
1778417740.7335737 [run_app-24] [0m[INFO] [1778417740.733026673] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=-0.450 pitch=0.000 relative=False[0m
1778417740.8675117 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:40,866] Materialisation performed by reasoner in 152.5ms[0m
1778417741.7597597 [run_app-24] [0m[INFO] [1778417741.759305480] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=0.000 pitch=-0.200 relative=False[0m
1778417742.0597568 [hri_face_detect_yunet-11] [33m[WARN] [1778417742.059088213] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.8sec[0m
1778417742.2032778 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:55:42,202] Removing expired statement <myself sees detected_blueberry_344_223> from <default> (expired on 2026-05-10T12:55:42+00:00)[0m
1778417742.2038889 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:55:42,202] Removing expired statement <detected_blueberry_344_223 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:55:42+00:00)[0m
1778417742.3048086 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:42,304] Materialisation performed by reasoner in 101.2ms[0m
1778417742.7850015 [run_app-24] [0m[INFO] [1778417742.784627263] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=0.000 pitch=0.200 relative=False[0m
1778417742.7870121 [run_app-24] [0m[INFO] [1778417742.786509992] [nao_orchestrator]: ORCH SCAN | target=person target_kind=person result_mode=success[0m
1778417742.8239813 [start_manager-23] [0m[INFO] [1778417742.822768270] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="__system__", text="The robot has finished executing a user-requested task.
1778417742.8245211 [start_manager-23] Reply to the human with one short, natural sentence about the completed task.
1778417742.8248756 [start_manager-23] Do not propose new actions, mention planner internals, or repeat the initial acknowledgement.
1778417742.8251755 [start_manager-23] Use only the execution result and suggested factual content as facts; if they do not answer whether a requested person, object, or target was found, say that no confirmed result was available instead of guessing.
1778417742.8254411 [start_manager-23] Original user request: Perfect, can you do a scan and tell me if you see the same person?
1778417742.8261025 [start_manager-23] Execution result: {"backend":"emorobcare_cv","objects":[{"center_x":337.4,"center_y":231.5,"entity_id":"detected_blueberry_336_219","kb_class":"Blueberry","label":"blueberry","last_seen_sec":1778417740.805,"score":0.514,"source":"emorobcare_cv","tracker_id":""},{"center_x":330.4,"center_y":203.4,"entity_id":"detected_blueberry_344_223","kb_class":"Blueberry","label":"blueberry","last_seen_sec":1778417738.943,"score":0.574,"source":"emorobcare_cv","tracker_id":""}],"observer":"myself"}
1778417742.8268394 [start_manager-23] Suggested factual content: {"backend":"emorobcare_cv","objects":[{"center_x":337.4,"center_y":231.5,"entity_id":"detected_blueberry_336_219","kb_class":"Blueberry","label":"blueberry","last_seen_sec":1778417740.805,"score":0.514,"source":"emorobcare_cv","tracker_id":""},{"center_x":330.4,"center_y":203.4,"entity_id":"detected_blueberry_344_223","kb_class":"Blueberry","label":"blueberry","last_seen_sec":1778417738.943,"score":0.574,"source":"emorobcare_cv","tracker_id":""}],"observer":"myself"}"[0m
1778417742.8324869 [start_manager-23] [0m[INFO] [1778417742.828839126] [dialogue_manager]: [PLANNER ACT] Requested chatbot wording for goal_id=goal_default___6[0m
1778417742.8354568 [start_node-22] [0m[INFO] [1778417742.834047666] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=__system__ turn=__default__:7 input=The robot has finished executing a user-requested task. Reply to the ...[0m
1778417742.8584495 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:42,857] Searching ['?entity', '?type'] in models {'default'} matching:
1778417742.8588140 [knowledge_core-5] 	- myself sees ?entity
1778417742.8590138 [knowledge_core-5] 	- ?entity rdf:type ?type[0m
1778417742.8661678 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:42,865] Found: [{'entity': 'anonymous_person_fffig', 'type': 'cyc:SolidTangibleThing'}, {'entity': 'anonymous_person_fffig', 'type': 'Human'}, {'entity': 'anonymous_person_fffig', 'type': 'foaf:Person'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:SpatialThing-Localized'}, {'entity': 'anonymous_person_fffig', 'type': 'Agent'}, {'entity': 'anonymous_person_fffig', 'type': 'foaf:Agent'}, {'entity': 'anonymous_person_fffig', 'type': 'owl:Thing'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:PartiallyTangible'}, {'entity': 'anonymous_person_fffig', 'type': 'EmbodiedAgent'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:SpatialThing'}, {'entity': 'anonymous_person_fffig', 'type': 'Location'}, {'entity': 'anonymous_person_fffig', 'type': 'cyc:EnduringThing-Localized'}, {'entity': 'detected_blueberry_336_219', 'type': 'Blueberry'}, {'entity': 'detected_blueberry_336_219', 'type': 'cyc:SpatialThing-Localized'}, {'entity': 'detected_blueberry_336_219', 'type': 'owl:Thing'}, {'entity': 'detected_blueberry_336_219', 'type': 'cyc:SpatialThing'}, {'entity': 'detected_blueberry_336_219', 'type': 'Location'}, {'entity': 'detected_blueberry_336_219', 'type': 'cyc:EnduringThing-Localized'}][0m
1778417742.8699589 [start_node-22] [0m[INFO] [1778417742.869592138] [chatbot_llm]: [turn:__default__:7] KB_SNAPSHOT | loaded 293 chars from /kb/query[0m
1778417742.8713076 [start_node-22] [0m[INFO] [1778417742.871005352] [chatbot_llm]: [turn:__default__:7] TURN_START | user="The robot has finished executing a user-requested task. Reply to the ..."[0m
1778417742.8724980 [start_node-22] [0m[INFO] [1778417742.872284494] [chatbot_llm]: [turn:__default__:7] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=15 timeout=60.0s[0m
1778417743.1602256 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:43,158] Updating ['default'] with:
1778417743.1611669 [knowledge_core-5] 	- myself sees detected_blueberry_331_248
1778417743.1613173 [knowledge_core-5] 	- detected_blueberry_331_248 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417743.1696000 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:43,167] This statement will expire on 10/05/2026, 12:55:47[0m
1778417743.2575481 [start_node-22] [0m[INFO] [1778417743.256988910] [chatbot_llm]: [turn:__default__:7] ROUTE_RESOLVED | route=execution intent=- source=llm_response_inferred_route confidence=0.00[0m
1778417743.2595031 [start_node-22] [0m[INFO] [1778417743.258407655] [chatbot_llm]: [turn:__default__:7] TURN_DONE | planner-mode response complete[0m
1778417743.2623968 [start_manager-23] [0m[INFO] [1778417743.262068910] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "I can see two blueberries in the current scene."[0m
1778417743.2635612 [start_manager-23] [0m[INFO] [1778417743.263312926] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1778417743.2645888 [start_manager-23] [0m[INFO] [1778417743.264301901] [dialogue_manager]: [TTS] Speaking text (priority=128): "I can see two blueberries in the current scene."[0m
1778417743.2680762 [start_skill-26] [0m[INFO] [1778417743.267790320] [nao_say_skill]: [turn:I can see two blueberries in the] TTS_ACCEPTED | goal accepted[0m
1778417743.2700043 [start_skill-26] [0m[INFO] [1778417743.269757620] [nao_say_skill]: [turn:I can see two blueberries in the] SAY_START | text_len=47 lang=en-US volume=1.00[0m
1778417743.2719982 [start_skill-26] [0m[INFO] [1778417743.271153171] [nao_say_skill]: [turn:I can see two blueberries in the] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=47[0m
1778417743.2735555 [start_skill-26] [0m[INFO] [1778417743.273404397] [nao_say_skill]: [turn:I can see two blueberries in the] DEBUG_TTS_FORWARD | action="/debug/say" text_len=47[0m
1778417743.2748914 [start_skill-26] [33m[WARN] [1778417743.274767327] [nao_say_skill]: [turn:I can see two blueberries in the] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1778417743.2765212 [start_skill-26] [0m[INFO] [1778417743.276342888] [nao_say_skill]: [turn:I can see two blueberries in the] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=47 subscribers=0[0m
1778417743.2860651 [start_skill-26] [0m[INFO] [1778417743.285690396] [nao_say_skill]: [turn:I can see two blueberries in the] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1778417743.2922809 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:43,291] Materialisation performed by reasoner in 123.9ms[0m
1778417743.6316390 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:43,630] Updating ['default'] with:
1778417743.6321173 [knowledge_core-5] 	- detected_blueberry_321_208 rdf:type Blueberry
1778417743.6332431 [knowledge_core-5] 	- myself sees detected_blueberry_321_208 (lifespan: 4.0sec)[0m
1778417743.6334791 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:43,631] This statement will expire on 10/05/2026, 12:55:47[0m
1778417743.7717617 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:43,770] Materialisation performed by reasoner in 138.4ms[0m
1778417744.6435430 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:55:44,642] Removing expired statement <detected_blueberry_336_219 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:55:44+00:00)[0m
1778417744.6449113 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:55:44,642] Removing expired statement <myself sees detected_blueberry_336_219> from <default> (expired on 2026-05-10T12:55:44+00:00)[0m
1778417744.7664597 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:44,765] Materialisation performed by reasoner in 122.7ms[0m
1778417746.8524165 [hri_face_detect_yunet-11] [33m[WARN] [1778417746.851797276] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.8sec[0m
1778417747.8841319 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:55:47,883] Removing expired statement <myself sees detected_blueberry_331_248> from <default> (expired on 2026-05-10T12:55:47+00:00)[0m
1778417747.8852332 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:55:47,883] Removing expired statement <detected_blueberry_331_248 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:55:47+00:00)[0m
1778417747.8854861 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:55:47,883] Removing expired statement <detected_blueberry_321_208 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:55:47+00:00)[0m
1778417747.8856425 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:55:47,883] Removing expired statement <myself sees detected_blueberry_321_208> from <default> (expired on 2026-05-10T12:55:47+00:00)[0m
1778417748.0159175 [knowledge_core-5] [37m[INFO] [2026-05-10 12:55:48,015] Materialisation performed by reasoner in 131.0ms[0m
1778417752.6336634 [hri_face_detect_yunet-11] [33m[WARN] [1778417752.633153287] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.8sec[0m
1778417780.4764421 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:20,475] Updating ['default'] with:
1778417780.4768131 [knowledge_core-5] 	- detected_blueberry_324_300 rdf:type Blueberry
1778417780.4771278 [knowledge_core-5] 	- myself sees detected_blueberry_324_300 (lifespan: 4.0sec)[0m
1778417780.4808865 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:20,476] This statement will expire on 10/05/2026, 12:56:24[0m
1778417780.8073881 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:20,806] Materialisation performed by reasoner in 329.8ms[0m
1778417781.4984567 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:21,495] Updating ['default'] with:
1778417781.4992595 [knowledge_core-5] 	- detected_blueberry_324_300 rdf:type Blueberry
1778417781.5002661 [knowledge_core-5] 	- myself sees detected_blueberry_324_300 (lifespan: 4.0sec)[0m
1778417781.5027101 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:21,497] Updating expiry date to 10/05/2026, 12:56:25[0m
1778417781.6401315 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:21,639] Materialisation performed by reasoner in 140.3ms[0m
1778417782.7429469 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:22,741] Updating ['default'] with:
1778417782.7432699 [knowledge_core-5] 	- detected_blueberry_324_300 rdf:type Blueberry
1778417782.7435036 [knowledge_core-5] 	- myself sees detected_blueberry_324_300 (lifespan: 4.0sec)[0m
1778417782.7443523 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:22,742] Updating expiry date to 10/05/2026, 12:56:26[0m
1778417782.8755925 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:22,874] Materialisation performed by reasoner in 131.6ms[0m
1778417785.0214417 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:25,020] Updating ['default'] with:
1778417785.0217423 [knowledge_core-5] 	- myself sees detected_blueberry_333_305
1778417785.0218472 [knowledge_core-5] 	- detected_blueberry_333_305 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417785.0224588 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:25,021] This statement will expire on 10/05/2026, 12:56:29[0m
1778417785.1855123 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:25,184] Materialisation performed by reasoner in 163.3ms[0m
1778417786.2661712 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:56:26,265] Removing expired statement <detected_blueberry_324_300 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:56:26+00:00)[0m
1778417786.2680204 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:56:26,265] Removing expired statement <myself sees detected_blueberry_324_300> from <default> (expired on 2026-05-10T12:56:26+00:00)[0m
1778417786.3912005 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:26,390] Materialisation performed by reasoner in 124.7ms[0m
1778417789.4957554 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:56:29,494] Removing expired statement <myself sees detected_blueberry_333_305> from <default> (expired on 2026-05-10T12:56:29+00:00)[0m
1778417789.4971075 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:56:29,494] Removing expired statement <detected_blueberry_333_305 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:56:29+00:00)[0m
1778417789.8397322 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:29,839] Materialisation performed by reasoner in 343.9ms[0m
1778417790.1940084 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:30,192] Deleting from ['default']:
1778417790.1944666 [knowledge_core-5] 	- anonymous_person_fffig rdf:type Human[0m
1778417790.3036065 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:30,302] Materialisation performed by reasoner in 109.1ms[0m
1778417790.4468462 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:30,446] Deleting from ['default']:
1778417790.4470675 [knowledge_core-5] 	- myself sees anonymous_person_fffig[0m
1778417790.5334098 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:30,532] Materialisation performed by reasoner in 86.1ms[0m
1778417792.8784118 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:32,877] Updating ['default'] with:
1778417792.8800001 [knowledge_core-5] 	- anonymous_person_fidfb rdf:type Human[0m
1778417793.0094776 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:33,008] Materialisation performed by reasoner in 131.2ms[0m
1778417793.1917586 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:33,190] Updating ['default'] with:
1778417793.1921191 [knowledge_core-5] 	- myself sees anonymous_person_fidfb[0m
1778417793.3245108 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:33,323] Materialisation performed by reasoner in 132.7ms[0m
1778417793.4720371 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:33,471] Updating ['default'] with:
1778417793.4723518 [knowledge_core-5] 	- anonymous_person_fidfb sees myself[0m
1778417793.8639469 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:33,862] Materialisation performed by reasoner in 391.4ms[0m
1778417797.3291426 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:37,327] Updating ['default'] with:
1778417797.3297417 [knowledge_core-5] 	- myself sees detected_blueberry_273_277
1778417797.3298657 [knowledge_core-5] 	- detected_blueberry_273_277 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417797.3325207 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:37,328] This statement will expire on 10/05/2026, 12:56:41[0m
1778417797.4889362 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:37,487] Materialisation performed by reasoner in 159.0ms[0m
1778417797.6366105 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:37,636] Updating ['default'] with:
1778417797.6368117 [knowledge_core-5] 	- detected_blueberry_316_284 rdf:type Blueberry
1778417797.6368966 [knowledge_core-5] 	- myself sees detected_blueberry_316_284 (lifespan: 4.0sec)[0m
1778417797.6374929 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:37,636] This statement will expire on 10/05/2026, 12:56:41[0m
1778417797.7983906 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:37,797] Materialisation performed by reasoner in 160.5ms[0m
1778417798.3286324 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:38,328] Updating ['default'] with:
1778417798.3288035 [knowledge_core-5] 	- myself sees detected_blueberry_273_277
1778417798.3288717 [knowledge_core-5] 	- detected_blueberry_273_277 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417798.3292944 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:38,328] Updating expiry date to 10/05/2026, 12:56:42[0m
1778417798.5301991 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:38,529] Materialisation performed by reasoner in 200.7ms[0m
1778417798.6779954 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:38,676] Deleting from ['default']:
1778417798.6783323 [knowledge_core-5] 	- anonymous_person_fidfb rdf:type Human[0m
1778417798.8076968 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:38,807] Materialisation performed by reasoner in 129.6ms[0m
1778417798.9507079 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:38,950] Deleting from ['default']:
1778417798.9508877 [knowledge_core-5] 	- myself sees anonymous_person_fidfb[0m
1778417799.0579598 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:39,056] Materialisation performed by reasoner in 105.6ms[0m
1778417799.5394325 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:39,538] Updating ['default'] with:
1778417799.5396140 [knowledge_core-5] 	- anonymous_person_jeigi rdf:type Human[0m
1778417799.9336548 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:39,932] Materialisation performed by reasoner in 393.7ms[0m
1778417800.0873458 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:40,085] Updating ['default'] with:
1778417800.0877681 [knowledge_core-5] 	- myself sees anonymous_person_jeigi[0m
1778417800.2285013 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:40,227] Materialisation performed by reasoner in 141.7ms[0m
1778417800.3881264 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:40,383] Updating ['default'] with:
1778417800.3910365 [knowledge_core-5] 	- anonymous_person_jeigi sees myself[0m
1778417800.5452380 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:40,544] Materialisation performed by reasoner in 159.8ms[0m
1778417801.6225684 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:56:41,621] Removing expired statement <detected_blueberry_316_284 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:56:41+00:00)[0m
1778417801.6235266 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:56:41,622] Removing expired statement <myself sees detected_blueberry_316_284> from <default> (expired on 2026-05-10T12:56:41+00:00)[0m
1778417801.7449768 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:41,744] Materialisation performed by reasoner in 122.2ms[0m
1778417802.7988691 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:56:42,791] Removing expired statement <myself sees detected_blueberry_273_277> from <default> (expired on 2026-05-10T12:56:42+00:00)[0m
1778417802.7992284 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:56:42,792] Removing expired statement <detected_blueberry_273_277 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:56:42+00:00)[0m
1778417802.9287555 [knowledge_core-5] [37m[INFO] [2026-05-10 12:56:42,927] Materialisation performed by reasoner in 135.3ms[0m
1778417826.1002479 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:06,098] Updating ['default'] with:
1778417826.1007605 [knowledge_core-5] 	- myself sees detected_tomato_242_298
1778417826.1009476 [knowledge_core-5] 	- detected_tomato_242_298 rdf:type Tomato (lifespan: 4.0sec)[0m
1778417826.1013412 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:06,099] This statement will expire on 10/05/2026, 12:57:10[0m
1778417826.2354262 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:06,234] Materialisation performed by reasoner in 135.2ms[0m
1778417827.6962316 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:07,694] Updating ['default'] with:
1778417827.6966457 [knowledge_core-5] 	- detected_blueberry_383_201 rdf:type Blueberry
1778417827.6970043 [knowledge_core-5] 	- myself sees detected_blueberry_383_201 (lifespan: 4.0sec)[0m
1778417827.6981966 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:07,695] This statement will expire on 10/05/2026, 12:57:11[0m
1778417827.8299334 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:07,829] Materialisation performed by reasoner in 132.8ms[0m
1778417828.3763218 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:08,375] Updating ['default'] with:
1778417828.3766944 [knowledge_core-5] 	- myself sees detected_blueberry_442_198
1778417828.3769164 [knowledge_core-5] 	- detected_blueberry_442_198 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417828.3787670 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:08,376] This statement will expire on 10/05/2026, 12:57:12[0m
1778417828.5084839 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:08,507] Materialisation performed by reasoner in 131.0ms[0m
1778417829.3195720 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:09,318] Updating ['default'] with:
1778417829.3198509 [knowledge_core-5] 	- myself sees detected_blueberry_442_198
1778417829.3199966 [knowledge_core-5] 	- detected_blueberry_442_198 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417829.3206518 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:09,319] Updating expiry date to 10/05/2026, 12:57:13[0m
1778417829.4415042 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:09,441] Materialisation performed by reasoner in 121.3ms[0m
1778417830.3563302 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:10,353] Updating ['default'] with:
1778417830.3570974 [knowledge_core-5] 	- myself sees detected_blueberry_442_198
1778417830.3576393 [knowledge_core-5] 	- detected_blueberry_442_198 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417830.3599534 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:10,356] Updating expiry date to 10/05/2026, 12:57:14[0m
1778417830.5096521 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:10,508] Materialisation performed by reasoner in 150.8ms[0m
1778417831.2907808 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:11,289] Removing expired statement <myself sees detected_tomato_242_298> from <default> (expired on 2026-05-10T12:57:10+00:00)[0m
1778417831.2918606 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:11,290] Removing expired statement <detected_tomato_242_298 rdf:type Tomato> from <default> (expired on 2026-05-10T12:57:10+00:00)[0m
1778417831.2920666 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:11,290] Removing expired statement <detected_blueberry_383_201 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:57:11+00:00)[0m
1778417831.2922070 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:11,290] Removing expired statement <myself sees detected_blueberry_383_201> from <default> (expired on 2026-05-10T12:57:11+00:00)[0m
1778417831.4107740 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:11,409] Materialisation performed by reasoner in 119.1ms[0m
1778417831.4698832 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:11,467] Updating ['default'] with:
1778417831.4706025 [knowledge_core-5] 	- myself sees detected_blueberry_442_198
1778417831.4710653 [knowledge_core-5] 	- detected_blueberry_442_198 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417831.4738107 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:11,469] Updating expiry date to 10/05/2026, 12:57:15[0m
1778417831.6055312 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:11,604] Materialisation performed by reasoner in 133.5ms[0m
1778417832.4852114 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:12,482] Updating ['default'] with:
1778417832.4855702 [knowledge_core-5] 	- myself sees detected_blueberry_442_198
1778417832.4863801 [knowledge_core-5] 	- detected_blueberry_442_198 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417832.4865749 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:12,483] Updating expiry date to 10/05/2026, 12:57:16[0m
1778417832.6119454 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:12,611] Materialisation performed by reasoner in 127.2ms[0m
1778417833.5593081 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:13,557] Updating ['default'] with:
1778417833.5597107 [knowledge_core-5] 	- myself sees detected_blueberry_442_198
1778417833.5600171 [knowledge_core-5] 	- detected_blueberry_442_198 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417833.5613794 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:13,559] Updating expiry date to 10/05/2026, 12:57:17[0m
1778417833.7125933 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:13,711] Materialisation performed by reasoner in 151.5ms[0m
1778417834.5059588 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:14,504] Updating ['default'] with:
1778417834.5062149 [knowledge_core-5] 	- myself sees detected_blueberry_442_198
1778417834.5063496 [knowledge_core-5] 	- detected_blueberry_442_198 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417834.5071042 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:14,505] Updating expiry date to 10/05/2026, 12:57:18[0m
1778417834.9089575 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:14,908] Materialisation performed by reasoner in 402.2ms[0m
1778417835.5786428 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:15,576] Updating ['default'] with:
1778417835.5790992 [knowledge_core-5] 	- myself sees detected_blueberry_442_198
1778417835.5794358 [knowledge_core-5] 	- detected_blueberry_442_198 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417835.5809898 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:15,578] Updating expiry date to 10/05/2026, 12:57:19[0m
1778417835.7240620 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:15,723] Materialisation performed by reasoner in 144.6ms[0m
1778417836.6999085 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:16,698] Updating ['default'] with:
1778417836.7006526 [knowledge_core-5] 	- myself sees detected_blueberry_442_198
1778417836.7007854 [knowledge_core-5] 	- detected_blueberry_442_198 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417836.7017524 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:16,700] Updating expiry date to 10/05/2026, 12:57:20[0m
1778417836.8990123 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:16,898] Materialisation performed by reasoner in 197.0ms[0m
1778417837.0420449 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:17,041] Deleting from ['default']:
1778417837.0421984 [knowledge_core-5] 	- anonymous_person_jeigi rdf:type Human[0m
1778417837.1398637 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:17,139] Materialisation performed by reasoner in 97.6ms[0m
1778417837.3326600 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:17,332] Deleting from ['default']:
1778417837.3330133 [knowledge_core-5] 	- myself sees anonymous_person_jeigi[0m
1778417837.4467206 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:17,445] Materialisation performed by reasoner in 113.5ms[0m
1778417837.5961077 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:17,595] Updating ['default'] with:
1778417837.5964403 [knowledge_core-5] 	- anonymous_person_jajdc rdf:type Human[0m
1778417837.7425761 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:17,741] Materialisation performed by reasoner in 146.4ms[0m
1778417837.8952808 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:17,893] Updating ['default'] with:
1778417837.8957138 [knowledge_core-5] 	- myself sees anonymous_person_jajdc[0m
1778417838.2588348 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:18,258] Materialisation performed by reasoner in 364.0ms[0m
1778417838.4376531 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:18,436] Updating ['default'] with:
1778417838.4378748 [knowledge_core-5] 	- anonymous_person_jajdc sees myself[0m
1778417838.5623882 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:18,561] Materialisation performed by reasoner in 124.8ms[0m
1778417840.3970354 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:20,396] Removing expired statement <myself sees detected_blueberry_442_198> from <default> (expired on 2026-05-10T12:57:20+00:00)[0m
1778417840.3975337 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:20,396] Removing expired statement <detected_blueberry_442_198 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:57:20+00:00)[0m
1778417840.5265667 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:20,524] Materialisation performed by reasoner in 127.3ms[0m
1778417841.4654408 [start_manager-23] [0m[INFO] [1778417841.464558670] [dialogue_manager]: [SPEECH INPUT] voice_id="anonymous_speaker": "DId you see any people in the last scan?" (locale=, confidence=0.00)[0m
1778417841.4698308 [start_manager-23] [0m[INFO] [1778417841.468003233] [dialogue_manager]: [SPEECH INPUT] Using default dialogue 973c0f1e-a654-42da-b888-6b3e1078600d[0m
1778417841.4732385 [start_manager-23] [0m[INFO] [1778417841.470728154] [dialogue_manager]: [CHATBOT REQUEST] chatbot_goal_id=3444e73b-3f02-4535-bc87-4762ba67d189, user_id="anonymous_speaker", text="DId you see any people in the last scan?"[0m
1778417841.4780416 [start_node-22] [0m[INFO] [1778417841.477381938] [chatbot_llm]: [CHATBOT] dialogue=3444e73b user=anonymous_speaker turn=__default__:8 input=DId you see any people in the last scan?[0m
1778417841.5091450 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:21,507] Searching ['?entity', '?type'] in models {'default'} matching:
1778417841.5093429 [knowledge_core-5] 	- myself sees ?entity
1778417841.5095041 [knowledge_core-5] 	- ?entity rdf:type ?type[0m
1778417841.5137680 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:21,513] Found: [{'entity': 'anonymous_person_jajdc', 'type': 'cyc:SolidTangibleThing'}, {'entity': 'anonymous_person_jajdc', 'type': 'Human'}, {'entity': 'anonymous_person_jajdc', 'type': 'foaf:Person'}, {'entity': 'anonymous_person_jajdc', 'type': 'cyc:SpatialThing-Localized'}, {'entity': 'anonymous_person_jajdc', 'type': 'Agent'}, {'entity': 'anonymous_person_jajdc', 'type': 'foaf:Agent'}, {'entity': 'anonymous_person_jajdc', 'type': 'owl:Thing'}, {'entity': 'anonymous_person_jajdc', 'type': 'cyc:PartiallyTangible'}, {'entity': 'anonymous_person_jajdc', 'type': 'EmbodiedAgent'}, {'entity': 'anonymous_person_jajdc', 'type': 'cyc:SpatialThing'}, {'entity': 'anonymous_person_jajdc', 'type': 'Location'}, {'entity': 'anonymous_person_jajdc', 'type': 'cyc:EnduringThing-Localized'}][0m
1778417841.5169866 [start_node-22] [0m[INFO] [1778417841.516561295] [chatbot_llm]: [turn:__default__:8] KB_SNAPSHOT | loaded 187 chars from /kb/query[0m
1778417841.5182283 [start_node-22] [0m[INFO] [1778417841.517635688] [chatbot_llm]: [turn:__default__:8] TURN_START | user="DId you see any people in the last scan?"[0m
1778417841.5189979 [start_node-22] [0m[INFO] [1778417841.518769825] [chatbot_llm]: [turn:__default__:8] LLM_REQUEST | stage=response model=QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ history=17 timeout=60.0s[0m
1778417842.0537539 [start_node-22] [0m[INFO] [1778417842.052652160] [chatbot_llm]: [turn:__default__:8] ROUTE_RESOLVED | route=execution intent=- source=llm_response_inferred_route confidence=0.00[0m
1778417842.0589337 [start_node-22] [0m[INFO] [1778417842.056527622] [chatbot_llm]: [turn:__default__:8] TURN_DONE | planner-mode response complete[0m
1778417842.0683453 [start_node-22] [0m[INFO] [1778417842.062222275] [chatbot_llm]: [turn:__default__:8] PLANNER_REQUEST | published planner request on /nao_orchestrator/planner_request goal_id=goal_default___8 kind=new_goal[0m
1778417842.0704188 [run_app-24] [0m[INFO] [1778417842.068871021] [nao_orchestrator]: Planner gate forwarded request | goal_id=goal_default___8 kind=new_goal active_goal=goal_default___8 topic=/planner/request[0m
1778417842.0739424 [start_manager-23] [0m[INFO] [1778417842.072077789] [dialogue_manager]: [CHATBOT RESPONSE] dialogue_id=973c0f1e-a654-42da-b888-6b3e1078600d: "No, I did not see any people in the last scan."[0m
1778417842.0767751 [start_manager-23] [0m[INFO] [1778417842.074858097] [dialogue_manager]: [CHATBOT RESPONSE] Speaking via TTS[0m
1778417842.0775642 [start_manager-23] [0m[INFO] [1778417842.076488006] [dialogue_manager]: [TTS] Speaking text (priority=128): "No, I did not see any people in the last scan."[0m
1778417842.0840290 [start_skill-26] [0m[INFO] [1778417842.083637874] [nao_say_skill]: [turn:No, I did not see any people in] TTS_ACCEPTED | goal accepted[0m
1778417842.0883355 [start_skill-26] [0m[INFO] [1778417842.087575404] [nao_say_skill]: [turn:No, I did not see any people in] SAY_START | text_len=46 lang=en-US volume=1.00[0m
1778417842.0932231 [start_skill-26] [0m[INFO] [1778417842.092927841] [nao_say_skill]: [turn:No, I did not see any people in] DEBUG_SPEECH_PUBLISHED | topic="/debug/nao_say/speech" text_len=46[0m
1778417842.0976820 [start_skill-26] [0m[INFO] [1778417842.094606384] [nao_say_skill]: [turn:No, I did not see any people in] DEBUG_TTS_FORWARD | action="/debug/say" text_len=46[0m
1778417842.0979106 [start_skill-26] [33m[WARN] [1778417842.096731905] [nao_say_skill]: [turn:No, I did not see any people in] SPEECH_TOPIC_NO_SUBSCRIBERS | topic="/speech" has no subscribers; robot audio will not play[0m
1778417842.1015990 [start_skill-26] [0m[INFO] [1778417842.098376363] [nao_say_skill]: [turn:No, I did not see any people in] SPEECH_TOPIC_PUBLISHED | topic="/speech" text_len=46 subscribers=0[0m
1778417842.1059709 [start_skill-26] [0m[INFO] [1778417842.105504239] [nao_say_skill]: [turn:No, I did not see any people in] DEBUG_TTS_ACCEPTED | action="/debug/say" accepted goal[0m
1778417843.4596641 [run_app-24] [0m[INFO] [1778417843.458587227] [nao_orchestrator]: ORCH SCAN | target=scene target_kind=scene result_mode=success[0m
1778417844.4810910 [run_app-24] [0m[INFO] [1778417844.480735794] [nao_orchestrator]: ORCH HEAD_DISPATCH | yaw=0.000 pitch=0.000 relative=False[0m
1778417844.7426324 [run_app-24] [33m[WARN] [1778417844.741233689] [nao_orchestrator]: Planned intent step failed | intent=raw_user_input source=planner_llm plan_id=plan_1778417843439 step={'id': 'step_3', 'type': 'skill', 'name': 'look_at', 'args': {'target_frame': 'head_center'}, 'requires': [], 'on_failure': 'fail', 'retry_budget': 0} reason=look_at_target action server unavailable[0m
1778417844.7512512 [start_manager-23] [33m[WARN] [1778417844.749524071] [dialogue_manager]: [PLANNER ACT] explain_failure suppressed for TTS goal_id=goal_default___8 reason=look_at_target action server unavailable text_hint=look_at_target action server unavailable[0m
1778417845.7523010 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:25,751] Updating ['default'] with:
1778417845.7533338 [knowledge_core-5] 	- myself sees detected_blueberry_342_261
1778417845.7541981 [knowledge_core-5] 	- detected_blueberry_342_261 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417845.7587636 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:25,751] This statement will expire on 10/05/2026, 12:57:29[0m
1778417845.9126184 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:25,911] Materialisation performed by reasoner in 159.7ms[0m
1778417847.3049934 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:27,300] Updating ['default'] with:
1778417847.3058817 [knowledge_core-5] 	- myself sees detected_blueberry_342_261
1778417847.3065021 [knowledge_core-5] 	- detected_blueberry_342_261 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417847.3092947 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:27,303] Updating expiry date to 10/05/2026, 12:57:31[0m
1778417847.6296453 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:27,627] Materialisation performed by reasoner in 323.2ms[0m
1778417848.4254005 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:28,424] Updating ['default'] with:
1778417848.4256244 [knowledge_core-5] 	- myself sees detected_blueberry_342_261
1778417848.4257207 [knowledge_core-5] 	- detected_blueberry_342_261 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417848.4258103 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:28,424] Updating expiry date to 10/05/2026, 12:57:32[0m
1778417848.5100870 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:28,509] Materialisation performed by reasoner in 84.1ms[0m
1778417849.4657371 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:29,464] Updating ['default'] with:
1778417849.4659932 [knowledge_core-5] 	- myself sees detected_blueberry_342_261
1778417849.4661229 [knowledge_core-5] 	- detected_blueberry_342_261 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417849.4668348 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:29,465] Updating expiry date to 10/05/2026, 12:57:33[0m
1778417849.5499794 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:29,549] Materialisation performed by reasoner in 83.4ms[0m
1778417850.6514626 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:30,650] Updating ['default'] with:
1778417850.6518710 [knowledge_core-5] 	- myself sees detected_blueberry_342_261
1778417850.6593843 [knowledge_core-5] 	- detected_blueberry_342_261 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417850.6604185 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:30,651] Updating expiry date to 10/05/2026, 12:57:34[0m
1778417850.7292905 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:30,728] Materialisation performed by reasoner in 77.1ms[0m
1778417851.6779346 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:31,676] Updating ['default'] with:
1778417851.6784160 [knowledge_core-5] 	- myself sees detected_blueberry_342_261
1778417851.6787786 [knowledge_core-5] 	- detected_blueberry_342_261 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417851.6802082 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:31,677] Updating expiry date to 10/05/2026, 12:57:35[0m
1778417851.8050563 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:31,803] Materialisation performed by reasoner in 125.4ms[0m
1778417852.7045147 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:32,703] Updating ['default'] with:
1778417852.7047112 [knowledge_core-5] 	- myself sees detected_blueberry_342_261
1778417852.7047830 [knowledge_core-5] 	- detected_blueberry_342_261 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417852.7052498 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:32,704] Updating expiry date to 10/05/2026, 12:57:36[0m
1778417852.7824664 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:32,781] Materialisation performed by reasoner in 77.2ms[0m
1778417853.8146293 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:33,812] Updating ['default'] with:
1778417853.8152435 [knowledge_core-5] 	- myself sees detected_blueberry_342_261
1778417853.8156469 [knowledge_core-5] 	- detected_blueberry_342_261 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417853.8176334 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:33,814] Updating expiry date to 10/05/2026, 12:57:37[0m
1778417854.1426394 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:34,141] Materialisation performed by reasoner in 326.7ms[0m
1778417854.8944552 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:34,893] Updating ['default'] with:
1778417854.8947721 [knowledge_core-5] 	- myself sees detected_blueberry_342_261
1778417854.8950508 [knowledge_core-5] 	- detected_blueberry_342_261 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417854.8957877 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:34,894] Updating expiry date to 10/05/2026, 12:57:38[0m
1778417854.9726832 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:34,972] Materialisation performed by reasoner in 77.5ms[0m
1778417855.0802429 [hri_face_detect_yunet-11] [33m[WARN] [1778417855.079583666] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 102.4sec[0m
1778417855.7735369 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:35,772] Updating ['default'] with:
1778417855.7739394 [knowledge_core-5] 	- detected_blueberry_370_257 rdf:type Blueberry
1778417855.7740979 [knowledge_core-5] 	- myself sees detected_blueberry_370_257 (lifespan: 4.0sec)[0m
1778417855.7750499 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:35,773] This statement will expire on 10/05/2026, 12:57:39[0m
1778417855.8459816 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:35,845] Materialisation performed by reasoner in 72.0ms[0m
1778417856.8723848 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:36,869] Updating ['default'] with:
1778417856.8731050 [knowledge_core-5] 	- detected_blueberry_370_257 rdf:type Blueberry
1778417856.8736582 [knowledge_core-5] 	- myself sees detected_blueberry_370_257 (lifespan: 4.0sec)[0m
1778417856.8758988 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:36,872] Updating expiry date to 10/05/2026, 12:57:40[0m
1778417857.0137393 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:37,013] Materialisation performed by reasoner in 139.5ms[0m
1778417857.3932385 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:37,391] Deleting from ['default']:
1778417857.3941677 [knowledge_core-5] 	- anonymous_person_jajdc rdf:type Human[0m
1778417857.5459561 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:37,545] Materialisation performed by reasoner in 153.1ms[0m
1778417857.6914527 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:37,689] Deleting from ['default']:
1778417857.6921728 [knowledge_core-5] 	- myself sees anonymous_person_jajdc[0m
1778417858.0237386 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:38,023] Materialisation performed by reasoner in 332.7ms[0m
1778417858.1834664 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:38,176] Updating ['default'] with:
1778417858.1838212 [knowledge_core-5] 	- anonymous_person_ibica rdf:type Human[0m
1778417858.3195024 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:38,318] Materialisation performed by reasoner in 141.6ms[0m
1778417858.3506770 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:38,349] Removing expired statement <myself sees detected_blueberry_342_261> from <default> (expired on 2026-05-10T12:57:38+00:00)[0m
1778417858.3514590 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:38,350] Removing expired statement <detected_blueberry_342_261 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:57:38+00:00)[0m
1778417858.4320164 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:38,431] Materialisation performed by reasoner in 81.1ms[0m
1778417858.4863322 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:38,485] Updating ['default'] with:
1778417858.4865222 [knowledge_core-5] 	- myself sees detected_blueberry_374_298
1778417858.4866061 [knowledge_core-5] 	- detected_blueberry_374_298 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417858.4870641 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:38,486] This statement will expire on 10/05/2026, 12:57:42[0m
1778417858.6086936 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:38,607] Materialisation performed by reasoner in 121.4ms[0m
1778417858.7701542 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:38,768] Updating ['default'] with:
1778417858.7707367 [knowledge_core-5] 	- myself sees anonymous_person_ibica[0m
1778417858.8728683 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:38,871] Materialisation performed by reasoner in 102.7ms[0m
1778417859.0199809 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:39,019] Updating ['default'] with:
1778417859.0203080 [knowledge_core-5] 	- anonymous_person_ibica sees myself[0m
1778417859.1037195 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:39,103] Materialisation performed by reasoner in 83.8ms[0m
1778417859.2466202 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:39,245] Updating ['default'] with:
1778417859.2468767 [knowledge_core-5] 	- myself sees detected_blueberry_374_298
1778417859.2469773 [knowledge_core-5] 	- detected_blueberry_374_298 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417859.2474411 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:39,246] Updating expiry date to 10/05/2026, 12:57:43[0m
1778417859.5697541 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:39,568] Materialisation performed by reasoner in 322.1ms[0m
1778417860.1915216 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:40,190] Updating ['default'] with:
1778417860.1919408 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417860.1921270 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417860.1933117 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:40,191] This statement will expire on 10/05/2026, 12:57:44[0m
1778417860.3232450 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:40,322] Materialisation performed by reasoner in 130.4ms[0m
1778417860.6428196 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:40,642] Removing expired statement <detected_blueberry_370_257 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:57:40+00:00)[0m
1778417860.6434681 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:40,642] Removing expired statement <myself sees detected_blueberry_370_257> from <default> (expired on 2026-05-10T12:57:40+00:00)[0m
1778417860.7537165 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:40,753] Materialisation performed by reasoner in 110.5ms[0m
1778417861.1638894 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:41,162] Updating ['default'] with:
1778417861.1640923 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417861.1641951 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417861.1647851 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:41,163] Updating expiry date to 10/05/2026, 12:57:45[0m
1778417861.2902691 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:41,289] Materialisation performed by reasoner in 126.0ms[0m
1778417862.3096130 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:42,308] Updating ['default'] with:
1778417862.3099065 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417862.3100777 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417862.3113348 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:42,309] Updating expiry date to 10/05/2026, 12:57:46[0m
1778417862.4298923 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:42,429] Materialisation performed by reasoner in 119.6ms[0m
1778417863.3928549 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:43,392] Updating ['default'] with:
1778417863.3930924 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417863.3932416 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417863.3940189 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:43,392] Updating expiry date to 10/05/2026, 12:57:47[0m
1778417863.4943094 [hri_face_detect_yunet-11] [33m[WARN] [1778417863.493709078] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 8.4sec[0m
1778417863.7142544 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:43,713] Materialisation performed by reasoner in 320.0ms[0m
1778417863.9633496 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:43,962] Removing expired statement <myself sees detected_blueberry_374_298> from <default> (expired on 2026-05-10T12:57:43+00:00)[0m
1778417863.9640851 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:43,962] Removing expired statement <detected_blueberry_374_298 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:57:43+00:00)[0m
1778417864.0839202 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:44,083] Materialisation performed by reasoner in 120.2ms[0m
1778417864.4966753 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:44,494] Updating ['default'] with:
1778417864.4974599 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417864.4978461 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417864.4993093 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:44,496] Updating expiry date to 10/05/2026, 12:57:48[0m
1778417864.6322067 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:44,631] Materialisation performed by reasoner in 134.3ms[0m
1778417865.3131630 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:45,311] Updating ['default'] with:
1778417865.3137002 [knowledge_core-5] 	- detected_tomato_281_356 rdf:type Tomato
1778417865.3140302 [knowledge_core-5] 	- myself sees detected_tomato_281_356 (lifespan: 4.0sec)[0m
1778417865.3161111 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:45,312] This statement will expire on 10/05/2026, 12:57:49[0m
1778417865.4554796 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:45,454] Materialisation performed by reasoner in 141.6ms[0m
1778417865.6000035 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:45,599] Updating ['default'] with:
1778417865.6001956 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417865.6002791 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417865.6007621 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:45,599] Updating expiry date to 10/05/2026, 12:57:49[0m
1778417865.7692957 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:45,768] Materialisation performed by reasoner in 168.4ms[0m
1778417866.6494930 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:46,648] Updating ['default'] with:
1778417866.6497211 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417866.6501491 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417866.6508710 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:46,649] Updating expiry date to 10/05/2026, 12:57:50[0m
1778417866.7434647 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:46,742] Materialisation performed by reasoner in 92.5ms[0m
1778417867.7129900 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:47,711] Updating ['default'] with:
1778417867.7135422 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417867.7138317 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417867.7152891 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:47,712] Updating expiry date to 10/05/2026, 12:57:51[0m
1778417868.1254432 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:48,124] Materialisation performed by reasoner in 411.6ms[0m
1778417868.4793012 [hri_face_detect_yunet-11] [33m[WARN] [1778417868.478787495] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.0sec[0m
1778417868.7820492 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:48,778] Updating ['default'] with:
1778417868.7823856 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417868.7824678 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417868.7825670 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:48,780] Updating expiry date to 10/05/2026, 12:57:52[0m
1778417868.8618255 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:48,861] Materialisation performed by reasoner in 80.8ms[0m
1778417869.4146249 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:49,413] Removing expired statement <detected_tomato_281_356 rdf:type Tomato> from <default> (expired on 2026-05-10T12:57:49+00:00)[0m
1778417869.4184082 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:49,414] Removing expired statement <myself sees detected_tomato_281_356> from <default> (expired on 2026-05-10T12:57:49+00:00)[0m
1778417869.5552044 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:49,549] Materialisation performed by reasoner in 135.4ms[0m
1778417869.8225787 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:49,821] Updating ['default'] with:
1778417869.8229303 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417869.8231344 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417869.8249462 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:49,822] Updating expiry date to 10/05/2026, 12:57:53[0m
1778417869.9683743 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:49,967] Materialisation performed by reasoner in 144.6ms[0m
1778417870.1856928 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:50,185] Updating ['default'] with:
1778417870.1858976 [knowledge_core-5] 	- detected_tomato_282_357 rdf:type Tomato
1778417870.1875792 [knowledge_core-5] 	- myself sees detected_tomato_282_357 (lifespan: 4.0sec)[0m
1778417870.1882870 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:50,185] This statement will expire on 10/05/2026, 12:57:54[0m
1778417870.2572343 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:50,256] Materialisation performed by reasoner in 70.9ms[0m
1778417871.0520873 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:51,051] Updating ['default'] with:
1778417871.0523298 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417871.0524313 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417871.0529506 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:51,051] Updating expiry date to 10/05/2026, 12:57:55[0m
1778417871.3836012 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:51,382] Materialisation performed by reasoner in 330.5ms[0m
1778417872.0422251 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:52,041] Updating ['default'] with:
1778417872.0432243 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417872.0442865 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417872.0490019 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:52,041] Updating expiry date to 10/05/2026, 12:57:56[0m
1778417872.1978033 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:52,196] Materialisation performed by reasoner in 155.0ms[0m
1778417873.1094217 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:53,105] Updating ['default'] with:
1778417873.1107497 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417873.1114995 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417873.1151416 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:53,109] Updating expiry date to 10/05/2026, 12:57:57[0m
1778417873.2656488 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:53,264] Materialisation performed by reasoner in 152.6ms[0m
1778417874.1392758 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:54,138] Updating ['default'] with:
1778417874.1394842 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417874.1395550 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417874.1400909 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:54,139] Updating expiry date to 10/05/2026, 12:57:58[0m
1778417874.3165760 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:54,314] Materialisation performed by reasoner in 175.1ms[0m
1778417874.7783425 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:54,777] Removing expired statement <detected_tomato_282_357 rdf:type Tomato> from <default> (expired on 2026-05-10T12:57:54+00:00)[0m
1778417874.7789066 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:57:54,777] Removing expired statement <myself sees detected_tomato_282_357> from <default> (expired on 2026-05-10T12:57:54+00:00)[0m
1778417874.8894341 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:54,888] Materialisation performed by reasoner in 110.6ms[0m
1778417875.0106676 [hri_face_detect_yunet-11] [33m[WARN] [1778417875.010187460] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.5sec[0m
1778417875.1679602 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:55,164] Updating ['default'] with:
1778417875.1685572 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417875.1689091 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417875.1705129 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:55,167] Updating expiry date to 10/05/2026, 12:57:59[0m
1778417875.3127131 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:55,311] Materialisation performed by reasoner in 141.9ms[0m
1778417876.1780250 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:56,177] Updating ['default'] with:
1778417876.1781983 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417876.1782498 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417876.1785834 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:56,177] Updating expiry date to 10/05/2026, 12:58:00[0m
1778417876.5601616 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:56,559] Materialisation performed by reasoner in 381.1ms[0m
1778417876.8466020 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:56,845] Deleting from ['default']:
1778417876.8469357 [knowledge_core-5] 	- anonymous_person_ibica rdf:type Human[0m
1778417876.9224291 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:56,921] Materialisation performed by reasoner in 75.6ms[0m
1778417877.0801377 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:57,077] Deleting from ['default']:
1778417877.0804863 [knowledge_core-5] 	- myself sees anonymous_person_ibica[0m
1778417877.2053971 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:57,204] Materialisation performed by reasoner in 125.6ms[0m
1778417877.3498294 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:57,349] Updating ['default'] with:
1778417877.3500156 [knowledge_core-5] 	- anonymous_person_jbbii rdf:type Human[0m
1778417877.4267254 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:57,426] Materialisation performed by reasoner in 76.6ms[0m
1778417877.4997683 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:57,498] Updating ['default'] with:
1778417877.5001168 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417877.5008998 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417877.5020480 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:57,499] Updating expiry date to 10/05/2026, 12:58:01[0m
1778417877.5827658 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:57,582] Materialisation performed by reasoner in 82.2ms[0m
1778417877.7328305 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:57,731] Updating ['default'] with:
1778417877.7331622 [knowledge_core-5] 	- myself sees anonymous_person_jbbii[0m
1778417878.0438399 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:58,042] Materialisation performed by reasoner in 310.9ms[0m
1778417878.2019033 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:58,201] Updating ['default'] with:
1778417878.2021854 [knowledge_core-5] 	- anonymous_person_jbbii sees myself[0m
1778417878.2886379 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:58,287] Materialisation performed by reasoner in 86.2ms[0m
1778417878.4388678 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:58,436] Updating ['default'] with:
1778417878.4395881 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417878.4408698 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417878.4428871 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:58,438] Updating expiry date to 10/05/2026, 12:58:02[0m
1778417878.5451794 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:58,544] Materialisation performed by reasoner in 105.3ms[0m
1778417879.2779858 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:59,277] Updating ['default'] with:
1778417879.2782702 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417879.2783782 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417879.2789054 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:59,277] Updating expiry date to 10/05/2026, 12:58:03[0m
1778417879.3569841 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:59,356] Materialisation performed by reasoner in 78.1ms[0m
1778417879.6520586 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:59,650] Updating ['default'] with:
1778417879.6524081 [knowledge_core-5] 	- myself sees detected_tomato_284_358
1778417879.6525478 [knowledge_core-5] 	- detected_tomato_284_358 rdf:type Tomato (lifespan: 4.0sec)[0m
1778417879.6534085 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:59,651] This statement will expire on 10/05/2026, 12:58:03[0m
1778417879.7809083 [knowledge_core-5] [37m[INFO] [2026-05-10 12:57:59,780] Materialisation performed by reasoner in 128.3ms[0m
1778417880.3203909 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:00,319] Updating ['default'] with:
1778417880.3206308 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417880.3207240 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417880.3214049 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:00,320] Updating expiry date to 10/05/2026, 12:58:04[0m
1778417880.4783838 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:00,477] Materialisation performed by reasoner in 157.0ms[0m
1778417881.4246387 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:01,423] Updating ['default'] with:
1778417881.4251010 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417881.4254320 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417881.4266703 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:01,424] Updating expiry date to 10/05/2026, 12:58:05[0m
1778417881.8669996 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:01,866] Materialisation performed by reasoner in 441.1ms[0m
1778417882.5289767 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:02,525] Updating ['default'] with:
1778417882.5302289 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417882.5309541 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417882.5348227 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:02,528] Updating expiry date to 10/05/2026, 12:58:06[0m
1778417882.6216948 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:02,621] Materialisation performed by reasoner in 91.1ms[0m
1778417883.3270762 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:58:03,326] Removing expired statement <myself sees detected_tomato_284_358> from <default> (expired on 2026-05-10T12:58:03+00:00)[0m
1778417883.3281157 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:58:03,326] Removing expired statement <detected_tomato_284_358 rdf:type Tomato> from <default> (expired on 2026-05-10T12:58:03+00:00)[0m
1778417883.4122398 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:03,411] Materialisation performed by reasoner in 84.7ms[0m
1778417883.6799593 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:03,678] Updating ['default'] with:
1778417883.6811576 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417883.6827290 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417883.6840460 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:03,679] Updating expiry date to 10/05/2026, 12:58:07[0m
1778417883.7637148 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:03,763] Materialisation performed by reasoner in 83.0ms[0m
1778417884.7066622 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:04,705] Updating ['default'] with:
1778417884.7069924 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417884.7072251 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417884.7083402 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:04,706] Updating expiry date to 10/05/2026, 12:58:08[0m
1778417884.8384485 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:04,837] Materialisation performed by reasoner in 130.6ms[0m
1778417885.7135017 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:05,712] Updating ['default'] with:
1778417885.7137618 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417885.7138710 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417885.7145669 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:05,713] Updating expiry date to 10/05/2026, 12:58:09[0m
1778417885.8783972 [hri_face_detect_yunet-11] [33m[WARN] [1778417885.877900720] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 10.9sec[0m
1778417886.0334446 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:06,032] Materialisation performed by reasoner in 318.4ms[0m
1778417886.7681959 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:06,767] Updating ['default'] with:
1778417886.7685211 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417886.7686446 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417886.7693429 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:06,767] Updating expiry date to 10/05/2026, 12:58:10[0m
1778417886.8846393 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:06,883] Materialisation performed by reasoner in 115.4ms[0m
1778417887.8264468 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:07,825] Updating ['default'] with:
1778417887.8266726 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417887.8269939 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417887.8276489 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:07,826] Updating expiry date to 10/05/2026, 12:58:11[0m
1778417887.9363172 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:07,935] Materialisation performed by reasoner in 108.7ms[0m
1778417888.8306270 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:08,829] Updating ['default'] with:
1778417888.8309019 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417888.8309896 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417888.8315237 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:08,830] Updating expiry date to 10/05/2026, 12:58:12[0m
1778417888.9455900 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:08,945] Materialisation performed by reasoner in 114.4ms[0m
1778417889.8983152 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:09,897] Updating ['default'] with:
1778417889.8985391 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417889.8986230 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417889.8986983 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:09,897] Updating expiry date to 10/05/2026, 12:58:13[0m
1778417889.9897122 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:09,989] Materialisation performed by reasoner in 90.9ms[0m
1778417890.9314344 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:10,930] Updating ['default'] with:
1778417890.9317074 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417890.9317994 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417890.9322965 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:10,931] Updating expiry date to 10/05/2026, 12:58:14[0m
1778417890.9955597 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:10,994] Materialisation performed by reasoner in 63.5ms[0m
1778417891.4344964 [hri_face_detect_yunet-11] [33m[WARN] [1778417891.433987666] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778417891.9452572 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:11,944] Updating ['default'] with:
1778417891.9454648 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417891.9455855 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417891.9460106 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:11,945] Updating expiry date to 10/05/2026, 12:58:15[0m
1778417892.2374299 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:12,236] Materialisation performed by reasoner in 291.5ms[0m
1778417893.0387871 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:13,038] Updating ['default'] with:
1778417893.0389977 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417893.0390892 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417893.0397484 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:13,038] Updating expiry date to 10/05/2026, 12:58:17[0m
1778417893.1976111 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:13,196] Materialisation performed by reasoner in 157.9ms[0m
1778417894.0971093 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:14,096] Updating ['default'] with:
1778417894.0974786 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417894.0977278 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417894.0987382 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:14,096] Updating expiry date to 10/05/2026, 12:58:18[0m
1778417894.2035658 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:14,202] Materialisation performed by reasoner in 105.5ms[0m
1778417895.2375154 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:15,236] Updating ['default'] with:
1778417895.2377319 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417895.2451272 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417895.2457464 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:15,237] Updating expiry date to 10/05/2026, 12:58:19[0m
1778417895.3096454 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:15,309] Materialisation performed by reasoner in 71.6ms[0m
1778417896.3210845 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:16,320] Updating ['default'] with:
1778417896.3212876 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417896.3213811 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417896.3219335 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:16,320] Updating expiry date to 10/05/2026, 12:58:20[0m
1778417896.3797622 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:16,379] Materialisation performed by reasoner in 58.2ms[0m
1778417897.3538921 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:17,351] Updating ['default'] with:
1778417897.3550074 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417897.3560040 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417897.3594320 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:17,352] Updating expiry date to 10/05/2026, 12:58:21[0m
1778417897.7379196 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:17,737] Materialisation performed by reasoner in 383.0ms[0m
1778417898.4085660 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:18,405] Updating ['default'] with:
1778417898.4096303 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417898.4101467 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417898.4124382 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:18,407] Updating expiry date to 10/05/2026, 12:58:22[0m
1778417898.5519269 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:18,550] Materialisation performed by reasoner in 141.8ms[0m
1778417899.3364563 [hri_face_detect_yunet-11] [33m[WARN] [1778417899.335267423] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 7.9sec[0m
1778417899.4425530 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:19,440] Updating ['default'] with:
1778417899.4433165 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417899.4437749 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417899.4453073 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:19,442] Updating expiry date to 10/05/2026, 12:58:23[0m
1778417899.5869818 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:19,586] Materialisation performed by reasoner in 143.1ms[0m
1778417900.4711390 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:20,469] Updating ['default'] with:
1778417900.4715097 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417900.4718585 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417900.4733460 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:20,470] Updating expiry date to 10/05/2026, 12:58:24[0m
1778417900.5570974 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:20,556] Materialisation performed by reasoner in 84.8ms[0m
1778417901.4290974 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:21,428] Updating ['default'] with:
1778417901.4293308 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417901.4294534 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417901.4300396 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:21,428] Updating expiry date to 10/05/2026, 12:58:25[0m
1778417901.5481498 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:21,547] Materialisation performed by reasoner in 118.5ms[0m
1778417902.5724061 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:22,570] Updating ['default'] with:
1778417902.5727057 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417902.5728705 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417902.5736113 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:22,571] Updating expiry date to 10/05/2026, 12:58:26[0m
1778417902.9975204 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:22,996] Materialisation performed by reasoner in 424.0ms[0m
1778417903.5822649 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:23,581] Updating ['default'] with:
1778417903.5826185 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417903.5828025 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417903.5840340 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:23,581] Updating expiry date to 10/05/2026, 12:58:27[0m
1778417903.7108970 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:23,710] Materialisation performed by reasoner in 127.9ms[0m
1778417904.3112965 [hri_face_detect_yunet-11] [33m[WARN] [1778417904.310775246] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.0sec[0m
1778417904.5887890 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:24,586] Updating ['default'] with:
1778417904.5894377 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417904.5897911 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417904.5918565 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:24,588] Updating expiry date to 10/05/2026, 12:58:28[0m
1778417904.7312710 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:24,730] Materialisation performed by reasoner in 141.1ms[0m
1778417905.7830720 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:25,781] Updating ['default'] with:
1778417905.7834671 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417905.7836564 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417905.7864959 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:25,782] Updating expiry date to 10/05/2026, 12:58:29[0m
1778417905.9222102 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:25,921] Materialisation performed by reasoner in 139.0ms[0m
1778417906.7551556 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:26,754] Updating ['default'] with:
1778417906.7553685 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417906.7554967 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417906.7559032 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:26,754] Updating expiry date to 10/05/2026, 12:58:30[0m
1778417906.8738501 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:26,872] Materialisation performed by reasoner in 117.6ms[0m
1778417907.7664797 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:27,765] Updating ['default'] with:
1778417907.7666543 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417907.7667208 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417907.7672493 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:27,766] Updating expiry date to 10/05/2026, 12:58:31[0m
1778417908.1488194 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:28,147] Materialisation performed by reasoner in 380.9ms[0m
1778417908.5290277 [hri_face_detect_yunet-11] [33m[WARN] [1778417908.528603443] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 4.2sec[0m
1778417908.8843341 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:28,883] Updating ['default'] with:
1778417908.8845809 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417908.8846698 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417908.8852887 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:28,884] Updating expiry date to 10/05/2026, 12:58:32[0m
1778417909.0274696 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:29,026] Materialisation performed by reasoner in 141.7ms[0m
1778417909.9161358 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:29,915] Updating ['default'] with:
1778417909.9163499 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417909.9165061 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417909.9170275 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:29,915] Updating expiry date to 10/05/2026, 12:58:33[0m
1778417910.0062482 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:30,005] Materialisation performed by reasoner in 89.4ms[0m
1778417910.9515514 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:30,950] Updating ['default'] with:
1778417910.9518030 [knowledge_core-5] 	- myself sees detected_blueberry_389_190
1778417910.9520361 [knowledge_core-5] 	- detected_blueberry_389_190 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417910.9521825 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:30,950] Updating expiry date to 10/05/2026, 12:58:34[0m
1778417911.0302806 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:31,029] Materialisation performed by reasoner in 78.7ms[0m
1778417914.9766774 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:58:34,975] Removing expired statement <myself sees detected_blueberry_389_190> from <default> (expired on 2026-05-10T12:58:34+00:00)[0m
1778417914.9779792 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:58:34,975] Removing expired statement <detected_blueberry_389_190 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:58:34+00:00)[0m
1778417915.0582416 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:35,057] Materialisation performed by reasoner in 81.6ms[0m
1778417917.3280735 [hri_face_detect_yunet-11] [33m[WARN] [1778417917.327479976] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 8.8sec[0m
1778417930.0655417 [hri_face_detect_yunet-11] [33m[WARN] [1778417930.064841950] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 12.7sec[0m
1778417935.1274555 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:55,126] Deleting from ['default']:
1778417935.1277258 [knowledge_core-5] 	- anonymous_person_jbbii rdf:type Human[0m
1778417935.2001026 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:55,199] Materialisation performed by reasoner in 72.9ms[0m
1778417935.2984571 [hri_face_detect_yunet-11] [33m[WARN] [1778417935.297676227] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.2sec[0m
1778417935.3437216 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:55,343] Deleting from ['default']:
1778417935.3439162 [knowledge_core-5] 	- myself sees anonymous_person_jbbii[0m
1778417935.4483418 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:55,447] Materialisation performed by reasoner in 104.0ms[0m
1778417935.9705508 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:55,969] Updating ['default'] with:
1778417935.9707706 [knowledge_core-5] 	- anonymous_person_gcaji rdf:type Human[0m
1778417936.0554132 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:56,054] Materialisation performed by reasoner in 84.3ms[0m
1778417936.2066762 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:56,205] Updating ['default'] with:
1778417936.2069404 [knowledge_core-5] 	- myself sees anonymous_person_gcaji[0m
1778417936.2861612 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:56,285] Materialisation performed by reasoner in 79.6ms[0m
1778417936.4334750 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:56,432] Updating ['default'] with:
1778417936.4338975 [knowledge_core-5] 	- anonymous_person_gcaji sees myself[0m
1778417936.8249385 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:56,824] Materialisation performed by reasoner in 391.5ms[0m
1778417938.0019951 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:58,001] Deleting from ['default']:
1778417938.0034997 [knowledge_core-5] 	- anonymous_person_gcaji rdf:type Human[0m
1778417938.0647309 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:58,064] Materialisation performed by reasoner in 62.8ms[0m
1778417938.2089653 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:58,208] Deleting from ['default']:
1778417938.2092679 [knowledge_core-5] 	- myself sees anonymous_person_gcaji[0m
1778417938.3356235 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:58,334] Materialisation performed by reasoner in 126.4ms[0m
1778417938.9813356 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:58,980] Updating ['default'] with:
1778417938.9816067 [knowledge_core-5] 	- anonymous_person_efcfc rdf:type Human[0m
1778417939.0682728 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:59,067] Materialisation performed by reasoner in 86.8ms[0m
1778417939.2627587 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:59,260] Updating ['default'] with:
1778417939.2647882 [knowledge_core-5] 	- myself sees anonymous_person_efcfc[0m
1778417939.4004593 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:59,399] Materialisation performed by reasoner in 138.1ms[0m
1778417939.5589194 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:59,557] Updating ['default'] with:
1778417939.5594640 [knowledge_core-5] 	- anonymous_person_efcfc sees myself[0m
1778417939.6946163 [knowledge_core-5] [37m[INFO] [2026-05-10 12:58:59,693] Materialisation performed by reasoner in 135.8ms[0m
1778417940.1274147 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:00,126] Deleting from ['default']:
1778417940.1276927 [knowledge_core-5] 	- anonymous_person_efcfc rdf:type Human[0m
1778417940.3643172 [hri_face_detect_yunet-11] [33m[WARN] [1778417940.363865630] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.1sec[0m
1778417940.4354284 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:00,434] Materialisation performed by reasoner in 308.0ms[0m
1778417940.5842702 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:00,583] Deleting from ['default']:
1778417940.5845261 [knowledge_core-5] 	- myself sees anonymous_person_efcfc[0m
1778417940.6597955 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:00,659] Materialisation performed by reasoner in 75.4ms[0m
1778417945.6835008 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:05,682] Updating ['default'] with:
1778417945.6836872 [knowledge_core-5] 	- anonymous_person_jeech rdf:type Human[0m
1778417945.8018000 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:05,801] Materialisation performed by reasoner in 118.1ms[0m
1778417945.9481583 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:05,947] Updating ['default'] with:
1778417945.9483747 [knowledge_core-5] 	- myself sees anonymous_person_jeech[0m
1778417946.0825658 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:06,080] Materialisation performed by reasoner in 132.4ms[0m
1778417946.2323003 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:06,231] Updating ['default'] with:
1778417946.2325506 [knowledge_core-5] 	- anonymous_person_jeech sees myself[0m
1778417946.3517697 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:06,350] Materialisation performed by reasoner in 118.6ms[0m
1778417947.6691549 [hri_face_detect_yunet-11] [33m[WARN] [1778417947.668597268] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 7.3sec[0m
1778417953.5502927 [hri_face_detect_yunet-11] [33m[WARN] [1778417953.549877564] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.9sec[0m
1778417958.8765373 [hri_face_detect_yunet-11] [33m[WARN] [1778417958.876133072] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778417964.4800718 [hri_face_detect_yunet-11] [33m[WARN] [1778417964.479732336] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778417969.3504763 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:29,349] Updating ['default'] with:
1778417969.3506677 [knowledge_core-5] 	- myself sees detected_blueberry_312_346
1778417969.3507442 [knowledge_core-5] 	- detected_blueberry_312_346 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417969.3512721 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:29,350] This statement will expire on 10/05/2026, 12:59:33[0m
1778417969.4126055 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:29,412] Materialisation performed by reasoner in 61.7ms[0m
1778417969.9877582 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:29,987] Deleting from ['default']:
1778417969.9881778 [knowledge_core-5] 	- anonymous_person_jeech rdf:type Human[0m
1778417970.0644152 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:30,063] Materialisation performed by reasoner in 76.6ms[0m
1778417970.2067552 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:30,206] Deleting from ['default']:
1778417970.2069995 [knowledge_core-5] 	- myself sees anonymous_person_jeech[0m
1778417970.2597950 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:30,258] Materialisation performed by reasoner in 52.6ms[0m
1778417970.4034386 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:30,402] Updating ['default'] with:
1778417970.4036410 [knowledge_core-5] 	- myself sees detected_blueberry_312_346
1778417970.4037166 [knowledge_core-5] 	- detected_blueberry_312_346 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417970.4041986 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:30,403] Updating expiry date to 10/05/2026, 12:59:34[0m
1778417970.5706828 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:30,570] Materialisation performed by reasoner in 166.7ms[0m
1778417970.7433019 [hri_face_detect_yunet-11] [33m[WARN] [1778417970.742937069] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.3sec[0m
1778417971.5034695 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:31,502] Updating ['default'] with:
1778417971.5037389 [knowledge_core-5] 	- anonymous_person_hfhfh rdf:type Human[0m
1778417971.5760133 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:31,574] Materialisation performed by reasoner in 71.5ms[0m
1778417971.7277353 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:31,727] Updating ['default'] with:
1778417971.7279217 [knowledge_core-5] 	- myself sees anonymous_person_hfhfh[0m
1778417971.7895811 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:31,788] Materialisation performed by reasoner in 61.4ms[0m
1778417971.8622053 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:31,861] Deleting from ['default']:
1778417971.8624680 [knowledge_core-5] 	- anonymous_person_hfhfh rdf:type Human[0m
1778417971.9406426 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:31,940] Materialisation performed by reasoner in 78.2ms[0m
1778417972.0124269 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,011] Updating ['default'] with:
1778417972.0126584 [knowledge_core-5] 	- myself sees detected_blueberry_312_346
1778417972.0127637 [knowledge_core-5] 	- detected_blueberry_312_346 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417972.0134406 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,012] Updating expiry date to 10/05/2026, 12:59:36[0m
1778417972.0841777 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,083] Materialisation performed by reasoner in 71.2ms[0m
1778417972.2289596 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,228] Updating ['default'] with:
1778417972.2291055 [knowledge_core-5] 	- anonymous_person_hfhfh sees myself[0m
1778417972.2885439 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,287] Materialisation performed by reasoner in 59.1ms[0m
1778417972.3606312 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,359] Deleting from ['default']:
1778417972.3609972 [knowledge_core-5] 	- myself sees anonymous_person_hfhfh[0m
1778417972.6777318 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,677] Materialisation performed by reasoner in 317.2ms[0m
1778417972.8295419 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,828] Updating ['default'] with:
1778417972.8298028 [knowledge_core-5] 	- myself sees detected_blueberry_312_346
1778417972.8298910 [knowledge_core-5] 	- detected_blueberry_312_346 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417972.8304105 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,829] Updating expiry date to 10/05/2026, 12:59:36[0m
1778417972.8926172 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:32,892] Materialisation performed by reasoner in 62.4ms[0m
1778417973.4638202 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:33,463] Updating ['default'] with:
1778417973.4640830 [knowledge_core-5] 	- anonymous_person_jedic rdf:type Human[0m
1778417973.5394619 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:33,538] Materialisation performed by reasoner in 75.4ms[0m
1778417973.6843479 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:33,683] Updating ['default'] with:
1778417973.6845460 [knowledge_core-5] 	- myself sees anonymous_person_jedic[0m
1778417973.7478011 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:33,747] Materialisation performed by reasoner in 63.3ms[0m
1778417973.8279023 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:33,826] Updating ['default'] with:
1778417973.8281817 [knowledge_core-5] 	- myself sees detected_blueberry_312_346
1778417973.8282678 [knowledge_core-5] 	- detected_blueberry_312_346 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417973.8290715 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:33,827] Updating expiry date to 10/05/2026, 12:59:37[0m
1778417973.9016550 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:33,900] Materialisation performed by reasoner in 72.8ms[0m
1778417974.0458064 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:34,045] Updating ['default'] with:
1778417974.0459778 [knowledge_core-5] 	- anonymous_person_jedic sees myself[0m
1778417974.3211420 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:34,320] Materialisation performed by reasoner in 275.0ms[0m
1778417974.4639421 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:34,463] Deleting from ['default']:
1778417974.4640985 [knowledge_core-5] 	- anonymous_person_jedic rdf:type Human[0m
1778417974.5149124 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:34,514] Materialisation performed by reasoner in 50.6ms[0m
1778417974.6589940 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:34,658] Deleting from ['default']:
1778417974.6628854 [knowledge_core-5] 	- myself sees anonymous_person_jedic[0m
1778417974.7720144 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:34,771] Materialisation performed by reasoner in 112.9ms[0m
1778417974.8556776 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:34,854] Updating ['default'] with:
1778417974.8559368 [knowledge_core-5] 	- myself sees detected_blueberry_312_346
1778417974.8561308 [knowledge_core-5] 	- detected_blueberry_312_346 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778417974.8569057 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:34,855] Updating expiry date to 10/05/2026, 12:59:38[0m
1778417974.9380260 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:34,937] Materialisation performed by reasoner in 81.7ms[0m
1778417975.0824163 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:35,081] Updating ['default'] with:
1778417975.0826154 [knowledge_core-5] 	- anonymous_person_ahcid rdf:type Human[0m
1778417975.1565526 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:35,155] Materialisation performed by reasoner in 73.7ms[0m
1778417975.3009541 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:35,300] Updating ['default'] with:
1778417975.3011203 [knowledge_core-5] 	- myself sees anonymous_person_ahcid[0m
1778417975.3740399 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:35,373] Materialisation performed by reasoner in 73.0ms[0m
1778417975.5188484 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:35,518] Updating ['default'] with:
1778417975.5191302 [knowledge_core-5] 	- anonymous_person_ahcid sees myself[0m
1778417975.8109982 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:35,810] Materialisation performed by reasoner in 291.8ms[0m
1778417977.3535120 [hri_face_detect_yunet-11] [33m[WARN] [1778417977.352980958] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.6sec[0m
1778417979.0512114 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:59:39,050] Removing expired statement <myself sees detected_blueberry_312_346> from <default> (expired on 2026-05-10T12:59:38+00:00)[0m
1778417979.0519769 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:59:39,050] Removing expired statement <detected_blueberry_312_346 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:59:38+00:00)[0m
1778417979.1253111 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:39,124] Materialisation performed by reasoner in 73.5ms[0m
1778417981.0326803 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:41,032] Updating ['default'] with:
1778417981.0329061 [knowledge_core-5] 	- detected_blueberry_441_208 rdf:type Blueberry
1778417981.0329998 [knowledge_core-5] 	- myself sees detected_blueberry_441_208 (lifespan: 4.0sec)[0m
1778417981.0337579 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:41,032] This statement will expire on 10/05/2026, 12:59:45[0m
1778417981.1078181 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:41,107] Materialisation performed by reasoner in 74.6ms[0m
1778417982.0404181 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:42,039] Updating ['default'] with:
1778417982.0405939 [knowledge_core-5] 	- detected_blueberry_441_208 rdf:type Blueberry
1778417982.0406597 [knowledge_core-5] 	- myself sees detected_blueberry_441_208 (lifespan: 4.0sec)[0m
1778417982.0411530 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:42,040] Updating expiry date to 10/05/2026, 12:59:46[0m
1778417982.1048033 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:42,104] Materialisation performed by reasoner in 63.5ms[0m
1778417982.6808171 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:42,680] Deleting from ['default']:
1778417982.6810057 [knowledge_core-5] 	- anonymous_person_ahcid rdf:type Human[0m
1778417982.7522593 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:42,751] Materialisation performed by reasoner in 71.2ms[0m
1778417982.8955703 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:42,894] Deleting from ['default']:
1778417982.8957834 [knowledge_core-5] 	- myself sees anonymous_person_ahcid[0m
1778417983.1361511 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:43,135] Materialisation performed by reasoner in 240.5ms[0m
1778417983.2686601 [hri_face_detect_yunet-11] [33m[WARN] [1778417983.267287173] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.9sec[0m
1778417983.2797227 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:43,279] Updating ['default'] with:
1778417983.2802317 [knowledge_core-5] 	- detected_blueberry_441_208 rdf:type Blueberry
1778417983.2803750 [knowledge_core-5] 	- myself sees detected_blueberry_441_208 (lifespan: 4.0sec)[0m
1778417983.2811620 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:43,279] Updating expiry date to 10/05/2026, 12:59:47[0m
1778417983.3891144 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:43,388] Materialisation performed by reasoner in 108.6ms[0m
1778417983.5422146 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:43,541] Updating ['default'] with:
1778417983.5424094 [knowledge_core-5] 	- anonymous_person_iafdg rdf:type Human[0m
1778417983.6177869 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:43,617] Materialisation performed by reasoner in 75.2ms[0m
1778417983.7622352 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:43,761] Updating ['default'] with:
1778417983.7624626 [knowledge_core-5] 	- myself sees anonymous_person_iafdg[0m
1778417983.8397539 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:43,839] Materialisation performed by reasoner in 77.3ms[0m
1778417983.9844184 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:43,983] Updating ['default'] with:
1778417983.9847033 [knowledge_core-5] 	- anonymous_person_iafdg sees myself[0m
1778417984.0663013 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:44,065] Materialisation performed by reasoner in 81.3ms[0m
1778417984.2097478 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:44,208] Updating ['default'] with:
1778417984.2100046 [knowledge_core-5] 	- detected_blueberry_441_208 rdf:type Blueberry
1778417984.2102962 [knowledge_core-5] 	- myself sees detected_blueberry_441_208 (lifespan: 4.0sec)[0m
1778417984.2107522 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:44,209] Updating expiry date to 10/05/2026, 12:59:48[0m
1778417984.4906139 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:44,490] Materialisation performed by reasoner in 280.3ms[0m
1778417985.2807541 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:45,280] Updating ['default'] with:
1778417985.2809918 [knowledge_core-5] 	- detected_blueberry_441_208 rdf:type Blueberry
1778417985.2810593 [knowledge_core-5] 	- myself sees detected_blueberry_441_208 (lifespan: 4.0sec)[0m
1778417985.2814336 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:45,280] Updating expiry date to 10/05/2026, 12:59:49[0m
1778417985.3716793 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:45,370] Materialisation performed by reasoner in 89.7ms[0m
1778417985.8796713 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:45,879] Deleting from ['default']:
1778417985.8799191 [knowledge_core-5] 	- anonymous_person_iafdg rdf:type Human[0m
1778417985.9580615 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:45,957] Materialisation performed by reasoner in 78.1ms[0m
1778417986.1032457 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:46,102] Deleting from ['default']:
1778417986.1035001 [knowledge_core-5] 	- myself sees anonymous_person_iafdg[0m
1778417986.2029853 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:46,201] Materialisation performed by reasoner in 99.3ms[0m
1778417986.4885085 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:46,487] Updating ['default'] with:
1778417986.4887719 [knowledge_core-5] 	- detected_blueberry_441_208 rdf:type Blueberry
1778417986.4888601 [knowledge_core-5] 	- myself sees detected_blueberry_441_208 (lifespan: 4.0sec)[0m
1778417986.4893913 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:46,488] Updating expiry date to 10/05/2026, 12:59:50[0m
1778417986.5734978 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:46,572] Materialisation performed by reasoner in 84.4ms[0m
1778417986.9433987 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:46,942] Updating ['default'] with:
1778417986.9436853 [knowledge_core-5] 	- anonymous_person_cahja rdf:type Human[0m
1778417987.0457218 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:47,044] Materialisation performed by reasoner in 101.9ms[0m
1778417987.1908448 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:47,190] Updating ['default'] with:
1778417987.1910455 [knowledge_core-5] 	- myself sees anonymous_person_cahja[0m
1778417987.5145652 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:47,513] Materialisation performed by reasoner in 323.2ms[0m
1778417987.6707637 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:47,670] Updating ['default'] with:
1778417987.6710138 [knowledge_core-5] 	- anonymous_person_cahja sees myself[0m
1778417987.7389619 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:47,738] Materialisation performed by reasoner in 67.9ms[0m
1778417989.2557075 [hri_face_detect_yunet-11] [33m[WARN] [1778417989.255215045] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.0sec[0m
1778417990.8316574 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:59:50,831] Removing expired statement <detected_blueberry_441_208 rdf:type Blueberry> from <default> (expired on 2026-05-10T12:59:50+00:00)[0m
1778417990.8321176 [knowledge_core-5] [33m[WARNING] [2026-05-10 12:59:50,831] Removing expired statement <myself sees detected_blueberry_441_208> from <default> (expired on 2026-05-10T12:59:50+00:00)[0m
1778417990.9027929 [knowledge_core-5] [37m[INFO] [2026-05-10 12:59:50,902] Materialisation performed by reasoner in 70.6ms[0m
1778417994.6668415 [hri_face_detect_yunet-11] [33m[WARN] [1778417994.666415342] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.4sec[0m
1778418000.2711253 [hri_face_detect_yunet-11] [33m[WARN] [1778418000.270644206] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418004.2472029 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:04,244] Deleting from ['default']:
1778418004.2475927 [knowledge_core-5] 	- anonymous_person_cahja rdf:type Human[0m
1778418004.3386824 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:04,337] Materialisation performed by reasoner in 93.5ms[0m
1778418004.4815304 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:04,481] Deleting from ['default']:
1778418004.4817080 [knowledge_core-5] 	- myself sees anonymous_person_cahja[0m
1778418004.6990356 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:04,698] Materialisation performed by reasoner in 217.3ms[0m
1778418005.5480692 [hri_face_detect_yunet-11] [33m[WARN] [1778418005.547545232] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778418007.9237120 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:07,923] Updating ['default'] with:
1778418007.9239376 [knowledge_core-5] 	- anonymous_person_gjcba rdf:type Human[0m
1778418007.9959805 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:07,995] Materialisation performed by reasoner in 72.1ms[0m
1778418008.1506903 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:08,150] Updating ['default'] with:
1778418008.1510367 [knowledge_core-5] 	- myself sees anonymous_person_gjcba[0m
1778418008.2370532 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:08,236] Materialisation performed by reasoner in 86.1ms[0m
1778418008.3805413 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:08,379] Updating ['default'] with:
1778418008.3807340 [knowledge_core-5] 	- anonymous_person_gjcba sees myself[0m
1778418008.4520299 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:08,451] Materialisation performed by reasoner in 71.3ms[0m
1778418010.8547413 [hri_face_detect_yunet-11] [33m[WARN] [1778418010.854266569] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778418016.3592393 [hri_face_detect_yunet-11] [33m[WARN] [1778418016.358862403] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.5sec[0m
1778418022.2652898 [hri_face_detect_yunet-11] [33m[WARN] [1778418022.264870525] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.9sec[0m
1778418027.9495728 [hri_face_detect_yunet-11] [33m[WARN] [1778418027.949047999] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418033.5608158 [hri_face_detect_yunet-11] [33m[WARN] [1778418033.560380123] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418039.1668293 [hri_face_detect_yunet-11] [33m[WARN] [1778418039.166448510] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418044.8449619 [hri_face_detect_yunet-11] [33m[WARN] [1778418044.844575073] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418050.7760966 [hri_face_detect_yunet-11] [33m[WARN] [1778418050.775557040] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.9sec[0m
1778418052.7403061 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:52,739] Deleting from ['default']:
1778418052.7415721 [knowledge_core-5] 	- anonymous_person_gjcba rdf:type Human[0m
1778418052.8741899 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:52,873] Materialisation performed by reasoner in 134.0ms[0m
1778418053.0175722 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:53,016] Deleting from ['default']:
1778418053.0178301 [knowledge_core-5] 	- myself sees anonymous_person_gjcba[0m
1778418053.0946360 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:53,093] Materialisation performed by reasoner in 76.4ms[0m
1778418055.5286269 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:55,527] Updating ['default'] with:
1778418055.5291765 [knowledge_core-5] 	- myself sees detected_blueberry_376_446
1778418055.5293865 [knowledge_core-5] 	- detected_blueberry_376_446 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418055.5302393 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:55,528] This statement will expire on 10/05/2026, 13:00:59[0m
1778418055.7408810 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:55,740] Materialisation performed by reasoner in 211.8ms[0m
1778418057.0792749 [hri_face_detect_yunet-11] [33m[WARN] [1778418057.078775688] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.3sec[0m
1778418057.3115654 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:57,311] Updating ['default'] with:
1778418057.3117239 [knowledge_core-5] 	- myself sees detected_blueberry_364_444
1778418057.3117738 [knowledge_core-5] 	- detected_blueberry_364_444 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418057.3165371 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:57,315] This statement will expire on 10/05/2026, 13:01:01[0m
1778418057.3804634 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:57,379] Materialisation performed by reasoner in 63.5ms[0m
1778418057.8899181 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:57,889] Updating ['default'] with:
1778418057.8900826 [knowledge_core-5] 	- myself sees detected_blueberry_322_446
1778418057.8901517 [knowledge_core-5] 	- detected_blueberry_322_446 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418057.8905370 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:57,889] This statement will expire on 10/05/2026, 13:01:01[0m
1778418057.9582763 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:57,957] Materialisation performed by reasoner in 67.9ms[0m
1778418059.5543413 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:00:59,553] Removing expired statement <myself sees detected_blueberry_376_446> from <default> (expired on 2026-05-10T13:00:59+00:00)[0m
1778418059.5550065 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:00:59,553] Removing expired statement <detected_blueberry_376_446 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:00:59+00:00)[0m
1778418059.6102178 [knowledge_core-5] [37m[INFO] [2026-05-10 13:00:59,609] Materialisation performed by reasoner in 55.7ms[0m
1778418061.7494032 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:01,748] Removing expired statement <myself sees detected_blueberry_364_444> from <default> (expired on 2026-05-10T13:01:01+00:00)[0m
1778418061.7500751 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:01,749] Removing expired statement <detected_blueberry_364_444 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:01+00:00)[0m
1778418061.7502813 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:01,749] Removing expired statement <myself sees detected_blueberry_322_446> from <default> (expired on 2026-05-10T13:01:01+00:00)[0m
1778418061.7504742 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:01,749] Removing expired statement <detected_blueberry_322_446 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:01+00:00)[0m
1778418061.8194773 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:01,818] Materialisation performed by reasoner in 69.0ms[0m
1778418062.7442768 [hri_face_detect_yunet-11] [33m[WARN] [1778418062.743852623] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418064.8045242 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:04,803] Updating ['default'] with:
1778418064.8048654 [knowledge_core-5] 	- anonymous_person_hchfb rdf:type Human[0m
1778418065.1093855 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:05,108] Materialisation performed by reasoner in 304.7ms[0m
1778418065.2606194 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:05,260] Updating ['default'] with:
1778418065.2608280 [knowledge_core-5] 	- myself sees anonymous_person_hchfb[0m
1778418065.3261635 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:05,325] Materialisation performed by reasoner in 64.6ms[0m
1778418065.4709458 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:05,470] Updating ['default'] with:
1778418065.4712093 [knowledge_core-5] 	- anonymous_person_hchfb sees myself[0m
1778418065.5471067 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:05,546] Materialisation performed by reasoner in 75.8ms[0m
1778418068.9453223 [hri_face_detect_yunet-11] [33m[WARN] [1778418068.944990896] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.2sec[0m
1778418073.0549986 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:13,054] Updating ['default'] with:
1778418073.0551729 [knowledge_core-5] 	- detected_blueberry_427_218 rdf:type Blueberry
1778418073.0552356 [knowledge_core-5] 	- myself sees detected_blueberry_427_218 (lifespan: 4.0sec)[0m
1778418073.0556099 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:13,054] This statement will expire on 10/05/2026, 13:01:17[0m
1778418073.1153407 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:13,114] Materialisation performed by reasoner in 59.8ms[0m
1778418074.1204131 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:14,119] Updating ['default'] with:
1778418074.1206815 [knowledge_core-5] 	- detected_blueberry_427_218 rdf:type Blueberry
1778418074.1207595 [knowledge_core-5] 	- myself sees detected_blueberry_427_218 (lifespan: 4.0sec)[0m
1778418074.1213160 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:14,120] Updating expiry date to 10/05/2026, 13:01:18[0m
1778418074.1904235 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:14,189] Materialisation performed by reasoner in 69.4ms[0m
1778418074.8617234 [hri_face_detect_yunet-11] [33m[WARN] [1778418074.861084454] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.9sec[0m
1778418075.1874359 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:15,186] Updating ['default'] with:
1778418075.1877666 [knowledge_core-5] 	- detected_blueberry_427_218 rdf:type Blueberry
1778418075.1878333 [knowledge_core-5] 	- myself sees detected_blueberry_427_218 (lifespan: 4.0sec)[0m
1778418075.1886361 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:15,187] Updating expiry date to 10/05/2026, 13:01:19[0m
1778418075.2544751 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:15,253] Materialisation performed by reasoner in 66.0ms[0m
1778418076.3286097 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:16,327] Updating ['default'] with:
1778418076.3288481 [knowledge_core-5] 	- detected_blueberry_427_218 rdf:type Blueberry
1778418076.3289130 [knowledge_core-5] 	- myself sees detected_blueberry_427_218 (lifespan: 4.0sec)[0m
1778418076.3289661 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:16,328] Updating expiry date to 10/05/2026, 13:01:20[0m
1778418076.4012709 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:16,400] Materialisation performed by reasoner in 72.4ms[0m
1778418077.3326254 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:17,331] Updating ['default'] with:
1778418077.3329051 [knowledge_core-5] 	- detected_blueberry_427_218 rdf:type Blueberry
1778418077.3329811 [knowledge_core-5] 	- myself sees detected_blueberry_427_218 (lifespan: 4.0sec)[0m
1778418077.3335407 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:17,332] Updating expiry date to 10/05/2026, 13:01:21[0m
1778418077.3978128 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:17,396] Materialisation performed by reasoner in 64.3ms[0m
1778418078.5434942 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:18,542] Updating ['default'] with:
1778418078.5437860 [knowledge_core-5] 	- detected_blueberry_427_218 rdf:type Blueberry
1778418078.5438592 [knowledge_core-5] 	- myself sees detected_blueberry_427_218 (lifespan: 4.0sec)[0m
1778418078.5444212 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:18,543] Updating expiry date to 10/05/2026, 13:01:22[0m
1778418078.6155264 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:18,615] Materialisation performed by reasoner in 71.7ms[0m
1778418079.6220477 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:19,620] Updating ['default'] with:
1778418079.6222608 [knowledge_core-5] 	- detected_blueberry_427_218 rdf:type Blueberry
1778418079.6223660 [knowledge_core-5] 	- myself sees detected_blueberry_427_218 (lifespan: 4.0sec)[0m
1778418079.6224439 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:19,620] Updating expiry date to 10/05/2026, 13:01:23[0m
1778418079.7013211 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:19,700] Materialisation performed by reasoner in 79.8ms[0m
1778418080.6479056 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:20,647] Updating ['default'] with:
1778418080.6482906 [knowledge_core-5] 	- detected_blueberry_427_218 rdf:type Blueberry
1778418080.6487987 [knowledge_core-5] 	- myself sees detected_blueberry_427_218 (lifespan: 4.0sec)[0m
1778418080.6488738 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:20,647] Updating expiry date to 10/05/2026, 13:01:24[0m
1778418080.7199087 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:20,719] Materialisation performed by reasoner in 71.5ms[0m
1778418081.1657529 [hri_face_detect_yunet-11] [33m[WARN] [1778418081.165304639] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.3sec[0m
1778418081.7211447 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:21,720] Updating ['default'] with:
1778418081.7215676 [knowledge_core-5] 	- detected_blueberry_427_218 rdf:type Blueberry
1778418081.7216389 [knowledge_core-5] 	- myself sees detected_blueberry_427_218 (lifespan: 4.0sec)[0m
1778418081.7224770 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:21,720] Updating expiry date to 10/05/2026, 13:01:25[0m
1778418081.7941089 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:21,793] Materialisation performed by reasoner in 72.6ms[0m
1778418083.2242949 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:23,223] Deleting from ['default']:
1778418083.2245495 [knowledge_core-5] 	- anonymous_person_hchfb rdf:type Human[0m
1778418083.3270621 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:23,326] Materialisation performed by reasoner in 98.5ms[0m
1778418083.4703426 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:23,469] Deleting from ['default']:
1778418083.4706545 [knowledge_core-5] 	- myself sees anonymous_person_hchfb[0m
1778418083.5472727 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:23,546] Materialisation performed by reasoner in 76.5ms[0m
1778418083.6210301 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:23,620] Updating ['default'] with:
1778418083.6211846 [knowledge_core-5] 	- detected_blueberry_434_446 rdf:type Blueberry
1778418083.6212485 [knowledge_core-5] 	- myself sees detected_blueberry_434_446 (lifespan: 4.0sec)[0m
1778418083.6217537 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:23,620] This statement will expire on 10/05/2026, 13:01:27[0m
1778418083.6812274 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:23,680] Materialisation performed by reasoner in 59.7ms[0m
1778418086.0292957 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:26,028] Removing expired statement <detected_blueberry_427_218 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:25+00:00)[0m
1778418086.0296991 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:26,029] Removing expired statement <myself sees detected_blueberry_427_218> from <default> (expired on 2026-05-10T13:01:25+00:00)[0m
1778418086.0750420 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:26,074] Materialisation performed by reasoner in 44.9ms[0m
1778418086.2718062 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:26,271] Updating ['default'] with:
1778418086.2719684 [knowledge_core-5] 	- myself sees detected_blueberry_442_450
1778418086.2720318 [knowledge_core-5] 	- detected_blueberry_442_450 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418086.2723854 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:26,271] This statement will expire on 10/05/2026, 13:01:30[0m
1778418086.3486102 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:26,348] Materialisation performed by reasoner in 76.3ms[0m
1778418086.7691762 [hri_face_detect_yunet-11] [33m[WARN] [1778418086.768708051] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418087.0854466 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:27,084] Removing expired statement <detected_blueberry_434_446 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:27+00:00)[0m
1778418087.0856855 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:27,084] Removing expired statement <myself sees detected_blueberry_434_446> from <default> (expired on 2026-05-10T13:01:27+00:00)[0m
1778418087.1486714 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:27,148] Materialisation performed by reasoner in 63.2ms[0m
1778418090.3581972 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:30,357] Removing expired statement <myself sees detected_blueberry_442_450> from <default> (expired on 2026-05-10T13:01:30+00:00)[0m
1778418090.3583705 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:30,357] Removing expired statement <detected_blueberry_442_450 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:30+00:00)[0m
1778418090.4301665 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:30,429] Materialisation performed by reasoner in 71.7ms[0m
1778418090.7671292 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:30,766] Updating ['default'] with:
1778418090.7673359 [knowledge_core-5] 	- myself sees detected_blueberry_429_449
1778418090.7674189 [knowledge_core-5] 	- detected_blueberry_429_449 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418090.7679105 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:30,766] This statement will expire on 10/05/2026, 13:01:34[0m
1778418090.8395205 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:30,839] Materialisation performed by reasoner in 72.0ms[0m
1778418091.8696091 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:31,869] Updating ['default'] with:
1778418091.8698134 [knowledge_core-5] 	- myself sees detected_blueberry_429_449
1778418091.8698952 [knowledge_core-5] 	- detected_blueberry_429_449 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418091.8702741 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:31,869] Updating expiry date to 10/05/2026, 13:01:35[0m
1778418091.9421887 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:31,941] Materialisation performed by reasoner in 71.8ms[0m
1778418092.4733326 [hri_face_detect_yunet-11] [33m[WARN] [1778418092.472923289] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418093.2346134 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:33,233] Updating ['default'] with:
1778418093.2350085 [knowledge_core-5] 	- detected_blueberry_430_449 rdf:type Blueberry
1778418093.2351100 [knowledge_core-5] 	- myself sees detected_blueberry_430_449 (lifespan: 4.0sec)[0m
1778418093.2357779 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:33,234] This statement will expire on 10/05/2026, 13:01:37[0m
1778418093.3235044 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:33,322] Materialisation performed by reasoner in 88.3ms[0m
1778418095.9960608 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:35,995] Removing expired statement <myself sees detected_blueberry_429_449> from <default> (expired on 2026-05-10T13:01:35+00:00)[0m
1778418095.9963930 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:35,995] Removing expired statement <detected_blueberry_429_449 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:35+00:00)[0m
1778418096.0709543 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:36,070] Materialisation performed by reasoner in 74.8ms[0m
1778418096.5500803 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:36,549] Updating ['default'] with:
1778418096.5503502 [knowledge_core-5] 	- detected_blueberry_452_453 rdf:type Blueberry
1778418096.5504551 [knowledge_core-5] 	- myself sees detected_blueberry_452_453 (lifespan: 4.0sec)[0m
1778418096.5508676 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:36,549] This statement will expire on 10/05/2026, 13:01:40[0m
1778418096.6194403 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:36,618] Materialisation performed by reasoner in 68.8ms[0m
1778418097.1456783 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:37,145] Removing expired statement <detected_blueberry_430_449 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:37+00:00)[0m
1778418097.1463361 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:37,145] Removing expired statement <myself sees detected_blueberry_430_449> from <default> (expired on 2026-05-10T13:01:37+00:00)[0m
1778418097.4202938 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:37,419] Materialisation performed by reasoner in 273.8ms[0m
1778418098.5783513 [hri_face_detect_yunet-11] [33m[WARN] [1778418098.577831503] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.1sec[0m
1778418099.0442889 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:39,043] Updating ['default'] with:
1778418099.0444832 [knowledge_core-5] 	- detected_blueberry_414_447 rdf:type Blueberry
1778418099.0445526 [knowledge_core-5] 	- myself sees detected_blueberry_414_447 (lifespan: 4.0sec)[0m
1778418099.0449429 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:39,044] This statement will expire on 10/05/2026, 13:01:43[0m
1778418099.0995467 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:39,099] Materialisation performed by reasoner in 54.5ms[0m
1778418100.6307254 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:40,622] Removing expired statement <detected_blueberry_452_453 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:40+00:00)[0m
1778418100.6309087 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:40,623] Removing expired statement <myself sees detected_blueberry_452_453> from <default> (expired on 2026-05-10T13:01:40+00:00)[0m
1778418100.6818326 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:40,680] Materialisation performed by reasoner in 57.7ms[0m
1778418101.5867326 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:41,586] Updating ['default'] with:
1778418101.5869977 [knowledge_core-5] 	- detected_blueberry_450_453 rdf:type Blueberry
1778418101.5871058 [knowledge_core-5] 	- myself sees detected_blueberry_450_453 (lifespan: 4.0sec)[0m
1778418101.5876539 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:41,586] This statement will expire on 10/05/2026, 13:01:45[0m
1778418101.6527054 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:41,652] Materialisation performed by reasoner in 65.4ms[0m
1778418102.7268281 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:42,726] Updating ['default'] with:
1778418102.7270236 [knowledge_core-5] 	- detected_blueberry_450_453 rdf:type Blueberry
1778418102.7271068 [knowledge_core-5] 	- myself sees detected_blueberry_450_453 (lifespan: 4.0sec)[0m
1778418102.7275836 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:42,726] Updating expiry date to 10/05/2026, 13:01:46[0m
1778418102.7780526 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:42,777] Materialisation performed by reasoner in 50.1ms[0m
1778418103.8790460 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:43,878] Removing expired statement <detected_blueberry_414_447 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:43+00:00)[0m
1778418103.8794222 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:43,878] Removing expired statement <myself sees detected_blueberry_414_447> from <default> (expired on 2026-05-10T13:01:43+00:00)[0m
1778418104.1597540 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:44,159] Materialisation performed by reasoner in 280.4ms[0m
1778418104.2132492 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:44,212] Updating ['default'] with:
1778418104.2134516 [knowledge_core-5] 	- detected_blueberry_441_450 rdf:type Blueberry
1778418104.2135134 [knowledge_core-5] 	- myself sees detected_blueberry_441_450 (lifespan: 4.0sec)[0m
1778418104.2139094 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:44,213] This statement will expire on 10/05/2026, 13:01:48[0m
1778418104.2620263 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:44,261] Materialisation performed by reasoner in 48.1ms[0m
1778418104.4448893 [hri_face_detect_yunet-11] [33m[WARN] [1778418104.444559221] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.9sec[0m
1778418105.4772677 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:45,476] Updating ['default'] with:
1778418105.4774759 [knowledge_core-5] 	- detected_blueberry_375_453 rdf:type Blueberry
1778418105.4776266 [knowledge_core-5] 	- myself sees detected_blueberry_375_453 (lifespan: 4.0sec)[0m
1778418105.4780829 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:45,477] This statement will expire on 10/05/2026, 13:01:49[0m
1778418105.5661294 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:45,565] Materialisation performed by reasoner in 88.2ms[0m
1778418106.2345579 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:46,233] Removing expired statement <detected_blueberry_450_453 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:46+00:00)[0m
1778418106.2352440 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:46,234] Removing expired statement <myself sees detected_blueberry_450_453> from <default> (expired on 2026-05-10T13:01:46+00:00)[0m
1778418106.3096504 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:46,309] Materialisation performed by reasoner in 74.8ms[0m
1778418106.8007464 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:46,796] Updating ['default'] with:
1778418106.8011444 [knowledge_core-5] 	- detected_blueberry_408_446 rdf:type Blueberry
1778418106.8013253 [knowledge_core-5] 	- myself sees detected_blueberry_408_446 (lifespan: 4.0sec)[0m
1778418106.8014724 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:46,797] This statement will expire on 10/05/2026, 13:01:50[0m
1778418106.8829639 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:46,882] Materialisation performed by reasoner in 84.4ms[0m
1778418108.4097552 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:48,409] Removing expired statement <detected_blueberry_441_450 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:48+00:00)[0m
1778418108.4099228 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:48,409] Removing expired statement <myself sees detected_blueberry_441_450> from <default> (expired on 2026-05-10T13:01:48+00:00)[0m
1778418108.4696064 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:48,469] Materialisation performed by reasoner in 59.7ms[0m
1778418109.7823741 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:49,781] Removing expired statement <detected_blueberry_375_453 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:49+00:00)[0m
1778418109.7832966 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:49,781] Removing expired statement <myself sees detected_blueberry_375_453> from <default> (expired on 2026-05-10T13:01:49+00:00)[0m
1778418109.8667848 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:49,866] Materialisation performed by reasoner in 84.2ms[0m
1778418110.5558619 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:50,555] Updating ['default'] with:
1778418110.5561559 [knowledge_core-5] 	- detected_blueberry_452_453 rdf:type Blueberry
1778418110.5562654 [knowledge_core-5] 	- myself sees detected_blueberry_452_453 (lifespan: 4.0sec)[0m
1778418110.5571222 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:50,555] This statement will expire on 10/05/2026, 13:01:54[0m
1778418110.6252041 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:50,624] Materialisation performed by reasoner in 68.6ms[0m
1778418110.9356170 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:50,935] Removing expired statement <detected_blueberry_408_446 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:50+00:00)[0m
1778418110.9360247 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:50,935] Removing expired statement <myself sees detected_blueberry_408_446> from <default> (expired on 2026-05-10T13:01:50+00:00)[0m
1778418110.9817982 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:50,981] Materialisation performed by reasoner in 45.9ms[0m
1778418111.0539804 [hri_face_detect_yunet-11] [33m[WARN] [1778418111.053425768] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.6sec[0m
1778418112.3904839 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:52,390] Updating ['default'] with:
1778418112.3908770 [knowledge_core-5] 	- detected_blueberry_433_446 rdf:type Blueberry
1778418112.3909998 [knowledge_core-5] 	- myself sees detected_blueberry_433_446 (lifespan: 4.0sec)[0m
1778418112.3917067 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:52,390] This statement will expire on 10/05/2026, 13:01:56[0m
1778418112.4680643 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:52,467] Materialisation performed by reasoner in 77.0ms[0m
1778418114.1355748 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:54,135] Removing expired statement <detected_blueberry_452_453 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:54+00:00)[0m
1778418114.1361632 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:54,135] Removing expired statement <myself sees detected_blueberry_452_453> from <default> (expired on 2026-05-10T13:01:54+00:00)[0m
1778418114.1885936 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:54,187] Materialisation performed by reasoner in 52.6ms[0m
1778418116.5285385 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:56,528] Removing expired statement <detected_blueberry_433_446 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:01:56+00:00)[0m
1778418116.5286863 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:01:56,528] Removing expired statement <myself sees detected_blueberry_433_446> from <default> (expired on 2026-05-10T13:01:56+00:00)[0m
1778418116.5791774 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:56,578] Materialisation performed by reasoner in 50.4ms[0m
1778418116.6559446 [hri_face_detect_yunet-11] [33m[WARN] [1778418116.655460548] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418119.9863222 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:59,985] Updating ['default'] with:
1778418119.9865837 [knowledge_core-5] 	- myself sees detected_blueberry_450_449
1778418119.9867153 [knowledge_core-5] 	- detected_blueberry_450_449 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418119.9874635 [knowledge_core-5] [37m[INFO] [2026-05-10 13:01:59,985] This statement will expire on 10/05/2026, 13:02:03[0m
1778418120.0530927 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:00,052] Materialisation performed by reasoner in 66.4ms[0m
1778418121.0564649 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:01,056] Updating ['default'] with:
1778418121.0566413 [knowledge_core-5] 	- myself sees detected_blueberry_388_443
1778418121.0567083 [knowledge_core-5] 	- detected_blueberry_388_443 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418121.0570624 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:01,056] This statement will expire on 10/05/2026, 13:02:05[0m
1778418121.1284654 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:01,127] Materialisation performed by reasoner in 71.2ms[0m
1778418122.1329007 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:02,132] Updating ['default'] with:
1778418122.1330826 [knowledge_core-5] 	- myself sees detected_blueberry_388_443
1778418122.1331472 [knowledge_core-5] 	- detected_blueberry_388_443 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418122.1338260 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:02,132] Updating expiry date to 10/05/2026, 13:02:06[0m
1778418122.1597493 [hri_face_detect_yunet-11] [33m[WARN] [1778418122.159245701] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.5sec[0m
1778418122.2069104 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:02,206] Materialisation performed by reasoner in 73.1ms[0m
1778418123.0128922 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:03,012] Removing expired statement <myself sees detected_blueberry_450_449> from <default> (expired on 2026-05-10T13:02:03+00:00)[0m
1778418123.0130746 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:03,012] Removing expired statement <detected_blueberry_450_449 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:03+00:00)[0m
1778418123.2044303 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:03,203] Materialisation performed by reasoner in 190.9ms[0m
1778418124.9664705 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:04,966] Updating ['default'] with:
1778418124.9666681 [knowledge_core-5] 	- detected_blueberry_459_450 rdf:type Blueberry
1778418124.9667451 [knowledge_core-5] 	- myself sees detected_blueberry_459_450 (lifespan: 4.0sec)[0m
1778418124.9671929 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:04,966] This statement will expire on 10/05/2026, 13:02:08[0m
1778418125.0217159 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:05,021] Materialisation performed by reasoner in 54.5ms[0m
1778418126.4062488 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:06,405] Removing expired statement <myself sees detected_blueberry_388_443> from <default> (expired on 2026-05-10T13:02:06+00:00)[0m
1778418126.4069495 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:06,405] Removing expired statement <detected_blueberry_388_443 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:06+00:00)[0m
1778418126.4809308 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:06,480] Materialisation performed by reasoner in 74.6ms[0m
1778418127.4629095 [hri_face_detect_yunet-11] [33m[WARN] [1778418127.462592921] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778418128.6491823 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:08,647] Removing expired statement <detected_blueberry_459_450 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:08+00:00)[0m
1778418128.6493874 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:08,647] Removing expired statement <myself sees detected_blueberry_459_450> from <default> (expired on 2026-05-10T13:02:08+00:00)[0m
1778418128.7422216 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:08,741] Materialisation performed by reasoner in 93.7ms[0m
1778418129.2948127 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:09,293] Updating ['default'] with:
1778418129.2950208 [knowledge_core-5] 	- detected_blueberry_355_454 rdf:type Blueberry
1778418129.2951319 [knowledge_core-5] 	- myself sees detected_blueberry_355_454 (lifespan: 4.0sec)[0m
1778418129.2952220 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:09,293] This statement will expire on 10/05/2026, 13:02:13[0m
1778418129.3728998 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:09,372] Materialisation performed by reasoner in 78.1ms[0m
1778418133.5262480 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:13,525] Updating ['default'] with:
1778418133.5265083 [knowledge_core-5] 	- myself sees detected_blueberry_386_448
1778418133.5265753 [knowledge_core-5] 	- detected_blueberry_386_448 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418133.5269701 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:13,526] This statement will expire on 10/05/2026, 13:02:17[0m
1778418133.8160911 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:13,815] Materialisation performed by reasoner in 289.3ms[0m
1778418133.9927318 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:13,990] Removing expired statement <detected_blueberry_355_454 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:13+00:00)[0m
1778418133.9933333 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:13,990] Removing expired statement <myself sees detected_blueberry_355_454> from <default> (expired on 2026-05-10T13:02:13+00:00)[0m
1778418134.0697906 [hri_face_detect_yunet-11] [33m[WARN] [1778418134.069045967] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.6sec[0m
1778418134.0741086 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:14,073] Materialisation performed by reasoner in 83.1ms[0m
1778418136.1219730 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:16,121] Updating ['default'] with:
1778418136.1221759 [knowledge_core-5] 	- detected_blueberry_397_447 rdf:type Blueberry
1778418136.1222529 [knowledge_core-5] 	- myself sees detected_blueberry_397_447 (lifespan: 4.0sec)[0m
1778418136.1226881 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:16,121] This statement will expire on 10/05/2026, 13:02:20[0m
1778418136.1867700 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:16,186] Materialisation performed by reasoner in 64.3ms[0m
1778418137.1889749 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:17,188] Updating ['default'] with:
1778418137.1891506 [knowledge_core-5] 	- detected_blueberry_397_447 rdf:type Blueberry
1778418137.1892126 [knowledge_core-5] 	- myself sees detected_blueberry_397_447 (lifespan: 4.0sec)[0m
1778418137.1895492 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:17,188] Updating expiry date to 10/05/2026, 13:02:21[0m
1778418137.2612493 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:17,260] Materialisation performed by reasoner in 71.5ms[0m
1778418137.2922268 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:17,291] Removing expired statement <myself sees detected_blueberry_386_448> from <default> (expired on 2026-05-10T13:02:17+00:00)[0m
1778418137.2927876 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:17,291] Removing expired statement <detected_blueberry_386_448 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:17+00:00)[0m
1778418137.3692253 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:17,368] Materialisation performed by reasoner in 76.7ms[0m
1778418139.4175184 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:19,416] Updating ['default'] with:
1778418139.4178996 [knowledge_core-5] 	- myself sees detected_blueberry_380_447
1778418139.4180050 [knowledge_core-5] 	- detected_blueberry_380_447 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418139.4185548 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:19,417] This statement will expire on 10/05/2026, 13:02:23[0m
1778418139.6787462 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:19,678] Materialisation performed by reasoner in 260.6ms[0m
1778418140.1419246 [hri_face_detect_yunet-11] [33m[WARN] [1778418140.141388718] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.1sec[0m
1778418140.9033821 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:20,902] Updating ['default'] with:
1778418140.9035566 [knowledge_core-5] 	- myself sees detected_blueberry_404_449
1778418140.9036305 [knowledge_core-5] 	- detected_blueberry_404_449 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418140.9039779 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:20,903] This statement will expire on 10/05/2026, 13:02:24[0m
1778418140.9743972 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:20,973] Materialisation performed by reasoner in 70.6ms[0m
1778418141.8547218 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:21,854] Removing expired statement <detected_blueberry_397_447 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:21+00:00)[0m
1778418141.8551815 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:21,854] Removing expired statement <myself sees detected_blueberry_397_447> from <default> (expired on 2026-05-10T13:02:21+00:00)[0m
1778418141.9256809 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:21,925] Materialisation performed by reasoner in 70.6ms[0m
1778418141.9781713 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:21,977] Updating ['default'] with:
1778418141.9783998 [knowledge_core-5] 	- myself sees detected_blueberry_404_449
1778418141.9784751 [knowledge_core-5] 	- detected_blueberry_404_449 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418141.9789727 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:21,978] Updating expiry date to 10/05/2026, 13:02:25[0m
1778418142.0623779 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:22,061] Materialisation performed by reasoner in 83.5ms[0m
1778418144.0130732 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:24,012] Removing expired statement <myself sees detected_blueberry_380_447> from <default> (expired on 2026-05-10T13:02:23+00:00)[0m
1778418144.0132616 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:24,012] Removing expired statement <detected_blueberry_380_447 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:23+00:00)[0m
1778418144.0683398 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:24,067] Materialisation performed by reasoner in 55.1ms[0m
1778418145.1390321 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:25,138] Removing expired statement <myself sees detected_blueberry_404_449> from <default> (expired on 2026-05-10T13:02:25+00:00)[0m
1778418145.1392345 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:25,138] Removing expired statement <detected_blueberry_404_449 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:25+00:00)[0m
1778418145.4186230 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:25,417] Materialisation performed by reasoner in 279.1ms[0m
1778418145.5775082 [hri_face_detect_yunet-11] [33m[WARN] [1778418145.577015902] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.4sec[0m
1778418145.7578905 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:25,757] Updating ['default'] with:
1778418145.7581418 [knowledge_core-5] 	- detected_blueberry_375_447 rdf:type Blueberry
1778418145.7582161 [knowledge_core-5] 	- myself sees detected_blueberry_375_447 (lifespan: 4.0sec)[0m
1778418145.7587280 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:25,757] This statement will expire on 10/05/2026, 13:02:29[0m
1778418145.8391337 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:25,838] Materialisation performed by reasoner in 80.8ms[0m
1778418147.4069376 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:27,406] Updating ['default'] with:
1778418147.4073901 [knowledge_core-5] 	- detected_blueberry_375_447 rdf:type Blueberry
1778418147.4074702 [knowledge_core-5] 	- myself sees detected_blueberry_375_447 (lifespan: 4.0sec)[0m
1778418147.4079444 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:27,406] Updating expiry date to 10/05/2026, 13:02:31[0m
1778418147.4710844 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:27,470] Materialisation performed by reasoner in 63.8ms[0m
1778418149.1897984 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:29,189] Updating ['default'] with:
1778418149.1899788 [knowledge_core-5] 	- myself sees detected_blueberry_285_449
1778418149.1900463 [knowledge_core-5] 	- detected_blueberry_285_449 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418149.1903877 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:29,189] This statement will expire on 10/05/2026, 13:02:33[0m
1778418149.2391605 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:29,238] Materialisation performed by reasoner in 49.0ms[0m
1778418150.8897893 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:30,889] Updating ['default'] with:
1778418150.8899868 [knowledge_core-5] 	- myself sees detected_blueberry_426_446
1778418150.8900371 [knowledge_core-5] 	- detected_blueberry_426_446 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418150.8904140 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:30,889] This statement will expire on 10/05/2026, 13:02:34[0m
1778418150.9641147 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:30,963] Materialisation performed by reasoner in 73.8ms[0m
1778418151.0484686 [hri_face_detect_yunet-11] [33m[WARN] [1778418151.048035615] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.5sec[0m
1778418151.7712400 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:31,770] Removing expired statement <detected_blueberry_375_447 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:31+00:00)[0m
1778418151.7716458 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:31,770] Removing expired statement <myself sees detected_blueberry_375_447> from <default> (expired on 2026-05-10T13:02:31+00:00)[0m
1778418152.0293727 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:32,028] Materialisation performed by reasoner in 257.9ms[0m
1778418152.0818565 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:32,081] Updating ['default'] with:
1778418152.0822322 [knowledge_core-5] 	- myself sees detected_blueberry_426_446
1778418152.0823221 [knowledge_core-5] 	- detected_blueberry_426_446 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418152.0829265 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:32,081] Updating expiry date to 10/05/2026, 13:02:36[0m
1778418152.1525786 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:32,152] Materialisation performed by reasoner in 70.1ms[0m
1778418153.1016889 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:33,101] Removing expired statement <myself sees detected_blueberry_285_449> from <default> (expired on 2026-05-10T13:02:33+00:00)[0m
1778418153.1021514 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:33,101] Removing expired statement <detected_blueberry_285_449 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:33+00:00)[0m
1778418153.1732130 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:33,172] Materialisation performed by reasoner in 71.2ms[0m
1778418153.3694170 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:33,368] Updating ['default'] with:
1778418153.3695893 [knowledge_core-5] 	- detected_blueberry_426_448 rdf:type Blueberry
1778418153.3696954 [knowledge_core-5] 	- myself sees detected_blueberry_426_448 (lifespan: 4.0sec)[0m
1778418153.3701134 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:33,369] This statement will expire on 10/05/2026, 13:02:37[0m
1778418153.4427006 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:33,441] Materialisation performed by reasoner in 72.5ms[0m
1778418156.0136890 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:36,013] Updating ['default'] with:
1778418156.0139711 [knowledge_core-5] 	- myself sees detected_blueberry_447_454
1778418156.0140831 [knowledge_core-5] 	- detected_blueberry_447_454 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418156.0148349 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:36,013] This statement will expire on 10/05/2026, 13:02:40[0m
1778418156.0875006 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:36,086] Materialisation performed by reasoner in 72.3ms[0m
1778418156.4038241 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:36,403] Removing expired statement <myself sees detected_blueberry_426_446> from <default> (expired on 2026-05-10T13:02:36+00:00)[0m
1778418156.4046092 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:36,403] Removing expired statement <detected_blueberry_426_446 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:36+00:00)[0m
1778418156.6592894 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:36,658] Materialisation performed by reasoner in 255.1ms[0m
1778418156.7116964 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:36,710] Updating ['default'] with:
1778418156.7118766 [knowledge_core-5] 	- detected_blueberry_375_447 rdf:type Blueberry
1778418156.7119405 [knowledge_core-5] 	- myself sees detected_blueberry_375_447 (lifespan: 4.0sec)[0m
1778418156.7123294 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:36,711] This statement will expire on 10/05/2026, 13:02:40[0m
1778418156.7881916 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:36,787] Materialisation performed by reasoner in 75.6ms[0m
1778418157.5022113 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:37,501] Updating ['default'] with:
1778418157.5023966 [knowledge_core-5] 	- anonymous_person_hbcba rdf:type Human[0m
1778418157.5812275 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:37,580] Materialisation performed by reasoner in 78.7ms[0m
1778418157.6831934 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:37,682] Removing expired statement <detected_blueberry_426_448 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:37+00:00)[0m
1778418157.6834202 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:37,682] Removing expired statement <myself sees detected_blueberry_426_448> from <default> (expired on 2026-05-10T13:02:37+00:00)[0m
1778418157.7642417 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:37,763] Materialisation performed by reasoner in 80.5ms[0m
1778418157.7742977 [hri_face_detect_yunet-11] [33m[WARN] [1778418157.773798271] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.7sec[0m
1778418157.8196309 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:37,818] Updating ['default'] with:
1778418157.8198938 [knowledge_core-5] 	- myself sees anonymous_person_hbcba[0m
1778418157.9034381 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:37,902] Materialisation performed by reasoner in 83.8ms[0m
1778418158.0492687 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:38,048] Updating ['default'] with:
1778418158.0494885 [knowledge_core-5] 	- anonymous_person_hbcba sees myself[0m
1778418158.1314957 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:38,130] Materialisation performed by reasoner in 81.7ms[0m
1778418161.1444907 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:41,144] Removing expired statement <myself sees detected_blueberry_447_454> from <default> (expired on 2026-05-10T13:02:40+00:00)[0m
1778418161.1449561 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:41,144] Removing expired statement <detected_blueberry_447_454 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:40+00:00)[0m
1778418161.1450689 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:41,144] Removing expired statement <detected_blueberry_375_447 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:40+00:00)[0m
1778418161.1451592 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:41,144] Removing expired statement <myself sees detected_blueberry_375_447> from <default> (expired on 2026-05-10T13:02:40+00:00)[0m
1778418161.2129569 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:41,212] Materialisation performed by reasoner in 67.8ms[0m
1778418163.8589635 [hri_face_detect_yunet-11] [33m[WARN] [1778418163.858421710] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.1sec[0m
1778418164.9747939 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:44,974] Deleting from ['default']:
1778418164.9750049 [knowledge_core-5] 	- anonymous_person_hbcba rdf:type Human[0m
1778418165.0443220 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:45,043] Materialisation performed by reasoner in 69.2ms[0m
1778418165.1871564 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:45,186] Deleting from ['default']:
1778418165.1874115 [knowledge_core-5] 	- myself sees anonymous_person_hbcba[0m
1778418165.2519395 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:45,251] Materialisation performed by reasoner in 64.8ms[0m
1778418168.8421922 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:48,841] Updating ['default'] with:
1778418168.8424189 [knowledge_core-5] 	- myself sees detected_blueberry_383_444
1778418168.8424945 [knowledge_core-5] 	- detected_blueberry_383_444 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418168.8429494 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:48,841] This statement will expire on 10/05/2026, 13:02:52[0m
1778418168.9100165 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:48,909] Materialisation performed by reasoner in 67.3ms[0m
1778418170.1461115 [hri_face_detect_yunet-11] [33m[WARN] [1778418170.145344500] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.3sec[0m
1778418170.4835463 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:50,483] Updating ['default'] with:
1778418170.4837241 [knowledge_core-5] 	- anonymous_person_fifcd rdf:type Human[0m
1778418170.7677178 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:50,767] Materialisation performed by reasoner in 284.0ms[0m
1778418170.9253292 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:50,924] Updating ['default'] with:
1778418170.9255159 [knowledge_core-5] 	- myself sees anonymous_person_fifcd[0m
1778418170.9913509 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:50,990] Materialisation performed by reasoner in 65.5ms[0m
1778418171.0679715 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,066] Updating ['default'] with:
1778418171.0682192 [knowledge_core-5] 	- myself sees detected_blueberry_423_363
1778418171.0682895 [knowledge_core-5] 	- detected_blueberry_423_363 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418171.0688088 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,067] This statement will expire on 10/05/2026, 13:02:55[0m
1778418171.1622972 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,161] Materialisation performed by reasoner in 94.1ms[0m
1778418171.3069696 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,306] Updating ['default'] with:
1778418171.3071675 [knowledge_core-5] 	- anonymous_person_fifcd sees myself[0m
1778418171.3863528 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,385] Materialisation performed by reasoner in 78.9ms[0m
1778418171.4582736 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,457] Deleting from ['default']:
1778418171.4584513 [knowledge_core-5] 	- anonymous_person_fifcd rdf:type Human[0m
1778418171.5249169 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,524] Materialisation performed by reasoner in 66.4ms[0m
1778418171.5998175 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,599] Updating ['default'] with:
1778418171.6000087 [knowledge_core-5] 	- myself sees detected_blueberry_464_364
1778418171.6000788 [knowledge_core-5] 	- detected_blueberry_464_364 rdf:type Blueberry (lifespan: 4.0sec)[0m
1778418171.6005328 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,599] This statement will expire on 10/05/2026, 13:02:55[0m
1778418171.6733305 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,672] Materialisation performed by reasoner in 73.0ms[0m
1778418171.8163450 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:51,815] Deleting from ['default']:
1778418171.8165863 [knowledge_core-5] 	- myself sees anonymous_person_fifcd[0m
1778418172.1008000 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:52,100] Materialisation performed by reasoner in 284.3ms[0m
1778418172.1386466 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:52,138] Removing expired statement <myself sees detected_blueberry_383_444> from <default> (expired on 2026-05-10T13:02:52+00:00)[0m
1778418172.1393211 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:52,138] Removing expired statement <detected_blueberry_383_444 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:52+00:00)[0m
1778418172.2046139 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:52,203] Materialisation performed by reasoner in 65.3ms[0m
1778418172.2597954 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:52,259] Updating ['default'] with:
1778418172.2599883 [knowledge_core-5] 	- detected_blueberry_605_230 rdf:type Blueberry
1778418172.2600493 [knowledge_core-5] 	- myself sees detected_blueberry_605_230 (lifespan: 4.0sec)[0m
1778418172.2605846 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:52,259] This statement will expire on 10/05/2026, 13:02:56[0m
1778418172.3349690 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:52,334] Materialisation performed by reasoner in 74.6ms[0m
1778418175.4216857 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:55,421] Removing expired statement <myself sees detected_blueberry_423_363> from <default> (expired on 2026-05-10T13:02:55+00:00)[0m
1778418175.4221897 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:55,421] Removing expired statement <detected_blueberry_423_363 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:55+00:00)[0m
1778418175.4222672 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:55,421] Removing expired statement <myself sees detected_blueberry_464_364> from <default> (expired on 2026-05-10T13:02:55+00:00)[0m
1778418175.4223311 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:55,421] Removing expired statement <detected_blueberry_464_364 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:55+00:00)[0m
1778418175.4797411 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:55,479] Materialisation performed by reasoner in 57.7ms[0m
1778418176.2667246 [hri_face_detect_yunet-11] [33m[WARN] [1778418176.266383033] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.1sec[0m
1778418176.5503068 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:56,549] Removing expired statement <detected_blueberry_605_230 rdf:type Blueberry> from <default> (expired on 2026-05-10T13:02:56+00:00)[0m
1778418176.5508626 [knowledge_core-5] [33m[WARNING] [2026-05-10 13:02:56,549] Removing expired statement <myself sees detected_blueberry_605_230> from <default> (expired on 2026-05-10T13:02:56+00:00)[0m
1778418176.6222618 [knowledge_core-5] [37m[INFO] [2026-05-10 13:02:56,621] Materialisation performed by reasoner in 71.6ms[0m
1778418181.8716130 [hri_face_detect_yunet-11] [33m[WARN] [1778418181.871246393] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418187.6394138 [hri_face_detect_yunet-11] [33m[WARN] [1778418187.638814864] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.8sec[0m
1778418193.3796730 [hri_face_detect_yunet-11] [33m[WARN] [1778418193.379315070] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418198.7456541 [hri_face_detect_yunet-11] [33m[WARN] [1778418198.745284690] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.4sec[0m
1778418204.1532669 [hri_face_detect_yunet-11] [33m[WARN] [1778418204.152763772] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.4sec[0m
1778418210.1598558 [hri_face_detect_yunet-11] [33m[WARN] [1778418210.159412657] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.0sec[0m
1778418215.4625657 [hri_face_detect_yunet-11] [33m[WARN] [1778418215.462102978] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778418220.9668689 [hri_face_detect_yunet-11] [33m[WARN] [1778418220.966437555] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.5sec[0m
1778418226.5706556 [hri_face_detect_yunet-11] [33m[WARN] [1778418226.570244357] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418232.2746255 [hri_face_detect_yunet-11] [33m[WARN] [1778418232.274276524] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418237.8802288 [hri_face_detect_yunet-11] [33m[WARN] [1778418237.879649664] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418243.4463475 [hri_face_detect_yunet-11] [33m[WARN] [1778418243.445969625] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418249.1531649 [hri_face_detect_yunet-11] [33m[WARN] [1778418249.152710440] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418254.9454312 [hri_face_detect_yunet-11] [33m[WARN] [1778418254.945082234] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.8sec[0m
1778418260.6405115 [hri_face_detect_yunet-11] [33m[WARN] [1778418260.640060390] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418266.7425666 [hri_face_detect_yunet-11] [33m[WARN] [1778418266.742073378] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 6.1sec[0m
1778418272.1452551 [hri_face_detect_yunet-11] [33m[WARN] [1778418272.144613421] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.4sec[0m
1778418277.5454562 [hri_face_detect_yunet-11] [33m[WARN] [1778418277.544758879] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.4sec[0m
1778418282.8474708 [hri_face_detect_yunet-11] [33m[WARN] [1778418282.847090257] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778418288.7515402 [hri_face_detect_yunet-11] [33m[WARN] [1778418288.750939826] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.9sec[0m
1778418294.3520353 [hri_face_detect_yunet-11] [33m[WARN] [1778418294.351630805] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418300.0435193 [hri_face_detect_yunet-11] [33m[WARN] [1778418300.043086055] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418305.7602432 [hri_face_detect_yunet-11] [33m[WARN] [1778418305.759838119] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418311.3649876 [hri_face_detect_yunet-11] [33m[WARN] [1778418311.364505248] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.6sec[0m
1778418317.1455743 [hri_face_detect_yunet-11] [33m[WARN] [1778418317.144828051] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.8sec[0m
1778418322.5446420 [hri_face_detect_yunet-11] [33m[WARN] [1778418322.544049435] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.4sec[0m
1778418327.5803065 [hri_face_detect_yunet-11] [33m[WARN] [1778418327.579854859] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.0sec[0m
1778418332.8499997 [hri_face_detect_yunet-11] [33m[WARN] [1778418332.849656071] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778418338.0513103 [hri_face_detect_yunet-11] [33m[WARN] [1778418338.050752289] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.2sec[0m
1778418343.5489767 [hri_face_detect_yunet-11] [33m[WARN] [1778418343.548517782] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.5sec[0m
1778418348.8468738 [hri_face_detect_yunet-11] [33m[WARN] [1778418348.846463622] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778418354.2639635 [hri_face_detect_yunet-11] [33m[WARN] [1778418354.263548756] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.4sec[0m
1778418359.9431658 [hri_face_detect_yunet-11] [33m[WARN] [1778418359.942617587] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418365.6424322 [hri_face_detect_yunet-11] [33m[WARN] [1778418365.642092219] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.7sec[0m
1778418370.9409144 [hri_face_detect_yunet-11] [33m[WARN] [1778418370.939927521] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778418376.2723105 [hri_face_detect_yunet-11] [33m[WARN] [1778418376.271892793] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.3sec[0m
1778418381.7716618 [hri_face_detect_yunet-11] [33m[WARN] [1778418381.771055923] [hri_face_detect_yunet]: Face_detect's processing too slow. Skipped 100 new incoming image over the last 5.5sec[0m
\`\`\`

## Suggested Focus Areas for GPT-5.5 Revision

1. Separate scan execution success from target detection success (add explicit target_detected + detected_ids fields).
2. For person targets, prefer tracked-person/KB evidence over object summaries when composing completion text.
3. Keep planner prompt high-level, but harden deterministic post-processing for runtime truth grounding and freshness windows.
4. Add action availability checks/graceful fallback for look_at_target unavailability.
