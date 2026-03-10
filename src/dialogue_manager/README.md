# dialogue_manager

Current standalone dialogue bridge for the NAO + ROS4HRI stack.

## Responsibility

`dialogue_manager_node` is the user-turn boundary node between ROS4HRI speech input and the rest of the chatbot pipeline.

Input:

- `hri_msgs/LiveSpeech` on `/humans/voices/anonymous_speaker/speech` by default
- `/chatbot/assistant_text`
- `/chatbot/intent`

Output:

- `/chatbot/user_text` as JSON payloads carrying `text` and `turn_id`
- `/chatbot/dialogue_state`
- `/speech` mirror for robot speech / ASR suppression path
- `/skill/say` dispatch when the say skill server is enabled

## Runtime Behavior

- consumes final ASR text by default
- optionally accepts incremental speech via `accept_incremental_speech`
- buffers and merges consecutive ASR finals for `user_turn_holdoff_sec`
- blocks overlapping user turns while the assistant is still processing/speaking when `ignore_user_speech_while_busy:=true`
- de-duplicates repeated user and assistant text inside short windows

## Launch

```bash
ros2 launch dialogue_manager dialogue_manager.launch.py
```

That launch now starts the same `dialogue_manager_node` executable used by the full NAO stack.

## Key Parameters

- `input_speech_topic`
- `user_text_topic`
- `assistant_text_topic`
- `intent_topic`
- `dialogue_state_topic`
- `naoqi_speech_topic`
- `use_say_skill`
- `say_skill_action`
- `say_skill_language`
- `say_skill_volume`
- `say_skill_dispatch_wait_sec`
- `also_publish_speech_topic`
- `fallback_publish_speech_topic`
- `dedupe_window_sec`
- `accept_incremental_speech`
- `ignore_user_speech_while_busy`
- `user_turn_holdoff_sec`
- `user_turn_min_chars`
- `user_turn_min_words`

Default values are installed in `config/00-defaults.yml`.
