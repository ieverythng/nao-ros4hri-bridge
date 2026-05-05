#!/bin/bash
#
# Usage: ./chat.sh [OPTIONS]
#
# Options:
#   --topic TOPIC       Set the conversation topic (overrides TOPIC below)
#   --save [FILE]       Save conversation history on exit (Ctrl+C or empty input)
#                       Defaults to history.json if no file is given
#   --use-example       Append the example transcript to the system prompt
#   --prompt "TEXT"     Use a custom system prompt instead of the default one
#                       Can be combined with --use-example to also append the transcript
#   --return-prompt     Print the resolved system prompt before the first turn
#
# Available topics:
#   dinosaurios, animales, juegos, familia, comida, escuela
#
# Examples:
#   ./chat.sh --topic dinosaurios
#   ./chat.sh --topic animales --save
#   ./chat.sh --topic comida --save session.json
#   ./chat.sh --topic juegos --use-example
#   ./chat.sh --topic comida --prompt prompt.txt
#   ./chat.sh --topic comida --prompt prompt.txt --return-prompt
#   ./chat.sh --topic comida --prompt prompt.txt --use-example --save session.json
#
# Requirements:
#   jq  — sudo apt install jq

HOST="http://10.7.138.215"
CHILD_ID="child_001"
EMOTION="happy"
TOPIC="comida"

# --- flag parsing ---
SAVE=false
SAVE_FILE="history.json"
USE_EXAMPLE=false
PROMPT=""
RETURN_PROMPT=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --save)
      SAVE=true
      if [[ -n "$2" && "$2" != --* ]]; then
        SAVE_FILE="$2"
        shift
      fi
      ;;
    --topic)
      TOPIC="$2"
      shift
      ;;
    --use-example)
      USE_EXAMPLE=true
      ;;
    --prompt)
      if [[ "$2" == *.txt ]]; then
        PROMPT=$(cat "$2")
      else
        PROMPT="$2"
      fi
      shift
      ;;
    --return-prompt)
      RETURN_PROMPT=true
      ;;
  esac
  shift
done

# --- save handler ---
save_history() {
  if [[ "$SAVE" == true ]]; then
    echo "$messages" | jq '.' > "$SAVE_FILE"
    echo ""
    echo "History saved to $SAVE_FILE"
  fi
}

trap 'save_history; exit 0' INT

messages="[]"

while true; do
  body=$(jq -n \
    --arg child_id "$CHILD_ID" \
    --arg emotion "$EMOTION" \
    --arg topic "$TOPIC" \
    --argjson messages "$messages" \
    --argjson use_example "$USE_EXAMPLE" \
    --arg prompt "$PROMPT" \
    --argjson return_prompt "$RETURN_PROMPT" \
    '{child_id: $child_id, messages: $messages, emotion: $emotion, topic: $topic, use_example: $use_example, return_prompt: $return_prompt}
     | if $prompt != "" then . + {prompt: $prompt} else . end')

  response=$(curl -s -X POST "$HOST/generation/generate" \
    -H "Content-Type: application/json" \
    -d "$body")

  assistant_text=$(echo "$response" | jq -r '.sentence')

  if [[ -z "$assistant_text" || "$assistant_text" == "null" ]]; then
    echo "Error or empty response: $response"
    save_history
    exit 1
  fi

  if [[ "$RETURN_PROMPT" == true ]]; then
    resolved_prompt=$(echo "$response" | jq -r '.system_prompt // empty')
    if [[ -n "$resolved_prompt" ]]; then
      echo "--- SYSTEM PROMPT ---"
      echo "$resolved_prompt"
      echo "---------------------"
    fi
    RETURN_PROMPT=false
  fi

  printf 'Emy: %s\n' "$assistant_text"

  messages=$(echo "$messages" | jq \
    --arg content "$assistant_text" \
    '. + [{"role": "assistant", "content": $content}]')

  echo -n "Tu: "
  read -r user_input

  if [[ -z "$user_input" ]]; then
    save_history
    break
  fi

  messages=$(echo "$messages" | jq \
    --arg content "$user_input" \
    '. + [{"role": "user", "content": $content}]')
done
