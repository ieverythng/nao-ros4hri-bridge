from pathlib import Path

from nao_chatbot.prompt_pack import default_prompt_pack
from nao_chatbot.prompt_pack import load_prompt_pack


def test_load_prompt_pack_from_yaml(tmp_path: Path) -> None:
    source = tmp_path / "pack.yaml"
    source.write_text(
        """
system_prompt: "hello {robot_name}"
response_prompt_addendum: "say short"
intent_prompt_addendum: "extract intent"
environment_description: "lab"
response_schema:
  type: object
  properties:
    verbal_ack:
      type: string
intent_schema:
  type: object
  properties:
    user_intent:
      type: object
""".strip(),
        encoding="utf-8",
    )

    pack = load_prompt_pack(str(source))

    assert pack.system_prompt == "hello {robot_name}"
    assert pack.environment_description == "lab"
    assert pack.response_schema["type"] == "object"


def test_load_prompt_pack_fallback_on_invalid_yaml(tmp_path: Path) -> None:
    source = tmp_path / "broken.yaml"
    source.write_text("response_schema: [", encoding="utf-8")

    pack = load_prompt_pack(str(source))
    defaults = default_prompt_pack()

    assert pack.system_prompt == defaults.system_prompt
    assert pack.response_schema == defaults.response_schema
