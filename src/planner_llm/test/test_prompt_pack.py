import pytest

from planner_llm.prompt_pack import default_prompt_pack
from planner_llm.prompt_pack import load_prompt_pack


def test_default_planner_prompt_pack_contains_core_fields() -> None:
    pack = default_prompt_pack()

    assert pack.prompt_pack_version == 'planner_llm_prompt_pack_v1'
    assert 'You are a planner for a robot called Pop' in pack.system_prompt
    assert 'step_type_skill' in pack.output_contract
    assert 'step_type_say' in pack.output_contract
    assert 'invalid_examples' in pack.output_contract
    assert 'planner contract errors' in pack.validation_retry['instruction']
    assert pack.validation_retry['previous_model_output_max_chars'] == 4000


def test_default_planner_prompt_pack_limits_routine_progress_speech() -> None:
    pack = default_prompt_pack()

    assert 'Keep emit_progress=false for short plans' in pack.system_prompt
    assert 'Routine internal step transitions' in pack.system_prompt
    assert '"report_result" already covers the completion' in pack.system_prompt


def test_default_planner_prompt_pack_rejects_composite_motion_objects() -> None:
    pack = default_prompt_pack()

    assert 'use one supplied "allowed_motion_objects" value per step' in pack.system_prompt
    assert 'never emit a composite label as "args.object"' in pack.system_prompt


def test_load_prompt_pack_supports_partial_override_merge(tmp_path) -> None:
    prompt_pack_path = tmp_path / 'planner_prompt_pack.yaml'
    prompt_pack_path.write_text(
        '\n'.join(
            [
                'prompt_pack_version: planner_llm_prompt_pack_custom',
                'system_prompt: "custom planner system"',
                'output_contract:',
                '  step_type_say:',
                '    args:',
                '      text: "custom say args"',
                'validation_retry:',
                '  previous_model_output_max_chars: 777',
            ]
        ),
        encoding='utf-8',
    )

    pack = load_prompt_pack(str(prompt_pack_path))

    assert pack.prompt_pack_version == 'planner_llm_prompt_pack_custom'
    assert pack.source_path == str(prompt_pack_path)
    assert pack.output_contract['step_type_say']['args']['text'] == 'custom say args'
    assert pack.output_contract['step_type_skill']['name'] == 'must be one allowed_skill_names entry'
    assert pack.validation_retry['previous_model_output_max_chars'] == 777


def test_load_prompt_pack_invalid_yaml_fails_without_required_system_prompt(tmp_path) -> None:
    prompt_pack_path = tmp_path / 'planner_prompt_pack.yaml'
    prompt_pack_path.write_text('::bad', encoding='utf-8')

    with pytest.raises(ValueError, match='root must be a mapping'):
        load_prompt_pack(str(prompt_pack_path))


def test_load_prompt_pack_missing_file_fails(tmp_path) -> None:
    with pytest.raises(FileNotFoundError):
        load_prompt_pack(str(tmp_path / 'missing.yaml'))
