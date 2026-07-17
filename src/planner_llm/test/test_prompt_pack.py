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
    assert 'context_ref' not in str(pack.output_contract)
    assert pack.validation_retry['previous_model_output_max_chars'] == 4000
    assert 'grounded_context.entities' in pack.system_prompt
    assert 'both currently visible people and objects' in pack.system_prompt
    assert 'explicit natural-language request to change the KB' in pack.system_prompt
    assert 'Never infer metric distance from' in pack.system_prompt
    assert 'communication_policy.emit_progress=true' in pack.system_prompt


def test_default_planner_prompt_pack_limits_routine_progress_speech() -> None:
    pack = default_prompt_pack()

    assert 'Keep emit_progress=false for short plans' in pack.system_prompt
    assert 'Routine internal step transitions' in pack.system_prompt
    assert '"report_result" already covers the completion' in pack.system_prompt
    assert 'Use emit_progress=true only for meaningful user-visible milestones' in pack.system_prompt


def test_default_planner_prompt_pack_rejects_composite_motion_objects() -> None:
    pack = default_prompt_pack()

    assert 'use one supplied "allowed_motion_objects" value per step' in pack.system_prompt
    assert 'never emit a composite label as "args.object"' in pack.system_prompt


def test_default_planner_prompt_pack_requires_canonical_grounded_targets() -> None:
    pack = default_prompt_pack()

    assert 'bind that reference to the matching "grounded_context.entities[].id"' in pack.system_prompt
    assert 'do not pass user-facing labels or names when a grounded id exists' in pack.system_prompt
    assert 'Pass canonical entity ids in skill args' in pack.system_prompt


def test_default_planner_prompt_pack_handles_every_object_with_valid_json() -> None:
    pack = default_prompt_pack()

    assert 'For quantified requests over visible objects' in pack.system_prompt
    assert 'return a valid "clarify" or "fail" JSON object' in pack.system_prompt


def test_default_planner_prompt_pack_translates_explicit_natural_language_kb_changes() -> None:
    prompt = default_prompt_pack().system_prompt

    assert 'do not require the user to provide RDF syntax' in prompt
    assert 'namespace-qualified predicates' in prompt
    assert 'unstated properties or mutate from a question' in prompt
    assert 'subject first, predicate second, and object' in prompt
    assert 'scan or find steps unless the requested fact explicitly depends' in prompt


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
