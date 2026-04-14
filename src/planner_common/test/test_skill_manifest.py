from planner_common import load_exported_skill_manifests


def test_shared_skill_manifest_loader_reads_yaml_and_json_exports() -> None:
    manifests = load_exported_skill_manifests(
        ['nao_skills', 'interaction_skills', 'nao_say_skill']
    )
    manifests_by_key = {
        (manifest.package, manifest.skill_id): manifest
        for manifest in manifests
    }

    assert ('nao_skills', 'do_head_motion') in manifests_by_key
    assert manifests_by_key[('nao_skills', 'do_head_motion')].input_names == (
        'yaw',
        'pitch',
        'speed',
        'relative',
    )

    assert ('interaction_skills', 'look_at') in manifests_by_key
    assert ('interaction_skills', 'ask_human_for_help') in manifests_by_key
    assert (
        manifests_by_key[('interaction_skills', 'ask_human_for_help')].content_type
        == 'json'
    )

    assert ('nao_say_skill', 'nao_say') in manifests_by_key
