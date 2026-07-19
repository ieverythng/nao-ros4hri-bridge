from planner_common.target_selection import validate_target_selection


def test_delivery_selection_requires_at_least_one_member() -> None:
    result = validate_target_selection(
        {
            'selection_kind': 'visible_objects',
            'operation': 'deliver',
            'member_ids': [],
            'recipient_id': 'person_1',
        },
        {
            'entities': [
                {'id': 'person_1', 'kind': 'person'},
            ],
        },
    )

    assert result.selection['operation'] == 'deliver'
    assert result.errors == ('target_selection.member_ids must not be empty',)
    assert result.valid is False


def test_delivery_selection_requires_object_members_and_person_recipient() -> None:
    result = validate_target_selection(
        {
            'selection_kind': 'explicit_members',
            'operation': 'deliver',
            'member_ids': ['cup_1', 'person_1'],
            'recipient_id': 'kitchen',
        },
        {
            'entities': [
                {'id': 'cup_1', 'kind': 'object'},
                {'id': 'person_1', 'kind': 'person'},
                {'id': 'kitchen', 'kind': 'location'},
            ],
        },
    )

    assert result.errors == (
        'delivery target_selection.member_ids must be grounded objects: person_1',
        'delivery target_selection.recipient_id must identify a grounded person: kitchen',
    )


def test_location_members_must_belong_to_grounded_source() -> None:
    result = validate_target_selection(
        {
            'selection_kind': 'location_members',
            'operation': 'deliver',
            'source_location_id': 'work_table',
            'member_ids': ['cup_1', 'book_1'],
            'recipient_id': 'person_1',
        },
        {
            'entities': [
                {'id': 'cup_1', 'kind': 'object'},
                {'id': 'book_1', 'kind': 'object'},
                {'id': 'person_1', 'kind': 'person'},
            ],
            'locations': [
                {
                    'id': 'work_table',
                    'contains': [{'id': 'cup_1', 'kind': 'object'}],
                },
            ],
        },
    )

    assert result.errors == (
        'target_selection members are outside source_location_id work_table: book_1',
    )


def test_visit_selection_rejects_unknown_members() -> None:
    result = validate_target_selection(
        {
            'selection_kind': 'explicit_members',
            'operation': 'visit',
            'member_ids': ['cup_1', 'kitchen', 'missing'],
        },
        {
            'entities': [{'id': 'cup_1', 'kind': 'object'}],
            'locations': [{'id': 'kitchen'}],
        },
    )

    assert result.errors == (
        'visit target_selection.member_ids must identify grounded entities or locations: missing',
    )


def test_delivery_recipient_must_be_distinct_from_members() -> None:
    result = validate_target_selection(
        {
            'selection_kind': 'explicit_members',
            'operation': 'deliver',
            'member_ids': ['cup_1'],
            'recipient_id': 'cup_1',
        },
        {'entities': [{'id': 'cup_1', 'kind': 'object'}]},
    )

    assert result.errors == (
        'delivery target_selection.recipient_id must identify a grounded person: cup_1',
        'delivery target_selection.recipient_id must differ from member_ids: cup_1',
    )
