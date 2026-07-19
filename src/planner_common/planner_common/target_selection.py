"""Normalize and validate authoritative grounded target selections."""

from __future__ import annotations

from dataclasses import dataclass


_SELECTION_KINDS = frozenset({'explicit_members', 'location_members', 'visible_objects'})
_OPERATIONS = frozenset({'deliver', 'visit'})
_ORDERINGS = frozenset({'none', 'sequential'})
_REPORT_POLICIES = frozenset({'none', 'per_target', 'final'})


@dataclass(frozen=True)
class TargetSelectionValidation:
    """Normalized target selection and deterministic admission errors."""

    selection: dict
    errors: tuple[str, ...]

    @property
    def valid(self) -> bool:
        return bool(self.selection) and not self.errors


def normalize_target_selection(value) -> dict:
    """Normalize the bounded authoritative target-selection contract."""
    data = value if isinstance(value, dict) else {}
    selection_kind = _choice(data.get('selection_kind'), _SELECTION_KINDS)
    operation = _choice(data.get('operation'), _OPERATIONS)
    if not selection_kind or not operation:
        return {}
    return {
        'selection_kind': selection_kind,
        'operation': operation,
        'source_location_id': str(data.get('source_location_id', '')).strip(),
        'member_ids': list(dict.fromkeys(_strings(data.get('member_ids', [])))),
        'recipient_id': str(data.get('recipient_id', '')).strip(),
        'ordering': _choice(data.get('ordering'), _ORDERINGS, 'none'),
        'report_policy': _choice(
            data.get('report_policy'),
            _REPORT_POLICIES,
            'none',
        ),
    }


def validate_target_selection(
    value,
    grounded_context: dict,
    *,
    expected_operation: str = '',
) -> TargetSelectionValidation:
    """Validate one normalized selection against current grounded context."""
    selection = normalize_target_selection(value)
    if not selection:
        return TargetSelectionValidation(
            selection={},
            errors=('target_selection is required for grounded execution',),
        )
    errors = []
    expected = _choice(expected_operation, _OPERATIONS)
    if expected and selection['operation'] != expected:
        errors.append(
            'target_selection.operation must be %s, got %s'
            % (expected, selection['operation'])
        )
    if not selection['member_ids']:
        errors.append('target_selection.member_ids must not be empty')
    entities = _records_by_id(grounded_context.get('entities', []))
    locations = _records_by_id(grounded_context.get('locations', []))
    if selection['operation'] == 'deliver':
        invalid_members = [
            member_id
            for member_id in selection['member_ids']
            if str(entities.get(member_id, {}).get('kind', '')).strip().lower()
            != 'object'
        ]
        if invalid_members:
            errors.append(
                'delivery target_selection.member_ids must be grounded objects: %s'
                % ', '.join(invalid_members)
            )
        recipient_id = selection['recipient_id']
        if str(entities.get(recipient_id, {}).get('kind', '')).strip().lower() != 'person':
            errors.append(
                'delivery target_selection.recipient_id must identify a grounded person: %s'
                % (recipient_id or '<missing>')
            )
        if recipient_id in selection['member_ids']:
            errors.append(
                'delivery target_selection.recipient_id must differ from member_ids: %s'
                % recipient_id
            )
    if selection['operation'] == 'visit':
        unknown_members = [
            member_id
            for member_id in selection['member_ids']
            if member_id not in entities and member_id not in locations
        ]
        if unknown_members:
            errors.append(
                'visit target_selection.member_ids must identify grounded entities or locations: %s'
                % ', '.join(unknown_members)
            )
    if selection['selection_kind'] == 'location_members':
        source_id = selection['source_location_id']
        source = locations.get(source_id, {})
        if not source:
            errors.append(
                'target_selection.source_location_id is not grounded: %s'
                % (source_id or '<missing>')
            )
        else:
            source_members = {
                str(member.get('id', '')).strip()
                for member in source.get('contains', [])
                if isinstance(member, dict) and str(member.get('id', '')).strip()
            }
            outside = [
                member_id
                for member_id in selection['member_ids']
                if member_id not in source_members
            ]
            if outside:
                errors.append(
                    'target_selection members are outside source_location_id %s: %s'
                    % (source_id, ', '.join(outside))
                )
    return TargetSelectionValidation(selection=selection, errors=tuple(errors))


def _choice(value, allowed: frozenset[str], fallback: str = '') -> str:
    clean = str(value or '').strip().lower()
    return clean if clean in allowed else fallback


def _strings(value) -> list[str]:
    if isinstance(value, str):
        return [value.strip()] if value.strip() else []
    if not isinstance(value, (list, tuple, set)):
        return []
    return [clean for clean in (str(item).strip() for item in value) if clean]


def _records_by_id(value) -> dict[str, dict]:
    if not isinstance(value, list):
        return {}
    return {
        str(record.get('id', '')).strip(): record
        for record in value
        if isinstance(record, dict) and str(record.get('id', '')).strip()
    }
