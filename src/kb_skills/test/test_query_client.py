from kb_skills.intent_labels import KB_QUERY_INTENTS
from kb_skills.intent_labels import KB_QUERY_SCENE_CHANGE
from kb_skills.intent_labels import KB_QUERY_VISIBLE_OBJECTS
from kb_skills.intent_labels import KB_QUERY_VISIBLE_PEOPLE
from kb_skills.mutation_client import KnowledgeCoreMutationClient
from kb_skills.query_client import KnowledgeCoreQueryClient


def test_parse_response_rows_accepts_object_or_list_payloads():
    assert KnowledgeCoreQueryClient.parse_response_rows('{"entity":"book"}') == [
        {"entity": "book"}
    ]
    assert KnowledgeCoreQueryClient.parse_response_rows('[{"entity":"book"}]') == [
        {"entity": "book"}
    ]


def test_parse_response_rows_rejects_invalid_payloads():
    assert KnowledgeCoreQueryClient.parse_response_rows("") == []
    assert KnowledgeCoreQueryClient.parse_response_rows("{not-json}") == []
    assert KnowledgeCoreQueryClient.parse_response_rows('"text"') == []


def test_dedupe_rows_preserves_first_occurrence_order():
    rows = [
        {"entity": "book", "type": "Book"},
        {"entity": "person", "type": "Human"},
        {"type": "Book", "entity": "book"},
    ]

    assert KnowledgeCoreQueryClient.dedupe_rows(rows) == [
        {"entity": "book", "type": "Book"},
        {"entity": "person", "type": "Human"},
    ]


def test_kb_query_intents_are_shared_constants():
    assert KB_QUERY_INTENTS == (
        KB_QUERY_VISIBLE_PEOPLE,
        KB_QUERY_VISIBLE_OBJECTS,
        KB_QUERY_SCENE_CHANGE,
    )


def test_mutation_client_methods_are_explicitly_not_implemented():
    client = KnowledgeCoreMutationClient()

    for method_name in ("add_fact", "revise_fact", "remove_fact"):
        method = getattr(client, method_name)
        try:
            method("test")
        except NotImplementedError as err:
            assert "Phase 2" in str(err)
        else:  # pragma: no cover - defensive assertion
            raise AssertionError(f"{method_name} should raise NotImplementedError")
