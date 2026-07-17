# Runtime Multi-Person Grounding Audit, 2026-07-16

## Target contract

The planner-facing grounded context must describe the current tracked people,
not the accumulated history of detector-generated handles in KnowledgeCore.
When authoritative `/humans/persons/tracked` state is available, inactive
`anonymous_person_*`, `sim_person_*`, and `person_*` handles must be removed
from both scene-derived and KnowledgeCore-derived projections. Named fixture
people and active generated handles must remain. When tracker state has not
arrived, the existing conservative behavior is preserved and no person is
silently removed.

The chatbot remains the owner of the user-facing answer. This change does not
add a response fallback, alter prompt policy, or make the planner infer a
person count independently.

## Baseline evidence

The v19 canonical runtime reproduced the suspected accumulation path. The
radar log generated eleven different anonymous handles in a short interval,
including repeated `New person ... detected` and `Person ... gone. Removing it.`
events. A contemporaneous `rdf:type Human` query still returned seven
anonymous handles. The first source-level filter removed only inactive
`anonymous_person_*` rows, leaving the `sim_person_*` class of generated
handles unaddressed.

The v20 clean overlay proved the initial tracker subscription and source
parity, but the two-turn live holdout exposed the remaining distinction:
KnowledgeCore contained a changing set of generated people while the tracker
published a current set. The second grounded context included
`sim_person_hyuse` and `anonymous_person_hgdbj` while the detector was also
emitting repeated slow-processing warnings.

## Approach registry

| Approach | Status | Reason |
| --- | --- | --- |
| Change detector ID generation or upstream person matching | Deferred | The upstream HRI packages are outside this repository boundary, and deterministic IDs do not establish identity across visibility gaps. |
| Cap or merge people heuristically in the chatbot | Rejected | It would fabricate a count and would damage real multi-person scenes. |
| Filter stale generated handles at the compact grounding projection | Accepted | It uses the authoritative tracker topic, preserves ownership boundaries, and leaves KB transport and detector behavior unchanged. |
| Add a response fallback for people questions | Rejected | The failure is stale evidence projection, not an inability to produce language. |

## Discriminating probes and results

### Source and unit gates

- `src/planner_common/test`: 70 passed.
- Focused chatbot grounding and handoff tests: 13 passed.
- Runtime questionnaire harness tests: 57 passed.
- Expanded chatbot domain holdout: 247 passed.
- `git diff --check`, Python compilation, and
  `scripts/ros4hri_change_audit.py --mode working`: passed.

### Clean runtime v21

Image: `iiia:nao-multiperson-20260716-v21-tracker-authoritative`

Image ID: `sha256:9354b56c86b60db2a261e024c13f70b1448352926644f86556ad93a27c376668`

Canonical overlay verification: `iiia:nao`, `iiia:naofrozen`, and the versioned
v21 tag all resolve to this same image ID. The prior canonical tags were not
removed as names; they now point to the accepted v21 overlay as requested.

The first v21 launch hit a known startup race in the unchanged
`dialogue_manager` action-client path (`ActionClient` had no `_lock`). A clean
full-container relaunch of the same image succeeded. The scored retry had all
required lifecycle nodes active, KnowledgeCore ready, zero LLM preflight
failures, zero errors, zero warnings, and zero fallback events before semantic
testing.

### Repeated current-scene holdout

The `grounding_people` set ran two speech turns in one dialogue group:

1. `What people can you see right now?`
2. `Please check again. What people can you see right now?`

On v21 both logged grounded contexts had `people: 0` and an empty entity list.
Both responses were produced by the chatbot and remained consistent with the
empty current projection. No report-result, planner, or chatbot fallback was
used. One repeated turn used the existing `route_repair` path to normalize the
knowledge-query intent. That did not alter grounded evidence.

### Adversarial stale-KB holdout

The `grounding_people_stale_kb_rows` case injected stale facts for
`anonymous_person_stale` and `sim_person_stale` before asking the same current
scene question. The raw KB snapshot contained 1,901 characters, while the
logged compact `GROUNDED_CONTEXT` contained zero entities and zero people.
The subsequent repeated turns also contained zero people. This demonstrates
that stale rows are removed at the chatbot handoff projection even when
KnowledgeCore has not yet retracted them.

## Adversarial audit

- People and objects remain separate in the compact contract.
- Active generated tracker IDs are retained by the new filter.
- Named fixture people remain available for fake-suite and planner tests.
- The no-tracker-state path still retains generated people, avoiding an
  accidental fail-closed projection during startup.
- Location grouping is recomputed after inactive generated people are removed,
  so stale people cannot remain as location members.
- The fix does not modify detector thresholds, tracker matching, prompt text,
  route policy, planner admission, or KnowledgeCore mutation semantics.
- Repeated detector slow-processing warnings remain visible. They are a
  perception-performance issue, not evidence that stale people leaked into
  the chatbot projection in this run.
- The separate `scene_targets` execution lineage was not silently rewritten by
  this audit. A targeted multi-person execution case remains necessary before
  claiming that planner target selection is fully covered.

The capability-extreme audit from commit `2765cb8eaf03b166da3801d466a4315669f77473`
was folded into the active questionnaire as a seeded seven-case manifest. Its
generated runtime reports were not copied or presented as v21 evidence because
that suite has not been run against this image.

## Decision

Accept the v21 source change as a bounded fix for the stale generated-person
grounding seam. The fix is source-tested and live-tested against both ordinary
repeated queries and injected stale KB rows. It is suitable for canonical image
tagging for this seam, subject to preserving the existing named image tags.

## Residual risk and next probe

The HRI detector still churns anonymous identities when image processing falls
behind. The filter prevents that churn from polluting chatbot grounding, but it
does not improve upstream identity continuity or reduce detector latency.
The next runtime pass should run the full and deep-fake suites, then add a
multi-person execution case that checks `scene_targets`, target selection,
execution feedback, and final report wording against a changing tracker state.

Runtime evidence files:

- `/tmp/multiperson_v19_snapshot.json`
- `/tmp/multiperson_v20_preflight.json`
- `/tmp/multiperson_v20_grounding_people.json`
- `/tmp/multiperson_v21_ready_retry.json`
- `/tmp/multiperson_v21_grounding_people.json`
- `/tmp/multiperson_v21_grounding_people_stale.json`
