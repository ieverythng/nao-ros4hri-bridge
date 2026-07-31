# Failure-position trace excerpts

These excerpts are derived from the raw per-case `log_excerpt` fields. The raw JSON and stack logs remain authoritative.

## `fake_deep_missing_object_recovery`

- Source: `F13_semantic_audit/raw/nao_fake_all_success_20260713_heavy_changes.json`
- Profile: `all_success`
- Stage: **beginning** (high confidence)
- Site: `find_object`
- Status: `pass`
- Basis: The trace shows the first executable find_object step failing, followed by a recovery/replan path.
- Reasons: matched expected trajectory
- Trace: `plan_1783978877653 [plan_accepted step_started:find_object step_failed:find_object plan_accepted step_started:find_object step_failed:find_object] | plan_1783978879895 [plan_accepted step_started:find_object step_failed:find_object plan_accepted step_started:find_object step_failed:find_object]`

## `fake_deep_gold_apple_multiturn`

- Source: `F13_semantic_audit/raw/nao_fake_all_success_20260713_heavy_changes.json`
- Profile: `all_success`
- Stage: **beginning** (high confidence)
- Site: `planner_admission_or_target_selection`
- Status: `fail`
- Basis: The planner publishes ask_clarification before executable plan steps, despite complete grounded fixture context.
- Reasons: expected planner request and execution feedback
- Trace: `no plan events captured`

## `fake_deep_grouped_work_table_delivery`

- Source: `F13_semantic_audit/raw/nao_fake_delivery_blocked_20260713_heavy_changes.json`
- Profile: `delivery_blocked`
- Stage: **middle** (high confidence)
- Site: `bring_object`
- Status: `degraded`
- Basis: The trace shows find/scan preparation before bring_object fails and replanning is observed.
- Reasons: recovery closure was not spoken after terminal evidence
- Trace: `plan_1783979425805 [plan_accepted step_started:find_object step_succeeded:find_object step_started:scan step_succeeded:scan step_started:find_object step_succeeded:find_object step_started:bring_object step_failed:bring_object plan_accepted step_started:find_object step_succeeded:find_object step_started:scan step_succeeded:scan step_started:find_object step_succeeded:find_object step_started:bring_object step_failed:bring_object] | plan_1783979441313 [plan_accepted step_started:bring_object step_failed:bring_object plan_accepted step_started:bring_object step_failed:bring_object]`

## `fake_deep_iiia_kitchen_delivery`

- Source: `F13_semantic_audit/raw/nao_fake_delivery_blocked_20260713_heavy_changes.json`
- Profile: `delivery_blocked`
- Stage: **unknown** (low confidence)
- Site: `bring_object`
- Status: `degraded`
- Basis: The delivery-blocked profile is configured, but the retained excerpt does not isolate a failed bring_object step for this case.
- Reasons: recovery closure was not spoken after terminal evidence
- Trace: `plan_1783979530706 [plan_accepted step_started:navigate_to step_succeeded:navigate_to step_started:scan step_succeeded:scan step_started:find_object step_succeeded:find_object step_started:pick_object step_succeeded:pick_object step_started:navigate_to step_succeeded:navigate_to step_started:place_object step_succeeded:place_object step_started:report_result step_succeeded:report_result plan_completed plan_accepted step_started:navigate_to step_succeeded:navigate_to step_started:scan step_succeeded:scan step_started:find_object step_succeeded:find_object step_started:pick_object step_succeeded:pick_object step_started:navigate_to step_succeeded:navigate_to step_started:place_object step_succeeded:place_object step_started:report_result step_succeeded:report_result plan_completed]`

## `fake_deep_grouped_work_table_delivery`

- Source: `F13_semantic_audit/raw/nao_fake_every_other_20260713_heavy_changes.json`
- Profile: `every_other`
- Stage: **middle_to_end** (medium confidence)
- Site: `mixed_fake_skill_policy`
- Status: `fail`
- Basis: The profile applies stochastic or alternating failure across the composite request; the artifact does not isolate one deterministic failed step.
- Reasons: asked for clarification despite complete fixture context | expected planner request and execution feedback | recovery closure was not spoken after terminal evidence
- Trace: `no plan events captured`

## `fake_deep_ordered_walk_report`

- Source: `F13_semantic_audit/raw/nao_fake_fail_once_navigation_20260713_heavy_changes.json`
- Profile: `fail_once_navigation`
- Stage: **beginning** (medium confidence)
- Site: `navigate_to`
- Status: `degraded`
- Basis: The targeted ordered-walk objective begins with navigation; the case reports an injected failure and missing recovery closure, but the retained excerpt does not expose a unique failed-step index.
- Reasons: recovery closure was not spoken after terminal evidence
- Trace: `plan_1783979229694 [plan_accepted step_started:navigate_to step_succeeded:navigate_to step_started:report_result step_succeeded:report_result step_started:navigate_to step_succeeded:navigate_to step_started:report_result step_succeeded:report_result plan_completed plan_accepted step_started:navigate_to step_succeeded:navigate_to step_started:report_result step_succeeded:report_result step_started:navigate_to step_succeeded:navigate_to step_started:report_result step_succeeded:report_result plan_completed]`

## `fake_deep_iiia_kitchen_delivery`

- Source: `F13_semantic_audit/raw/nao_fake_fail_once_navigation_20260713_heavy_changes.json`
- Profile: `fail_once_navigation`
- Stage: **unknown** (low confidence)
- Site: `navigate_to`
- Status: `degraded`
- Basis: The navigation failure profile was configured, but this case's retained excerpt shows only a successful bring_object plan; applicability is not proven.
- Reasons: recovery closure was not spoken after terminal evidence
- Trace: `plan_1783979298515 [plan_accepted step_started:bring_object step_succeeded:bring_object plan_completed plan_accepted step_started:bring_object step_succeeded:bring_object plan_completed]`

## `skill_pick_phone_generic`

- Source: `F13_semantic_audit/raw/nao_fake_fail_once_pick_20260713_heavy_changes.json`
- Profile: `fail_once_pick`
- Stage: **middle** (high confidence)
- Site: `pick_object`
- Status: `pass`
- Basis: The trace shows find_object succeeding before pick_object fails, followed by a successful retry plan.
- Reasons: matched expected trajectory
- Trace: `plan_1783979351305 [plan_accepted step_started:find_object step_succeeded:find_object step_started:pick_object step_failed:pick_object plan_accepted step_started:find_object step_succeeded:find_object step_started:pick_object step_failed:pick_object] | plan_1783979355769 [plan_accepted step_started:scan step_succeeded:scan step_started:find_object step_succeeded:find_object step_started:pick_object step_succeeded:pick_object step_started:report_result step_succeeded:report_result plan_completed plan_accepted step_started:scan step_succeeded:scan step_started:find_object step_succeeded:find_object step_started:pick_object step_succeeded:pick_object step_started:report_result step_succeeded:report_result plan_completed]`

## `fake_deep_grouped_work_table_delivery`

- Source: `F13_semantic_audit/raw/nao_fake_random_seeded_20260713_heavy_changes.json`
- Profile: `random_seeded`
- Stage: **middle_to_end** (medium confidence)
- Site: `mixed_fake_skill_policy`
- Status: `fail`
- Basis: The profile applies stochastic or alternating failure across the composite request; the artifact does not isolate one deterministic failed step.
- Reasons: asked for clarification despite complete fixture context | expected planner request and execution feedback | recovery closure was not spoken after terminal evidence
- Trace: `no plan events captured`

## `fake_deep_grouped_work_table_delivery`

- Source: `F13_semantic_audit/raw/nao_fake_recipient_missing_20260713_heavy_changes.json`
- Profile: `recipient_missing`
- Stage: **end** (high confidence)
- Site: `bring_object_recipient_boundary`
- Status: `fail`
- Basis: The trace reaches navigation and object finding before recipient-bound delivery fails; the case then reports clarification despite complete fixture context.
- Reasons: asked for clarification despite complete fixture context
- Trace: `plan_1783979595798 [plan_accepted step_started:navigate_to step_succeeded:navigate_to step_started:find_object step_succeeded:find_object step_started:bring_object step_failed:bring_object plan_accepted step_started:navigate_to step_succeeded:navigate_to step_started:find_object step_succeeded:find_object step_started:bring_object step_failed:bring_object]`

## `fake_deep_gold_apple_multiturn`

- Source: `F13_semantic_audit/raw/nao_fake_recipient_missing_20260713_heavy_changes.json`
- Profile: `recipient_missing`
- Stage: **end** (medium confidence)
- Site: `post_failure_closure`
- Status: `degraded`
- Basis: The case assessment identifies missing recovery closure after terminal evidence; this is an end-of-plan speech/supervision observation.
- Reasons: recovery closure was not spoken after terminal evidence
- Trace: `plan_1783979685322 [plan_accepted step_started:find_object step_succeeded:find_object step_started:pick_object step_succeeded:pick_object step_started:navigate_to step_succeeded:navigate_to step_started:place_object step_succeeded:place_object step_started:report_result step_succeeded:report_result plan_completed plan_accepted step_started:find_object step_succeeded:find_object step_started:pick_object step_succeeded:pick_object step_started:navigate_to step_succeeded:navigate_to step_started:place_object step_succeeded:place_object step_started:report_result step_succeeded:report_result plan_completed]`

## `kb_mutation_add_red_cup`

- Source: `F13_semantic_audit/raw/nao_main_20260713_heavy_changes_clean.json`
- Profile: `none`
- Stage: **beginning** (high confidence)
- Site: `kb_add_compiler_boundary`
- Status: `pass`
- Basis: The first planned kb_add step is rejected for non-RDF-style statements, then the harness records the expected recovery trajectory.
- Reasons: matched expected trajectory
- Trace: `plan_1783977887396 [plan_accepted step_started:kb_add step_failed:kb_add plan_accepted step_started:kb_add step_failed:kb_add]`

## `fake_deep_gold_apple_multiturn`

- Source: `F14_targeted_hardening/raw/nao_v6_targeted_success.json`
- Profile: `all_success`
- Stage: **beginning** (high confidence)
- Site: `planner_admission_or_target_selection`
- Status: `fail`
- Basis: The planner publishes ask_clarification before executable plan steps, despite complete grounded fixture context.
- Reasons: asked for clarification despite complete fixture context | expected planner request and execution feedback | missing terminal or speech evidence
- Trace: `no plan events captured`
