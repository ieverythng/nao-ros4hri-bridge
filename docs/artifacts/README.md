# Documentation Artifacts

Historical and reference material lives here. These files are not day-to-day
sources of truth.

## Structure

- `plan_archive/` — superseded/absorbed plan files (`.md` + `.html`).
- `handoffs/` — dated implementation handoffs moved out of top-level `docs/`.
- root of `artifacts/` — ad-hoc research, inventories, and one-off technical notes.

## Recent runtime evidence

- `runtime_review_2026-07-10_sv_seams.md` — live `nao_ros2` review of
  supervisor-critical seams: KB mutation, grouped location delivery,
  report_result wording, fake-deep recovery, and container source fingerprinting.
- `runtime_review_2026-07-11_qwen_personal_stack.md` — live Qwen personal-stack
  ablation covering main, environment, architecture, and deep fake/replan
  profiles.

## Active docs (authoritative)

- `../contracts.md`
- `../current_workflow.md`
- `../launch_profiles.md`
- `../plans/nao_ros4hri_masterplan.md`
- `../plans/ISSUE_TRACKER_FULL_SUITE.html`
- `../plans/ISSUE_TRACKER_FAKE_SUITE.html`
- `../plans/FINAL_RUNTIME_REVIEW.html`

When a historical artifact still contains required current behavior, copy the
relevant part into an active doc and keep the artifact as provenance.
