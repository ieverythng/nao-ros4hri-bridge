# Reporting LLM-Based Embodied-Agent Validation Results

**Date:** 2026-07-17
**Purpose:** Primary-source precedents and actionable recommendations for restructuring Chapters 7 and 8.
**Scope:** Results presentation only. This note does not assess the current runtime, alter the validation protocol, or claim that the pending canonical run has passed.

## Executive recommendation

Chapter 7 should be led by one frozen canonical validation tuple and organized by validation question or test family. The chronological F-series should not remain its organizing device. Reader-facing labels should describe the evidence, such as *canonical full-runtime suite*, *deterministic recovery matrix*, *stateful KB profile*, and *alternate-model ablation*. The F identifiers should remain in the evidence manifest and appendix as provenance keys.

The final results should keep the distinctions that exposed the earlier false positive:

1. plan or route validity;
2. complete task success;
3. semantic goal-condition satisfaction;
4. execution and recovery behavior;
5. post-execution state correctness;
6. dialogue closure and speech multiplicity;
7. operational efficiency;
8. physical embodiment evidence.

A nominal 100% pass should therefore be stated as `N/N cases passed the declared contract`, followed by the denominators for each semantic and operational metric. It should not be presented as unrestricted robot task success.

## Primary-source reporting patterns

| Primary source | Reporting structure used by the authors | Applicable precedent for this thesis |
|---|---|---|
| [SayCan, Ahn et al. (2022)](https://arxiv.org/abs/2204.01691), Section 5 and Table 2 | Separates **plan success** from **execution success**, reports both by instruction family and environment, and attributes errors to the LLM or affordance model. The appendix retains the complete instruction-level results. | Report planning or admission success separately from completed semantic execution. Break down the canonical result by dialogue, KB, simple execution, composite execution, and long-horizon families. Keep complete case records in the appendix or evidence bundle. |
| [Inner Monologue, Huang et al. (2022)](https://arxiv.org/abs/2207.05608), Tables 1–3 and Figure 4 | Reports feedback variants as controlled ablations, separates simulated tabletop, real tabletop, and real mobile-manipulation experiments, injects disturbances to test recovery, states repetition counts, and presents failure causes separately from aggregate success. | Present deterministic fake policies as recovery experiments, not as physical skill evidence. Separate undisturbed success from injected-failure recovery. Include a recovery/failure taxonomy and preserve one representative versioned replan trace. |
| [ALFRED, Shridhar et al. (2020)](https://openaccess.thecvf.com/content_CVPR_2020/html/Shridhar_ALFRED_A_Benchmark_for_Interpreting_Grounded_Instructions_for_Everyday_Tasks_CVPR_2020_paper.html), Section 5 and Table 3 | Reports binary **Task Success** alongside **Goal-Condition Success**, where the latter measures the fraction of required final-state conditions satisfied. It also reports path-weighted forms and subgoal results. | Treat requested, selected, planned, executed, reported, and post-state agreement as explicit goal conditions. A terminal trajectory cannot replace the semantic-condition denominator. A phase funnel is an appropriate secondary view of partial completion. |
| [CALVIN, Mees et al. (2022)](https://arxiv.org/abs/2112.03227), Section III-C and Figure 8 | Uses 10 rollouts per individual task and evaluates 1,000 feasible five-instruction chains by the number of consecutively completed instructions. Environment state determines whether the next instruction is issued. | Show all frozen robustness repetitions and report a chain or phase funnel with exact denominators. Three repetitions provide bounded repeatability evidence, not a population-level estimate. |
| [RePLan, Skreta et al. (2024)](https://arxiv.org/abs/2401.04157), Sections 4.4–4.5, Table 1, and Figures 2–3 | Compares the full replanning system against no-verifier, no-perceiver, and no-replan variants over repeated tasks. It reports action counts with dispersion and provides a representative failed-plan and revised-plan trajectory. | Include one plan-version lineage trace in Chapter 7. Reserve mechanism-rich examples for Chapter 8. Call a comparison an ablation only when it disables or changes one declared component under controlled conditions. |
| [ReAct, Yao et al. (2023)](https://arxiv.org/abs/2210.03629), Section 4 and Table 3 | Breaks success down by task type, reports average and best results across six controlled prompt permutations, and isolates an external-feedback ablation. | Report the frozen robustness repetitions explicitly, preferably per case plus an aggregate stability column. Do not silently retain only the best repetition. Compare ablations only when the changed factor and fixed controls are declared. |
| [RT-2, Brohan et al. (2023)](https://arxiv.org/abs/2307.15818), Section 4 and Appendix Tables 4–6 | Separates seen tasks from controlled generalization categories, reports capability-specific subsets, and gives model-size and training-strategy ablations separately from the main result. | Keep the frozen main questionnaire separate from robustness or generalization cases. Do not merge an alternate model, route mode, or detector profile into the canonical score. |
| [OK-Robot, Chang et al. (2024)](https://arxiv.org/abs/2401.12202), Section III and Figures 5–6 | Reports full pick-and-drop success across real homes, component ablations, environment cleanup conditions, and a module-level failure breakdown. Its appendix lists individual tasks, trial outcomes, and failure categories. | Add an error taxonomy tied to owning seams, for example grounding/selection, plan generation, admission, skill execution, report coverage, KB postcondition, and speech closure. A phase-completion funnel should expose multiplicative losses across modules even when the overall result is green. |
| [BEHAVIOR-1K, Li et al. (2023)](https://proceedings.mlr.press/v205/li23a.html), Section 6 and Tables 2–4 | Reports task success together with distance, simulated time, and disarrangement; ablates simplifying execution assumptions; then evaluates simulation and a physical robot in separate subsections with separate run counts and failure distributions. | Report latency and timeout evidence separately from correctness. Label fake skills and simplified adapters as software/integration validation. Place detector-enabled or real-robot observations in a separate bounded subsection with their own denominator and limitations. |
| [BAKU, Haldar et al. (2024)](https://arxiv.org/abs/2406.07539), Section 4 and Appendix E.1 | Reports simulated and real-robot task sets separately, states 10 simulated rollouts and five real evaluations per task, provides aggregate tables in the main text, moves task-wise results to the appendix, and changes one factor at a time in ablations. | Put aggregate family results in Chapter 7 and complete per-case/repetition data in the appendix or evidence bundle. The two thesis ablations should be described as controlled comparisons only if every other relevant tuple component is fixed. |
| [Latency-Aware Benchmarking of LLMs for ROS 2 Navigation, Das et al. (2026)](https://www.mdpi.com/1424-8220/26/2/608), Section 3, Table 2, and Figures 5–6 | Logs latency, success, path metrics, and token use per trial, then reports sample counts, central tendency, variability, and distribution plots for simulated ROS 2 navigation. | Preserve the per-case timestamps and report latency distributions with explicit denominators. Separate language or planning latency from execution and end-to-end latency whenever the trace supports that decomposition. |

These papers are precedents rather than a universal reporting standard. They nevertheless converge on three practices relevant here: separate levels of success, state denominators and repetitions, and avoid treating simulated or simplified execution as physical-robot evidence.

## Proposed Chapter 7 shape

### 7.1 Canonical validation tuple and headline result

Open with the exact source, nested revisions, image digest, imported source hashes, model configuration, prompt and registry hashes, questionnaire hash, harness hash, launch arguments, fixture, and timestamps. Follow this with a compact headline table:

| Evidence family | Passed | Degraded | Failed | Not scored | Primary claim |
|---|---:|---:|---:|---:|---|
| Full-runtime questionnaire | `[ ]/[ ]` | `[ ]` | `[ ]` | `[ ]` | Route, semantic contract, execution, and closure |
| Environment and KB suite | `[ ]/[ ]` | `[ ]` | `[ ]` | `[ ]` | State query, mutation, freshness, and postconditions |
| Deterministic fake matrix | `[ ]/[ ]` | `[ ]` | `[ ]` | `[ ]` | Dispatch, feedback, recovery, and truthful failure |
| Frozen robustness set | `[ ]/[ ] repetitions` | `[ ]` | `[ ]` | `[ ]` | Stability under fixed paraphrases and multi-turn references |
| Bounded detector or robot profile | `[ ]/[ ]` | `[ ]` | `[ ]` | `[ ]` | Perception or adapter observations only |

The pending result may populate these placeholders with 100% values only after the canonical evidence bundle and semantic oracle agree.

### 7.2 Full-runtime semantic contract results

Use one table with the 21 cases grouped by family. The main columns should be:

`case family`, `cases`, `route correct`, `plan valid`, `target-set exact`, `role/order correct`, `execution complete`, `report coverage`, `postcondition correct`, `speech exactly once`, and `overall pass`.

This follows the plan/execution separation in SayCan and the task/goal-condition separation in ALFRED. A short paragraph may use the earlier gold-apple result to explain why the semantic columns are necessary, but the historical run should not dominate the subsection.

### 7.3 Stateful environment and KB validation

Report the environment suite separately because its unit of correctness is a state transition or query answer, not only an executable trajectory. Include:

- pre-query result;
- expected mutation or no-mutation rule;
- post-query result;
- state-freshness verdict;
- natural-language answer correctness;
- denominator and concise failure reason.

Keep symbolic state and wording as separate columns. A fluent response does not establish the KB effect, while a correct direct query does not establish adequate user-facing wording.

### 7.4 Deterministic failure and recovery matrix

Use a policy-by-case heatmap. Suggested cells are `pass`, `degraded`, `fail`, and `not applicable`, with a short reason code. Report at least:

- `all_success`;
- `fail_once_navigation`;
- applicable `fail_once_pick` cases;
- applicable `delivery_blocked` cases;
- applicable `recipient_missing` cases.

Beside the heatmap, report recovery denominators: injected failures observed, correct feedback emitted, valid replan lineage, safe retry or clarification, terminal outcome truthful, and exactly-once speech. Include one representative lineage trace with `goal_id`, `plan_id`, `plan_version`, stable step identity where applicable, failure observation, revised action, and final dialogue act.

### 7.5 Robustness repetitions

Show all three repetitions of each frozen case rather than a selected best run. A compact table can use `P/P/P`, `P/P/F`, or equivalent case-level marks, plus `stable pass rate` and the observed failure seam. Report descriptive counts only. Five prompts repeated three times do not support broad statistical inference about arbitrary language.

### 7.6 Controlled ablations

Keep only actual controlled comparisons in this section. Each ablation table should declare the changed factor and the fixed tuple components. Suitable candidates are:

- response-first versus intent-first, only if intent-first remains an executed and provenance-complete profile;
- canonical model versus alternate model, only if prompt pack, fixture, image, questionnaires, policies, and scoring oracle are held fixed.

If those controls are not satisfied, label the evidence a diagnostic comparison and move it to the developmental appendix. Do not infer causality from F-series runs that changed several variables simultaneously.

### 7.7 Bounded detector and physical-robot evidence

Report this independently from deterministic fake execution. State the number of trials and the physical seams actually exercised. Use separate columns for detector correctness, adapter admission, physical action observation, and post-action evidence. Do not let an `all_success` fake-skill result imply navigation or manipulation accuracy.

### 7.8 Error taxonomy, operational index, and RQ synthesis

Even if every canonical case passes, retain a zero-count taxonomy so the evaluated failure space is visible. Populate it with failures or degradations from the canonical tuple only, then refer to historical examples in Chapter 8. Report all eight operational-index components with their denominators. Finish with the existing RQ table, updated from the canonical evidence first and bounded by the declared profile.

## Recommended Chapter 7 visuals

Use a small number of figures with distinct purposes:

1. **Canonical evidence overview:** counts by evidence family and semantic depth.
2. **Phase-completion funnel:** injected turn, correct route, valid plan, admitted plan, completed execution, verified postcondition, complete report, exactly-once closure.
3. **Failure-policy heatmap:** case against deterministic policy.
4. **Latency distribution:** median, IQR, p95, maximum, timeout count, and denominator by comparable case family.
5. **Representative recovery trace:** plan-version lineage rather than an unstructured log excerpt.
6. **Error taxonomy:** counts by owning seam, shown only when the denominator and classification rule are explicit.

The funnel and operational index must remain secondary to the semantic acceptance table. A weighted score can conceal one safety-critical failure.

## Proposed Chapter 8 allocation

Chapter 8 should explain mechanisms and implications rather than repeat the result tables.

- **Architectural interpretation:** what deterministic admission and owner separation did and did not guarantee.
- **Planning and recovery:** why feedback enabled or failed to enable revised plans under each injected policy.
- **Grounding and state:** what the target-set and KB postcondition checks reveal about compact grounding.
- **ROS4HRI integration:** what was validated at the public seams and what remains adapter-specific.
- **Methodological finding:** why trajectory completion and semantic contract satisfaction require separate oracles.
- **External validity:** why fake-skill, detector, and physical-robot evidence have different claim strengths.

The gold-apple, incomplete-report, stale-KB, and late-speech incidents fit here as concise diagnostic case studies. Each should support a general finding, state the responsible seam, identify the added oracle or guard, and say whether the canonical run resolves it. Detailed event chronology belongs in the appendix or evidence bundle.

## Treatment of the F-series

The F identifiers remain useful for internal traceability but are poor reader-facing section labels because they encode development order rather than experimental meaning.

Recommended handling:

- use a semantic canonical-run identifier in Chapter 7, tied to its immutable manifest;
- cite F identifiers parenthetically only when tracing a diagnostic claim;
- move the full F-series table to an appendix headed **Developmental and diagnostic evidence**;
- group historical runs by failure class or intervention, not by date;
- retain dates, image changes, and scores in the evidence manifest;
- do not average or trend scores across incomparable tuples;
- describe the historical sequence in Chapter 8 only when it supports a methodological or architectural conclusion.

A suitable appendix table is:

| Evidence ID | Semantic label | Tuple difference | Diagnostic finding | Resulting guard or oracle | Canonical status |
|---|---|---|---|---|---|
| F13 | Semantic re-audit | Stronger oracle | Trajectory pass concealed wrong task semantics | Cross-phase target and postcondition equality | Diagnostic only |
| F14 | Targeted post-hardening probes | Multiple rebuilt images | Focused target/report improvements with one unresolved selection case | Exact selection and report coverage checks | Diagnostic only |

This retains the developmental value without making a sequence of mutable builds appear to be the final experiment.

## Acceptance rules for the writing agent

The agent updating Chapters 7 and 8 should not:

- write `100%` without an explicit numerator and denominator;
- merge isolated reruns into a full-sweep denominator;
- use a late diagnostic pass to replace the original policy sweep;
- claim physical navigation or manipulation from fake-skill execution;
- call an uncontrolled historical comparison an ablation;
- let final speech establish postcondition correctness;
- lead Chapter 7 with the F-series chronology;
- remove counterexamples that motivated the stronger oracle.

The agent should:

- derive every numeric statement from the final generated bundle;
- retain placeholders until the CRITIC run and repair tuple are frozen;
- lead each subsection with its aggregate denominator, then show the decisive semantic breakdown;
- keep one canonical run as the basis of the RQ answers;
- distinguish canonical, bounded, ablation, diagnostic, and historical evidence in captions and prose;
- place detailed case outputs and provenance in the appendix or repository bundle.

## Suggested evidence products for the later handoff bundle

The final handoff should reference generated artifacts rather than restating them manually:

- immutable `manifest.json` with all source and configuration hashes;
- `case_results.csv` with one row per case and semantic component;
- `policy_matrix.csv` for deterministic failures;
- `robustness_repetitions.csv` containing all 15 observations;
- `latency_summary.json` and the underlying timestamped observations;
- `operational_index.json` with all eight components and denominators;
- `error_taxonomy.csv`;
- one machine-readable lineage trace;
- generated figures used in Chapter 7;
- an RQ-to-evidence ledger;
- a compact appendix table mapping F identifiers to semantic findings and source bundles.

This structure allows another agent to update the thesis without interpreting raw logs or reconstructing denominators from prose.
