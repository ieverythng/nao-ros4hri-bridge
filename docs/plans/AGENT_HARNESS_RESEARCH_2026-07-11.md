# Agent Harness Research Plan

Date: 2026-07-11

This plan captures a research direction for a Watson/Hermes-style harness that
can serve local LLMs into ROS, NAO, code, and later applied systems such as
iTrader. The motivation from the runtime ablations is simple: model capability
matters, but the harness decides what the model can see, what it may do, how
errors are represented, and whether success is checked against the world.

## Position

The harness should not start as a replacement for Neural Workbench. It should
start as a thin, typed environment layer around the seams we already validate:
dialogue, planner request, execution feedback, KnowledgeCore mutation,
artifacts, and runtime traces. Once those contracts are stable, the same
architecture can support non-robot applied-agent work.

## Design Principles

- Keep environment state typed and replayable.
- Separate model roles: response, intent, planner, critic, summarizer, and tool
  executor.
- Treat postconditions as first-class evidence, not optional logging.
- Keep ROS ownership boundaries intact. The harness can observe and request; it
  must not bypass `nao_orchestrator`, `planner_llm`, or `kb_skills`.
- Make model backends swappable through one role-aware adapter over LiteLLM,
  llama.cpp, vLLM, OpenAI-compatible APIs, and local process tools.
- Store traces as research artifacts that can be replayed without the live
  robot.

## Proposed Architecture

```text
User / evaluator
  -> Harness session controller
  -> Role router
     -> response model
     -> intent model
     -> planner model
     -> critic model
  -> Environment adapter layer
     -> ROS4HRI adapter
     -> workspace/code adapter
     -> browser/service adapter
     -> future trading/simulation adapter
  -> Tool schema registry
  -> Postcondition checker
  -> Trace and replay store
  -> HTML/operator dashboard
```

## ROS4HRI Harness Boundary

The ROS harness should expose a small set of typed operations:

- Capture runtime state: node graph, lifecycle states, launch args, endpoint
  reachability, duplicate node names, and model role bindings.
- Inject user turns through the same dialogue service used by the stack.
- Read planner requests, planner outputs, execution feedback, planner dialogue
  acts, chatbot traces, and KnowledgeCore query results.
- Apply deterministic fixtures for humans, objects, locations, and fake-skill
  policies.
- Score postconditions after every claimed mutation or completed action.

The harness should not directly mutate KnowledgeCore except through explicit
fixture setup and teardown. Planner-visible mutations during a user turn must
continue to pass through `nao_orchestrator` and `kb_skills`.

## Lessons From The Ablations

- Qwen/Watson produced more natural wording, but fluency did not imply reliable
  execution. The fake-deep and KB postconditions remained the deciding evidence.
- Dynamic parameter mutation is not a reliable model-ablation boundary. The
  launch contract must name response, intent, and planner models explicitly.
- Planner JSON validity is only one layer. The mixed run reduced schema pressure
  but still failed KB revise/remove and fake-deep handoff.
- Grouped location effects need a postcondition model that can detect stale
  predicates and object-like facts leaking onto humans.
- Missing speech evidence can make a good turn look degraded. The harness should
  distinguish speech instrumentation failures from planner/executor failures.

## Grilling Checkpoints

Recommended answer: start from the existing runtime-review scripts, not from
scratch. They already know the ROS graph, fake-skill fixtures, and trace
channels. The research harness should extract them into a cleaner session
controller.

Recommended answer: keep the ROS harness as one environment adapter inside a
general harness. A robot-specific harness will move faster now, but a generic
core avoids rebuilding the same role router and postcondition machinery for
iTrader or code-agent work later.

Recommended answer: use model roles rather than one global model. The ablations
already show that wording quality, route selection, planning schema compliance,
and critique are different jobs.

Recommended answer: make postconditions mandatory for research-grade runs. A
successful answer without a world-state check should be scored as unproven.

Recommended answer: build a small HTML dashboard early. It should show role
bindings, endpoint reachability, graph health, current fixture, model outputs,
execution feedback, postconditions, and replay links.

## Milestones

1. Ablation ledger: record model roles, launch args, endpoint checks, artifacts,
   and scores for every run.
2. Replayable runtime session: wrap the current questionnaire and architecture
   sweep behind a session manifest.
3. Role router: configure response, intent, planner, critic, and summarizer
   models independently.
4. Postcondition DSL: express KB, execution, report-result, and human/object
   assertions in one format.
5. Watson ROS adapter: package the ROS4HRI operations as a reusable local
   harness module.
6. Applied-agent expansion: add workspace, browser/service, and iTrader-style
   simulation adapters.

## Risks

- Overfitting the harness to the NAO thesis stack.
- Treating prompt text as the only control surface.
- Allowing direct KB writes outside accepted fixture setup.
- Scoring launch-wiring failures as model failures.
- Letting natural language reports pass without postcondition evidence.

## Immediate Next Work

- Add a session manifest format for runtime reviews.
- Add an endpoint preflight that blocks scored ablations when the configured
  backend is unreachable.
- Add a model-role verification check that requires startup logs and runtime
  traces to agree with launch arguments.
- Add grouped-location postcondition checks for stale `isIn`, `isAt`, `isOn`,
  `contains`, and inverse human/object leakage.
