# TFM Architecture and Implementation Reference

Date: 2026-06-03
Audience: TFM writing, supervisor review, implementation handoff
Scope: Architecture, package ownership, runtime flow, and thesis chapter integration.

---

## Purpose and Thesis Positioning

This document turns the current implementation into thesis-ready prose. It supports the architecture, implementation, and discussion chapters of the TFM draft extracted from the Overleaf structure. The system under study is a modular ROS 2 and ROS4HRI stack for the NAO robot in which natural language, symbolic grounding, LLM planning, deterministic execution, and user-facing dialogue are deliberately separated.

The core claim is that LLM-based planning becomes more inspectable and experimentally useful when the language model is not treated as a direct robot controller. Instead, the LLM is placed inside a typed planning layer. It receives bounded context, emits structured JSON, and depends on deterministic ROS nodes for admission, validation, execution, and feedback.

<div class="diagram">
  <svg viewBox="0 0 1400 570" role="img" aria-label="Layered planner architecture">
    <defs>
      <marker id="arr" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="#111"></path>
      </marker>
    </defs>
    <rect x="40" y="34" width="1280" height="78" fill="#f7f7f7" stroke="#222"></rect>
    <text x="65" y="80" font-size="28" font-family="Liberation Serif, Times New Roman, serif">Human interaction layer: dialogue_manager + chatbot_llm</text>
    <rect x="40" y="146" width="1280" height="90" fill="#fff" stroke="#222"></rect>
    <text x="65" y="190" font-size="26" font-family="Liberation Serif, Times New Roman, serif">Semantic planning layer: planner_llm + planner_common contracts</text>
    <text x="65" y="218" font-size="18" font-family="Liberation Serif, Times New Roman, serif">Goal supervision, plan generation, replanning, and planner dialogue acts.</text>
    <rect x="40" y="270" width="1280" height="90" fill="#f7f7f7" stroke="#222"></rect>
    <text x="65" y="314" font-size="26" font-family="Liberation Serif, Times New Roman, serif">Deterministic execution layer: nao_orchestrator + skill registry projection</text>
    <text x="65" y="342" font-size="18" font-family="Liberation Serif, Times New Roman, serif">Admission, validation, action dispatch, and execution feedback.</text>
    <rect x="40" y="394" width="600" height="86" fill="#fff" stroke="#222"></rect>
    <text x="65" y="438" font-size="24" font-family="Liberation Serif, Times New Roman, serif">Grounding layer: KB + scene grounding</text>
    <text x="65" y="466" font-size="17" font-family="Liberation Serif, Times New Roman, serif">Knowledge snapshots, scene summaries, and state_t0.</text>
    <rect x="720" y="394" width="600" height="86" fill="#fff" stroke="#222"></rect>
    <text x="745" y="438" font-size="24" font-family="Liberation Serif, Times New Roman, serif">Robot adapter layer: AB=1 skills</text>
    <text x="745" y="466" font-size="17" font-family="Liberation Serif, Times New Roman, serif">say, look_at, scan, report_result, and head motion.</text>
    <g stroke="#111" stroke-width="2" marker-end="url(#arr)">
      <line x1="700" y1="112" x2="700" y2="146"></line>
      <line x1="700" y1="236" x2="700" y2="270"></line>
      <line x1="470" y1="360" x2="470" y2="394"></line>
      <line x1="1010" y1="360" x2="1010" y2="394"></line>
      <line x1="720" y1="437" x2="640" y2="437"></line>
    </g>
    <text x="64" y="530" font-size="17" font-family="Liberation Serif, Times New Roman, serif">Invariant: the LLM proposes structured intent and plans; deterministic ROS nodes own validation, dispatch, and evidence.</text>
  </svg>
  <div class="caption">Figure 1. Layered decomposition of the active NAO ROS4HRI planner stack.</div>
</div>

## Architectural Thesis

The architecture follows a layered design. The dialogue layer is responsible for receiving user turns and producing semantic intent frames. The planning layer turns those intent frames into structured plans over an abstract skill registry. The orchestrator validates the plan and dispatches only supported actions. Skill servers interact with robot adapters, scene sources, and the knowledge base, then publish typed feedback that the planner can use for continuation, clarification, or replanning.

This split gives a concrete answer to the problem statement: an LLM planner can be integrated into a ROS4HRI robotic system by making it a supervised, contract-bound component rather than an unconstrained action generator. The planner decides what should happen at the symbolic level, while ROS components decide whether and how it may happen.

## Runtime Ownership Model

| Layer | Main packages | Responsibility | Thesis role |
|---|---|---|---|
| Dialogue | `dialogue_manager`, `chatbot_llm` | Dialogue lifecycle, turn handling, intent declaration, user-facing language | Shows that conversational behavior is separated from robot execution |
| Planning | `planner_llm`, `planner_common` | Plan generation, supervisor state, request/output/feedback contracts | Shows how LLM output is made structured and auditable |
| Execution | `nao_orchestrator`, AB=1 skills | Request admission, plan validation, action dispatch, feedback | Shows deterministic control around the LLM |
| Grounding | `nao_scene_grounding`, `kb_skills`, KnowledgeCore | Detector-to-KB grounding and prompt-facing state | Shows how the planner receives bounded symbolic context |
| Robot adapters | `nao_say_skill`, `nao_look_at`, motion/replay skills | Robot-specific execution details | Shows portability through abstract capability contracts |

## End-to-End Runtime Flow

<div class="diagram">
  <svg viewBox="0 0 1500 430" role="img" aria-label="Runtime sequence">
    <defs>
      <marker id="arr2" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
        <path d="M 0 0 L 10 5 L 0 10 z" fill="#111"></path>
      </marker>
    </defs>
    <g font-family="Liberation Serif, Times New Roman, serif" font-size="15">
      <text x="45" y="35">User</text><text x="190" y="35">dialogue_manager</text><text x="390" y="35">chatbot_llm</text>
      <text x="575" y="35">nao_orchestrator</text><text x="820" y="35">planner_llm</text><text x="1030" y="35">Skills</text><text x="1210" y="35">KB / Scene</text>
    </g>
    <g stroke="#aaa" stroke-dasharray="4 4">
      <line x1="70" y1="45" x2="70" y2="365"></line><line x1="260" y1="45" x2="260" y2="365"></line>
      <line x1="450" y1="45" x2="450" y2="365"></line><line x1="660" y1="45" x2="660" y2="365"></line>
      <line x1="875" y1="45" x2="875" y2="365"></line><line x1="1060" y1="45" x2="1060" y2="365"></line><line x1="1260" y1="45" x2="1260" y2="365"></line>
    </g>
    <g stroke="#111" stroke-width="1.5" fill="none" marker-end="url(#arr2)">
      <line x1="70" y1="75" x2="260" y2="75"></line><line x1="260" y1="105" x2="450" y2="105"></line>
      <line x1="450" y1="135" x2="660" y2="135"></line><line x1="660" y1="165" x2="875" y2="165"></line>
      <line x1="875" y1="195" x2="660" y2="195"></line><line x1="660" y1="225" x2="1060" y2="225"></line>
      <line x1="1060" y1="255" x2="1260" y2="255"></line><line x1="1060" y1="285" x2="660" y2="285"></line>
      <line x1="660" y1="315" x2="875" y2="315"></line><line x1="875" y1="345" x2="260" y2="345"></line>
    </g>
    <g font-family="Liberation Serif, Times New Roman, serif" font-size="13">
      <text x="112" y="68">utterance</text><text x="300" y="98">turn request</text><text x="488" y="128">planner_request</text>
      <text x="700" y="158">admitted request</text><text x="735" y="188">plan JSON</text><text x="790" y="218">dispatch</text>
      <text x="1105" y="248">evidence lookup</text><text x="790" y="278">skill result</text><text x="706" y="308">execution_feedback</text>
      <text x="565" y="338">dialogue_act / completion</text>
    </g>
  </svg>
  <div class="caption">Figure 2. Main runtime sequence from natural-language request to execution feedback.</div>
</div>

The runtime flow begins with a human utterance. The dialogue manager forwards the conversational turn to `chatbot_llm`, which classifies the turn as dialogue, knowledge query, or execution. Execution-eligible turns are converted into planner request JSON and published through the orchestrator planner gate. The planner produces a plan, the orchestrator dispatches each step, and feedback is returned as structured JSON. User-facing status is emitted through planner dialogue acts and handled by the dialogue/speech owner.

## Node and Package Responsibilities

### `chatbot_llm`

`chatbot_llm` is the user-facing LLM backend. Its role is not to generate robot plans directly. It interprets the user turn, creates a normalized intent frame, injects knowledge and scene context when available, and publishes planner requests when the turn requires execution. It also remains the appropriate owner for user-facing wording when planner dialogue completion is routed through chatbot-owned language generation.

### `planner_llm`

`planner_llm` is the high-level supervisor. It consumes admitted planner requests, reasons over the skill registry, and emits plan JSON. It tracks goal continuity through `goal_id` and plan lineage through `plan_id` and `plan_version`. It receives execution feedback and may continue, replan, ask for clarification, explain a failure, or emit completion dialogue.

### `planner_common`

`planner_common` is the contract source of truth. It normalizes planner requests, plan steps, execution feedback, scene summaries, and dialogue acts. This package is critical for thesis reproducibility because it makes the JSON interfaces explicit rather than leaving them implicit in prompt text.

### `nao_orchestrator`

`nao_orchestrator` is the deterministic execution boundary. It admits planner requests, validates plan steps, dispatches actions, and publishes feedback. It is intentionally not an LLM policy node. This is the main safety and reproducibility boundary: planner outputs become executable only after deterministic validation.

### `nao_scene_grounding` and `kb_skills`

`nao_scene_grounding` converts detector output into symbolic facts and scene summaries. `kb_skills` keeps KnowledgeCore access behind a package boundary. Together, they provide the grounding path that prevents the planner from relying only on language priors.

### AB=1 skills and robot adapters

Skills such as `say`, `look_at`, `scan`, `report_result`, and head-motion actions form the execution vocabulary. They are represented to the planner as abstract capabilities but implemented through ROS actions and robot-specific adapters. This is the thesis bridge between symbolic planning and embodied robot behavior.

## Architectural Invariants

- The planner emits structured plan JSON, not direct robot API calls.
- The orchestrator validates and dispatches; it does not generate LLM policy.
- The dialogue stack owns user-facing speech and conversational lifecycle.
- Scene and KB state are passed as bounded JSON context, not as raw detector or KB transport dumps.
- Execution feedback is the mechanism that connects embodied outcomes back to the planner.
- People and objects are represented separately in grounded context to avoid treating humans as generic objects.

## Relation to ROS4HRI and SocialMinds Principles

The system follows ROS4HRI-style modularity by keeping public behavior in ROS messages, services, and actions. Continuous observations such as scene state are topic-like, short request-response operations remain service-like, and long-running robot behaviors are action-oriented. The architecture also preserves upstream-sensitive package boundaries: dialogue remains in dialogue packages, grounding remains in scene/KB packages, and robot execution remains in skill/adaptor packages.

For the thesis, this matters because the contribution is not simply that an LLM can produce a plausible task list. The contribution is that the LLM planner is embedded in a robotics architecture where responsibilities are inspectable and replaceable.

## Chapter Integration Guidance

- Chapter 1: motivate the need for modular, grounded, feedback-aware LLM planning.
- Chapter 3: convert the invariants into design principles and non-functional requirements.
- Chapter 4: use the layered diagram and package responsibility table as the system architecture section.
- Chapter 6: expand the node descriptions into implementation subsections.
- Chapter 9: discuss why ownership boundaries reduce brittleness and improve observability.
