# TFM Presentation Guide

Presentation source: `TFM_JUAN_BENDEK_FINAL.pdf`, dated 23 July 2026.

## Official format

The 2025/2026 UAB teaching guide for Master's Degree Dissertation 42257 states
that each student has 25 to 30 minutes to frame the question, present the
objectives, explain and contextualise the results, and state the conclusions.
The committee may then use up to 30 minutes for questions and discussion. The
committee has three members. The oral dissertation contributes 20 percent of
the final grade.

Official source:
<https://apps.uab.cat/guies/public/portal/html/2025/assignatura/42257/en>

## Recommended timing

| Segment | Time | Slides |
| --- | ---: | ---: |
| Motivation, problem, and questions | 4 minutes | 1 to 4 |
| Architecture and implementation | 8 minutes | 5 to 11 |
| Validation and results | 9 minutes | 12 to 17 |
| Limitations, freeze, and future work | 5 minutes | 18 to 20 |
| Contributions and conclusion | 3 minutes | 21 to 22 |

The template contains appendix slides for questions. They are not part of the
timed main presentation.

## Central presentation argument

The main claim is containment and qualification. The architecture does not make
LLM output deterministic. It constrains execution authority, preserves typed
lineage, and makes semantic failures observable before or after dispatch.

Use this sequence:

1. establish why a planner call is insufficient for interactive robotics;
2. show the ownership boundaries and runtime contracts;
3. explain how deterministic admission contains generated proposals;
4. show the validation design and primary Qwen3-VL result;
5. separate model variance from stack coherence;
6. state the physical and statistical limitations without weakening the
   software contribution;
7. close with the repeatable qualification procedure and v2 embodied roadmap.

## Live demonstration rule

The live demo supports the presentation but is not the only evidence. Keep one
recorded trace or screenshot ready for every live step. Use `iiia:nao-final` and
pin the selected model at startup. If the remote endpoint is unavailable, use
the tested Ollama fallback and state that the model changed. Do not claim that a
fallback run reproduces the Qwen3-VL qualification.

Recommended live sequence:

1. ordinary dialogue;
2. KnowledgeCore scene query;
3. one real posture or head/gaze action;
4. one multi-step fake-skill plan with visible execution feedback;
5. one controlled failure and replan or truthful stop.

## Likely committee questions

- Why use an LLM planner instead of PDDL or HTN planning?
- What part of the architecture is deterministic?
- What does model-agnostic mean if models produce different results?
- Why are fake skills valid evidence?
- Which results were executed on the physical robot?
- How is false completion prevented?
- How are people, objects, and locations kept separate?
- What would be required to make navigation or manipulation a supported claim?
- How would the architecture transfer to another robot?
- What is the main scientific contribution beyond software integration?

Concise answers are included in the appendix slides of the HTML template.
