# Motivation Letter: AI Agent & Tool-Use Engineer (RE2)

**Juan David Bendek Williamson**  
Barcelona, Spain | +34 623 124 525 | juandavidbendeck@hotmail.com

28 July 2026

Recruitment Panel  
Barcelona Supercomputing Center - Centro Nacional de Supercomputación

**Subject: Application for AI Agent & Tool-Use Engineer (RE2), reference 187_26_AII_LM_RE2**

Dear Members of the Recruitment Panel,

I am applying for the AI Agent & Tool-Use Engineer position because it sits at
the intersection I have been building towards: reliable agentic workflows,
well-defined tool interfaces, and evaluation that distinguishes model failures
from failures in the surrounding system. I am completing an MSc in Modelling
for Science and Engineering at the Universitat Autonoma de Barcelona,
specialising in Data Science. Alongside my studies, I work on enterprise agent
systems at Aily Labs and completed a JAE Intro ICU 2025 research scholarship at
IIIA-CSIC.

At Aily Labs, I work on the engineering needed to turn language models into
production analytical agents. My contributions have included prompt-defined
MCP capabilities, an MCP server package for go-to-market workflows, and the
transition towards task-scoped skills that control which instructions and
tools are exposed. I have also worked with a semantic access layer in which
curated SQL table descriptions encode business meaning and intended usage.
This allows an SQL agent to query approved data through a bounded interface
instead of inferring how to use a database from raw schema alone.

Evaluation closes that production loop. Package-owned question sets declare
the expected tables, skills, tools, and answer criteria for each capability.
Benchmark runs combine deterministic assertions, LLM-as-a-judge scoring, and
trace inspection, with the resulting checks incorporated into CI/CD and
promotion decisions. This makes failures attributable: the evidence can show
whether a weak answer arose from model reasoning, tool selection, semantic
context, routing, or runtime execution. It also provides a disciplined way to
improve an agent through better contracts, context, and tools before assuming
that fine-tuning is the only remedy.

At IIIA-CSIC, I applied the same approach to an embodied system. My MSc thesis
integrates dialogue, symbolic knowledge, language-model planning, deterministic
execution, and typed feedback in a ROS 2 architecture for a NAO robot. The
system constrains planning through a reviewed skill registry, passes compact
knowledge context to the language-model components, preserves execution
lineage, and validates both simulated and physical skill adapters through
structured traces. I designed the validation around dialogue, knowledge
interaction, single and composite skills, and failures introduced at the
beginning, middle, or end of execution. This work reinforced a practical
lesson: fluent model output is not evidence of reliable tool use unless the
capability exposure, world state, action result, and trace are also correct.

<!-- pagebreak -->

These lessons also motivated an unpublished proof of concept for a Universal
Agentic Harness. It projects task-specific capabilities from a registry,
applies deterministic reach and effect checks, and records append-only traces
through a portable environment interface. The proof of concept has focused
tests but is not yet live-integrated or externally validated. Future work will
test the same contracts against the other agentic environments in my project
portfolio. The objective is concrete: make capability exposure, permissions,
and evidence requirements explicit enough that an agent can be evaluated
consistently across different domains.

My strongest personal machine-learning project, iTrader, provides complementary
model-side experience. Its implemented research environment uses a PPO/GAE
solver, task-conditioned FiLM policy and value networks, validated task
specifications, and a decomposed reward engine covering PnL, drawdown,
turnover, and transaction-related effects. Runs preserve configuration, seed,
code, and dataset lineage. Rule-based and local LLM proposer backends generate
candidate curricula behind validation and deterministic controls. Planned work
will evaluate grouped proposer candidates and GRPO-like optimisation once the
downstream reward signal is sufficiently robust. This has given me direct
experience with PyTorch, policy optimisation, reward shaping, experimental
controls, and the failure modes of learning systems that optimise against an
imperfect objective.

My hands-on experience is strongest in model-tool systems, PPO/GAE, reward
design, and evaluation harnesses. I have less production experience with TRL,
FastChat, NeMo-RL, DPO, GSPO, and CISPO, and I would treat these as a focused
area of development. I would not, however, approach them without the necessary
engineering foundation. My work already relies on typed task contracts,
separate development and holdout evidence, reproducible runs, reward-hacking
checks, and evaluation that can distinguish a better policy from a more
permissive runtime.

The Language Modeling Team is therefore a strong fit for both what I can
contribute immediately and where I want to deepen my research engineering. I
can bring practical experience in MCP integration, function calling,
task-oriented tool exposure, evaluation pipelines, trace analysis, and Python
ML development. I am equally motivated to apply that experience to instruction
data curation and post-training experiments on BSC infrastructure. I work
fluently in English and Spanish, and I value the opportunity to contribute to
open language technologies within a multidisciplinary research environment.

Thank you for considering my application. I would welcome the opportunity to
discuss how my experience in agent evaluation, model-tool integration, and
reproducible ML experimentation could support the team.

Yours sincerely,

**Juan David Bendek Williamson**

## References

1. **Roger Montane**, Mid Data Scientist, Aily Labs  
   Mentor | roger.montane@ailylabs.com
2. **Dr. Balogh Miklos**, Professor, BME  
   BSc Thesis Supervisor | balogh.miklos@gpk.bme.hu
