# Motivation Letter: Research Engineer - Agentic Systems in AI for Science (RE2)

**Juan David Bendek Williamson**  
Barcelona, Spain | +34 623 124 525 | juandavidbendeck@hotmail.com

26 August 2026

Recruitment Panel  
Barcelona Supercomputing Center - Centro Nacional de Supercomputación

**Subject: Application for Research Engineer - Agentic Systems in AI for Science (RE2), reference 383_26_AII_AIT_RE2**

Dear Members of the Recruitment Panel,

I am applying for the Research Engineer position in Agentic Systems for AI for
Science because the initiative closely matches the systems problem that has
defined my recent work: how to give language-model agents useful capabilities
while keeping deployment, evaluation, and reported outcomes reliable. I
recently completed an MSc in Modelling for Science and Engineering at the
Universitat Autonoma de Barcelona, specialising in Data Science, after a BSc in
Mechanical Engineering. I completed my BSc under the Hungarian Government's
Stipendium Hungaricum scholarship, which covered my tuition and provided living
and accommodation support. I was later selected for CSIC's JAE Intro ICU 2025
research-introduction scholarship at IIIA-CSIC, where I developed my MSc thesis.
My experience combines enterprise agent workflows, interactive robotics, and
computational modelling.

At Aily Labs, I took responsibility for integration seams connecting
go-to-market data capabilities to SuperAgent, the company's general-purpose,
customer-facing LLM reasoning agent. My work progressed from MCP server tools
to task-scoped skills that packaged approved instructions, tools, and evaluation
criteria around each workflow. I then helped migrate these skills into a
governed semantic layer, representing approved SQL tables through curated
Markdown resources with their business meaning, relationships, permitted use,
and access constraints. I developed and maintained GTM capabilities used by
Aily tenants including Sanofi.

When answer quality regressed or a skill, resource mapping, or workflow
contract changed, my team was responsible for evaluating and correcting the
behaviour from development through production. My contribution was to trace
failures across data availability, semantic descriptions, table allowlists,
tool selection, DAG payloads, and post-processing, then verify the correction
before promotion. Code review, automated tests, continuous integration,
pre-production validation, question sets, and trace inspection provided the
release evidence. This gave me practical LLMOps experience with deployment
controls, production observability, regression evaluation, and tenant-scoped
data and capabilities.

My MSc thesis at IIIA-CSIC brought the same concerns into an embodied system. I
designed and validated a modular ROS 2 and ROS4HRI architecture for NAO that
separates dialogue, grounded symbolic state, language-model planning,
deterministic skill execution, and feedback-driven recovery. Planning is
bounded by a reviewed capability registry, execution preserves plan lineage,
and completion reports must remain tied to runtime evidence. I validated the
integrated stack through a scenario-based framework covering dialogue,
knowledge-base cases, composite actions, injected failures, recovery, final
reporting, and physical robot checks. The result is an agentic workflow whose
model, harness, ROS runtime, state, and execution path can be inspected
separately.

The MSc also provided formal training relevant to BSC's HPC environment.
Parallel Programming covered C, CUDA, OpenMP, MPI, SLURM, GPU programming, and
performance analysis. Distributed Systems covered cluster and supercomputer
infrastructure, batch queues, job arrays and dependencies, heterogeneous
resources, cloud systems, and fault tolerance. Through laboratory and project
work at UAB and IIIA-CSIC, I submitted and monitored cluster workloads and
worked with parallel and distributed execution models. This is hands-on
academic HPC experience, which I am prepared to deepen within BSC's production
infrastructure.

<!-- pagebreak -->

My personal projects extend the same direction. An unpublished Universal
Agentic Harness proof of concept projects task-specific capabilities from a
registry, applies deterministic reach and effect gates, and records append-only
traces through portable environment adapters. Watson is my working personal
agent platform: Hermes provides the harness and tool workflow, LiteLLM provides
model routing and an OpenAI-compatible proxy, llama.cpp serves a quantised Qwen
model, and ZeroTier exposes the endpoint across my machines. I maintain its
launch, recovery, routing, and performance gates.

iTrader provides complementary model-side experience as an autonomous trading
platform under staged development. Its implemented offline stack learns from
OHLCV windows and engineered features through validated task specifications,
PPO/GAE, FiLM-conditioned policy and value networks, decomposed rewards, local
LLM proposers, and reproducible manifests. Its online design progresses through
replay and shadow operation before live execution, with measured solver
capability, deterministic decision gates, and separate risk controls.

I can contribute immediately to the review and implementation of agent
frameworks, Python-based integration, reproducible evaluation, documentation,
and operational diagnosis. My direct experience is with contributing to and
maintaining production LLM workflows rather than operating an institutional
HPC agent platform at BSC's scale. The combination of production-facing
ownership at Aily, full-stack agent validation in my thesis, and formal
parallel-computing training provides a relevant foundation for that next level
of responsibility.

BSC's objective of hosting an in-house agentic solution for researchers is
especially compelling because it treats model choice, software architecture,
HPC integration, user support, and evaluation as one engineering problem. I
would bring disciplined ownership, strong Python and Linux practice, fluent
English and Spanish, and experience communicating across data, research, and
software teams. I would welcome the opportunity to help develop a system that
improves scientific productivity while remaining reproducible and observable.

Thank you for considering my application. I would welcome a discussion about
how my experience in agentic systems, deployment discipline, and evidence-based
evaluation could support the AI Institute's AI4Science initiative.

Yours sincerely,

**Juan David Bendek Williamson**

## References

1. **Francisco Martin**, Principal Data Scientist, Aily Labs  
   Manager | francisco.martin@ailylabs.com
2. **Dr. Balogh Miklos**, Professor, BME  
   BSc Thesis Supervisor | balogh.miklos@gpk.bme.hu
3. **Ben Wolfe**, Senior Engineer, ABS Consulting UK  
   Supervisor | bwolfe@eagle.org
