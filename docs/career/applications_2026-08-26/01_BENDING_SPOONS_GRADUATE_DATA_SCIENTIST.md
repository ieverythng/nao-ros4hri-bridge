# Cover Letter: Graduate Data Scientist

**Juan David Bendek Williamson**  
Barcelona, Spain | +34 623 124 525 | juandavidbendeck@hotmail.com

26 August 2026

Talent Acquisition Team  
Bending Spoons

**Subject: Application for Graduate Data Scientist**

Dear Members of the Talent Acquisition Team,

I am applying for the Graduate Data Scientist position because it combines the
work I most enjoy: turning incomplete operational questions into dependable
data products and improving the systems that produce them. I recently completed
an MSc in Modelling for Science and Engineering at the Universitat Autonoma de
Barcelona, specialising in Data Science. My work spans enterprise data and
agent products at Aily Labs, scientific machine learning, and end-to-end LLM
systems.

At Aily Labs, I took responsibility for integration seams connecting
go-to-market data capabilities to SuperAgent, the company's general-purpose,
customer-facing LLM reasoning agent. My work progressed from MCP server tools
to task-scoped skills that packaged approved instructions, tools, and evaluation
criteria around each workflow. I then helped migrate these skills into a
governed semantic layer, representing approved SQL tables through curated
Markdown resources with their business meaning, relationships, permitted use,
and access constraints. Specialised agents could therefore discover and query
the right data instead of inferring intent from raw schemas.

I developed and maintained GTM capabilities used by Aily tenants including
Sanofi. When an agent's answer quality regressed or a skill, resource mapping,
or workflow contract changed, my team was responsible for evaluating and
correcting the behaviour from development through production. My contribution
was to trace failures across data availability, semantic descriptions, table
allowlists, tool selection, DAG payloads, and post-processing, then verify the
correction before promotion. The pace required fast intervention without
compromising tenant isolation, data correctness, or customer-facing quality.

Changes moved through code review, automated tests, continuous integration,
pre-production validation, and controlled production promotion. Question sets
declared expected tables, skills, tools, and answer criteria, while traces,
deterministic validators, and model-based evaluation checked each result. I
also built reporting components that passed analytical outputs, artifact paths,
row counts, and execution metadata through explicit contracts to Airflow. This
made me responsible for the complete data-product path from analysis and
orchestration to validation and consumption.

iTrader is my autonomous trading platform under staged development. Its
implemented offline stack learns from OHLCV market windows and solver-specific
engineered features rather than hand-authored buy or sell labels. It combines
market simulation, validated task specifications, a PPO/GAE solver,
task-conditioned FiLM policy and value networks, decomposed rewards for return,
drawdown and turnover, proposer backends, and reproducible manifests. Its
online design progresses through replay and shadow operation before live
execution. Candidate tasks are scored against measured solver capability;
deterministic gates choose whether to trade or wait, while risk and execution
controls remain authoritative. The goal is continuous improvement without
sacrificing reproducibility or safe non-action.

My BSc thesis provides complementary supervised-learning experience. I built
and validated a neural network for turbulent-intensity prediction from
wind-profile data, connecting feature preparation and model selection to the
underlying physical system. Both projects required meaningful targets, separate
evaluation evidence, and restraint when an attractive result was not yet
reliable.

<!-- pagebreak -->

My MSc thesis demonstrates my ability to integrate an agent into an unfamiliar,
distributed ecosystem. I designed and validated a modular ROS 2 and ROS4HRI
stack for NAO, separating dialogue, symbolic knowledge, language-model
planning, deterministic orchestration, and robot skills. The model proposes
plans, while reviewed capabilities and typed execution contracts determine
what the robot may do; feedback supports clarification, replanning, or truthful
failure. I also built the scenario-based framework used to validate dialogue,
knowledge queries, composite tasks, injected failures, recovery, and final
reporting. The work treats the LLM, harness, and ROS runtime as one measurable
system without obscuring ownership at their interfaces.

Watson is the personal agentic platform I built for my own software development
and research. Hermes provides the agent harness and tool workflow, LiteLLM
provides model routing and an OpenAI-compatible proxy, llama.cpp serves a
quantised Qwen model on the GPU, and ZeroTier makes the endpoint available
across my machines. I maintain its launch, health, recovery, and routing paths,
benchmarking context size, time to first token, throughput, memory use, and
tool-call fidelity before promoting a configuration. Watson accelerates my
daily development, but it also makes me operate the serving path behind that
value, including observability, recovery, and performance-quality trade-offs.

Bending Spoons appeals to me because the Graduate Data Scientist role places
data work inside product and company decisions. The responsibility is not only
to analyse data, but to improve its pipelines, define metrics with engineers,
and build reporting or forecasting tools that remain useful across products.
At Aily, I maintained production-facing data and agent workflows for several
consumers. iTrader makes me accountable for a modelling and decision pipeline
from raw market data to controlled action. NAO and Watson show that I can
integrate AI into unfamiliar systems, build the surrounding software, and
evaluate the result instead of trusting plausible output.

I would bring strong Python and SQL foundations, first-principles reasoning,
and a high standard of ownership. At ABS Consulting, engineering work on
seismic qualification and structural assessment for UK nuclear-power assets
reinforced careful validation and reviewable communication. I use AI tools in
daily development, but understand their limits because I have built and
maintained the data, serving, tool, and evaluation layers around them.

I will continue living in Barcelona after completing my MSc. I maintain
permanent residence in the United Kingdom and would welcome sponsorship for a
Spain-based role, with flexibility to visit the Madrid office as needed. Thank
you for considering my application. I would welcome the opportunity to discuss
how my data engineering, applied modelling, and production AI ownership could
contribute to Bending Spoons.

Yours sincerely,

**Juan David Bendek Williamson**

## References

1. **Francisco Martin**, Principal Data Scientist, Aily Labs | Manager | francisco.martin@ailylabs.com
2. **Dr. Balogh Miklos**, Professor, BME | BSc Thesis Supervisor | balogh.miklos@gpk.bme.hu
3. **Ben Wolfe**, Senior Engineer, ABS Consulting UK | Supervisor | bwolfe@eagle.org
