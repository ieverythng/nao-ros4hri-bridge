# Juan David Bendek Williamson

[juandavidbendeck@hotmail.com](mailto:juandavidbendeck@hotmail.com) | +34 623 124 525 | [LinkedIn](https://www.linkedin.com/in/juan-david-bendek-williamson-958186216) | [GitHub](https://github.com/ieverythng) | Barcelona, Spain

## EDUCATION

### MSc, Modelling for Science and Engineering
*Universitat Autònoma de Barcelona, Barcelona, Spain | September 2025 - July 2026*
- Data Science specialisation with formal training in scientific modelling, distributed systems, parallel programming, and high-performance computing using C, CUDA, OpenMP, MPI, and SLURM.
- MSc thesis: "Design, Integration, and Validation of a Modular LLM-Based Interactive Architecture."
- Selected for the CSIC JAE Intro ICU 2025 research-introduction scholarship at IIIA-CSIC, supporting research on LLM-based architectures for interactive systems.

### BSc, Mechanical Engineering
*Budapest University of Technology and Economics, Budapest, Hungary | September 2020 - July 2024*
- Process Engineering specialisation; final grade: 4.33/5.
- Completed under the Hungarian Government's Stipendium Hungaricum scholarship, including full tuition coverage and living and accommodation support.
- BSc thesis: "Application of Neural Networks to Improve Wind Measurements."

## RESEARCH EXPERIENCE

### MSc Thesis Project
*Universitat Autònoma de Barcelona and IIIA-CSIC, Barcelona, Spain | Academic year 2025-2026*
**Supervisor:** Dr Raquel Ros
*"Design, Integration, and Validation of a Modular LLM-Based Interactive Architecture"*
- Designed and implemented a modular ROS 2 architecture for the NAO robot using ROS4HRI, a standard set of interfaces for representing people and social signals. The system separates dialogue, route selection, symbolic world knowledge, language-model planning, deterministic action control, robot execution, and user-facing reporting.
- Specified the structured information exchanged between components, including user goals, model-proposed plans, action results, execution feedback, dialogue responses, and subject-predicate-object world facts. Stable identifiers preserve the history of revised goals and plans, while a reviewed catalogue limits the robot to permitted actions.
- Integrated the KnowledgeCore symbolic knowledge base, detector-normalised scene information, and real or simulated robot actions. Execution feedback can trigger clarification, replanning, or truthful failure, while one dialogue component remains responsible for speech.
- Developed a scenario-based software and integration validation framework covering route correctness, entity and role preservation, action order, expected changes to knowledge-base state, controlled failures, recovery, and truthful terminal reporting.
- Produced a final evidence package of 108 case records from 33 structured traces. The frozen primary configuration passed 64/72 records, including 20/22 end-to-end requirements.
- Demonstrated recovery from controlled action failures through revised plans and rejection of incomplete long multi-capability plans before execution; documented remaining errors in model-authored target selection, role preservation, clarification wording, and terminal closure.

### JAE Intro ICU 2025 Research Scholar - IIIA-05 LLM-Based Architectures for Interactive Systems
*Artificial Intelligence Research Institute (IIIA-CSIC), Bellaterra, Spain | January 2026 - August 2026*
- Investigated how language-model dialogue and planning can be integrated into an inspectable human-robot interaction stack for domestic-like tasks without assigning execution authority to the model.
- Re-engineered and extended a distributed ROS 2 workspace so dialogue, planning, knowledge, perception, action control, and robot-specific hardware code remained independently testable. Reusable ROS4HRI interfaces represented tracked people and speech without coupling them to the planner.
- Integrated a custom YOLO object detector with the NAO camera stream; normalised detections from different sources, applied confidence and label controls, maintained short-lived object identities, and converted accepted observations into symbolic facts carrying source and freshness information.
- Conducted containerised runtime diagnosis and iterative validation across knowledge queries, multi-step plans, controlled skill failures, replanning, duplicate-speech prevention, and physical robot checks.
- The appointment provided the institutional research setting and experimental platform from which the linked MSc thesis formalised the architecture, validation method, and final evidence package.

### iTrader - Autonomous Trading Agent and Platform
*Independent Research and Development Project | July 2025 - Present*
[Project repository](https://github.com/ieverythng/itrader-azr)
- Architected iTrader as a continuously improving autonomous trading system with distinct offline and online versions. The offline version learns and measures trading behaviour; the online design later frames market opportunities, scores them against measured capability, and retains deterministic authority over whether to trade or wait.
- Implemented the offline training environment over open, high, low, close, and volume market windows with engineered features selected for the policy model rather than hand-authored buy or sell labels. Parameterised task descriptions vary market horizon, episode start, reward objective, and conditioning context without changing the core environment.
- Applied Proximal Policy Optimization with Generalized Advantage Estimation (PPO/GAE). Feature-wise linear modulation (FiLM) conditions the policy and value networks on the active task, allowing one policy model to learn across different objectives and market contexts.
- Designed finite, decomposed rewards for return, drawdown, turnover, and risk-adjusted behaviour. Explicit validation prevents malformed tasks or non-finite rewards from silently entering training and makes reward exploitation easier to diagnose.
- Added rule-based and language-model curriculum generators that propose training tasks through interchangeable local or OpenAI-compatible model services. Generated tasks are checked against allowed ranges and structures, with deterministic fallback when a model response is invalid or unavailable.
- Stabilised the implemented offline baseline through seed control, environment and reward guards, checkpoints, run manifests recording code/configuration/data identity, health checks, Population Based Training, and comparable JSON/CSV metrics with Matplotlib ablation reports. Development exposed recurrent challenges around non-finite training signals, dependency and checkpoint compatibility, and unreliable model-generated task descriptions.
- The online version remains staged: historical replay and shadow operation must demonstrate calibrated policy capability before live execution. Candidate actions will pass through deterministic trade/wait decisions and separate risk controls, so language-model output cannot directly place a trade.

### BSc Thesis Project
*Budapest University of Technology and Economics, Department of Fluid Mechanics | 2024*
**Supervisor:** Dr Miklos Balogh
*"Application of Neural Networks to Improve Wind Measurements"*
- Prepared and cleaned 10-minute averaged light detection and ranging (LIDAR) wind-profile measurements collected at University College Cork across five heights, removing invalid readings and restructuring the data into a cyclic height representation.
- Engineered physically motivated features from horizontal and vertical wind components, height gradients, extrema, and turbulence quantities, with normalisation and explicit controls against trivially reconstructing the target.
- Implemented and compared a one-dimensional convolution plus gated recurrent unit model (Conv1D-GRU), a baseline long short-term memory network (LSTM), and recurrent models supplied with turbulence intensity at one measured height. The work used Python, TensorFlow/Keras, pandas, scikit-learn, CUDA/cuDNN, TensorBoard, and Matplotlib.
- Used held-out train, validation, and test splits, regularisation, early stopping, mean absolute and mean squared error, and model/feature comparisons to assess convergence, generalisation, and sensitivity to height-dependent inputs.
- Selected the recurrent model supplied with turbulence intensity at 90 metres (ATI-90m) as the strongest tested architecture. It achieved more than 15% lower mean absolute error than the baseline LSTM in the reported comparison and reconstructed the turbulence-intensity profile across all five heights on unseen data.
- Identified limitations including sensitivity to hyperparameters, reduced accuracy during the first test-window samples, dependence on the available measurement period and heights, and the need for broader high-resolution validation before operational use.

### Individual Research Project - Applications of PINNs for CFD and Fluid Mechanics
*Budapest University of Technology and Economics, Department of Fluid Mechanics | February 2023 - June 2023*
**Supervisor:** Dr Joshua Patrick Davidson
- Conducted a literature-based technical assessment of Physics-Informed Neural Networks (PINNs) for fluid mechanics and their relationship to conventional Computational Fluid Dynamics (CFD) workflows.
- Analysed a proposed PINN architecture in which the Navier-Stokes equations, initial and boundary conditions, observational data, and collocation-point residuals contribute to a composite training objective.
- Developed workflow diagrams and pseudocode for geometry/domain definition, sampling, automatic differentiation, loss balancing, training, and validation against analytical or CFD reference solutions.
- Compared potential advantages such as mesh-free approximation, data/physics fusion, inverse-problem support, and differentiable surrogate modelling with limitations involving optimisation stiffness, loss-term imbalance, boundary-condition enforcement, computational cost, and uncertain extrapolation.
- The project remained an analytical design study; it did not implement or benchmark a coupled PINN-CFD solver.

## RELEVANT TECHNICAL EXPERIENCE

### Intern Data Scientist - Aily Labs
*Barcelona, Spain | January 2026 - August 2026*
- Contributed to SuperAgent, Aily's general-purpose customer-facing language-model agent, by connecting approved SQL data, documented business meaning, reusable agent capabilities, automated workflows, reports, and evaluation evidence.
- Progressed from Model Context Protocol (MCP) tools to task-scoped agent skills that packaged instructions, tools, and evaluation criteria. These later fed a governed semantic layer describing what approved tables meant, how they related, which agents could query them, and what access restrictions applied.
- Developed structured reporting outputs containing CSV artifacts, summaries, row counts, dates, and diagnostic metadata for Airflow-orchestrated workflows. Validators, integration tests, representative questions, and execution traces supported promotion from development through pre-production into production.
- Diagnosed regressions involving missing data, inaccurate semantic descriptions, resource discovery, tool choice, credentials, workflow inputs, and post-processing; maintained capabilities used by Aily tenants including Sanofi.

### Engineer I - ABS Consulting
*Warrington, United Kingdom | November 2024 - October 2025*
- Supported seismic qualification and structural assessment for safety-related assets within the United Kingdom's nuclear-power sector.
- Built and reviewed finite-element and engineering-calculation work using Abaqus, Mathcad, Autodesk Inventor, and STAAD.Pro, applying Eurocodes and British Standards.
- Produced reviewable calculations and billable technical reports in which assumptions, modelling choices, standards, and results remained traceable for internal and client review.

## SELECTED TECHNICAL PROJECT

### Gamma / FlowTrack Supply-Chain Visibility Demonstrator
*Collaborative Full-Stack Product Engineering Project | April 2026 - May 2026*
[Project repository](https://github.com/ieverythng/iTrack-Supply-Chain)
- Co-developed a modular supply-chain visibility demonstrator with a React/TypeScript frontend, shared validated data schemas, deterministic demonstration data, and a FastAPI/SQLAlchemy backend supporting SQLite and optional PostgreSQL.
- Created one frontend data interface that could switch between deterministic demonstration data and the local API. This allowed the team to validate persistent reads and updates, authentication, role-based access, and organisation-scoped audit records without breaking the standalone demonstration.
- Implemented or integrated shipment, document, purchase-order, tracking, notification, billing, portal, and agent-workbench surfaces with automated TypeScript, API, and browser-smoke validation.
- Designed external tracking and AI-assisted action paths around explicit evidence, signed provider callbacks, integration-status records, deterministic fallbacks, and human approval before consequential actions.
- Classified as a technical product project rather than academic research; live-provider coverage, durable agent incidents, and several production integrations remained bounded development work.

## RESEARCH METHODS AND TECHNICAL SKILLS

- **Machine learning and evaluation:** PyTorch, TensorFlow/Keras, scikit-learn, Proximal Policy Optimization, Generalized Advantage Estimation, FiLM conditioning, recurrent networks, reward design, ablations, held-out validation, trace-based evaluation, and error taxonomy.
- **Scientific and high-performance computing:** Python, MATLAB, C, CUDA, OpenMP, MPI, SLURM, Ray/RLlib, numerical modelling, finite-element analysis, and cluster job execution.
- **Language-model and agent systems:** Language-model planning, tool use, Model Context Protocol, semantic data layers, LiteLLM, llama.cpp, Ollama, OpenAI-compatible serving, structured outputs, reviewed action catalogues, and deterministic validation gates.
- **Robotics and perception:** ROS 2 Jazzy, ROS4HRI, NAO, the KnowledgeCore symbolic knowledge base, scene grounding, YOLO integration, action feedback, replanning, and simulated/physical validation.
- **Software and data engineering:** SQL, pandas, FastAPI, SQLAlchemy, React, TypeScript, Docker, Git/GitHub, CI/CD, Airflow, structured JSON/CSV artifacts, APIs, WebSockets, testing, observability, and technical documentation.
- **Languages:** English and Spanish (fluent); Hungarian (basic).
