# BSC Motivation Letter Kit

This folder contains one reusable motivation letter and four letters tailored to
the BSC vacancies reviewed on 28 July 2026.

## Recommended Order

1. `02_18726_AI_AGENT_TOOL_USE_RE2.md`
2. `03_33626_GENERATIVE_AI_APPLICATIONS_RE3.md`
3. `04_9026_HPC_ML_APPLICATIONS_RE3.md`
4. `05_17926_HYPERSONIC_CFD_PHD.md`

The first application is the closest match to Juan's demonstrated MCP,
tool-use, agent-evaluation, Python, and Spanish-language profile. The remaining
letters deliberately distinguish relevant experience from capabilities that
are still being developed.

## Before Submission

1. Confirm both listed referees' consent and contact details.
2. Recheck the vacancy status, reference number, and closing date on the BSC
   website.
3. Update the date if the letter is submitted after 28 July 2026.
4. Attach the English CV requested by BSC.
5. Keep the role-specific subject line. Do not submit the master letter when a
   tailored version is available.
6. Export again by running:

   ```bash
   python3 render_letters.py
   ```

The personal projects are intentionally described as unpublished or
experimental. Do not remove those maturity labels unless their status changes.

## Files

- `00_APPLICATION_STRATEGY.md`: fit analysis and claim boundaries.
- `01_MASTER_MOTIVATION_LETTER.md`: reusable base letter.
- `02_18726_AI_AGENT_TOOL_USE_RE2.md`: Language Modeling Team application.
- `03_33626_GENERATIVE_AI_APPLICATIONS_RE3.md`: AI Factory application.
- `04_9026_HPC_ML_APPLICATIONS_RE3.md`: Wave Phenomena application.
- `05_17926_HYPERSONIC_CFD_PHD.md`: LS/CFD PhD application.
- `pdf/`: rendered application-ready PDFs.
