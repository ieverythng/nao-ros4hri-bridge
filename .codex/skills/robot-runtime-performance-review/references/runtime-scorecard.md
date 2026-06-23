# Runtime Scorecard

Use this scorecard when the user asks for an overall rating.

Start at 10 and subtract:

- 2.0 for any duplicate spoken semantic event.
- 2.0 for raw planner/model failure text reaching the user.
- 1.5 for execution reported successful when observable success failed.
- 1.5 for grounded-context facts present but ignored on repeated turns.
- 1.0 for KB facts expiring or entity ids changing during normal follow-up
  dialogue.
- 1.0 for dialogue-only turns entering planner execution.
- 0.5 for repeated detector frame-skip warnings or slow KB materialisation.
- 0.5 for missing trace evidence that blocks root-cause separation.

Never score below 0. If the evidence is incomplete, cap the score at 7 until
the missing probe is collected.
