# Provenance gaps

The comparison preserves the authoritative questionnaire JSON, startup snapshots, image identity, endpoint inventory, derived CSV/JSONL dataset, and seam audit.

Launch logs were inspected live during each model startup. The container was reused between cells, so the transient launch-log files for the earlier model cells were not all copied into the final evidence directory before the container switched models. The startup snapshots and per-case JSON retain the preflight state, structured trace summaries, speech observations, failure markers, and case-level excerpts needed for the reported attribution.

This gap does not change the semantic scores. It limits retrospective log-level inspection of the earliest startup messages. A future rerun should copy the launch log immediately after each startup and include its SHA-256 in the comparison manifest.
