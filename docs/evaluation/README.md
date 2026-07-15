# TFM Runtime Evidence

This directory contains immutable, hash-addressed runtime evidence used by the
thesis. Each run directory contains the original artifacts under `raw/`, a
manifest with source hashes and provenance, normalized per-case rows, and a
small aggregate. Historical, diagnostic, and final evidence are not pooled.

Generate a bundle from the repository root with:

```bash
python3 scripts/build_tfm_evidence_bundle.py \
  --run-id <run_id> \
  --classification <final|historical|diagnostic|source_only> \
  --metadata-json <metadata.json> \
  --artifact <questionnaire.json>
```

Files larger than one MiB are stored as deterministic gzip streams. The
manifest records hashes for both the original source and the bundled copy.
`aggregate_metrics.json` preserves the harness verdicts present in the source
artifacts. It must not be interpreted as a semantic or physical score unless
the corresponding case manifest and evidence depth support that claim.

## Evidence classes

- `final`: complete frozen run with source, image, model, prompt, registry, and
  fixture provenance.
- `historical`: accepted prior runtime evidence with known provenance limits.
- `diagnostic`: focused or adversarial evidence that does not define a complete
  run-level denominator.
- `source_only`: tests or source inspection without a rebuilt live run.

The final thesis run remains pending until the repository revision, nested
revision, container image, model configuration, prompt packs, skill registry,
fixtures, and questionnaires are frozen together.
