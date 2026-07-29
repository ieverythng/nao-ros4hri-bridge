# v1 Frozen Runtime Provenance

Capture date: 28 July 2026

## Frozen image

- canonical tag: `iiia:nao-final`
- image ID: `sha256:bb82c158092a69870dae48272b3b20fd8cd4ecfe97a0e61a2253a2cca00c663e`
- previous validation tag: `iiia:nao-runtime-v34-final-frozen-review`
- architecture: `linux/amd64`
- created: 19 July 2026 at 14:38:02 UTC
- uncompressed local size reported by Docker: 10,132,060,674 bytes

The active `nao_ros2` container was running this exact image ID when the
provenance was captured. Removing the previous tag does not stop or modify the
running container.

## Source identity

The retained workspace and the files installed in the frozen image have
matching SHA-256 hashes for:

- `planner_llm/planner_engine.py`
- the focused planner-engine test file
- `nao_chatbot/stack_launch.py`
- `planner_common/report_outcome.py`
- `planner_common/target_selection.py`

These files cover the final planner hardening, launch wiring, truthful outcome
reporting, and target-selection contracts. The root release tag records the
complete Git identity after all freeze documentation and evidence is committed.

## Files

- `docker_image_identity.json`: bounded Docker image metadata.
- `active_container_identity.json`: active container image and network identity,
  without environment variables or secrets.
- `docker_history_iiia_nao_final.txt`: image layer history.
- `docker_image_inventory_before.txt`: full local inventory before cleanup.
- `docker_image_inventory_after.txt`: retained inventory after cleanup.
- `workspace_runtime_hashes.sha256`: selected workspace hashes.
- `image_runtime_hashes.sha256`: corresponding hashes inside the frozen image.

## Retention decision

Only `iiia:nao` and `iiia:nao-final` are retained as named local images.
`iiia:nao` remains the rebuild base. `iiia:nao-final` is the thesis and
presentation runtime. Historical image tags and unused intermediate images are
removed after this inventory is written.

This record establishes software and container provenance. It does not extend
the thesis claims to unqualified physical navigation, manipulation, perception
accuracy, or safety performance.
