# Nested Repository Manifest

Capture date: 28 July 2026

The root repository ignores these independently versioned source trees. Their
exact Git identities are recorded because several are copied into the frozen
container image.

| Repository | Branch | Commit | Publication state |
| --- | --- | --- | --- |
| `src/chatbot_llm` | `feat/planner_llm_hooks` | `a1cddc2cf100ac9e1f7a33c1b67d55cd7bf48e37` | Pushed to `ieverythng/nao_chatbot_llm` |
| `src/dialogue_manager` | `juan-feat-1` | `336211fc9070d14dd7652ddb300050328966e909` | Pushed to `ieverythng/dialogue_manager` |
| `src/emorobcare_cv_object_detection` | `codex/v2-detector-contract-hardening` | `1fb786f7444f0c364d57aa227011e7bc9a29622f` | Local commit; configured EMOROBOCARE remote is inaccessible to the current GitHub identity |
| `src/emorobcare_cv_msgs` | `codex/v2-interface-contracts` | `208ce07b01ff8982775adbd0b95411779913eb26` | Local commit; configured EMOROBOCARE remote is inaccessible to the current GitHub identity |
| `src/Neural-Wokbench` | `codex/ignore-generated-skill-cache` | `5983a6e1f53c47b6980a1f0afdeda398d5b9bb35` | Pushed to `juanbendek-aily/Neural-Wokbench` |
| `src/motions_skills` | `main` | `e90ff7e3fccac7261d6ded3c384bd5843b45038e` | Clean worktree; existing branch divergence (`ahead 14, behind 1`) was not rewritten or pushed |

## Validation

- `chatbot_llm`: 66 focused planner-adapter and Ollama transport tests passed
  inside `iiia:nao-final`.
- `dialogue_manager`: 27 planner-act and Say-client tests passed inside
  `iiia:nao-final`.
- `emorobcare_cv_msgs`: clean ROS 2 Jazzy interface build with system Python.
- `emorobcare_cv_object_detection`: Python entrypoints and launch files compile;
  its ament lint tests are unavailable on the host and the package remains v2
  experimental work.

The two EMOROBOCARE commits are not part of the frozen v1 capability claim.
They are retained as v2 groundwork until an authorized remote or fork is
provided.
