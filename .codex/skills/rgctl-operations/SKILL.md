---
name: rgctl-operations
description: Manage RESEARCH-GLOBAL tasks with rgctl, including project-scoped task creation/updates, log entries, and GitHub sync preflight/dry-run/live checks. Use when the user asks to update rgctl tasks, sync tasks to GitHub, validate task YAML, or check cross-repo task status.
---

# rgctl Operations

## Quick start

1. Use `rgctl` from any directory (`command -v rgctl`).
2. Validate and inspect tasks before edits.
3. Sync with `--preflight`, then `--dry-run`, then live sync.

## Core workflow

1. Confirm repo and tool health:
   - `cd /Users/juanbendek/repos/RESEARCH-GLOBAL`
   - `rgctl doctor`
   - `rgctl list --project <project-name>`
2. Create or update tasks:
   - create: `rgctl create RG-XXXX --project <project> --title "..." --objective "..."`
   - claim: `rgctl claim RG-XXXX --agent codex`
   - status: `rgctl status RG-XXXX active|review|done|blocked`
   - log: `rgctl log RG-XXXX --agent codex --message "..."`
3. Validate YAML after edits:
   - `rgctl validate tasks/backlog/<file>.yaml`
4. Sync safely:
   - `rgctl sync --preflight`
   - `rgctl sync --task-id RG-XXXX --dry-run --direction two-way`
   - `rgctl sync --task-id RG-XXXX --direction two-way`

## Safety checks

- Prefer `--task-id` scoped sync to avoid accidental bulk issue creation.
- Keep `project` set explicitly for cross-repo work (for example `nao-ros4hri-bridge`).
- If sync reports repo/auth errors, verify remotes and `gh auth status` before retrying.
- If parsing/sync behavior differs by `gh` version, patch `rgctl/github_sync.py` and rerun targeted tests.

## References

- See [references/rgctl-workflows.md](references/rgctl-workflows.md) for command recipes.
