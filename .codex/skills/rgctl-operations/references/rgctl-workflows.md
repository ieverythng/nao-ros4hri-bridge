# rgctl Workflow Notes

## Global command availability

```bash
mkdir -p ~/.local/bin
ln -sfn /Users/juanbendek/repos/RESEARCH-GLOBAL/.venv/bin/rgctl ~/.local/bin/rgctl
command -v rgctl
```

`~/.local/bin` must be on PATH.

## Common commands

```bash
rgctl list
rgctl list --project nao-ros4hri-bridge
rgctl show RG-0016
rgctl validate tasks/backlog/RG-0016-*.yaml
```

## Task create pattern

```bash
rgctl create RG-0016 \
  --project nao-ros4hri-bridge \
  --priority high \
  --title "Neural Workbench semantic AB0 registry rollout" \
  --objective "Track and validate semantic AB0 decomposition rollout."
```

## Sync pattern

```bash
rgctl sync --preflight
rgctl sync --task-id RG-0016 --dry-run --direction two-way
rgctl sync --task-id RG-0016 --direction two-way
```

## GitHub repo/auth diagnostics

```bash
gh auth status -h github.com
gh repo view juanbendek-aily/RESEARCH-GLOBAL --json nameWithOwner,url
```

If a repo slug is unreachable, use the reachable slug in the git remote and rerun preflight.
