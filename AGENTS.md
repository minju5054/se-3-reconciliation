# Research Repository Rules

These rules apply to future work in this repository unless the user explicitly says
otherwise for a specific task.

- Preserve raw and derived data separately. Derived artifacts must identify their raw
  inputs and processing configuration.
- Never silently transform coordinates. Record and document every source frame, target
  frame, transform, axis convention, and unit.
- Never overwrite raw VLA outputs.
- Record coordinate frames and observation, readiness, and execution timestamps
  explicitly.
- Keep experiments reproducible through versioned configuration, deterministic fixtures,
  documented commands, and append-only work logs.
- Write tests for every SE(2) operation and timing convention.
- Synthetic fixtures are tests and demonstrations only. Never describe them as
  experimental evidence.
- Do not implement future research stages, including correspondence factors or graph
  optimization, unless explicitly requested.
- Do not vendor or modify LightNav in this repository.
- Do not commit model weights, large datasets, raw experiment recordings, Isaac Sim
  caches/data, virtual environments, or source from external repositories.
- Before edits, inspect Git status, branch, and remotes. Preserve unrelated user changes.
- After a logical task, run the strongest relevant tests, append `docs/WORK_LOG.md`, review
  the diff and staged diff, make one focused commit, and normally push the current branch.
