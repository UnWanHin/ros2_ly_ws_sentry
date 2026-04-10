# Repository Guidelines

## Project Structure & Module Organization
This repository is a ROS2 workspace built with `colcon`.
- `src/`: all runtime packages (`gimbal_driver`, `detector`, `tracker_solver`, `predictor`, `behavior_tree`, `buff_hitter`, `outpost_hitter`, `shooting_table_calib`, `auto_aim_common`).
- `scripts/`: operational scripts (for example, `selfcheck.sh`, `start.sh`).
- `docs/`: contributor-facing documentation, organized by `architecture/`, `guides/`, `modules/`, `sentry/`, `reports/`, `rules/`.
- Generated artifacts: `build/`, `install/`, `log/` (do not commit).

## Build, Test, and Development Commands
- `colcon build`  
  Build all ROS2 packages in this workspace.
- `colcon build --packages-select detector behavior_tree gimbal_driver`  
  Faster iterative build for selected modules.
- `source install/setup.bash`  
  Load built packages into the current shell.
- `./scripts/selfcheck.sh sentry --skip-hz`  
  Static + graph contract checks (without frequency sampling).
- `./scripts/selfcheck.sh sentry --launch --wait 10`  
  Launch stack and run end-to-end runtime checks.
- `colcon test && colcon test-result --verbose`  
  Run package tests where available.

## Coding Style & Naming Conventions
- Primary languages: C++ (ROS2 nodes), Python (launch/scripts), YAML/JSON (config).
- Follow existing file-level style; do not reformat unrelated code.
- Use descriptive names aligned with existing patterns:
  - Topics: `/ly/<domain>/<name>` (snake_case).
  - Config keys: keep dot/slash compatibility when touching detector-related params.
- Avoid hardcoded hardware values (camera SN, device name, baud rate); keep them in YAML.

## Testing Guidelines
- Minimum before PR: targeted build for changed packages + `selfcheck.sh sentry --skip-hz`.
- For runtime/link changes, include one launched self-check run or explain why unavailable.
- If changing topics/messages/params, validate publisher-subscriber contracts and update docs.

## Commit & Pull Request Guidelines
- Recent history favors short, module-focused messages (often Chinese/English mixed), e.g. `posture 寫入bt樹`, `相機硬參數yaml化`.
- Recommended commit format: `<module>: <what changed>` (concise, imperative).
- PRs should include:
  - Scope and motivation
  - Affected packages/files
  - Verification commands and results
  - Config/doc updates (especially under `docs/`) when interfaces change.

## Security & Configuration Tips
- Never commit secrets, device-specific credentials, or local absolute paths.
- Keep runtime configuration centralized in `src/detector/config/auto_aim_config.yaml` and related launch parameters.

## Default Engineer Mode (Project)
- Default to the `$cautious-super-engineer` working style for all tasks unless the user explicitly overrides it.
- Understand architecture, data/call chain, and current behavior before making edits.
- Apply minimal local diffs only; do not refactor structure, move modules, or rename interfaces unless explicitly requested.
- Do not add broad defensive guards, fallback layers, retries, or compatibility shims unless a concrete failure mode is proven and approved.
- Do not perform chain/link rerouting or coverage-style rewiring unless explicitly required and validated.
- Preserve external contracts (topics, messages, APIs, config keys) unless interface change is the explicit task.
