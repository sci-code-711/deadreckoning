# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project overview

Deadreckoning is a Python toolkit for reconstructing motion trajectories from
IMU (gyroscope + accelerometer) measurements, using direct integration and
Extended Kalman Filtering (EKF) with attitude correction from gravity vector
measurements.

- `deadrec/` — the installable package:
  - `quaternion.py`, `kinematics.py`, `attitude.py` — attitude math and
    gravity-vector estimation.
  - `calibration.py`, `filtering.py` — sensor calibration and low-pass
    filtering.
  - `samples.py` — shared `ImuSample`/`TrajectoryState` data types.
  - `dead_reckoning.py`, `ekf.py` — trajectory reconstruction
    (`DeadReckoner`, `GravityCorrectedEKF`, `WindowedGravityCorrectedEKF`).
  - `io.py`, `cli.py` — CSV read/write and the `deadrec` command-line tool.
  - `core.py`, `connectors.py`, `transformers.py`, `runners.py`, `logger.py`
    — a multiprocessing streaming-pipeline scaffold, not yet wired up to
    the reconstruction code above.
- `test/` — pytest test suite.
- `example_data/` — sample IMU data used by tests.

## Tooling

- **Package management:** [uv](https://docs.astral.sh/uv/). Dependencies and
  dev tools are declared in `pyproject.toml`; the default Python version is
  pinned in `.python-version`.
- **Linting & formatting:** [ruff](https://docs.astral.sh/ruff/) (`ruff
  check` for lint, `ruff format` for formatting — both enforced in CI).
- **Testing:** [pytest](https://docs.pytest.org/).

## Common commands

```bash
# Install/sync the environment (creates .venv, installs deps + dev tools)
uv sync

# Add a runtime or dev dependency
uv add <package>
uv add --dev <package>

# Lint
uv run ruff check .

# Auto-fix lint issues where possible
uv run ruff check --fix .

# Format (and check formatting without writing changes)
uv run ruff format .
uv run ruff format --check .

# Run the test suite
uv run pytest
```

## Working conventions

- Always use `uv run <cmd>` (or `uv sync` first) rather than invoking
  `python`/`pytest`/`ruff` directly, so the correct locked environment is
  used.
- Run `uv run ruff check .`, `uv run ruff format --check .`, and
  `uv run pytest` before considering a change complete — all three are
  enforced in CI and must pass cleanly.
- `pyproject.toml` + `uv` is the sole source of truth for dependencies;
  there is no conda `environment.yml` anymore.
