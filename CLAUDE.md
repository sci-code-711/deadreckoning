# CLAUDE.md

Guidance for Claude Code when working in this repository.

## Project overview

Deadreckoning is a Python toolkit for reconstructing motion trajectories from
IMU (gyroscope + accelerometer) measurements, using direct integration and
Extended Kalman Filtering (EKF) with attitude correction from gravity vector
measurements.

- `deadrec/` — the installable package (quaternion math, connectors,
  transformers, runners, core pipeline).
- `test/` — pytest test suite.
- `Deadrec.py`, `EKF.py`, `EKF_fut.py`, `Calibration.py`, `Filter.py`,
  `Functions.py` — standalone scripts/prototypes at the repo root (not part
  of the `deadrec` package).
- `example_data/` — sample IMU data used by the scripts/notebooks.
- `*.ipynb` — exploratory notebooks; not linted or covered by tests.

## Tooling

- **Package management:** [uv](https://docs.astral.sh/uv/). Dependencies and
  dev tools are declared in `pyproject.toml`.
- **Linting:** [ruff](https://docs.astral.sh/ruff/).
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

# Run the test suite
uv run pytest
```

## Working conventions

- Always use `uv run <cmd>` (or `uv sync` first) rather than invoking
  `python`/`pytest`/`ruff` directly, so the correct locked environment is
  used.
- Run `uv run ruff check .` and `uv run pytest` before considering a change
  complete.
- The root-level `.py` scripts and notebooks are legacy/exploratory and
  carry pre-existing lint issues; don't fix unrelated lint findings outside
  the files you're already touching.
- `environment.yml` is legacy (conda-based); `pyproject.toml` + `uv` is the
  source of truth for dependencies going forward.
