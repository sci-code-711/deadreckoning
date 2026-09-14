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
  `uv run pytest` before considering a change complete.
- The root-level `.py` scripts and notebooks are legacy/exploratory and
  carry pre-existing lint issues; don't fix unrelated lint findings outside
  the files you're already touching.
- `pyproject.toml` + `uv` is the sole source of truth for dependencies;
  there is no conda `environment.yml` anymore.
