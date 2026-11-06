# Installation

Conda manages the runtime environment. The supplied `environment.yml` selects Python 3.12 and installs the toolkit in editable mode inside that environment. The toolkit supports Python 3.10–3.12. The CI workflow targets Linux with Python 3.10 and 3.12. Scoring uses CPU; planner inference has its own device requirements. Clone your hosted repository or extract the repository archive, then run from its root:

```bash
conda env create -f environment.yml
conda activate extremedrive
extremedrive --assets assets verify
bash scripts/evaluation/run_smoke.sh
```

`assets/` is included in the repository and source release. It contains compact annotations and metadata, not raw sensor files. A code-only wheel still requires a separate path to these assets. No Git LFS or external simulator installation is required.

For development: `pip install -e '.[dev]'`. The runtime is defined by `environment.yml` and `pyproject.toml`. Benchmark rules are versioned independently of toolkit packaging.

## Environment management

Run environment commands from the repository root. Python and pip are managed by Conda; package dependencies are installed by pip inside the `extremedrive` environment according to `pyproject.toml`.

```bash
# Apply environment updates after pulling repository changes.
conda env update -n extremedrive -f environment.yml
conda activate extremedrive

# Run a command without activating the environment in your shell.
conda run -n extremedrive extremedrive --assets assets verify
```

For development, activate the environment before `python -m pip install -e '.[dev]'`. The CI and release workflows use the same Conda environment file. Raw data and evaluation assets remain outside the environment.
