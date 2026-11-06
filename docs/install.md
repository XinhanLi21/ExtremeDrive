# Installation

The toolkit supports Python 3.10–3.12. The CI workflow targets Linux with Python 3.10 and 3.12. Scoring uses CPU; planner inference has its own device requirements. Clone your hosted repository or extract the repository archive, then run from its root:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
extremedrive --assets assets verify
bash scripts/evaluation/run_smoke.sh
```

`assets/` is included in the repository and source release. It contains compact annotations and metadata, not raw sensor files. A code-only wheel still requires a separate path to these assets. No Git LFS or external simulator installation is required.

For development: `pip install -e '.[dev]'`. For the existing tested dependency snapshot, see `tested-environment.txt`. Benchmark rules are versioned independently of toolkit packaging.
