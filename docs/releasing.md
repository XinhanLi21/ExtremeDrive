# Current toolkit distribution

`main` is the single maintained distribution. Use its current code and documentation; no parallel release branches or version-tagged downloads are maintained. Git history is retained for reproducibility.

Install through Conda as described in [installation](install.md). Build the complete download with:

```bash
conda activate extremedrive
python scripts/dev/build_release.py
```

The archive has a stable name, `ExtremeDrive_Toolkit.zip`, with a SHA-256 checksum. It includes compact benchmark assets and the offline tutorial; raw sensor data and trained checkpoints are not included.

The manually triggered release workflow validates the toolkit and uploads only this archive and checksum. After a successful upload, older toolkit download artifacts are removed. CI retains its run logs without accumulating downloadable smoke-test artifacts. This workflow does not publish to PyPI or create a GitHub Release.

Protocol identities and asset hashes remain available for reproducibility. They identify experimental conditions, not alternative supported toolkit versions.
