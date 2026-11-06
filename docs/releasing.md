# GitHub repository and release

This is a standalone Git repository. Choose the owning GitHub account/organization and create an empty repository; add its actual URL as `origin`. No remote address is embedded in the toolkit.

```bash
git remote add origin <your-repository-url>
git push -u origin main
```

These commands publish the repository. Check the intended visibility and project-specific license before public distribution. Third-party licenses and the upstream dataset license are already recorded; a project open-source license has not been selected automatically.

CI runs tests, verifies assets, executes regression cases and builds the Python distributions. Pushing a version tag or manually running the release workflow builds downloadable artifacts; it does not publish to PyPI or automatically create a public GitHub Release.

```bash
conda activate extremedrive
python -m build
python scripts/dev/build_release.py
```

The wheel is code-only. The full source archive includes compact benchmark assets and the offline tutorial. The files in `dist/` can be attached to a GitHub Release after the repository owner chooses to publish it. No trained checkpoint or raw sensor data is included.
