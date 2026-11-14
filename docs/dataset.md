# Dataset download

Source: [Stary108/Extreme_Driving_Conditions_Dataset](https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset), curated by Shiyue Zhao and the Tsinghua University Intelligent Chassis Team.

## Access and authentication

Open the [dataset page](https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset), sign in to Hugging Face and accept its access conditions. Authenticate locally with a read-access token from an account with dataset access:

```bash
hf auth login
```

## Download and inspect

```bash
extremedrive --assets assets download --scope execution --dataroot data --dry-run
extremedrive --assets assets download --scope execution --dataroot data
extremedrive --assets assets inspect --dataroot data --split test
```

The downloader pins the revision in `assets/huggingface.json`, resumes through Hugging Face's cache, downloads per-episode archives and extracts the original directory layout. `HF_HOME` controls the cache; credentials are managed by `huggingface_hub`. If access is denied, verify that the authenticated account has accepted the dataset access conditions.

Scopes: `execution` selects 56 source episodes, `train`, `val`, and `test` follow benchmark splits, and `all` selects the 336 source episodes needed by those three splits. It does not mean every episode in the upstream dataset.

Existing data may be used directly with `--dataroot`; no redownload is necessary. The upstream data include camera/LiDAR streams, vehicle records and CoC annotations. The released planner interface selects current images and causal history; it does not pass future CoC or future actor labels to the agent.

Source data are CC BY-NC 4.0. The reviewed benchmark annotations and corrected labels in `assets/` are a separate versioned resource. Source folder train/val names must not be used to reconstruct benchmark splits.

## License, privacy and hosting

See [data governance](DATA_GOVERNANCE.md) for the source-data license, published privacy measures, DOI and hosting. The source data card governs dataset access; the repository [LICENSE](../LICENSE) distinguishes source data from project-specific code and annotations.
