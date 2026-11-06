# Source attribution and redistribution

- **Original Extreme Driving Conditions Dataset:** Tsinghua University Intelligent Chassis Team; source curated by Shiyue Zhao. Repository: https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset ; project: https://sean-shiyuez.github.io/extreme-driving-dataset-web/ . Upstream card declares CC BY-NC 4.0. The downloader fetches upstream data under that license; the release archive does not redistribute raw camera/LiDAR data.
- **LQR implementation:** NAVSIM, https://github.com/autonomousvision/navsim , commit `c0b6fb05d9060f665dc98830285fe236312dc2ba`. Selected dependency modules are included; import namespaces alone were relocated. License: `extremebench/_vendor/NAVSIM_LICENSE`. Original and packaged SHA-256 values: `extremebench/_vendor/manifest.json`.
- **Vehicle/time data structures:** nuplan-devkit 1.2.0, https://github.com/motional/nuplan-devkit . Selected dependency modules are included, with import namespaces relocated. License: `extremebench/_vendor/NUPLAN_LICENSE`.
- **Workflow references:** nuScenes devkit and NAVSIM. Their names describe references and attribution, not endorsement of ExtremeDrive results.

The ExtremeDrive-specific code and reviewed annotation assets are supplied to the project owner as a research toolkit. This packaging step does not grant a new public redistribution license for project-specific materials. No repository or dataset has been publicly uploaded by this task. Keep third-party notices when redistributing permitted components.
