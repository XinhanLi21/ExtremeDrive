# Data license, privacy and hosting

## Data license and access

The [source dataset](https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset) is distributed under [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/). Its Hugging Face data card is the reference for the source-data license and access conditions. Sign in and accept those conditions before downloading.

The source-data license does not assign a license to the toolkit's independently authored code or additional benchmark annotations. See the repository [LICENSE](../LICENSE) for the applicable scope.

## Privacy

According to the data card's [Privacy & Anonymization section](https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset#6ter-privacy--anonymization), faces and vehicle license plates in the released RGB streams are blurred. Source metadata provide velocity components and heading rather than absolute latitude and longitude; audio was not recorded. Depth and LiDAR arrays were not altered by the image-blurring process.

The publisher notes that occluded or low-resolution instances may escape detection. Report residual personal information through the dataset's contact channel so that a corrected release can be issued. These statements describe the source release, not a separate privacy audit of every toolkit illustration or locally supplied recording.

## Hosting and reproducibility

The data are hosted on Hugging Face with DOI [10.57967/hf/8637](https://doi.org/10.57967/hf/8637). The toolkit downloads the source revision recorded in `assets/huggingface.json`, using resumable caching and per-episode archives. The DOI identifies the dataset; the pinned revision identifies the files used by the toolkit.

Code and compact benchmark assets are distributed through this repository. See [dataset access](dataset.md) and [toolkit distribution](releasing.md). The current hosting arrangement does not specify a guaranteed retention period.

## CAN export

The aligned CAN export in `assets/can/` is distributed under the source dataset's CC BY-NC 4.0 license. It contains vehicle channels and timing metadata; no RGB images, absolute geographic coordinates or local filesystem paths are included. Source logger identifiers are represented by opaque IDs. See [CAN access and schema](can.md).
