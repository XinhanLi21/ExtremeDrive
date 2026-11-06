# Fixed splits

| Split | Windows | Interface |
|---|---:|---|
| train | 758 | Agent inputs and supervised trajectory targets |
| val | 89 | Agent inputs and trajectory evaluation |
| test | 341 | Agent inputs and full-split ADE/FDE |
| execution cohort | 131 | Fixed subset of test with released execution annotations |

The original split files are preserved byte-for-byte. The toolkit remaps sensor paths at access time. `assets/cohort.json` fixes the execution tokens. Several windows may belong to one episode or map; these counts are not independent scene counts.

Use test labels only for evaluation. Any model's prior exposure to source episodes must be disclosed independently: benchmark split labels do not erase pretraining overlap.

The execution cohort combines critical driving, low light and snow into one aggregate table. Comfort applies to the noncritical windows; emergency-safety success applies to critical-driving windows. Scope and applicability denominators are present in output metadata.
