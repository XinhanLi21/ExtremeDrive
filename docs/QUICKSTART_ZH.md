# ExtremeDrive Toolkit 使用说明

ExtremeDrive Toolkit 提供数据读取、规划模型接入、轨迹执行与评测接口。本指南介绍环境配置、数据准备和模型评测流程。

## 1. 安装

解压完整发布包，进入目录：

```bash
conda env create -f environment.yml
conda activate extremedrive
extremedrive --assets assets verify
```

使用 Conda 管理运行环境；`environment.yml` 默认创建 Python 3.12 环境，并在环境内安装工具库及依赖。评分在 CPU 上运行，模型推理按各自 GPU 要求配置。代码 wheel 不含传感器和评测资产；使用 wheel 时仍需提供完整包中的 `assets` 目录。

## 2. 数据准备

数据地址：<https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset>

原始数据由 Shiyue Zhao 与清华大学智能底盘团队整理。下载工具使用指定的数据版本，按 episode 下载并解压到统一目录。

```bash
# 查看当前执行评测需要下载的 56 个原始片段
extremedrive --assets assets download --scope execution --dataroot /path/to/data --dry-run
# 下载并解压
extremedrive --assets assets download --scope execution --dataroot /path/to/data
# 下载固定 train/val/test 所需要的 336 个原始片段
extremedrive --assets assets download --scope all --dataroot /path/to/data
# 检查图像文件是否齐全
extremedrive --assets assets inspect --dataroot /path/to/data --split test
```

`--scope train/val/test` 可单独选择一个 benchmark 划分。`all` 指三个固定划分所需数据，并非上游全库所有片段。已有本地数据可通过 `--dataroot` 指定其根目录。原始数据许可为 CC BY-NC 4.0。

上游的 CoC、车辆记录与传感器数据保留在各 episode 中。本工具包另带人工修正地图、可行驶面、动态目标及修正后的轨迹标签；这些资产不会被原始数据下载覆盖。原数据目录的 train/val 不能替代 benchmark 的 train/val/test。

## 3. 接入模型

编写一个类，提供 `predict(inputs)`，返回 8×3（2 Hz）或 16×3（4 Hz）数组，每行是未来的 `[x, y, yaw]`，覆盖 4.0 秒。x 向前、y 向左，角度单位为 rad，使用当前自车坐标。

```python
class Agent:
    def predict(self, inputs):
        # inputs: images、language、history、history_valid、token
        return model_predict(inputs)  # 按 inputs["target_times_s"] 返回轨迹
```

```bash
extremedrive --assets assets predict --agent your_module:Agent \
  --dataroot /path/to/data --execution-only --out predictions.jsonl
extremedrive --assets assets validate --submission predictions.jsonl
extremedrive --assets assets evaluate --submission predictions.jsonl --workers 4 --out results/my_model
```

移除 `--execution-only` 可生成完整 test 的 2,574 个片段预测；用 `displacement` 计算 ADE/FDE。执行评分默认要求完整 2,574 个片段；调试少数窗口必须显式加 `--subset`。

## 4. 运行评测示例

```bash
extremedrive --assets assets evaluate \
  --submission examples/smoke_predictions.jsonl --subset --workers 3 --out results/smoke
```

示例使用三条 PLUTO 预测，依次执行轨迹跟踪、车辆仿真和指标计算。打开 `results/smoke/index.html` 查看分项结果。

## 5. 参考结果

```bash
extremedrive --assets assets reference --model pluto --out results/reference_pluto
```

可替换为 latent_transfuser、tcp、transfuser、diffusiondrive、plantf、diffusion_planner。该命令校验并导出随库提供的参考结果。参考结果中的人工复核记录绑定对应预测；新提交按自动评价流程评分。各指标的适用范围、判定状态与汇总方式见评测协议。

## 6. 评分范围

- 碰撞、白黄线合规、可行驶面及行驶方向、TTC、进度、危急操作安全性。
- 舒适性仅排除真正的危急驾驶；雪天、低照度仍计算。
- 不包含 EC、交通信号灯规则或偏离记录轨迹后的交互式后继场景。
- 2,574 个片段是 test 的执行评测集合，来自 56 段原始记录，不是独立采集的 2,574 个场景。

具体阈值见 `assets/protocol.json`，定义见 [PROTOCOL.md](PROTOCOL.md)。`assets/manifest.json` 提供全部评测资源校验；场景归属不变，片段按文件名每隔 0.25 秒提取；读取时转换数据路径。

## 7. 导出 BEV 检查图

```bash
extremedrive --assets assets visualize \
  --result results/smoke/scores/11b8fad23a415420_000001_adjacent.json --out bev.png
```

图中包含道路、起始时刻的目标框、自车执行轨迹和沿途车身轮廓。周车仅显示起始时刻快照。


## 8. 数据存储

评测标注采用共享地图与无损压缩存储。数据接口在加载时还原场景并校验完整性；使用方式与未压缩标注一致。存储格式见 [DATA_FORMAT.md](DATA_FORMAT.md)。

## 9. 环境维护

在仓库根目录执行：

```bash
conda env update -n extremedrive -f environment.yml
conda activate extremedrive
```

开发依赖可在激活环境后通过 `python -m pip install -e '.[dev]'` 安装。也可使用 `conda run -n extremedrive extremedrive --assets assets verify` 直接运行命令。

## 10. 2 Hz / 4 Hz 配置

`predict` 支持 `--input-hz 2|4` 与 `--trajectory-hz 2|4`，默认均为 2 Hz。前者控制历史输入采样，后者控制预测轨迹网格。`validate`、`evaluate`、`displacement` 可从提交的时间戳自动识别轨迹频率，也可显式指定 `--trajectory-hz` 校验。详见 [sampling.md](sampling.md)。
