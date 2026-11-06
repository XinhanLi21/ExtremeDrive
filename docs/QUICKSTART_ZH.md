# ExtremeDrive Toolkit 使用说明

本工具包将目前的 benchmark 整理为独立的安装、数据读取、预测提交和评分流程。保留原划分、0 cm 白黄线规则、SL03 近似车身轮廓、LQR 与车辆数据拟合动力学参数，不需要依赖原服务器上的实验目录。

## 1. 安装

解压完整发布包，进入目录：

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
extremedrive --assets assets verify
```

建议 Python 3.10–3.12。评分在 CPU 上运行，模型推理按各自 GPU 要求配置。代码 wheel 不含传感器和评测资产；使用 wheel 时仍需提供完整包中的 `assets` 目录。

## 2. 连接 Shiyue Zhao 整理的原始数据

数据地址：<https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset>

工具包已记录数据版本，不使用随时变化的 main 分支。原始数据采用逐 episode 的 tar 存储，下载命令会还原目录结构。

```bash
# 查看当前执行评测需要下载的 13 个原始片段
extremedrive --assets assets download --scope execution --dataroot /path/to/data --dry-run
# 下载并解压
extremedrive --assets assets download --scope execution --dataroot /path/to/data
# 下载固定 train/val/test 所需要的 336 个原始片段
extremedrive --assets assets download --scope all --dataroot /path/to/data
# 检查图像文件是否齐全
extremedrive --assets assets inspect --dataroot /path/to/data --split test
```

`--scope train/val/test` 可单独选择一个 benchmark 划分。`all` 指三个固定划分所需数据，并非上游全库所有片段。已有本地数据可直接使用 `/data/Extreme_Driving_Conditions_Dataset`，无需重新下载。原始数据许可为 CC BY-NC 4.0。

上游的 CoC、车辆记录与传感器数据保留在各 episode 中。本工具包另带人工修正地图、可行驶面、动态目标及修正后的轨迹标签；这些资产不会被原始数据下载覆盖。原数据目录的 train/val 不能替代 benchmark 的 train/val/test。

## 3. 接入模型

编写一个类，提供 `predict(inputs)`，返回 8×3 数组，每行是未来的 `[x, y, yaw]`，对应 0.5 至 4.0 秒。x 向前、y 向左，角度单位为 rad，使用当前自车坐标。

```python
class Agent:
    def predict(self, inputs):
        # inputs: images、language、history、history_valid、token
        return model_predict(inputs)  # 8 × 3
```

```bash
extremedrive --assets assets predict --agent your_module:Agent \
  --dataroot /path/to/data --execution-only --out predictions.jsonl
extremedrive --assets assets validate --submission predictions.jsonl
extremedrive --assets assets evaluate --submission predictions.jsonl --workers 4 --out results/my_model
```

移除 `--execution-only` 可生成完整 test 的 341 个窗口预测；用 `displacement` 计算 ADE/FDE。执行评分默认要求完整 131 个窗口；调试少数窗口必须显式加 `--subset`。不会开始训练模型。

## 4. 不下载图像，先运行完整执行链路

```bash
extremedrive --assets assets evaluate \
  --submission examples/smoke_predictions.jsonl --subset --workers 3 --out results/smoke
```

示例采用三个已有 PLUTO 预测，真实重新执行 LQR、执行器与动力学仿真，然后评分。不是读取预存总分。直接打开 `results/smoke/index.html` 可看分项结果。

## 5. 查看已有七模型结果

```bash
extremedrive --assets assets reference --model pluto --out results/reference_pluto
```

可替换为 latent_transfuser、tcp、transfuser、diffusiondrive、plantf、diffusion_planner。此入口是历史复核结果的验证和导出，不是重新推理。已核验的 TTC 人工判断绑定原模型预测和结果，不能沿用到新的预测。新评测仍缺证据时会保留 unknown 与总分范围。

## 6. 评分范围

- 碰撞、白黄线合规、可行驶面及行驶方向、TTC、进度、危急操作安全性。
- 舒适性仅排除真正的危急驾驶；雪天、低照度仍计算。
- 不包含 EC、交通信号灯规则或偏离记录轨迹后的交互式后继场景。
- 131 个窗口是 test 的执行评测集合，不是 131 个独立原始片段。

具体阈值见 `assets/protocol.json`，定义见 [PROTOCOL.md](PROTOCOL.md)。`assets/manifest.json` 提供全部评测资源校验；原固定划分文件保留原字节，读取时才转换路径。

## 7. 导出 BEV 检查图

```bash
extremedrive --assets assets visualize \
  --result results/smoke/scores/11b8fad23a415420_000001_adjacent.json --out bev.png
```

图中包含道路、起始时刻的目标框、自车执行轨迹和沿途车身轮廓。周车仅显示起始时刻快照。


## 0.4.1 无损压缩更新

131 个窗口的标注由约 121 MiB 压缩至约 5.1 MiB。完全相同的地图与道路几何仅保存一次，窗口引用对应内容；其余数据采用 LZMA/XZ 压缩。没有删帧、降低坐标精度或简化线形。

逐窗口还原后的规范 JSON 哈希与 0.4.0 全部一致。加载时也自动校验共享数据和完整窗口的哈希。原命令、数据划分及评分规则不变，旧格式仍可读取。
