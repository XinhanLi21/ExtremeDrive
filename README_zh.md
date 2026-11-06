<div align="center">
<h1>ExtremeDrive</h1>
<h3>面向危急驾驶的自动驾驶数据集与评测基准</h3>
<p><a href="https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset">数据集</a> · <a href="docs/QUICKSTART_ZH.md">使用说明</a> · <a href="docs/agents.md">模型接口</a> · <a href="docs/PROTOCOL.md">评价指标</a> · <a href="README.md">English</a></p>
</div>

ExtremeDrive 面向危急驾驶及恶劣条件下的自动驾驶研究，将真实道路驾驶记录与道路结构、交通参与者标注相结合，评价环境约束和车辆动力学约束下的运动规划能力。

本工具库提供数据访问、规划模型接入、轨迹执行、指标计算及结果可视化功能。模型提交的轨迹通过统一控制器与车辆模型执行，用于比较规划结果及其对应的车辆行为。

## 基准概览

- **驾驶场景：** 覆盖危急操作及恶劣天气、光照条件，规划数据同时包含常规驾驶。
- **场景标注：** 提供与车辆运动对齐的道路标线、可行驶区域和动态交通参与者信息。
- **规划评价：** 支持轨迹误差，以及执行后的碰撞规避、道路合规、碰撞时间、行驶进度和舒适性评价。
- **方法比较：** 使用预定义数据划分、统一轨迹执行协议，并提供七种适配规划方法的参考结果。

训练、验证、测试集分别包含 758、89、341 个规划窗口；执行评测采用测试集中的 131 个窗口。详见[数据划分](docs/splits.md)与[评价协议](docs/PROTOCOL.md)。

## 快速开始

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
extremedrive --assets assets verify
bash scripts/evaluation/run_smoke.sh
```

示例使用随库提供的场景标注，在 CPU 上评测三条轨迹。打开 `results/smoke/index.html` 查看结果。

## 使用文档

1. [安装与使用](docs/QUICKSTART_ZH.md)
2. [数据下载](docs/dataset.md)
3. [数据格式与接口](docs/DATA_FORMAT.md)
4. [数据划分](docs/splits.md)
5. [规划模型接入](docs/agents.md)
6. [预测提交与评测](docs/submission.md)
7. [评价指标](docs/PROTOCOL.md)
8. [交互教程](tutorials/quickstart.ipynb)

## 参考结果

提供 Latent TransFuser、TCP、TransFuser、DiffusionDrive、PlanTF、PLUTO 和 Diffusion-Planner 的适配预测及参考评测结果。

```bash
extremedrive --assets assets reference --model pluto --out results/reference_pluto
```

`reference` 用于导出已有参考结果，`evaluate` 用于评测新的轨迹提交。结果来源及复核规则见[评测说明](docs/submission.md)。

## 开发与维护

```bash
pip install -e '.[dev]'
python -m unittest discover -s tests -v
python scripts/dev/build_release.py
```

参见[贡献指南](CONTRIBUTING.md)、[版本记录](CHANGELOG.md)和[发布说明](docs/releasing.md)。

## 数据来源与许可

原始数据由 Shiyue Zhao 与清华大学智能底盘团队整理，托管于 [Hugging Face](https://huggingface.co/datasets/Stary108/Extreme_Driving_Conditions_Dataset)，采用 CC BY-NC 4.0 许可。

工具库、源数据和第三方组件的许可与来源分别见 [LICENSE](LICENSE) 和 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
