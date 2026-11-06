# ExtremeDrive Toolkit

面向危急驾驶的规划与车辆执行评测工具库。

**流程：传感器输入 → 规划轨迹 → LQR → 执行器 → 车辆动力学 → 评分。**

- 原训练 / 验证 / 测试划分保持 758 / 89 / 341 个窗口。
- 当前执行评测覆盖 test 中的 131 个窗口，统一汇总危急驾驶、低照度与雪天。
- 车道、道路与动态目标标注已无损压缩为约 5.1 MiB。
- 白黄线合并合规、0 cm 压线规则、SL03 轮廓、现有车辆参数与评分机制保持不变。

## 快速开始

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e .
extremedrive --assets assets verify
bash scripts/evaluation/run_smoke.sh
```

打开 `results/smoke/index.html` 查看执行结果。此示例实际运行控制器和动力学，不需要 GPU 或下载原始图像。

## 使用文档

- [中文完整操作说明](docs/QUICKSTART_ZH.md)
- [数据下载](docs/dataset.md)：直接连接 Shiyue Zhao 整理的 Hugging Face 数据。
- [模型接口](docs/agents.md)：接入自己的规划器。
- [提交及评分](docs/submission.md)：预测校验、完整评测和结果导出。
- [数据划分](docs/splits.md)、[评分协议](docs/PROTOCOL.md)、[交互教程](tutorials/quickstart.ipynb)。
- [GitHub 发布操作](docs/releasing.md)。

原始数据和 benchmark 修正标注分别管理。历史人工复核仅绑定原预测，新模型不能自动继承。完整 test 可算轨迹误差，当前完整执行标注集合为 131 个窗口。

## 开发

```bash
pip install -e '.[dev]'
python -m unittest discover -s tests -v
python scripts/dev/build_release.py
```

GitHub Actions 已配置自动测试和安装包构建。仓库未预填虚构的远程地址、论文信息或排行榜链接。许可与来源见 [LICENSE](LICENSE) 和 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
