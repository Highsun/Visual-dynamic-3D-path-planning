# Visual-dynamic-3D-path-planning

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.12.2](https://img.shields.io/badge/python-3.12.2-blue.svg)](https://www.python.org/downloads/)
[![Platform: macOS](https://img.shields.io/badge/platform-macOS-lightgrey.svg)](https://github.com/)

> **电子科技大学本科毕业设计** > 基于动态信息域的三维空间最优路径规划算法——以《三角洲行动》中“麦晓雯”技能模型为例

## 介绍

这是一个电子科技大学数学专业的本科毕业设计课题。本项目旨在研究三维动态环境下的高效寻路问题。受国产 FPS 游戏《三角洲行动》中信息位角色“麦晓雯”大招探测技能的启发，本项目构建了一个**动态信息域**模型。

通过改进 A\* 算法，结合**增量式搜索**与**时变加权图**，实现在复杂 3D 体素空间中，根据实时扫描信息动态优化从移动源到目标的指引路径。

### 核心特性

- **3D 体素模型**: 基于体素网格的高性能三维空间建模。
- **动态信息域**: 110° 扇形锥面动态扫描体，模拟实时探测机制。
- **优化路径规划**: 针对质点模型设计的 Any-Angle 寻路优化，生成顺滑的玩家指引线。
- **高性能实时渲染**: 实现 3D 实时仿真与可视化。

---

## 项目结构（更新中）

```text
├── README.md
├── requirements.txt
└── src
    └── env
        └── voxel_env.py
```

---

## 快速开始（计划中）

---

## 研究计划（计划中）

---

## 许可证

本项目遵循 [MIT LICENSE](./LICENSE) 。

---

## 联系方式

- **Author**: Gaoyang FENG
- **Institution**: University of Electronic Science and Technology of China (UESTC)
- **Email**: highsun910@gmail.com
