# Visual-Dynamic-3D-Path-Planning

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.12.2](https://img.shields.io/badge/python-3.12.2-blue.svg)](https://www.python.org/downloads/)
[![Platform: macOS](https://img.shields.io/badge/platform-macOS-lightgrey.svg)](https://github.com/)

> **Undergraduate Graduation Thesis | UESTC** > Optimal Path Planning in 3D Space Based on Dynamic Information Fields: A Case Study of Mai Xiaowen's Ability Model in _Delta Force_

## Introduction

This project is an undergraduate graduation thesis from the **School of Mathematical Sciences, University of Electronic Science and Technology of China (UESTC)**. The research focuses on efficient pathfinding in 3D dynamic environments.

Inspired by the tactical detection ultimate of the character "Mai Xiaowen" in the FPS game _Delta Force_, this project constructs a **Dynamic Information Field (DIF)** model. By enhancing the **A\* algorithm** with **incremental search** and **time-varying weighted graphs**, the system achieves real-time path optimization from a moving source to a target within a complex 3D voxel space based on live scanning data.

## Key Features

- **3D Voxel Modeling**: High-performance spatial representation using 3D voxel grids.
- **Dynamic Information Field**: A 110° sector-cone dynamic scanning volume simulating real-time detection mechanisms.
- **Optimized Path Planning**: Implementation of Any-Angle pathfinding (e.g., Theta\*) designed for point-mass models to generate smooth tactical guidance lines.
- **Real-time Simulation**: High-performance 3D visualization and algorithmic feedback loop.

---

## Project Structure (In Progress)

```text
├── LICENSE
├── README.md
├── requirements.txt
└── src
    └── modeling
        └── 3d_voxel.py
```

## Getting Started (Planned)

Instructional content for environment setup and running the simulation will be updated soon.

## License

This project is licensed under the [MIT LICENSE](./LICENSE).

## Contact

- **Author**: Gaoyang FENG
- **Institution**: University of Electronic Science and Technology of China (UESTC)
- **Department**: School of Mathematical Sciences
- **Email**: highsun910@gmail.com
