# ROS2 相机驱动框架

一个模块化的 ROS2 相机驱动框架，采用基于插件的架构，支持工业相机。

## 概述

本仓库包含三个 ROS2 包，构成一个灵活的相机驱动框架：

- **camera_node**: 主 ROS2 节点，通过 pluginlib 动态加载相机驱动插件，管理参数，采集图像，并以 ROS2 话题发布。
- **huaray_camera**: 华睿工业相机的插件驱动，使用 MVSDK。
- **hik_camera**: 海康威视工业相机的插件驱动，使用 MvCameraControl SDK。

该框架提供了统一的相机操作接口 (`CameraBase`)，便于通过插件开发集成新的相机型号。

## 功能特性

- **基于插件的架构**: 运行时加载相机驱动，无需重新编译主节点。
- **多相机支持**: 目前支持华睿和海康威视工业相机。
- **自动恢复**: 守护线程监控相机状态，在停止时尝试重启。
- **灵活的参数配置**: 所有参数可通过 ROS2 参数或 YAML 文件配置。
- **ROS2 集成**: 以 `sensor_msgs/Image` 格式发布图像，附带相机信息。
- **图像旋转**: 发布前可选图像旋转（90°、180°、270°）。
- **触发模式**: 支持连续采集和外部硬件触发。
- **线程安全设计**: 独立的采集线程和守护线程，确保稳定运行。

## 软件架构

```
camera_node (main node)
├── PluginLoader (pluginlib)
├── CameraBase (abstract interface)
├── Capture Thread
├── Guard Thread
├── ROS2 Publishers
└── Parameter Server
```

**驱动插件**（实现 CameraBase）：
- `HuarayCamera` → 使用 MVSDK
- `HikCamera` → 使用 MvCameraControl SDK

## 依赖环境

- **ROS2**（推荐 Humble 或更高版本）
- **OpenCV** (4.x)
- **pluginlib** (ROS2)
- **CMake** (3.8+)
- **C++14** 编译器

**相机SDK依赖**：
- 华睿相机：MVSDK（位于 `huaray_camera/huaraySDK/`）
- 海康相机：MvCameraControl SDK（位于 `hik_camera/hikSDK/`）

## 安装与构建

### 1. 克隆仓库
```bash
git clone <repository-url>
cd camera_packages
```

### 2. 使用 colcon 构建
```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
```

### 3. 激活工作空间
```bash
source install/setup.bash
```

## 使用说明

### 启动相机节点
```bash
ros2 launch camera_node camera_launch.py
```

### 查看发布的话题
```bash
ros2 topic list
ros2 topic echo /image_raw
```

### 通过参数更改相机类型
修改 `camera_node/config/camera_params.yaml`：
```yaml
camera_type: "hik_camera"  # 或 "huaray_camera"
```

## 参数配置

主要参数（完整列表见 `camera_params.yaml`）：

| 参数 | 类型 | 默认值 | 描述 |
|------|------|--------|------|
| `camera_type` | string | `"hik_camera"` | 要加载的相机驱动插件 |
| `exposure_time` | double | `1000.0` | 曝光时间（微秒） |
| `gain` | double | `16.0` | 增益值 |
| `autocap` | bool | `true` | 自动采集模式（false 为外部触发） |
| `frame_rate` | double | `249.0` | 帧率（Hz） |
| `rotate_image` | bool | `false` | 启用图像旋转 |
| `rotation_code` | int | `-1` | 旋转代码：0=180°，1=90° 顺时针，2=90° 逆时针 |
| `interface_type` | int | `0` | 华睿相机：0=全部，1=GigE，2=USB3 |

## 插件开发指南

要为新的相机型号添加支持：

1. **创建一个新的 ROS2 包**，依赖 `camera_node` 和 `pluginlib`。
2. **实现 `CameraBase` 接口**（参见 `camera_node/include/camera_node/camera_base.hpp`）。
3. **重写所有纯虚方法**：
   - `Initialize()`：连接相机并开始采集。
   - `Stop()`：停止采集并释放资源。
   - `Read()`：捕获单帧图像。
   - `GetState()`：返回当前相机状态。
   - `SetParams()` / `GetParams()`：管理相机参数。
   - `GetImageWidth()` / `GetImageHeight()`：返回图像尺寸。
   - `SetFloatValue()` / `SetEnumValue()`：设置相机特定参数。
4. **使用 `PLUGINLIB_EXPORT_CLASS` 宏导出插件**。
5. **创建 `plugins.xml`** 描述插件类。
6. **更新 CMakeLists.txt** 以导出插件描述。

参考示例插件：`huaray_camera` 和 `hik_camera`。

## 许可证

本项目基于 Apache License 2.0 许可证。详见 `LICENSE` 文件。

## 贡献指南

欢迎贡献！请提交 Pull Request 或针对问题和功能请求创建 Issue。

---

# ROS2 Camera Driver Framework

A modular ROS2 camera driver framework with plugin-based architecture for industrial cameras.

## Overview

This repository contains three ROS2 packages that form a flexible camera driver framework:

- **camera_node**: Main ROS2 node that dynamically loads camera driver plugins via pluginlib, manages parameters, captures images, and publishes them as ROS2 topics.
- **huaray_camera**: Plugin driver for Huaray (华睿) industrial cameras using MVSDK.
- **hik_camera**: Plugin driver for Hikvision (海康威视) industrial cameras using MvCameraControl SDK.

The framework provides a unified interface (`CameraBase`) for camera operations, allowing easy integration of new camera models through plugin development.

## Features

- **Plugin-based architecture**: Load camera drivers at runtime without recompiling the main node.
- **Multi-camera support**: Currently supports Huaray and Hikvision industrial cameras.
- **Automatic recovery**: Guard thread monitors camera state and attempts to restart if stopped.
- **Flexible parameter configuration**: All parameters configurable via ROS2 parameters or YAML files.
- **ROS2 integration**: Publishes images as `sensor_msgs/Image` with camera info.
- **Image rotation**: Optional image rotation (90°, 180°, 270°) before publishing.
- **Trigger modes**: Supports both continuous acquisition and external hardware trigger.
- **Thread-safe design**: Separate capture thread and guard thread for robust operation.

## Architecture

```
camera_node (main node)
├── PluginLoader (pluginlib)
├── CameraBase (abstract interface)
├── Capture Thread
├── Guard Thread
├── ROS2 Publishers
└── Parameter Server
```

**Driver Plugins** (implement CameraBase):
- `HuarayCamera` → uses MVSDK
- `HikCamera` → uses MvCameraControl SDK

## Dependencies

- **ROS2** (Humble or later recommended)
- **OpenCV** (4.x)
- **pluginlib** (ROS2)
- **CMake** (3.8+)
- **C++14** compiler

**Camera SDK Dependencies**:
- Huaray cameras: MVSDK (provided in `huaray_camera/huaraySDK/`)
- Hikvision cameras: MvCameraControl SDK (provided in `hik_camera/hikSDK/`)

## Installation & Build

### 1. Clone the repository
```bash
git clone <repository-url>
cd camera_packages
```

### 2. Build with colcon
```bash
source /opt/ros/<distro>/setup.bash
colcon build --symlink-install
```

### 3. Source the workspace
```bash
source install/setup.bash
```

## Usage

### Launch camera node
```bash
ros2 launch camera_node camera_launch.py
```

### View published topics
```bash
ros2 topic list
ros2 topic echo /image_raw
```

### Change camera type via parameters
Modify `camera_node/config/camera_params.yaml`:
```yaml
camera_type: "hik_camera"  # or "huaray_camera"
```

## Parameters

Key parameters (see `camera_params.yaml` for full list):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_type` | string | `"hik_camera"` | Camera driver plugin to load |
| `exposure_time` | double | `1000.0` | Exposure time in microseconds |
| `gain` | double | `16.0` | Gain value |
| `autocap` | bool | `true` | Automatic capture mode (false for external trigger) |
| `frame_rate` | double | `249.0` | Frame rate (Hz) |
| `rotate_image` | bool | `false` | Enable image rotation |
| `rotation_code` | int | `-1` | Rotation code: 0=180°, 1=90° CW, 2=90° CCW |
| `interface_type` | int | `0` | For Huaray: 0=All, 1=GigE, 2=USB3 |

## Plugin Development

To add support for a new camera model:

1. **Create a new ROS2 package** depending on `camera_node` and `pluginlib`.
2. **Implement the `CameraBase` interface** (see `camera_node/include/camera_node/camera_base.hpp`).
3. **Override all pure virtual methods**:
   - `Initialize()`: Connect to camera and start acquisition.
   - `Stop()`: Stop acquisition and release resources.
   - `Read()`: Capture a single frame.
   - `GetState()`: Return current camera state.
   - `SetParams()` / `GetParams()`: Manage camera parameters.
   - `GetImageWidth()` / `GetImageHeight()`: Return image dimensions.
   - `SetFloatValue()` / `SetEnumValue()`: Set camera-specific parameters.
4. **Export the plugin** using `PLUGINLIB_EXPORT_CLASS` macro.
5. **Create `plugins.xml`** describing the plugin class.
6. **Update CMakeLists.txt** to export plugin description.

Example plugins: `huaray_camera` and `hik_camera`.

## License

Licensed under the Apache License 2.0. See `LICENSE` file for details.

## Contributing

Contributions are welcome! Please submit pull requests or open issues for bugs and feature requests.