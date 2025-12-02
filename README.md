# heartbeat
![3D Heart Animation](heartbeat/heart_animation.gif)
# 3D 爱心动画项目说明

## 1. 项目简介
这个项目实现了一个 3D 爱心的动画效果，模拟 **心跳** 的视觉效果。动画通过 **Matplotlib** 和 **NumPy** 实现，并可以保存为 **GIF** 或 **MP4** 格式。此外，动画视角会随时间变化，模拟不同角度观看爱心的效果。
同时提供了C++版本的实现。

## 2. 依赖库

### 必要依赖：
1. **NumPy** - 用于数值计算和矩阵操作。
2. **Matplotlib** - 用于绘制和保存动画。
3. **scikit-image** - 用于网格提取 (Marching Cubes)。
4. **Pillow** - 用于保存 GIF 格式动画。

C++版本依赖PCL库

### 推荐依赖：
1. **FFmpeg** - 用于保存 MP4 格式的动画文件（可选）。

### 安装依赖：
确保你有一个 Python 环境，可以使用 **pip** 安装必要的库：

```bash
pip install numpy matplotlib scikit-image pillow
