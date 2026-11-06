# dart_sim — 独立的飞镖仿真包

这个包只做一件事：**生成一个几何上正确的假世界**，通过 ROS 话题发布
「相机图像 + 相机内参 + 隐藏真值」，并可以把整段飞行存成数据集。

它和你的测量工程是**两个独立的包**：

- `dart_sim` 不 import 你工程里的任何代码；
- 你的工程也不应该 import `dart_sim`（生产代码只用 OpenCV / numpy）；
- 两者唯一的接口是 ROS 话题：`/camera/image_raw`、`/camera/camera_info`；
- 你可以整个删掉、或者移动到别的 workspace，不会影响你的工程。

仿真只是**开发期的输入源**。等你的测量节点写好了，把 `sim_world` 换成真实
相机驱动（例如 `v4l2_camera` / `usb_cam`），测量节点一行都不用改。

---

## 目录

```
src/dart_sim/
  dart_sim/
    scene.py       场景 + 投影模型（纯 numpy，不依赖 ROS）
    render.py      渲染 + PNG 编码（纯 numpy）
    sim_world.py   渲染节点：发图像、内参、真值，可存数据集
    sim_viewer.py  显示节点（可选，需要 OpenCV）
  config/scene.yaml  场景参数：相机位姿/内参、四个灯、飞镖初速、渲染参数
  launch/sim.launch.py
  tools/
    preview.py     不开 ROS 的离线预览 / 自检
    dataset.py     读数据集（meta + 图像 + 真值）
    env_check.sh   检查 WSL/ROS 环境
```

---

## 编译

在 workspace 根目录（`dart_ws/`）：

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> 在 `/mnt/e` 下不要用 `--symlink-install`，普通 `colcon build` 即可。

---

## 运行

```bash
# 假世界 + 显示窗口
ros2 launch dart_sim sim.launch.py

# 不开窗口（服务器 / 没装 OpenCV）
ros2 launch dart_sim sim.launch.py gui:=false

# 慢动作
ros2 launch dart_sim sim.launch.py time_scale:=0.3

# 跑一遍就停，并存成数据集到 dart_ws/runs/<时间戳>/
ros2 launch dart_sim sim.launch.py gui:=false loop:=false save:=true
```

批量生成时让节点自己退出：

```bash
ros2 run dart_sim sim_world --ros-args \
    -p save:=true -p loop:=false -p exit_when_done:=true
```

---

## 话题

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/Image` | 假相机图像，`bgr8`，640×480 |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 内参 K、畸变 D、投影矩阵 P |
| `/sim/truth/dart` | `geometry_msgs/PointStamped` | 飞镖中心在发射系的坐标 |
| `/sim/truth/dart_velocity` | `geometry_msgs/Vector3Stamped` | 飞镖速度 |
| `/sim/truth/dart_pixel` | `geometry_msgs/PointStamped` | 飞镖中心像素 (u, v, 深度) |
| `/sim/truth/camera_pose` | `geometry_msgs/PoseStamped` | 相机位姿（C + 四元数） |
| `/sim/truth/light_pixels` | `std_msgs/Float64MultiArray` | 四个灯像素，8 个数 |
| `/sim/truth/light_points` | `std_msgs/Float64MultiArray` | 四个灯的世界坐标，12 个数 |
| `/sim/status` | `std_msgs/String` | 帧号 / 时间 |

真值话题只用于**打分**，不要拿它当输入。测量节点只应订阅前两个话题
（这样换成真相机时才能无缝切换）。

---

## 数据集格式

```
runs/<时间戳>/
  frame_0000.png ...   每帧图像
  meta.json            静态参数：K、D、R_wc、C、灯的坐标、飞镖参数、渲染参数
  truth.jsonl          每行一帧：t、dart、velocity、dart_px、dart_px_strip 两端、light_px、depth、valid
```

`meta.json` 里的 `R_wc` 三列是相机 x/y/z 轴在发射系下的方向，`C` 是相机中心；
世界→相机 `p_cam = R_wc.T @ (p_world - C)`。

读取：

```bash
python3 src/dart_sim/tools/dataset.py runs/<时间戳>
```

或在代码里：

```python
from tools.dataset import load_dataset
meta, frames, truth = load_dataset('runs/<时间戳>')
```

> 注意：`draw_truth:=true` 会把绿色真值圈画进**存下来的图**里，只用于检查渲染
> 和投影是否一致，生成数据时不要开。

---

## 离线自检（不需要 ROS / OpenCV）

Windows 或 WSL 都可以（只要 numpy + PyYAML）：

```bash
python3 src/dart_sim/tools/preview.py --frames 0,25,50 --montage 6
```

输出 `runs/preview/`，并打印每帧飞镖的投影、深度、四个灯是否都在画面内。

---

## 场景参数

`config/scene.yaml` 里可以改：

- 相机内参、畸变、位置（`eye`/`target`/`up`）
- 四个灯的坐标和半径
- 飞镖的发射点、初速、仰角、灯条尺寸
- 渲染噪声、曝光、运动模糊

改完直接重新 `ros2 launch` 即可，不需要重新编译。
