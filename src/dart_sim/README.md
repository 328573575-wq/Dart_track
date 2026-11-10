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
    sim_viewer.py  显示窗口（需要 OpenCV，默认由 sim_world 自动拉起）
    sim_eval.py    打分节点（默认由 sim_world 自动拉起）
  config/scene.yaml                场景参数：相机位姿、飞镖、渲染
  config/real_light_centers.yaml   四个引导灯的坐标（和识别包同格式）
  config/ost.yaml                  相机内参（ost.yaml 格式）
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

`sim_world` 默认会**自动**把显示窗口（`sim_viewer`）和打分（`sim_eval`）一起拉起来，
所以只要开仿真，图像、窗口、打分三样都齐了；上层 launch（比如 `dart_track` 的
`use_sim:=true`）不需要额外做什么。

```bash
# 假世界 + 显示窗口 + 打分（默认全开）
ros2 launch dart_sim sim.launch.py

# 不开窗口（服务器 / 没装 OpenCV）
ros2 launch dart_sim sim.launch.py gui:=false

# 不要窗口 / 不要打分
ros2 launch dart_sim sim.launch.py gui:=false
ros2 launch dart_sim sim.launch.py eval:=false

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
| `/sim/truth/dart_front` | `geometry_msgs/PointStamped` | 灯条**前端**（沿速度方向）在发射系的坐标 |
| `/sim/truth/dart_back` | `geometry_msgs/PointStamped` | 灯条**后端**在发射系的坐标 |
| `/sim/truth/dart_angle_deg` | `std_msgs/Float64` | 速度与 +y 轴夹角（度） |
| `/sim/truth/camera_pose` | `geometry_msgs/PoseStamped` | 相机位姿（C + 四元数） |
| `/sim/truth/light_pixels` | `std_msgs/Float64MultiArray` | 四个灯像素，8 个数 |
| `/sim/truth/light_points` | `std_msgs/Float64MultiArray` | 四个灯的世界坐标，12 个数 |
| `/sim/status` | `std_msgs/String` | 帧号 / 时间 |

真值话题只用于**打分**，不要拿它当输入。测量节点只应订阅前两个话题
（这样换成真相机时才能无缝切换）。

`dart_front` / `dart_back` 和你 `position_node` 输出的 `dart_real_front` /
`dart_real_back` 是同一个定义（发射系下的灯条两端），时间戳也相同，可以直接逐帧比。

---

## 两个和识别包共用的配置文件

仿真**不把灯和相机内参写死**，而是读这两个文件：

| 文件 | 内容 | 格式 |
| --- | --- | --- |
| `config/real_light_centers.yaml` | 四个引导灯在 throw 下的坐标 | 和 `dart_track/config/real_light_centers.yaml` 一模一样 |
| `config/ost.yaml` | 分辨率、K、畸变、P | OpenCV `FileStorage` 方言（`%YAML:1.0` + `!!opencv-matrix` + `dt`），这样 C++ 的 `cv::FileStorage` 能直接读 |

工作流就是：

1. 改 `dart_sim/config/` 里这两个文件 → 重启仿真，画面/真值/打分都跟着变；
2. 把这两个文件**复制**到识别包的 `config/` 下 → 解算用的就是同一套参数；
3. 两边都要 `colcon build`（节点读的是 install/share 里的副本，不是 src）。

> `ost.yaml` 用的是 OpenCV 方言而不是标准 `camera_calibration` 格式，因为
> `solvepose_node` / `position_node` 用 `cv::FileStorage` 读它。仿真这边的
> Python 读取器两种格式都认，所以以后真机标定导出的标准 ost.yaml 也能直接丢进来。

`sim_world` 启动时不再往终端打印坐标；要核对就看话题（`frame_id` 都是 `throw`）。

运行中的真值看话题（`frame_id` 都是 `throw`）：

```bash
ros2 topic echo /sim/truth/camera_pose     # 相机在 throw 下的位姿（和你的 TF 同一定义）
ros2 topic echo /sim/truth/dart_front
ros2 topic echo /sim/truth/dart_back
ros2 topic echo /sim/truth/light_points    # 4 个灯 xyz
```

要存盘就用 `save:=true`，每帧的 `truth.jsonl` 里已经包含头尾两端和夹角。

---

## 用 sim_eval 给测量代码打分

`sim_eval` 订阅 `/dart_point`、`/tf`（`throw -> camera`）和上面三个真值话题，
按 `header.stamp` 逐帧配对，每秒打印一次：

```
n=  120 | dart  front  1.24 mm  back  2.01 mm | cam  pos 0.85 mm  rot 0.052 deg
          max    3.10 mm       4.20 mm |      1.90 mm       0.113 deg
```

- `dart front/back`：你的解算点和真值点的欧氏距离（毫米）；
- `cam pos/rot`：你的 TF 平移 vs 真值相机中心、旋转 vs 真值 `R_wc`；
- 行尾出现 `[head/tail swapped: N]` 说明有 N 帧把镖头镖尾判反了（误差会正好是灯条长度）。

```bash
ros2 run dart_sim sim_eval
# 或者随仿真一起起：
ros2 launch dart_sim sim.launch.py eval:=true
```

`sim_world` 默认会自动起它（`eval:=false` 可关）。

它需要你的 `dart_point_interfaces`（因为要读 `/dart_point`）；没有时只会打印一句提示就跳过，
`dart_sim` 本身不依赖你的工程。

> 如果 `sim_eval` 一直不打印 `n=...`，先看 `/dart_detections` 有没有输出：
> 绿色灯条会被 guidance-light 的**面积**筛选当成第 5 个灯，导致 recognition 直接 return。
> 解决：把 `light_min_area` 调到比灯条面积大（当前场景约 130）、`dart_min_area` 调到比它小（约 40），
> 或者给 recognition 的灯筛选加形状判据。

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
