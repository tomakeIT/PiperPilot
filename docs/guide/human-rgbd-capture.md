# 人类操作 RGB-D 数据采集(piper-human)

用一台 RealSense D435 拍摄桌面场景的 RGB-D 静态快照,用于人类操作演示数据的
采集。与 `piper-collect` 不同,这条链路**不涉及机械臂**,也不产出 LeRobot
时序数据集——每次快门是一组独立的静态帧。

## 适用场景

| 需求 | 用哪个工具 |
|---|---|
| 遥操作机械臂,录连续轨迹(LeRobot v2.0) | `piper-collect` |
| 拍桌面场景的 RGB-D 静态图,做检测/标注/点云 | `piper-human`(本文) |

典型用途:手部与物体检测、抓取点标注、场景点云重建、深度先验的预训练素材。

## 硬件

| 项 | 值 |
|---|---|
| 相机 | Intel RealSense D435 / D435i |
| 分辨率 / 帧率 | 640×480 @ 30 fps(USB3);USB2 口自动降到 15 fps |
| 深度模式 | 已对齐到彩色相机(`rs.align`) |
| 深度比例 | 0.001 m/单位(即原始值单位为毫米) |

未传 `--serial` 时程序使用第一台已连接的 RealSense。多相机环境应显式传入
序列号，避免设备枚举顺序变化。

## 使用方法

```bash
conda activate piper_teleop
piper-human --name <数据集名>
```

然后浏览器打开 **http://127.0.0.1:8790**。

| 操作 | 快捷键 | 说明 |
|---|---|---|
| 拍照 | `空格` 或点 **shoot** | 保存当前帧的三份文件 |
| 撤销 | `u` 或点 **undo last** | 删掉刚拍那张的全部三份文件,编号回退复用 |

界面左右两栏分别是实时彩色画面和伪彩深度画面。**建议按快门前先看一眼深度图**:
大片黑色代表无效像素(反光、透明物体、超出量程),这种帧事后基本没法用。

常用参数:

| 参数 | 默认 | 说明 |
|---|---|---|
| `--serial` | 第一台已连接设备 | 指定 RealSense 序列号 |
| `--name` | `human_<日期>` | 数据集目录名 |
| `--root` | `~/piper_datasets` | 数据集根目录 |
| `--host` | `127.0.0.1` | 改成 `0.0.0.0` 可从手机/其他机器打开界面 |
| `--port` | `8790` | Web 界面端口 |
| `--ir` | 关 | 额外保存左右两路 IR 图 |
| `--raw-depth` | 关 | 保存原生深度(不对齐到彩色) |

程序重启后打开同一个目录会**接着上次的编号往下拍**,不会覆盖已有数据。

## 数据格式

```
<数据集名>/
├── calib.json             相机内参 / 外参 / 深度比例(全数据集共用)
├── shots.csv              每张快照的时间戳索引
├── 000000_color.png       8-bit RGB,无损,约 150 KB
├── 000000_depth.png       16-bit 原始深度 ← 真正的数据,约 80 KB
├── 000000_depth_viz.png   8-bit 伪彩深度(仅供人眼查看),约 40 KB
└── ...
```

编号相同的三个文件来自**同一帧**,严格对齐。单组约 270 KB。

### 为什么原始深度用 16-bit PNG

实测对比(同一帧真实深度图):

| 格式 | 大小 | 写入 | 无损 |
|---|---|---|---|
| **16-bit PNG(采用)** | **80 KB** | 6.8 ms | 是 |
| `.npy` | 600 KB | 0.5 ms | 是 |
| `.npz`(压缩) | 60 KB | 22.3 ms | 是 |

选 PNG 的理由:体积只有 npy 的 1/7.5,且 16-bit PNG 是 RGB-D 数据集的事实标准
(NYUv2、TUM RGB-D、ScanNet、BOP 都用它),对接现成 pipeline 不用转格式。
深度本身就是整数量化的,存 float32 没有信息增益。

### `shots.csv` 字段

| 列 | 含义 |
|---|---|
| `idx` | 快照编号,对应文件名前缀 |
| `wall_time` | 主机墙钟时间(ISO 8601,毫秒) |
| `host_mono_s` | 主机单调时钟(秒),算间隔用,不受对时影响 |
| `device_ts_ms` | RealSense 设备端时间戳(毫秒) |
| `device_frame_number` | 设备端帧计数器 |

## 怎么读深度

深度是 **uint16 的绝对物理距离**(不是相对深度),转米:

```python
import cv2, numpy as np

depth_raw = cv2.imread("000000_depth.png", cv2.IMREAD_UNCHANGED)  # uint16
depth_m = depth_raw.astype(np.float32) * 0.001
```

!!! danger "必须加 `cv2.IMREAD_UNCHANGED`"
    不加的话 OpenCV 会**静默地**把 16-bit 单通道降成 8-bit 三通道 BGR,
    深度值全毁而且不报错。这是这份数据最容易踩的坑。

**像素值 0 表示无效**(遮挡、镜面反光、超出量程),不是 0 米,统计前先掩掉。

### 反投影成点云

深度已对齐到彩色相机,所以 `depth[v,u]` 和 `color[v,u]` 是同一条光线,
**直接用彩色内参**即可,不需要深度内参和 depth→color 外参:

```python
import cv2, numpy as np, json

calib = json.load(open("calib.json"))
K = calib["color_intrinsics"]
fx, fy, ppx, ppy = K["fx"], K["fy"], K["ppx"], K["ppy"]

depth_m = cv2.imread("000000_depth.png", cv2.IMREAD_UNCHANGED).astype(np.float32) \
          * calib["depth_scale_m"]
rgb = cv2.cvtColor(cv2.imread("000000_color.png"), cv2.COLOR_BGR2RGB)

v, u = np.mgrid[0:depth_m.shape[0], 0:depth_m.shape[1]]
valid = depth_m > 0
Z = depth_m[valid]
X = (u[valid] - ppx) / fx * Z
Y = (v[valid] - ppy) / fy * Z

points = np.stack([X, Y, Z], axis=-1)   # (N, 3) 相机坐标系,单位米
colors = rgb[valid]                     # (N, 3) uint8,与 points 一一对应
```

深度是**沿光轴的 Z 分量**,不是沿视线的欧氏距离,所以上式成立。

`calib.json` 里的 `depth_intrinsics` 和 `depth_to_color_extrinsics` 是未对齐
深度流的参数,对齐模式下用不到,仅作记录保留。

### `depth_viz.png` 的定位

8-bit 彩色图,按 **0–2000 mm** 固定量程映射到 TURBO 色带,近蓝远红、黑色为无效。
量程固定所以不同快照之间颜色可横向比较。

**这是有损可视化,绝不能当深度数据读回去。** 训练和几何计算一律用 `_depth.png`。

## 深度精度

D435 是主动双目立体相机(双 IR + 红外散斑投射),出厂已完成立体标定,输出为
**度量级绝对深度**,不需要用户标定。

实测健康检查(RANSAC 提取主平面):

| 指标 | 实测值 |
|---|---|
| 平面拟合残差 | 4.5 mm @ 2.4 m |
| 逐像素时序噪声 | 5.2 mm @ 2.4 m |
| 理论噪声上限 | 约 20–25 mm @ 2.4 m |

远好于理论上限,说明出厂标定没有退化(摔过或受热变形的相机会在这一步暴露)。
立体深度误差大致随距离平方增长,桌面工作距离(0.5–1 m)上精度更高。

!!! note "绝对尺度未标定"
    平面拟合只证明"精密",不能排除 1–2% 的系统性尺度偏差。0.7 m 处 2% 就是
    14 mm。若下游任务对绝对尺度敏感,建议用卷尺实测一个已知距离核对;
    真有偏差可用 RealSense 的 on-chip tare 校正。

## Web 界面的实现要点

界面用 stdlib `http.server`,但有三处关键优化,起因是最初版本明显卡顿:

1. **关闭 Nagle 算法(TCP_NODELAY)** —— 响应的头和体分两次 `send()` 发出,
   Nagle 会压住第二次等对方 ACK,而 ACK 又被延迟确认拖住,每个请求白等
   **40 ms**。关掉后 `/status` 延迟从 40 ms 降到 **0.31 ms**。这是最大的一笔。
2. **HTTP/1.1 keep-alive** —— `BaseHTTPRequestHandler` 默认是 HTTP/1.0,
   每个请求都要重建 TCP 连接、重开线程。
3. **MJPEG 推流** —— 原来用 JS 定时器改 `img.src` 轮询,换 src 会让图片先清空
   再加载(肉眼可见的闪烁)。改成 `multipart/x-mixed-replace` 常连接后,
   页面上一行刷新 JS 都不需要,浏览器自己渲染。

配套优化:深度伪彩用预建的 65536 项查找表(比 float32 运算快 2.5 倍);
JPEG 按帧号缓存,多个客户端看同一帧只编码一次。

实测:预览稳定 15 fps,深度流 190 KB/s,推流开着时快门往返 15 ms。

## 已知限制

- **没有相机→机器人基座的外参**,所有 3D 点都在相机坐标系下。要把轨迹映射到
  机械臂坐标系需另行标定(AprilTag 或末端触点法)。参考 RoboMIND 数据集的
  坐标系不匹配问题——这一步不能靠假设。
- **没有位姿标注、分割掩码或抓取标签**,是原始数据。
- **不是时序数据**,每张之间不连续,不能当视频或轨迹用。
- 默认不存 IR 图(需要 `--ir`)。

## 相关代码

- 采集程序:`piper_teleop/apps/collect_human.py`
- CLI 入口:`piper-human`(在 `pyproject.toml` 的 `[project.scripts]` 注册)
