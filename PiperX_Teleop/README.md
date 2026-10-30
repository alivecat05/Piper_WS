# PiperX 双臂遥操

Windows · Meta Quest 3 · 官方 pyAgxArm · 头部 D555 + 双腕 D405

## 功能

- 双臂相对位姿遥操、扳机比例控制夹爪、自定义初始位。
- **电脑本地**三视角 RGB 窗口，显示相机 FPS、action Hz、录制状态。
- 30 Hz 软件对齐的 RGB + 双臂 action/state；一键转 LeRobot v3。
- Quest 支持 USB / Wi-Fi；录制独立于机械臂运动。

## 1. 安装（首次）

Windows 10/11 x64，联网；项目路径建议不含空格。

```powershell
git clone https://github.com/alivecat05/PiperX_Teleop.git
cd PiperX_Teleop
.\setup.cmd
```

自动安装项目内 Conda、Python 3.12、RealSense、机器人 SDK 和 ADB，不覆盖已有 `config.json`。USB-CAN 须安装硬件厂商提供的 Windows 驱动。

## 2. 配置硬件

```powershell
.\teleop.cmd discover
```

编辑自动生成的 `config.json`：

- `cameras.*.serial`：填头部、左腕、右腕相机序列号。
- `arms.*.channel`：核对左右 USB-CAN 编号；示例左 1、右 0。
- `arms.*.firmware`：匹配 SDK 固件 profile；当前配置 `v189`。
- `tcp_offset / xr_to_base / workspace_*`：按工具和机械臂安装方向调整。

先把双臂摆到展开的工作姿态，避免收拢零位；进入程序后长按 **X** 保存初始位。相机参数和初始位仅保存在本机。

## 3. 连接 Quest（二选一）

**USB（建议先用）**：Quest 开启开发者模式，连 USB，在头显内允许 USB 调试。

```powershell
.\teleop.cmd --transport usb
```

Quest 浏览器打开 **http://localhost:8012**，点击“启动手柄追踪”。

**Wi-Fi**：两台设备连接同一 Wi-Fi。把下面地址换成电脑 **WLAN 网卡的 IPv4**；防火墙允许 TCP 8012。

```powershell
.\teleop.cmd cert --ip 192.168.1.100
.\teleop.cmd --transport wifi --cert certs/cert.pem --key certs/key.pem
```

Quest 打开终端显示的 **https://地址:8012**，无需 token。证书首次生成、IP 改变后重建；自签证书可能提示不受信任，USB 无此提示。

**挂胸前使用**：先连 USB 执行 `.\teleop.cmd awake`，再启动追踪；摄像头朝前且无遮挡。结束执行 `.\teleop.cmd sleep`；头显重启后需重新 `awake`。无需 MQDH，但仍依赖头显看得到手柄，挂胸前稳定性需实测。

## 4. 手柄操作

- **左右中指键**：按住对应臂跟随，松开保持当前位置。
- **左右食指扳机**：半按半闭，全按闭合，松开张开；不必按中指键。
- **B**：开始录制。
- **A**：停止并保存；等待 `SAVING` 结束后再开下一段。
- **X 长按 2 秒**：保存当前双臂位置为初始位。
- **Y 长按 1.5 秒**：回初始位；松开再按 Y 可取消。

X/Y 前松开双中指键，并结束录制、等待保存。电脑窗口按 **Q / Esc** 退出。

## 5. 导出数据

```powershell
.\setup.cmd -Export    # 首次安装导出环境
.\export.cmd           # 一键转换
```

原始数据：`outputs/episodes/`；LeRobot：`datasets/piperx/`，默认不上传、排除未完成/模拟数据。

`action` 是已发送目标，`state` 是实际反馈；均为左右各 `[x,y,z,qx,qy,qz,qw,夹爪宽度]`，共 16 维，长度单位米、各臂基座坐标。同步为软件时间对齐，不是曝光级硬同步。

## 排查

- **先测手柄，不动机器人**：`.\teleop.cmd --transport usb --input-test`；看窗口中的追踪、中指和扳机状态。
- **无硬件模拟**：`.\teleop.cmd --transport usb --mock`。
- **paused**：恢复所需手柄追踪，松开按键/扳机 0.3 秒后再按；夹爪需重新捏扳机。闲置手柄丢失不阻断另一只手。
- **fault**：查看 `outputs/logs/teleop.log`。若需清除支持恢复的控制器故障，先退出程序、**支撑好双臂**，再执行 `.\teleop.cmd recover --supported`；恢复会断开电机动力，不是回初始位。

代码：`piperx_teleop/`；测试：`tests/`；第三方 SDK：`vendor/`（保留各自许可证）。[验证与限制](docs/validation.md)
