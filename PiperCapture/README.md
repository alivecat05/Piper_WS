# PiperCapture

**两组主从 PiperX 示教采集**，D555 + 双 D405，不需要 Quest。

当前接线是两路 USB-CAN，各自在 CAN 侧分接一组主从，原有硬件跟随保持不变。
实机的“保存从臂参考位”只保存从臂位置；**主从返回自定义初始位尚未实现，按钮已禁用**。
官方 `move_leader_follower_to_home()` 是主从回零，不能替代自定义工作位。录制、预览、同步和导出仍可使用。

## 使用

```powershell
cd D:\Piper_WS\PiperCapture
.\run.cmd --mock    # 无硬件模拟
.\run.cmd           # 实机采集
```

当前电脑复用相邻 `PiperX_Teleop` 的 Conda 环境。新电脑先运行 `setup.cmd`，然后在 `config.json` 填相机序列号和左右 CAN 编号；`run.cmd discover` 查询相机。

`run.cmd discover-arms` 只读枚举 USB-CAN 编号和适配器标识，不会让机械臂运动；当前两组主从检测到两路是正常的。

**不要同时运行 PiperX_Teleop。** 用机械臂自身的示教控制进入可拖动状态；程序录制时只读反馈，不自动断电或切换示教模式。

## 电脑窗口按钮

录制前填写 **任务名、采集序号**（正整数；自动建议下一个序号，也可手动修改；已有目录不覆盖）。录制/保存期间锁定；下段录制可修改。

- **开始录制 / 结束保存**：录一段数据，等“保存中”结束再录下一段。
- **保存从臂参考位**：保存两台从臂关节角和夹爪开度，不包含主臂。
- **返回初始位 / 取消返回**：目前仅在模拟或两臂独立控制配置中可用；当前主从共线实机不发送此类命令。
- **退出**：录制中会先保存再关闭；回位中会先取消。

直接点击窗口下方的中文按钮，执行结果或拒绝原因会显示在按钮上方。R/S/H/G/C/Q 快捷键仍可在预览窗口内使用，支持大小写；终端按键不作为控制输入。

保存参考位前结束录制并等待保存完成。程序不会自动松电机，也不自动更改主从角色。

窗口实时显示三路 RGB/FPS、机器人反馈 Hz、有效 action Hz、保存帧数。`fault` 停止采集并标记未完成；检查连接/机器人状态后重启。

## 数据与同步

- 原始数据：`outputs/<task_name>/episode_001/`；初始位：`home.json`（模拟另存）。
- 每段保存 `head.mp4`、`left_wrist.mp4`、`right_wrist.mp4`、`action.csv`、`samples.jsonl`、`manifest.json`，不再写 JPG。
- 任务名和采集序号用于目录命名，不额外写进数据文件。旧的 `outputs/episodes/` 数据原样保留。
- 三路视频第 N 帧 = CSV 第 N 行数据 = JSONL 第 N 行；视频丢帧处不补帧，真实间隔看时间戳。
- `action.csv` 只有 17 列：`timestamp_unix_s` + 左臂 8 维 + 右臂 8 维，无关节角/state。
- 所有落盘采集时间戳使用 **Unix 秒**（UTC）；CSV 保留 6 位小数。内部同步仍用单调时钟，每段固定映射到 Unix 时间。
- 30 Hz 软件时间对齐；联合选取三路帧，单路偏差 ≤20 ms、三路跨度 ≤25 ms，不是曝光级硬同步。
- 相机优先启用 `global_time`（设备时间映射到电脑时钟）；`system_time` 使用 SDK 帧时间。来源记录在 JSONL 的 `timestamp_basis`，不可用时才标记 `host_receive` 回退。
- 窗口显示丢帧原因，`manifest.json` 保存 `drop_reasons` 和最长连续缺帧秒数。`complete` 表示正常保存，不代表没有丢帧。
- `state[t]`：当前实测状态；`action[t]`：**下一采样时刻的实测状态**，不是 SDK 下发命令。
- 左右各 `[x,y,z,qx,qy,qz,qw,夹爪宽度]`，共 16 维；基座坐标、米、四元数 xyzw。
- CSV 时间对应当前 RGB；action 的下一时刻时间另存在 JSONL 的 `action_timestamp_unix_s`。JSONL 保留 state、关节角及来源 Unix 时间；action 独立匹配下一时刻机器人反馈，不依赖下一组三路图像。超出录制终点的尾帧舍弃。

## 转 LeRobot

```powershell
.\setup.cmd -Export  # 新电脑首次；当前电脑可省略
.\export.cmd
```

输出 `datasets/pipercapture/`，LeRobot 0.4.2 / v3；支持新 MP4 和旧 JPG 录制，按缺帧切分，跳过未完成数据，拒绝混入模拟数据，不上传。

默认递归读取 `outputs/`，也可 `--source outputs/pick_cup` 只导出一个任务；任务描述取自目录名。旧数据用 `--source outputs/episodes`。

## Python 接口

```python
import time
from piper_capture.capture import Capture, load_config

with Capture(load_config("config.json")) as cap:
    while cap.status()["mode"] == "warming":
        time.sleep(0.05)
    cap.record(task="pick_cup", capture_index=1)
    input("拖动示教，按回车结束录制")
    cap.stop()
    while cap.status()["saving"]:
        time.sleep(0.05)
    print(cap.status())
```

另有 `set_home()/go_home()/cancel_home()`；操作返回状态字典。`status()` 包含 `mode/message/recording/saving/frames/dropped/episode/fps`。

代码：`capture` 调度，`core` 同步，`cameras/robot` 设备，`recording/export` 数据，`app` 窗口。
