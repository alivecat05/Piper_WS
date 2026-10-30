# 验证与边界

## 参考取舍

- [Unitree XR / TeleVuer](https://github.com/unitreerobotics/televuer)：借鉴浏览器 XR 位姿输入与视频/控制分离；本项目使用简短原生 WebXR 实现，没有移植 Unitree 机器人驱动。
- [Marvin Apex Quest](https://klmmotion.github.io/Apex_doc_center/en/xr-teleop/meta)：借鉴 Grip 接管；本项目按需求改为 B 开始录制、A 结束录制，录制与运动独立。
- [Meta USB 调试](https://developers.meta.com/vr/documentation/web/browser-remote-debugging/)：USB 使用 `adb reverse`，与 Wi-Fi 使用相同协议。
- [LeRobot 0.4.2](https://github.com/huggingface/lerobot/tree/v0.4.2)：锁定官方写入 API，生成 v3 数据；仓库原有 vendor 转换脚本不直接复用。
- [EVA-CLIENT WebXR](https://github.com/Noietch/EVA-CLIENT/tree/main/examples/input_sources/vr_webxr)：已对照输入节点、相对位姿映射和录制对齐代码，并独立实现有序按键事件、处理确认、去重及手柄震动。逐段同步质量报告尚未引入。其 Grip 长按切换、全局 ARM 与 token 不适用于本项目已确定的操作方式。
- [ARX_VR](https://github.com/ARXroboticsX/ARX_VR)：当前 main 主要提供 Quest APK、安装和操作说明，未找到 Unity 工程或可直接适配 PiperX 的通信协议。可参考保持位姿、复位和语音状态提示；其食指控制机械臂、侧键控制夹爪的映射与本项目相反，保留本项目按键定义。
- EVA 的 USB 例程使用 ADB reverse 和浏览器；ARX 文档建议延长休眠至 4 小时，两者都未要求 MQDH，也未证实无需额外设置即可挂脖追踪。本项目采用 [Meta 官方离脸 ADB 方法](https://developers.meta.com/vr/documentation/unity/meta-xr-operator/quest/)：`awake` 发送 `com.oculus.vrpowermanager.prox_close`，`sleep` 发送 `com.oculus.vrpowermanager.automation_disable` 恢复物理传感器；不修改追踪有效性判断。命令执行成功不等于已验证具体 Quest 固件的持续追踪，按 [README](../README.md#挂脖采集) 实测。

## 时间和动作

- 采集、反馈、命令统一用 `perf_counter()` 高精度单调时钟（Windows QPC），避免当前 Python `monotonic()` 的 15.6 ms 分辨率；不直接比较独立相机的 hardware clock。
- RealSense global time 可用时优先使用；否则采用接收时间，并记录 `host_receive`。`latency_ms` 可修正实测固定延迟。
- 默认：相机对目标时间 ±20 ms、三路跨度 ≤25 ms、反馈各字段 ±40 ms；等待 80 ms 后对齐。
- `action` 是观测时刻之前最后一条由 SDK 发送的目标，包含限速/限幅结果；不是手柄原始位姿，也不保证机器人已到达。
- 两臂 CAN 顺序发送，各自记录发送完成时间；没有双臂总线原子发送保证。
- 来源时间戳、实际关节反馈和配置保存在原始 episode 中；LeRobot 以连续 grid 构建均匀时间轴。
- 当前保留的是对齐后图像及来源元数据，不保留所有未匹配原始帧。
- A 固定录制结束时刻；继续对齐截止前的尾帧，再由后台线程完成写盘。`SAVING` 期间遥操和断联检查继续运行；保存完成后可开始下一段。
- 就绪检查要求三路 RGB、两臂反馈和 Quest 跟踪均有效；RGB 默认超过 250 ms 没更新即停机。复位中再次按 Y，用实际关节位置替换剩余路径。
- 位姿取最新帧；四个面键的按下/释放逐一编号、排队，收到处理确认前重发，已处理事件去重。断联不重放操作，过期事件停止控制。确认编号表示处理完毕；具体执行/拒绝结果通过编号通知返回。
- 声音在独立线程播放，故障优先；手柄震动按通知编号去重。状态每 30 ms 推送，未收到新手柄输入时也能报告故障；只有写盘提交成功后才发“保存完成”。

## 实机前需配置

- D555 的 PoE、DDS 发现、Windows 防火墙与 SDK/固件配套；依据 `discover` 选择真实支持的 RGB 模式。
- 两臂 CAN 对应关系、固件 profile、TCP 标定、坐标方向与空间范围。
- 复位路径必须适合当前安装；依次执行左臂/右臂关节路径并等待反馈确认；不生成避障轨迹。
- 手柄保存初始位只替换各臂路径的最后一个路点；已有中间路点仍需确认适配新初始位。
- 当前只有单臂空间/速度限制，未实现双臂碰撞检测。首次联调须小范围、低速，保留物理急停。
- SDK `reset()` 是控制器状态复位，不是回初始位姿；本项目回位调用 `move_j()`。
- 软件对齐阈值是时间戳一致性指标；D555 网络延迟和 D405 时钟误差仍需实测，不能据此承诺物理曝光同步。

## 验证入口

手柄诊断：`--input-test` 保留真实相机，但只构造模拟机器人；测试数据标记为模拟，不用于正式导出。`/status` 是只读接口，不占用或重置控制连接。故障原因区分超时、失焦和左右手追踪状态；界面持续显示按键输入，接口累计中指键和食指扳机的按下次数，Quest 输入丢失且机器人反馈正常时进入 paused，发送实测位置保持；恢复须有效追踪及所有按键/扳机释放至少 0.3 秒，再重新按键。夹爪恢复前保持当前开度，不因释放动作自动张开。机器人/相机/反馈故障和保持失败仍进入 fault，不能自动解除。共 105 项 Python 检查通过。保持实测关节角只检查已启用的 SDK 软件限位，不修改开关或裁剪角度；覆盖负 J2 反馈、无效反馈、不同手柄绝对起始姿态、接管抖动、起始姿态拒绝跟随后夹爪仍独立工作，官方夹爪命令的使能标志，以及扳机慢捏/快捏/保持/反向时的开度比例跟随（无额外速度限制）。保持目标可发送不代表末端 IK 可行；初始关节距 SDK 预设限位不超过 2° 时，阻止接管使能和运动命令、保存工作初始位及回到该工作位，不自动生成工作姿态。

```powershell
.\.conda\Scripts\conda.exe run -p .\.conda\envs\teleop python -m pytest -q
.\.conda\Scripts\conda.exe run -p .\.conda\envs\teleop python -m piperx_teleop.export --validate-only
```

模拟数据仅验证程序链路；Quest 实机跟踪、相机同步精度、回位路径和机器人运动均需现场联调。

105 项 Python 检查通过，覆盖 SDK 限位开关、挂胸前相对起点、接管时先保持再使能、静止/使能等待时不发笛卡尔目标、食指扳机只控制夹爪、保持目标禁止静默裁剪、端口冲突时不接硬件、未控制时不发急停、显式恢复及新鲜反馈验证、简短启动错误、硬件配置检查、ADB 保活/恢复与失败提示、慢写盘时遥操/断联检查、队列满时结束录制、尾帧截止边界、相机停帧、取消复位、短按事件队列/去重/超时、保存完成确认和声音线程。Quest 脚本已用模拟 WebXR 验证短按捕获、确认重发、震动去重、无震动支持时继续输入及失焦停止；接管修复、挂脖状态的实际追踪范围、声音和震动效果仍需实机验证。历史模拟数据导出已用官方 LeRobot 回读 15 个连续片段、38 帧、三路 RGB 与 16 维 action/state。
Windows 回读先初始化 PyAV，避免媒体 DLL 的加载顺序问题：

```python
import av
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset
data = LeRobotDataset("local/piperx", root=Path("datasets/piperx").resolve(), video_backend="pyav")
```

显式恢复仅允许 NORMAL、急停或目标越限且 err_code=0；双臂预检通过后，每臂最多一次 reset，等待新鲜 NORMAL 反馈，不发送使能/运动。2026-10-29 实机已清除左臂急停与右臂目标越限。

闲置手柄丢追踪不触发全局暂停；未追踪手柄不发控制命令，重新取得追踪后须释放中指与扳机再接管。活动手柄丢追踪仍保持暂停；录制及回位仍要求双手有效。新增覆盖单臂持续控制、闲置拿起、X 保存、录制拒绝及单活动手柄重新接管。
