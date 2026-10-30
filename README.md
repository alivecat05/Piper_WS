# Piper 工作空间

- [PiperCapture](PiperCapture/README.md)：主从机械臂示教采集，三视角预览、RGB/action 同步、MP4/CSV/JSONL、LeRobot 导出。
- [PiperX_Teleop](PiperX_Teleop/README.md)：Quest 3 双臂遥操，支持 USB / Wi-Fi。

## 采集

Windows 10/11，首次运行安装项目内 Conda：

```powershell
cd PiperCapture
.\setup.cmd
# 在 config.json 填相机序列号和 CAN 编号
.\run.cmd
```

在窗口填写任务名、采集序号，点击“开始录制 / 结束保存”：

```text
PiperCapture/output/pick_cup/episode_001/
  head.mp4
  left_wrist.mp4
  right_wrist.mp4
  action.csv
  samples.jsonl
  manifest.json
```

序号自动建议下一号，也可手填；不覆盖已有数据。当前主从共线接法暂不支持自定义回位。

## 导出

```powershell
.\setup.cmd -Export
.\export.cmd
```

默认读取 `PiperCapture/output/`，输出到 `PiperCapture/datasets/pipercapture/`。

仓库包含代码、测试、SDK 源码及许可证、配置示例和安装脚本；不包含本地 Conda、录制数据、证书、缓存及设备配置。
