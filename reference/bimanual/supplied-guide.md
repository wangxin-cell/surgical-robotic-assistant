# 双臂关节点动台

Windows PowerShell 点动工具，基于团队提供的 LeftArmTeach v1.4 和 XLeRobot 的右臂 ID 配置。一次只连接一侧机械臂，不控制底盘、不使用相机。

## 启动

双击 `Start-BimanualArmTeach.cmd`，输入 L（左臂）或 R（右臂）。也可以在 PowerShell 中运行：

```powershell
powershell.exe -NoProfile -STA -ExecutionPolicy Bypass -File .\BimanualArmTeach.ps1 -Arm left
powershell.exe -NoProfile -STA -ExecutionPolicy Bypass -File .\BimanualArmTeach.ps1 -Arm right
```

程序用 USB 序列号识别控制板，不依赖固定 COM 号：左臂 `5A7C122569`，右臂 `5AAF288004`。右侧只操作 ID 1–6；ID 7 及以上的底盘电机不在控制白名单内。

连接后先只读查看位置、目标、限位和力矩。要启用保持，需确认机械臂有支撑、夹爪空置、人员离开活动范围，并勾选确认框。点动仅提供 8/16/24 编码器计数；先从 8 开始，一次只动一个关节。释放力矩可能导致机械臂下落，必须先托住机械臂。

## 验证范围

`-SelfTest` 在模拟状态下验证保持、单关节点动、限位拦截、释放和姿态导出：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\BimanualArmTeach.ps1 -SelfTest
```

模拟自检通过不等于实机验证。该交付尚未在实机上驱动验证；右臂只读状态、实际供电、夹爪方向、机械净空和所有限位都应在现场确认后再进行点动。不得连接真实锐器。操控日志写入程序目录的 `bimanual-arm-diagnostics.jsonl`。
## 记录人工示教

连接并启用保持后，在“任务标签”框输入任务名，点击“开始记录一轮”。操作者逐步点动机械臂完成演示，随后点击“结束并保存”。每轮 JSONL 保存 episode_start/end、约 1 秒间隔的实测关节状态，以及每次点动的前态、目标、步长和动作反馈。文件写入 `episodes` 子目录。保持窗口开启直到轮次保存完成。

这份无相机记录可用于固定场景的关节轨迹分析/回放原型，不能单独提供器械位置或视觉状态，因此不能据此实现对任意新摆位手术器具的自主定位抓取。后续训练请加入相机帧、目标标注、坐标标定和成功/失败标签，并转换成与所选 LeRobot 模型匹配的数据集格式。
## 成功示教回放

先用原始起始姿态、夹爪空载进行预览：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\BimanualArmTeach.ps1 -Arm left -ReplayEpisode .\episodes\left_manual_instrument_pick_20261124_215818.jsonl
```

预览只检查文件并报告动作数，不会连接舵机。实机运行前先在点动界面将左臂带到该轮首个状态附近（六关节各自误差不超过 30 计数），保持力矩开启；清空运动范围、托扶臂身、移走锐器并准备好物理断电。执行命令会再检查序列号、起始位姿、力矩及所有记录目标与硬限位的 24 计数余量，随后要求输入 `REPLAY LEFT`：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\BimanualArmTeach.ps1 -Arm left -ReplayEpisode .\episodes\left_manual_instrument_pick_20261124_215818.jsonl -ExecuteReplay
```

回放按已接受动作的时间顺序连续下发位置目标，最长间隔压缩到 0.30 秒；舵机仍按自身位置控制曲线运动。遇到通信、力矩或限位异常会停止继续下发目标。该脚本只做固定起始姿态下的动作复刻；它不会检测抓取成功，也没有相机闭环。先用空夹爪验证轨迹，真实器具应使用钝头模拟器具。实机回放尚未由本工具环境验证。