# Surgical Robotic Assistant · 双臂示教与器械抓取演示

用 Windows 笔记本通过 USB 示教左臂或右臂，保存关节轨迹，并在模拟总线上复现右夹爪的夹持、抬腕和放回流程。瑞莎 Orion O6 的相机检测、台面坐标转换和训练代码在 `vision/`。两端可独立部署。

## Windows 一键部署

准备 Windows 10/11、Windows PowerShell 5.1（含 .NET Framework 和 WinForms）和带 `venv` 的 Python 3.10 或更新版本。Git 仅用于克隆，也可在 GitHub 选择 **Code → Download ZIP** 并解压到可写目录。取得代码后，部署只需运行最后一行。

```powershell
git clone https://github.com/wangxin-cell/surgical-robotic-assistant.git
cd surgical-robotic-assistant
.\Deploy.cmd
```

部署会在内存中自检两侧的保持、点动、限位、释放和姿态导出，然后在项目内建立 `.venv`，从仓库随附的 wheel 离线安装 pySerial 3.5 并运行模拟测试。最后应看到 `OK` 和 `DEPLOY OK (teach console and gripper demo)`。验收时无需连接 O6、控制板或电机。重复运行会复用环境。

| 使用场景 | 在仓库根目录运行 |
|---|---|
| 仅安装并自检 PowerShell 示教台，无 Python | `.\Deploy.cmd -TeachOnly` |
| 指定本机已有的 Python | `.\Deploy.cmd -Python "D:\miniconda\python.exe"`（替换为实有路径） |
| 打开右臂模拟界面 | `powershell.exe -NoProfile -STA -ExecutionPolicy Bypass -File .\BimanualArmTeach.ps1 -Arm right -Simulate` |
| 复现夹持和松开 | `.\.venv\Scripts\python.exe -X utf8 .\demo\film_grip.py --simulate` |
| 复现右腕抬起与放回 | `.\.venv\Scripts\python.exe -X utf8 .\demo\film_pick_lift.py` |

## 连接实机与记录示教

1. 将目标控制板的 USB 数据线接到运行本程序的 Windows 电脑，在设备管理器确认出现串口。使用 WCH CH343 对应的驱动。程序按板卡序列号选择 COM 口，本套设备左侧是 `5A7C122569`，右侧是 `5AAF288004`，串口参数为 1,000,000 baud、8N1。另一台设备需先核对板卡映射和关节配置。
2. 双击 `Start-BimanualArmTeach.cmd`，按 L 或 R 选择一侧，点“连接并读取”。窗口显示六个关节的实测位置、目标、限位和力矩。右侧与底盘共用总线，示教台的关节白名单是 ID 1–6。
3. 按界面提示支撑臂体、清空活动范围，再启用当前位置保持。选择关节并点动；记录完整一轮时填写任务标签，点击“开始记录一轮”，完成后点击“结束并保存”。新文件位于 `episodes/`。
4. 预览自己录制的轨迹时，从 `episodes/` 选择与 `-Arm` 一致的文件。以下命令会打印动作数。关闭 GUI 前先依界面提示承托并释放机械臂。

```powershell
$episode = Get-ChildItem .\episodes\left_*.jsonl | Sort-Object LastWriteTime -Descending | Select-Object -First 1
if ($episode) { powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\BimanualArmTeach.ps1 -Arm left -ReplayEpisode $episode.FullName }
```

实机回放还需要 `-ExecuteReplay`、与记录相符的起始姿态和现场确认。请先用空夹爪核对路径。右夹爪演示使用有独立支撑的前臂和无刃轻质练习件；离台与放回需由侧面观察或录像确认。交接时实机已观察到夹爪张合和两垫接触剪柄，最近一次抬腕尝试因位置反馈未跟随而停止，夹取离台尚待验证。

## O6 视觉与台面定位

环境为 Linux ARM64、Python 3.11、`python3.11-venv` 和 Bash。下列命令在 O6 的仓库根目录运行。安装脚本使用独立虚拟环境，安装 CPU 版 PyTorch 2.10.0、torchvision 0.25.0、Ultralytics 8.4.0 等依赖，随后运行几何自检。示例像素 (160, 120) 应映射为 (60, 40) mm。

```bash
bash vision/setup-o6.sh
```

已有旧部署 wheel 目录时，可用 `WHEELHOUSE=/home/radxa/instrument_vision/wheels bash vision/setup-o6.sh` 离线安装。识别使用团队训练的 `best.pt`。在已有项目的 O6 上，可用下列命令复用已部署的权重并校验版本；全新设备可从团队 O6 复制同一文件。仓库包括训练脚本，也可替换成自训权重。

```bash
install -Dm644 /home/radxa/instrument_vision/weights/best.pt vision/weights/best.pt
echo '490381f0fb60c19a0f297054d53bba343af56502a8bdcda58f63004c0e458653  vision/weights/best.pt' | sha256sum -c -
vision/.venv/bin/python vision/predict.py --source /path/to/tray.jpg --weights vision/weights/best.pt --device cpu --output vision/results/tray
```

实拍时用 `ls -l /dev/v4l/by-path/` 找到已确认的头部镜头，将其 `video-index0` 路径传给 `vision/head_camera_check.py --device <路径> --seconds 12`，解释器仍为 `vision/.venv/bin/python`。单张图推理保存带框图和逐帧 JSONL。台面定位可调用 `vision/table_plane.py <现场标定.json> <u> <v>`。仓库的 `example-plane.json` 是几何自检用的合成标定。相机、底板或底盘的位置改变后，需要用现场标志点核对变换。模型的检测框中心与实物夹持点须单独验证。

## 从数据复现训练

Windows 训练环境使用 Python 3.13 x64、NVIDIA CUDA 12.8 兼容驱动和 `vision/requirements-training-windows.txt`。以下命令在仓库根目录运行。预训练初始权重为 Ultralytics YOLOv8n 的 `yolov8n.pt`，请放到 `vision/weights/yolov8n.pt`。下载脚本使用 Windows 的 `curl.exe` 获取 Labeled Surgical Tools 第 1 版。训练使用固定切分与随机种子，硬件差异可能影响结果。

```powershell
py -3.13 -m venv vision\.venv
.\vision\.venv\Scripts\python.exe -m pip install -r vision\requirements-training-windows.txt
.\vision\.venv\Scripts\python.exe vision\download_dataset.py
.\vision\.venv\Scripts\python.exe vision\prepare_dataset.py
.\vision\.venv\Scripts\python.exe vision\train.py --data vision\datasets\labeled_surgical_tools\yolo\data.yaml --epochs 50 --batch 8 --name instruments_50epochs --no-amp
.\vision\.venv\Scripts\python.exe vision\evaluate.py
```

## 文件对应关系

| 路径 | 内容 |
|---|---|
| `BimanualArmTeach.ps1` | 带轮次记录与轨迹预览的双臂示教版，一次选一侧 |
| `reference/bimanual/` | 此次交付的原始脚本、启动器和操作说明。原说明的轮次记录和回放功能对应根目录增强版 |
| `reference/left/` | 早期独立左臂示教台和原始说明 |
| `examples/diagnostics/` | 团队提供的两份历史诊断 JSONL，用于查看指令与反馈差异。轨迹预览读取单独录制的 episode |
| `demo/` | 右臂串口、夹爪和腕部演示、配套模拟测试 |
| `vision/` | 训练、推理、O6 相机、在盘计数、按位置取用核对和台面换算 |
| `vendor/` | pySerial 3.5 官方 wheel，含包内许可证，安装时校验 SHA-256 |

串口显示“拒绝访问”时，先关闭占用同一 COM 口的另一个示教窗口或串口助手。示教台开在 Windows 上时，目标控制板也应接到该电脑。O6 的 SSH 只用于板端任务。

项目许可证见 [LICENSE](LICENSE)。上游项目和训练数据的许可见各自发布页。[XLeRobot](https://github.com/Vector-Wangel/XLeRobot) · [Ultralytics](https://github.com/ultralytics/ultralytics) · [Labeled Surgical Tools](https://www.kaggle.com/datasets/dilavado/labeled-surgical-tools)
