"""Read measured arm episodes for the workstation's offline timeline."""
from datetime import datetime
import json
from pathlib import Path
import re


SERIALS = {"left": "5A7C122569", "right": "5AAF288004"}
FOLDERS = {"main": "episodes", "archive": "episodes/archive", "sim": "simulation/episodes"}
PREFERRED = "main:left_manual_instrument_pick_20261126_221353"
EPISODE_NAME = re.compile(r"(left|right)_[\w-]+_\d{8}_\d{6}(?:_[0-9a-f]{8})?")


def _positions(joints):
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError("示教观测缺少六轴实测位置")
    positions = {}
    for joint in joints:
        if not isinstance(joint, dict):
            raise ValueError("无效的轴记录")
        axis, position = joint.get("id"), joint.get("position")
        if (type(axis) is not int or axis not in range(1, 7)
                or type(position) is not int or not 0 <= position <= 4095):
            raise ValueError("无效的轴编号或编码器读数")
        positions[axis] = position
    if len(positions) != 6:
        raise ValueError("轴编号重复")
    return [positions[axis] for axis in range(1, 7)]


class ArmEpisodes:
    """Accept only native recording names from the three designated folders."""

    def __init__(self, root):
        self.root = Path(root).resolve()
        self._summaries = {}

    def _path(self, arm, episode_id):
        if arm not in SERIALS or not isinstance(episode_id, str):
            raise ValueError("无效的机械臂或示教编号")
        source, _, stem = episode_id.partition(":")
        name = EPISODE_NAME.fullmatch(stem)
        if source not in FOLDERS or not name or name[1] != arm:
            raise ValueError("示教编号与所选机械臂不匹配")
        directory = self.root / FOLDERS[source]
        path = directory / (stem + ".jsonl")
        if directory.resolve() != directory or path.resolve() != path:
            raise ValueError("示教文件路径无效")
        if not path.is_file():
            raise FileNotFoundError("所选示教文件不存在")
        return path

    def list(self, arm):
        if arm not in SERIALS:
            raise ValueError("无效的机械臂")
        summaries = []
        for source, folder in FOLDERS.items():
            for candidate in (self.root / folder).glob(arm + "_*.jsonl"):
                episode_id = source + ":" + candidate.stem
                try:
                    path = self._path(arm, episode_id)
                    st = path.stat()
                    stamp = (st.st_size, st.st_mtime_ns)
                    cache = self._summaries.get(episode_id)
                    if not cache or stamp != cache[0]:
                        full = self.detail(arm, episode_id)
                        summary = {k: v for k, v in full.items() if k not in ("frames", "joints", "phases")}
                        cache = (stamp, summary)
                        self._summaries[episode_id] = cache
                    summaries.append(cache[1].copy())
                except (OSError, ValueError):
                    # A newly opened recording may still be waiting for its first complete row.
                    continue
        return sorted(summaries, key=lambda item: (item["preferred"], item["started_at"]), reverse=True)

    def detail(self, arm, episode_id):
        path = self._path(arm, episode_id)
        simulated = episode_id.startswith("sim:")
        start = None
        last = None
        task = ""
        frames, phases = [], []
        action_count = 0
        complete = False
        with path.open("rb") as stream:
            for raw_line in stream:
                if not raw_line.strip():
                    continue
                try:
                    row = json.loads(raw_line)
                except (UnicodeError, json.JSONDecodeError):
                    if raw_line.endswith(b"\n"):
                        raise ValueError("示教记录中有损坏的行") from None
                    break  # The open recorder has not committed its final row yet.
                if (not isinstance(row, dict) or row.get("schema") != 1 or row.get("arm") != arm
                        or row.get("controller_serial") != SERIALS[arm]):
                    raise ValueError("示教记录的格式、机械臂或控制板序列号不匹配")
                if any(key in row and row[key] is not simulated for key in ("simulated", "simulation")):
                    raise ValueError("示教记录的实机或模拟来源不匹配")
                kind, data = row.get("kind"), row.get("data")
                if not isinstance(data, dict) or not isinstance(row.get("time_utc"), str):
                    raise ValueError("示教记录缺少数据或时间")
                current = datetime.fromisoformat(row["time_utc"].replace("Z", "+00:00"))
                if current.tzinfo is None or last is not None and current < last:
                    raise ValueError("示教时间戳无效")
                if complete or start is not None and kind == "episode_start":
                    raise ValueError("示教起止记录次序异常")
                if start is None:
                    if kind != "episode_start":
                        raise ValueError("缺少示教开始记录")
                    start = current
                    task = str(data.get("task") or "")
                last = current
                t = round((current - start).total_seconds(), 3)
                if kind == "observation":
                    frames.append({"t": t, "positions": _positions(data.get("joints"))})
                elif kind == "action":
                    action_count += 1
                elif kind == "phase_marker":
                    phases.append({"t": t, "label": str(data.get("phase") or "阶段标记")})
                elif kind == "episode_end":
                    complete = True
        if start is None:
            raise ValueError("尚未取得示教开始记录")
        task_name = "手动取件" if task == "manual_instrument_pick" else task or "示教"
        label = f"{start.astimezone():%m-%d %H:%M:%S} · {task_name}"
        return {
            "id": episode_id, "label": label, "arm": arm, "preferred": episode_id == PREFERRED,
            "simulated": simulated, "complete": complete, "started_at": start.isoformat(),
            "source": episode_id.partition(":")[0], "sample_count": len(frames), "action_count": action_count,
            "duration_s": round((last - start).total_seconds(), 3),
            "joints": frames[0]["positions"].copy() if frames else [], "frames": frames, "phases": phases,
        }
