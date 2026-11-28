"""Feetech STS bus access on the O6. Only arm motor IDs 1..6 are addressable."""
from contextlib import contextmanager
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import time
from uuid import uuid4

SERIALS = {"left": "5A7C122569", "right": "5AAF288004"}
IDS = range(1, 7)
HOME = (2208, 1740, 2133, 2177, 1003, 1647)
MARGIN = 24
LOCKS = {side: threading.Lock() for side in SERIALS}
RECORDING = {}
MANUAL_OWNER = {}
SIM_LIMITS = ((1204,3040),(993,3248),(837,3053),(911,3234),(47,3938),(1391,2953))


def validate_arm(arm):
    if arm not in SERIALS:
        raise ValueError("请选择左臂或右臂。")
    return arm


def in_range(j, value, margin=0):
    if j["mode"] != 0 or not 0 <= j["min"] < j["max"] <= 4095:
        raise ValueError(f"ID{j['id']} 位置模式或硬限异常。")
    lo, hi = j["min"] + margin, j["max"] - margin
    if not lo <= value <= hi:
        raise ValueError(f"ID{j['id']} 实测或目标 {value} 不在允许范围 {lo}–{hi} 格。")


def port_for(arm):
    serial_number = SERIALS[validate_arm(arm)]
    paths = list(Path("/dev/serial/by-id").glob(f"*_{serial_number}-*"))
    if len(paths) != 1 or not paths[0].is_char_device():
        raise RuntimeError(f"O6 未找到唯一的 {arm} 控制板 {serial_number}，请检查板端 USB 数据线。")
    return str(paths[0])


class ServoBus:
    simulated = False

    def __init__(self, arm):
        import serial
        self.path = port_for(arm)
        self.port = serial.Serial(self.path, 1000000, timeout=.012, write_timeout=.1, exclusive=True)

    def send(self, axis, opcode, address, data):
        if type(axis) is not int or axis not in IDS:
            raise ValueError("舵机 ID 必须在 1 至 6。")
        body = bytes((axis, len(data) + 3, opcode, address)) + data
        packet = b"\xff\xff" + body + bytes((~sum(body) & 255,))
        self.port.reset_input_buffer()
        if self.port.write(packet) != len(packet):
            raise OSError("串口指令未完整发送。")
        return packet

    def read(self, axis, address, count):
        for _ in range(2):
            sent = self.send(axis, 2, address, bytes((count,)))
            data = bytearray()
            deadline = time.monotonic() + .11
            while time.monotonic() < deadline and len(data) < 120:
                chunk = self.port.read(1)
                if not chunk:
                    continue
                data.extend(chunk)
                for size in (count + 6, 6):
                    frame = data[-size:]
                    if (len(frame) != size or frame[:4] != bytes((255,255,axis,size-4))
                            or sum(frame[2:]) & 255 != 255 or frame == sent):
                        continue
                    if frame[4]:
                        raise RuntimeError(f"ID{axis} 舵机报告状态错误 {frame[4]}。")
                    if size > 6:
                        return bytes(frame[5:-1])
        raise TimeoutError(f"ID{axis} 读取寄存器 {address} 超时。")

    def number(self, axis, address, count):
        return int.from_bytes(self.read(axis, address, count), "little")

    def joint(self, axis):
        limits = self.read(axis, 9, 4)
        # Read torque, goal and intervening bytes together to reduce USB round trips.
        control = self.read(axis, 40, 4)
        return dict(id=axis, min=int.from_bytes(limits[:2],"little"), max=int.from_bytes(limits[2:],"little"),
                    mode=self.number(axis,33,1), torque=control[0], goal=int.from_bytes(control[2:],"little"),
                    position=self.number(axis,56,2), moving=self.number(axis,66,1))

    def goal(self, axis, value):
        if type(value) is not int or not 0 <= value <= 4095:
            raise ValueError("无效的编码器目标。")
        self.send(axis,3,41,bytes((5,value & 255,value >> 8,0,0,45,0)))
        if self.number(axis,42,2) != value:
            raise RuntimeError(f"ID{axis} 目标回读不符。")

    def torque(self, axis, enabled):
        self.send(axis,3,40,bytes((int(enabled),)))
        if self.number(axis,40,1) != int(enabled):
            raise RuntimeError(f"ID{axis} 力矩回读不符。")

    def close(self):
        self.port.close()


class SimBus:
    simulated = True
    path = "SIMULATION"

    def __init__(self):
        self.joints = [dict(id=i, min=lo, max=hi, mode=0, torque=0, goal=p, position=p, moving=0)
                       for i, ((lo,hi),p) in enumerate(zip(SIM_LIMITS, HOME),1)]

    def joint(self, axis):
        return self.joints[axis-1].copy()

    def goal(self, axis, value):
        self.joints[axis-1]["goal"] = value
        if self.joints[axis-1]["torque"]:
            self.joints[axis-1]["position"] = value

    def torque(self, axis, enabled):
        self.joints[axis-1]["torque"] = int(enabled)

    def close(self):
        pass


@contextmanager
def lease(arm, *, simulation=None, pick=False):
    lock = LOCKS[validate_arm(arm)]
    if not lock.acquire(blocking=False):
        raise RuntimeError("本臂串口正在执行另一项操作。")
    try:
        if pick and arm in RECORDING:
            raise RuntimeError("左臂正在记录示教，请先结束记录。")
        if pick:
            MANUAL_OWNER.pop(arm,None)
        bus = simulation if simulation is not None else ServoBus(arm)
        try:
            yield bus
        finally:
            bus.close()
    finally:
        lock.release()


def snapshot(bus):
    return [bus.joint(axis) for axis in IDS]


def engage(bus, ref):
    current = bus.joint(ref["id"])
    p = current["position"]
    in_range(current, p, MARGIN)
    if (abs(p-ref["position"]) > 4 or current["torque"] != ref["torque"] or current["moving"]
            or current["torque"] and abs(current["goal"]-p) > 16):
        raise RuntimeError(f"ID{ref['id']} 在建立保持期间发生变化。")
    axis = current["id"]
    bus.goal(axis,p)
    if not current["torque"]:
        bus.torque(axis,True)
    after = bus.joint(axis)
    if after["torque"] != 1 or abs(after["position"]-p) > 8 or after["goal"] != p:
        raise RuntimeError(f"ID{axis} 保持未通过核验。")
    return p


def read_quiet(bus, before):
    for _ in range(2):
        if not bus.simulated:
            time.sleep(.15)
        now = snapshot(bus)
        for old,j in zip(before,now):
            if (j["moving"] or abs(j["position"]-old["position"]) > 2
                    or any(j[key]!=old[key] for key in ("mode","torque","min","max"))):
                raise RuntimeError(f"ID{j['id']} 起姿尚未停稳。")
    return now


class ArmSessions:
    def __init__(self):
        self.mode = {}
        self.sim = {arm:SimBus() for arm in SERIALS}
        self.events = {}
        self.session_lock = threading.RLock()

    def _sample_recording(self, arm, event):
        # A minimized or closed browser must not interrupt the board's sample clock.
        while not event["finished"].wait(1):
            with self.session_lock:
                if self.events.get(arm) is not event:
                    return
                try:
                    mode=self.mode[arm]
                    with lease(arm,simulation=self.sim[arm] if mode=="simulation" else None) as bus:
                        self._event(arm,"observation",{"joints":snapshot(bus)})
                except (ValueError,RuntimeError,OSError) as exc:
                    event["error"]=str(exc)
                    return

    def _event(self, arm, kind, data):
        event = self.events.get(arm)
        if event is None:
            return
        event["seq"] += 1
        if kind == "action":
            event["actions"] += 1
        row = dict(schema=1, sequence=event["seq"], kind=kind, time_utc=datetime.now(timezone.utc).isoformat(),
                   arm=arm, simulation=self.mode[arm]=="simulation", controller_serial=SERIALS[arm],
                   action_count=event["actions"], data=data)
        with event["file"].open("a",encoding="utf8") as f:
            f.write(json.dumps(row, ensure_ascii=False)+"\n")

    def _result(self, arm, bus, joints):
        stamp=datetime.now(timezone.utc).isoformat()
        error=self.events.get(arm,{}).get("error")
        msg=("示教采样已中断，请结束本次记录："+error) if error else "六轴读数已核对。"
        tele=dict(arm=arm,simulated=bus.simulated,state="connected",updated_at=stamp,joints=joints,
                  holding=all(j["torque"]==1 for j in joints),locked=False,recording=arm in self.events,
                  phase=None,message=msg,port=bus.path)
        return dict(arm=arm,mode=self.mode[arm],session="open",telemetry=tele,stale=False,message=msg)

    def active(self):
        return bool(self.mode)

    def open(self, arm, mode):
        validate_arm(arm)
        if mode not in ("simulation","hardware"):
            raise ValueError("无效示教模式。")
        with self.session_lock:
            if arm in self.events and mode != self.mode.get(arm):
                raise ValueError("请先结束本次记录。")
            with lease(arm,simulation=self.sim[arm] if mode=="simulation" else None) as bus:
                joints=snapshot(bus)
                self.mode[arm]=mode
                return self._result(arm,bus,joints)

    def status(self, arm):
        validate_arm(arm)
        with self.session_lock:
            mode = self.mode.get(arm)
            if not mode:
                return dict(arm=arm,mode=None,session="idle",stale=False,telemetry=None,message="尚未连接。")
            try:
                with lease(arm,simulation=self.sim[arm] if mode=="simulation" else None) as bus:
                    joints=snapshot(bus)
                    return self._result(arm,bus,joints)
            except (OSError,RuntimeError) as exc:
                return dict(arm=arm,mode=mode,session="open",stale=True,telemetry=None,message=str(exc))

    def action(self, arm, operation, *, joint=None, delta=None, supported=False, mode="hardware"):
        validate_arm(arm)
        with self.session_lock:
            if self.mode.get(arm) != mode:
                raise ValueError("请先连接所选模式。")
            if operation=="close" and arm not in self.events:
                del self.mode[arm]
                MANUAL_OWNER.pop(arm,None)
                return self.status(arm)
            if operation in ("hold","release") and supported is not True:
                raise ValueError("请先确认独立支撑已就位。")
            if operation=="record_stop":
                event=self.events.get(arm)
                if event is None:
                    raise ValueError("没有进行中的记录。")
                event["finished"].set()
                # Closing a recording and releasing the software lease must also work
                # when the USB cable has been unplugged during teaching.
                with LOCKS[arm]:
                    self._event(arm,"episode_end",{"sampling_error":event.get("error")})
                    del self.events[arm]
                    RECORDING.pop(arm,None)
                return self.status(arm)
            with lease(arm,simulation=self.sim[arm] if mode=="simulation" else None) as bus:
                if operation=="close":
                    raise ValueError("请先结束示教记录。")
                if operation=="release":
                    MANUAL_OWNER.pop(arm,None)
                    failures=[]
                    for axis in IDS:
                        try:
                            bus.torque(axis,False)
                        except (OSError,RuntimeError) as exc:
                            failures.append(str(exc))
                    if failures:
                        raise RuntimeError("卸力未完全核实："+"；".join(failures))
                else:
                    joints=snapshot(bus)
                    if operation=="hold":
                        for j in joints:
                            in_range(j,j["position"],MARGIN)
                            if j["torque"] or j["moving"]:
                                raise ValueError("请独立支撑后释放六轴并待其停稳，再建立保持。")
                        for ref in read_quiet(bus,joints):
                            engage(bus,ref)
                        MANUAL_OWNER[arm]=(self,mode)
                    elif operation=="jog":
                        if MANUAL_OWNER.get(arm)!=(self,mode):
                            raise ValueError("请在当前示教会话中启用保持后再点动。")
                        if type(joint) is not int or joint not in IDS or type(delta) is not int or abs(delta) not in (8,16,24):
                            raise ValueError("请选择六轴之一，步长为 8、16 或 24 格。")
                        for j in joints:
                            in_range(j,j["position"])
                            if j["torque"]!=1 or j["moving"] or abs(j["position"]-j["goal"])>12:
                                raise ValueError(f"ID{j['id']} 未稳定保持。")
                        j=joints[joint-1]
                        target=max(j["min"]+MARGIN,min(j["max"]-MARGIN,j["position"]+delta))
                        if (target-j["position"])*delta<=0 or abs(target-j["position"])>abs(delta):
                            raise ValueError("该方向的点动已到限位。")
                        bus.goal(joint,target)
                        self._event(arm,"action",{"id":joint,"target":target,"delta":delta})
                        if arm in self.events:
                            self._event(arm,"observation",{"joints":snapshot(bus)})
                    elif operation=="record_start":
                        if arm in self.events:
                            raise ValueError("正在记录。")
                        root=Path(os.environ.get("O6_WORKSTATION_RUNTIME",Path.home()/"o6_workstation"/"runtime"))
                        folder=root/("simulation/episodes" if mode=="simulation" else "episodes")
                        folder.mkdir(parents=True,exist_ok=True)
                        name=f"{arm}_manual_instrument_pick_{datetime.now():%Y%m%d_%H%M%S}_{uuid4().hex[:8]}.jsonl"
                        self.events[arm]={"file":folder/name,"seq":0,"actions":0,"finished":threading.Event()}
                        try:
                            self._event(arm,"episode_start",{"task":"manual_instrument_pick"})
                            self._event(arm,"observation",{"joints":joints})
                        except Exception:
                            del self.events[arm]
                            raise
                        RECORDING[arm]=True
                        threading.Thread(target=self._sample_recording,args=(arm,self.events[arm]),
                                         name="o6-record-"+arm,daemon=True).start()
                    else:
                        raise ValueError("未知示教操作。")
                return self._result(arm,bus,snapshot(bus))

