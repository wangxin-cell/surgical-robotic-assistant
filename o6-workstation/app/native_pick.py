"""O6 native four-point replay; execute only after a current visual request."""
from contextlib import contextmanager
from datetime import datetime
from hashlib import sha256
import json
import os
from pathlib import Path
import threading
import time
from native_motor import HOME, MARGIN, SimBus, engage, in_range, lease, read_quiet, snapshot

HOME_TOLERANCE = 24
HOME_LEAD = 32
HOME_RATE = 16  # encoder counts per second on the longest axis
HOME_MAX_ATTEMPTS = 3
RECORDINGS = {
    1: ("20261127_1", "3f7c50da3ae7287f40f75fac45e6ad47236ced4642a46c3acdda6d45416e7c05"),
    2: ("20261126_2", "16f0b8d9b511377e29d1d3d59b5d65c444807bbc61fcc12dba28576f86460f12"),
    3: ("20261127_3", "7807ed1297d619fbd99ddd50cc37fd6f8e4bb38699b61bed4d06dcdc3266fd50"),
    4: ("20261127_4fuwei", "81ced0cabe03b784add7c7c32deb030ef1f10e11bffe3db366ac0607013ea7a0"),
}


class ReplayStopped(RuntimeError):
    pass


class PoseStepStalled(TimeoutError):
    def __init__(self, axis, detail):
        super().__init__(detail)
        self.axis = axis


def plan_for(slot):
    if type(slot) is not int or slot not in RECORDINGS:
        raise ValueError("点位必须为 1 至 4。")
    suffix, digest = RECORDINGS[slot]
    file = Path(__file__).parent/"arm_tools"/"pick_points"/f"left_manual_instrument_pick_{suffix}.jsonl"
    raw = file.read_bytes()
    if sha256(raw).hexdigest() != digest:
        raise ValueError("点位示教文件与原件不一致。")
    initial = end = prev = None
    actions = []
    phase = ""
    for line in raw.splitlines():
        row = json.loads(line)
        data = row["data"]
        if "joints" in data:
            end = sorted(data["joints"], key=lambda j:j["id"])
            if initial is None:
                initial = end
        if row["kind"]=="phase_marker":
            phase = data["phase"]
        if row["kind"]=="action":
            stamp=datetime.fromisoformat(row["time_utc"])
            actions.append((max(0,(stamp-prev).total_seconds()) if prev else 0,
                            data["id"],data["target"],phase))
            prev=stamp
    return initial,actions,end


def recorded_origins():
    return {str(slot):[j["position"] for j in plan_for(slot)[0]] for slot in RECORDINGS}


def stop_check(stop):
    if stop.is_set():
        raise ReplayStopped("本轮动作已停止，请核对现场保持状态。")


def check_held(j, expected, tolerance):
    in_range(j,j["position"])
    if j["torque"]!=1 or j["goal"]!=expected or abs(j["position"]-expected)>tolerance:
        raise RuntimeError(f"ID{j['id']} 保持或跟随异常：期望 {expected}，目标回读 {j['goal']}，"
                           f"实测 {j['position']}，力矩使能 {j['torque']}，运动标志 {j['moving']}。")


def within_pose(position, target, tolerance, reference=None):
    if abs(position-target) <= tolerance:
        return True
    # A stationary residual measured near the operator's HOME is valid
    # only on that side of the target and only within the measured band.
    return (reference is not None and abs(position-target) <= HOME_TOLERANCE
            and min(target, reference)-2 <= position <= max(target, reference)+2)


def pose_check(bus, targets, tolerance, references=None):
    feedback=snapshot(bus)
    settled=True
    for i,(j,target) in enumerate(zip(feedback,targets)):
        check_held(j,target,36)
        if j["moving"] or not within_pose(j["position"],target,tolerance,references[i] if references else None):
            settled=False
    return settled


def verified_home_axes():
    """Allow only axes that passed the observed HOME correction on this release."""
    runtime = Path(os.environ.get("O6_WORKSTATION_RUNTIME",Path.home()/"o6_workstation/runtime"))
    try:
        receipt = json.loads((runtime/"left-home-trim.json").read_bytes())
        axes = receipt["verified_axes"]
        if (receipt["release"] == Path(__file__).resolve().parents[1].name
                and receipt["policy"] == "left-home-trim-6-v1"
                and receipt["controller"] == "5A7C122569" and receipt["home"] == list(HOME)
                and receipt["passed"] is True and isinstance(axes,list)
                and all(type(axis) is int and axis in (3,4) for axis in axes)):
            return set(axes)
    except (OSError,ValueError,KeyError,TypeError):
        pass
    return set()


def trim_health(bus, axis):
    # The additional movement is restricted to the measured, lightly loaded
    # bench configuration. Read environmental limits from the actual servo;
    # current is diagnostic only (the unit is model-specific).
    model = bus.number(axis,3,2)
    max_temp,max_volt,min_volt = bus.read(axis,13,3)
    feedback = bus.read(axis,60,6)
    load = int.from_bytes(feedback[:2],"little")
    voltage, temperature, status = feedback[2],feedback[3],feedback[5]
    current = bus.number(axis,69,2)
    if (model!=777 or status!=0 or not 0<min_volt<max_volt or not min_volt<=voltage<=max_volt
            or not temperature<max_temp or load>2047 or (load & 1023)>100):
        raise RuntimeError(f"ID{axis} 未满足末端校正工况：型号 {model}，状态 {status}，"
                           f"电压 {voltage/10:.1f} V，温度 {temperature} °C，"
                           f"负载原始值 {load}，电流原始值 {current}。")
    return load,current,status


def correct_home_axis(bus, axis, baseline, stop, absolute_limit, report, trimmed_axes):
    """Single monitored correction for an already-held elbow or wrist. No torque writes."""
    target = HOME[axis-1]
    initial = baseline[axis-1]["position"]
    error = initial-target
    if axis not in (3,4) or not 9<=abs(error)<=16:
        raise ValueError("本轮静差超出单次末端校正范围。")
    direction = 1 if error<0 else -1
    offset = min(6,20-abs(error))
    command = target + direction*offset
    for j,q in zip(baseline,HOME):
        check_held(j,q,16)
        if j["moving"]:
            raise RuntimeError("校正前请等待六轴停稳。")
    in_range(baseline[axis-1],command,MARGIN)
    trim_health(bus,axis)
    j = bus.joint(axis)
    check_held(j,target,16)
    if j["moving"] or abs(j["position"]-command)>20 or abs(j["position"]-initial)>1:
        raise RuntimeError(f"ID{axis} 末端校正前姿态已变化。")
    deadline = min(absolute_limit,time.monotonic()+1.0)
    if deadline-time.monotonic()<.9:
        raise TimeoutError("复位剩余时间不足，取消末端校正。")
    report(f"ID{axis} · 单次末端校正 {command-target:+d} 格，随后恢复公共目标 {target}。",0)
    stop_check(stop)
    bus.goal(axis,command)
    if trimmed_axes is not None:
        trimmed_axes.add(axis)
    best = 0
    reached = False
    while time.monotonic()<deadline:
        stop_check(stop)
        load,current,status = trim_health(bus,axis)
        complete = True
        for k,ref in enumerate(baseline,1):
            stop_check(stop)
            if time.monotonic()>=deadline:
                complete = False
                break
            j = bus.joint(k)
            check_held(j,command if k==axis else HOME[k-1],20 if k==axis else 16)
            if k==axis:
                pos = j["position"]
                progress = (pos-initial)*direction
                if progress<best-2 or (pos-command)*direction>2:
                    raise RuntimeError(f"ID{axis} 校正方向或位移异常：实测 {pos}，原目标 {target}。")
                best = max(best,progress)
                reached = abs(pos-target)<=8
            elif j["moving"] or abs(j["position"]-ref["position"])>2:
                raise RuntimeError(f"ID{k} 在 ID{axis} 校正期间移动，已停止。")
        reached = reached and complete
        if reached or not complete:
            break
        stop.wait(.03)
    stop_check(stop)
    j = bus.joint(axis)
    check_held(j,command,20)
    load,current,status = trim_health(bus,axis)
    stop_check(stop)
    bus.goal(axis,target)
    if not reached:
        # Capture the actual failed read before outer stop logic changes any goal.
        raise TimeoutError(f"ID{axis} 单次校正未到位，已恢复目标 {target}。校正末次实测 "
                           f"{j['position']}，最大向目标移动 {best} 格，负载原始值 {load}，"
                           f"电流原始值 {current}，舵机状态 {status}。")


class PoseAttempts:
    """Share a single retry budget across travel and final settling."""

    def __init__(self, report, stage):
        self.number = 1
        self.report = report
        self.stage = stage

    def repeat(self, bus, expected, axes, stop, deadline, max_gap, reason):
        stop_check(stop)
        if self.number >= HOME_MAX_ATTEMPTS:
            raise TimeoutError(f"{self.stage} {self.number} 轮后仍未到位。{reason}")
        before = snapshot(bus)
        current = read_quiet(bus, before)
        stop_check(stop)
        for j, goal in zip(current, expected):
            check_held(j, goal, max_gap)
            in_range(j, goal, MARGIN)
            in_range(j, j["position"], MARGIN)
            if before[j["id"]-1]["goal"] != goal or j["moving"]:
                raise RuntimeError(f"ID{j['id']} 在重试前保持状态发生变化。{reason}")
        self.number += 1
        self.report(f"{self.stage} · 第 {self.number}/{HOME_MAX_ATTEMPTS} 轮，重新发送 ID"
                    + "、".join(map(str, axes)) + " 当前的保持目标。", 0)
        for axis in axes:
            stop_check(stop)
            goal = expected[axis-1]
            j = bus.joint(axis)
            check_held(j, goal, max_gap)
            in_range(j, goal, MARGIN)
            in_range(j, j["position"], MARGIN)
            if j["moving"] or abs(j["position"]-before[axis-1]["position"]) > 2:
                raise RuntimeError(f"ID{axis} 在重试前发生移动。{reason}")
            if hasattr(bus, "read"):
                feedback = bus.read(axis, 60, 6)
                load = int.from_bytes(feedback[:2], "little")
                if feedback[5] or load > 2047 or (load & 1023) > 100:
                    raise RuntimeError(f"ID{axis} 重试已停止，状态 {feedback[5]}，负载原始值 {load}。{reason}")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{self.stage}已到等待上限。{reason}")
            stop_check(stop)
            bus.goal(axis, goal)


def check_servo_state(bus, joint, *, check_load):
    if not hasattr(bus, "read"):
        return
    data = bus.read(joint["id"], 60, 6)
    load = int.from_bytes(data[:2], "little")
    if data[5] or (check_load and (load > 2047 or (load & 1023) > 100)):
        raise RuntimeError(f"ID{joint['id']} 状态 {data[5]}，负载原始值 {load}，停止运动。")


def settle_pose(bus, targets, stop, stage, report, *, trim_axes=(), trimmed_axes=None, attempts=None, references=None):
    """Keep the final goals latched and get three consecutive stationary scans."""
    start = time.monotonic()
    deadline = start + 5.
    limit = start + 10.
    still_from = None
    baseline = None
    best_error = None
    reissued = False
    eligible = set(trim_axes) & {3,4} if stage=="公共复位" and tuple(targets)==HOME else set()
    candidates = set()
    attempted = set()
    good = 0
    ordinary_home = attempts is not None and stage in ("公共复位","结束复位") and tuple(targets)==HOME
    tolerance = HOME_TOLERANCE if ordinary_home else 8
    retry_gap = HOME_LEAD if ordinary_home else 20
    first_good = None
    report(f"{stage} · 已到最终指令，正在核对六轴保持反馈。", 0)
    while True:
        stop_check(stop)
        joints = snapshot(bus)
        now = time.monotonic()
        errors = [abs(j["position"]-q) for j,q in zip(joints,targets)]
        for j,target in zip(joints,targets):
            check_held(j,target,36)
        stop_check(stop)
        positions = [j["position"] for j in joints]
        blocked = [i for i,j in enumerate(joints) if j["moving"] or not within_pose(
            j["position"], targets[i], tolerance, references[i] if references else None)]
        if attempts is not None and (blocked or first_good is None or
                any(abs(p-q)>2 for p,q in zip(positions,first_good))):
            good = 0
            first_good = None if blocked else positions
        good = 0 if blocked else good+1
        if good >= 3:
            if ordinary_home or references:
                for j, error in zip(joints, errors):
                    stop_check(stop)
                    check_servo_state(bus, j, check_load=error>8)
                stop_check(stop)
            return joints
        # Allow a slowly converging servo more time, without extending for jitter.
        worst = max(errors)
        if best_error is None or worst <= best_error-2:
            best_error = worst
            deadline = min(limit, max(deadline, now+2.))
        if now >= deadline and (not good or now >= limit):
            details = "；".join(
                f"ID{j['id']} 实测 {j['position']}，目标 {targets[i]}（回读 {j['goal']}），"
                f"差 {j['position']-targets[i]:+d} 格，力矩 {j['torque']}，运动 {j['moving']}"
                for i,j in enumerate(joints) if i in blocked
            )
            rounds = f"（第 {attempts.number}/{HOME_MAX_ATTEMPTS} 轮）" if attempts is not None else ""
            raise TimeoutError(f"{stage}{rounds}未稳定，连续合格 {good}/3 次。"
                               + (details or "达到等待上限。"))
        if any(j["moving"] for j in joints):
            still_from = baseline = None
        elif baseline is None or any(abs(a-b)>2 for a,b in zip(baseline,positions)):
            baseline = positions
            still_from = now
        elif (attempts is not None and blocked and attempts.number < HOME_MAX_ATTEMPTS
              and now-still_from >= 1.2 and worst<=retry_gap):
            attempts.repeat(bus, targets, [i+1 for i in blocked], stop, limit, retry_gap, "最终目标尚未到位。")
            still_from,baseline = None,None
            deadline = min(limit,max(deadline,time.monotonic()+3.))
        elif attempts is None and blocked and not reissued and now-still_from >= 1.2 and worst<=16:
            # A stationary servo just outside tolerance may have missed the final
            # small change. Repeat only that exact goal once, while already held.
            for i in blocked:
                stop_check(stop)
                j = bus.joint(i+1)
                check_held(j,targets[i],16)
                if not j["moving"] and abs(j["position"]-targets[i])>8:
                    stop_check(stop)
                    bus.goal(i+1,targets[i])
                    candidates.add(i+1)
            reissued = True
            still_from,baseline = None,None
            deadline = min(limit, max(deadline, now+3.))
            report(f"{stage} · 已复核保持目标，等待偏差收敛。", 0)
        elif (reissued and blocked and still_from is not None and now-still_from>=1.
                and candidates.issubset(eligible)
                and {i+1 for i in blocked}.issubset((eligible & candidates)-attempted)
                and worst<=16):
            axis = blocked[0]+1
            attempted.add(axis)
            correct_home_axis(bus,axis,joints,stop,limit,report,trimmed_axes)
            good = 0
            still_from,baseline = None,None
            deadline = min(limit,max(deadline,time.monotonic()+3.))
        if not bus.simulated:
            stop.wait(.1)


def follow_feedback(bus, j):
    text = (f"ID{j['id']} 保持目标 {j['goal']}，实测 {j['position']}，"
            f"力矩 {j['torque']}，运动 {j['moving']}")
    if hasattr(bus,"read"):
        data = bus.read(j["id"],60,11)
        text += (f"，负载原始值 {int.from_bytes(data[:2],'little')}，"
                 f"电流原始值 {int.from_bytes(data[9:11],'little')}，状态 {data[5]}")
        if data[5]:
            raise RuntimeError(text)
    return text


def advance_pose_step(bus, expected, proposed, stop, absolute_limit, *, stage, targets, attempts=None, leads=None):
    """Complete this same interpolation step before allowing path time to advance."""
    waiting = {}
    paused = False
    def record_wait(j, step, blocked):
        axis = j["id"]
        blocked.add(axis)
        info = follow_feedback(bus,j)
        now = time.monotonic()
        gap = abs(step-j["position"])
        best,since = waiting.get(axis,(gap,now))
        if gap<=best-2:
            best,since = gap,now
        waiting[axis] = (best,since)
        if now-since>=1.8:
            target = targets[axis-1]
            raise PoseStepStalled(axis, f"{stage}等待跟随，1.8 秒无有效进展。{info}。"
                                  f"本轴最终目标 {target} 格，实测与最终目标相差 {j['position']-target:+d} 格。")
        if attempts is not None and axis not in waiting_reported:
            attempts.report(f"{stage} · 等待 ID{axis} 跟随，当前指令 {j['goal']}，实测 {j['position']}。", 0)
            waiting_reported.add(axis)

    def read_held(axis,old):
        j = bus.joint(axis)
        try:
            check_held(j,old,leads[axis-1]+4 if leads else (20 if attempts is not None else 16))
        except (ValueError,RuntimeError) as exc:
            raise RuntimeError(f"{stage} · {exc}") from exc
        return j

    waiting_reported = set()
    while True:
        stop_check(stop)
        if time.monotonic() >= absolute_limit:
            raise TimeoutError(f"{stage}轨迹超时。")
        fraction = 1.
        blocked = set()
        catching_up = False
        for index,(old,step) in enumerate(zip(expected,proposed)):
            stop_check(stop)
            j = read_held(index+1,old)
            lead = leads[index] if leads else 16
            at_endpoint = old == step == targets[index]
            if abs(old-j["position"])>lead and not at_endpoint:
                record_wait(j,old,blocked)
                catching_up = True
                continue
            delta = step-old
            gap = abs(step-j["position"])
            if not delta or gap<=lead:
                continue
            record_wait(j,step,blocked)
            direction = 1 if delta>0 else -1
            available = lead - direction*(old-j["position"])
            fraction = min(fraction,available/abs(delta))
        # If the first scan needs a shorter step, send the feasible fraction of
        # the original segment once. Hold it until the whole proposed step fits.
        if not catching_up and (fraction==1. or not paused):
            for index,(old,step) in enumerate(zip(expected,proposed)):
                goal = old+int((step-old)*fraction)
                if goal==old:
                    continue
                stop_check(stop)
                j = read_held(index+1,old)
                lead = leads[index] if leads else 16
                if abs(old-j["position"])>lead:
                    record_wait(j,old,blocked)
                    break
                if abs(goal-j["position"])>lead:
                    # The encoder moved between the group scan and this write.
                    record_wait(j,step,blocked)
                    break
                in_range(j,goal,MARGIN)
                if time.monotonic()>=absolute_limit:
                    raise TimeoutError(f"{stage}轨迹超时。")
                stop_check(stop)
                bus.goal(index+1,goal)
                expected[index] = goal
        if not catching_up and expected==proposed:
            return paused
        waiting = {axis:state for axis,state in waiting.items() if axis in blocked}
        paused = True
        if not bus.simulated:
            stop.wait(.05)


def read_pose_start(bus, targets, stop, stage, report, allow_wait, *, ordinary_home=False, leads=None):
    initial=snapshot(bus)
    # A preceding controller may have left a small tracking lag. Observe the
    # already latched goals before claiming the axes or sending any command.
    pending = []
    for i,(j,target) in enumerate(zip(initial,targets)):
        in_range(j,j["position"],MARGIN)
        in_range(j,target,MARGIN)
        max_gap = leads[i] if leads else (20 if allow_wait else 16)
        if ((not ordinary_home and abs(j["position"]-target)>1000) or j["moving"] or j["torque"] not in (0,1)
                or j["torque"] and abs(j["goal"]-j["position"])>max_gap):
            raise ValueError(f"{stage} · ID{j['id']} 起始状态超出允许范围。保持目标 {j['goal']}，"
                             f"实测 {j['position']}，目的位置 {target}，力矩 {j['torque']}，运动 {j['moving']}。")
        if leads:
            check_servo_state(bus, j, check_load=j["torque"] and abs(j["goal"]-j["position"])>16)
        elif j["torque"] and abs(j["goal"]-j["position"])>16:
            pending.append(j)
    if pending:
        report(f"{stage} · 正在等待读数追上原保持目标。", 0)
        deadline = time.monotonic()+1.8
        original = initial
        while pending:
            stop_check(stop)
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{stage}启动前保持偏差持续。{follow_feedback(bus,pending[0])}。")
            if not bus.simulated:
                stop.wait(.05)
            initial = snapshot(bus)
            pending = []
            for j,old,target in zip(initial,original,targets):
                in_range(j,j["position"],MARGIN)
                was_lagging = old["torque"] and abs(old["goal"]-old["position"])>16
                if (abs(j["position"]-target)>1000 or
                        any(j[k]!=old[k] for k in ("id","goal","torque","min","max","mode")) or
                        (not was_lagging and (j["moving"] or abs(j["position"]-old["position"])>2))):
                    raise RuntimeError(f"{stage} · ID{j['id']} 起始保持状态发生变化。")
                if j["torque"]:
                    check_held(j,old["goal"],20)
                    if j["moving"] or abs(j["position"]-j["goal"])>16:
                        pending.append(j)
    quiet = read_quiet(bus,initial)
    if any(j["goal"] != ref["goal"] for j, ref in zip(quiet,initial)):
        raise RuntimeError(f"{stage} · 等待静止期间原保持目标发生变化。")
    return quiet


def move_pose(bus, targets, stop, stage, report, touched, *, trim_axes=None, trimmed_axes=None, references=None):
    attempts = PoseAttempts(report, stage) if trim_axes is None else None
    ordinary_home = (attempts is not None and stage in ("公共复位","结束复位") and tuple(targets)==HOME)
    leads = [HOME_LEAD]*6 if ordinary_home or references is not None else None
    current = read_pose_start(bus, targets, stop, stage, report, attempts is not None,
                              ordinary_home=ordinary_home, leads=leads)
    starts=[]
    for j,target in zip(current,targets):
        stop_check(stop)
        axis = j["id"]
        goal = j["goal"]
        # Continue from an existing, verified setpoint if it already lies on
        # the path to the target. Re-latching a loaded joint can add sag.
        if j["torque"]==1 and (leads or min(j["position"],target)<=goal<=max(j["position"],target)):
            live = bus.joint(axis)
            in_range(live,live["position"],MARGIN)
            in_range(live,goal,MARGIN)
            check_held(live,goal,leads[axis-1] if leads else 16)
            if (live["moving"] or abs(live["position"]-j["position"])>4
                    or not ordinary_home and abs(live["position"]-target)>1000):
                raise RuntimeError(f"ID{axis} 在确认当前位置保持期间移动。")
            starts.append(goal)
            touched.add(axis)
        else:
            touched.add(axis)
            starts.append(engage(bus,j))
    expected=list(starts)
    travel=max(abs(p-q) for p,q in zip(starts,targets))
    duration=max(1.,travel/HOME_RATE)
    origin=last=time.monotonic()
    elapsed=0.
    shown=-1
    while travel and elapsed < duration:
        stop_check(stop)
        now=time.monotonic()
        if now-origin>2*duration+10:
            raise TimeoutError("复位或起姿对齐超时。")
        elapsed=min(duration,elapsed+(.08 if bus.simulated else min(.15,now-last)))
        last=now
        t=elapsed/duration
        linear = ordinary_home or references is not None
        fraction=t if linear else t*t*(3-2*t)
        proposed=[]
        for index,(p,q) in enumerate(zip(starts,targets)):
            step=round(p+(q-p)*fraction)
            # Leave the last (at most eight) counts as one bounded update at
            # the scheduled endpoint instead of restarting the servo profile
            # for a series of one-count updates during deceleration.
            if not linear and abs(q-expected[index])<=8 and step!=q:
                step=expected[index]
            proposed.append(step)
        limit = origin+2*duration+10
        while True:
            try:
                if advance_pose_step(bus,expected,proposed,stop,limit,stage=stage,targets=targets,attempts=attempts,leads=leads):
                    last=time.monotonic()
                break
            except PoseStepStalled as exc:
                if attempts is None:
                    raise
                attempts.repeat(bus, expected, [exc.axis], stop, limit, max(leads)+4 if leads else 20, str(exc))
                last=time.monotonic()
        percent=int(100*t)
        if percent//5!=shown:
            current_round = f"第 {attempts.number}/{HOME_MAX_ATTEMPTS} 轮 · " if attempts is not None else ""
            report(f"{stage} · {current_round}{percent}%",0)
            shown=percent//5
        if expected == list(targets):
            break
        if not bus.simulated:
            stop.wait(.05)
    return settle_pose(bus,targets,stop,stage,report,trim_axes=trim_axes or (),trimmed_axes=trimmed_axes,
                       attempts=attempts,references=references)


def verify_at_pose(check_start, stop, verify):
    check_start()
    stop_check(stop)
    finished=threading.Event()
    errors=[]
    def worker():
        try:
            verify()
        except BaseException as err:
            errors.append(err)
        finally:
            finished.set()
    threading.Thread(target=worker, name="o6-pick-vision",daemon=True).start()
    timeout=time.monotonic()+90
    next_scan=time.monotonic()+.6
    while not finished.wait(.04):
        stop_check(stop)
        if time.monotonic()>timeout:
            raise TimeoutError("起姿复拍超过 90 秒。")
        if time.monotonic()>=next_scan:
            check_start()
            next_scan=time.monotonic()+.6
    stop_check(stop)
    if errors:
        raise errors[0]
    check_start()


@contextmanager
def hold_on_failure(bus):
    """Keep feedback hold on claimed axes after either a reset or a pick stops."""
    touched = set()
    try:
        yield touched
    except BaseException as failure:
        unknown = []
        for axis in sorted(touched):
            try:
                j = bus.joint(axis)
                if j["torque"] == 1:
                    in_range(j, j["position"])
                    bus.goal(axis, j["position"])
            except Exception:
                unknown.append(axis)
        if unknown:
            raise RuntimeError(f"{failure} 另有 ID{unknown} 保持状态未确认。") from failure
        raise


def reset_left_home(stop, report):
    stop_check(stop)
    with lease("left", pick=True) as bus, hold_on_failure(bus) as touched:
        measured = move_pose(bus, HOME, stop, "公共复位", report, touched)
        stop_check(stop)
        positions = "、".join(str(j["position"]) for j in measured)
        return f"六轴已连续三次停稳。ID1 至 ID6 实测依次为 {positions} 格。"


def check_plan_start(bus, plan, stop):
    first,actions,_=plan
    stop_check(stop)
    current=snapshot(bus)
    targets=[j["position"] for j in first]
    for j,ref in zip(current,first):
        in_range(j,j["position"],MARGIN)
        if (j["id"],j["min"],j["max"])!=(ref["id"],ref["min"],ref["max"]):
            raise ValueError(f"ID{j['id']} 与示教限位不符。")
        in_range(j,ref["position"],MARGIN)
    for _,axis,target,_ in actions:
        in_range(current[axis-1],target,MARGIN)
    return read_quiet(bus,current), targets


def align_origin(bus, slot, start, targets, stop, report, touched):
    stop_check(stop)
    near_home = all(abs(j["position"]-home)<=HOME_TOLERANCE for j,home in zip(start,HOME))
    references = [j["position"] if near_home and j["torque"]==1 and abs(j["goal"]-home)<=8
                  and abs(home-target)<=8 and abs(j["position"]-target)<=HOME_TOLERANCE
                  and abs(j["position"]-home)>8 else None
                  for j,home,target in zip(start,HOME,targets)]
    move_pose(bus,targets,stop,f"对齐点位 {slot} 起姿",report,touched,references=references)
    return references


def prepare_slot_origin(slot, stop, report):
    plan = plan_for(slot)
    with lease("left",pick=True) as bus, hold_on_failure(bus) as touched:
        start, targets = check_plan_start(bus,plan,stop)
        return align_origin(bus,slot,start,targets,stop,report,touched)


def recorded_pause(delay):
    # Forward playback cadence used by the supplied BimanualArmTeach.ps1.
    return max(.06, min(.30, delay))


def wait_for_recorded_step(bus, joint, expected, target, stop):
    deadline = time.monotonic() + 1.2
    axis = joint["id"]
    while True:
        stop_check(stop)
        in_range(joint, joint["position"])
        in_range(joint, target, MARGIN)
        if joint["torque"] != 1 or joint["goal"] != expected:
            raise RuntimeError(f"ID{axis} 保持状态改变，轨迹已停止。")
        if abs(joint["position"] - expected) <= 36 and abs(joint["position"] - target) <= 48:
            return
        if time.monotonic() >= deadline:
            raise TimeoutError(f"ID{axis} 未跟上录制路径。已发 {expected}，实测 {joint['position']}，下一步 {target}。")
        check_servo_state(bus, joint, check_load=True)
        stop.wait(.04)
        joint = bus.joint(axis)


def execute(bus, slot, plan, stop, report, verify):
    _,actions,last=plan
    start,targets=check_plan_start(bus,plan,stop)
    expected = [joint["goal"] for joint in start]

    def check_start():
        for joint, goal, target, record in zip(snapshot(bus), expected, targets, plan[0]):
            measured = joint["position"]
            if (joint["moving"] or abs(measured - target) > 30
                    or min(abs(goal - target), abs(goal - record["goal"])) > 30):
                raise ValueError(f"点位 {slot} 起姿不符：ID{joint['id']} 实测 {measured}，录制起姿 {target}。"
                                 "请先手动恢复录制起姿并启用保持，再发起请求。")
            check_held(joint, goal, 36)

    verify_at_pose(check_start, stop, verify)
    with hold_on_failure(bus) as touched:
        next_scan=time.monotonic()+.65
        for n,(delay,axis,target,phase) in enumerate(actions,1):
            stop_check(stop)
            if time.monotonic() >= next_scan:
                for joint, goal in zip(snapshot(bus), expected):
                    wait_for_recorded_step(bus, joint, goal, goal, stop)
                next_scan = time.monotonic() + .65
            wait_for_recorded_step(bus, bus.joint(axis), expected[axis-1], target, stop)
            stop_check(stop)
            touched.add(axis)
            bus.goal(axis,target)
            expected[axis-1]=target
            if n==1 or n%5==0 or n==len(actions):
                report(f"{phase or '示教回放'} · {n}/{len(actions)}",n/len(actions))
            if not bus.simulated:
                stop.wait(recorded_pause(delay))
        deadline=time.monotonic()+4
        first_still = None
        still = 0
        while True:
            stop_check(stop)
            joints=snapshot(bus)
            for j,goal in zip(joints,expected):
                check_held(j,goal,36)
            positions = [j["position"] for j in joints]
            if all(not j["moving"] and abs(j["position"]-end["position"])<=24 for j,end in zip(joints,last)):
                if first_still is None or any(abs(a-b)>2 for a,b in zip(positions,first_still)):
                    first_still, still = positions, 0
                still += 1
                if still >= 3:
                    break
            else:
                first_still, still = None, 0
            if time.monotonic()>deadline:
                raise TimeoutError("录制结束姿态未能核实。")
            if not bus.simulated:
                stop.wait(.1)
        stop_check(stop)
        return f"点位 {slot} 的 {len(actions)} 步原始路径已回放，左臂停在文件终点。请核对实物。"


def replay_slot(slot,stop,report,verify_target):
    plan=plan_for(slot)
    stop_check(stop)
    with lease("left",pick=True) as bus:
        return execute(bus,slot,plan,stop,report,verify_target)

