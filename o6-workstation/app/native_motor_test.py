"""Offline checks for O6 motor limits, stop handling and original four-point motion."""
import os
import json
import itertools
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event
import unittest
from unittest.mock import Mock, patch
import native_motor as m
import native_pick as p


class RecordingBus(m.SimBus):
    def __init__(self):
        super().__init__()
        self.writes=[]

    def goal(self, axis, target):
        self.writes.append((axis,target))
        super().goal(axis,target)

    def torque(self, axis, enabled):
        self.writes.append((axis,enabled))
        super().torque(axis,enabled)


class RetryElbowBus(RecordingBus):
    """The encoder follows each phase only after its latched goal is repeated."""
    def __init__(self, travel_repeats, endpoint_repeats):
        super().__init__()
        for j in self.joints:
            j["torque"] = 1
        self.joints[2].update(goal=2190, position=2196)
        self.travel_repeats = travel_repeats
        self.endpoint_repeats = endpoint_repeats
        self.repeats = []
        self.before_write = []
        self.status = 0

    def goal(self, axis, target):
        j = self.joint(axis)
        self.before_write.append((axis, j["goal"], j["position"], target))
        super().goal(axis, target)
        if axis != 3:
            return
        terminal = target == m.HOME[2]
        if target == j["goal"]:
            self.repeats.append(target)
            if terminal:
                self.endpoint_repeats -= 1
            else:
                self.travel_repeats -= 1
        if terminal and self.endpoint_repeats > 0:
            self.joints[2]["position"] = m.HOME[2] + 26
        elif not terminal and self.travel_repeats > 0:
            self.joints[2]["position"] = j["position"]

    def read(self, axis, address, count):
        assert (address, count) in ((60, 11), (60, 6))
        return bytes((36, 0, 126, 38, 0, self.status, 0, 0, 0, 2, 0))[:count]


class MotorTests(unittest.TestCase):
    class StickyHomeBus(RecordingBus):
        def __init__(self):
            super().__init__()
            for j in self.joints:
                j["torque"]=1
            self.joints[2]["position"]+=14
            self.joints[3]["position"]+=13
            self.load=36
            self.stuck=False

        def goal(self,axis,target):
            self.writes.append((axis,target))
            j=self.joints[axis-1]
            j["goal"]=target
            if target != m.HOME[axis-1] and not self.stuck:
                j["position"]=m.HOME[axis-1]+6

        def number(self,axis,address,count):
            return {3:777,69:0}[address]

        def read(self,axis,address,count):
            if address==13 and count==3:
                return bytes((80,140,90))
            if address==60 and count==6:
                return self.load.to_bytes(2,"little")+bytes((126,38,0,0))
            raise AssertionError("unexpected register")

    def test_home_trim_is_sequential_and_restores_both_exact_goals(self):
        bus=self.StickyHomeBus()
        exercised=set()
        with patch.object(p.time,"monotonic",side_effect=itertools.count(0,.02)):
            p.settle_pose(bus,m.HOME,Event(),"公共复位",Mock(),trim_axes={3,4},trimmed_axes=exercised)
        self.assertEqual(exercised,{3,4})
        self.assertEqual(bus.writes,[(3,2133),(4,2177),(3,2127),(3,2133),(4,2171),(4,2177)])
        self.assertTrue(p.pose_check(bus,m.HOME,8))

    def test_servo_reaches_trial_goal_then_qualifies_at_nominal(self):
        class Responsive(self.StickyHomeBus):
            def goal(self,axis,target):
                super().goal(axis,target)
                if target!=m.HOME[axis-1]:
                    self.joints[axis-1]["position"]=target
        b=Responsive()
        with patch.object(p.time,"monotonic",side_effect=itertools.count(0,.02)):
            p.settle_pose(b,m.HOME,Event(),"公共复位",Mock(),trim_axes={3,4})
        self.assertEqual([j["goal"] for j in b.joints],list(m.HOME))
        self.assertTrue(p.pose_check(b,m.HOME,8))

    def test_peer_movement_stops_trim_without_further_goal_writes(self):
        b=self.StickyHomeBus()
        goal=b.goal
        def drift(axis,target):
            goal(axis,target)
            b.joints[0]["position"]+=3
        b.goal=drift
        with self.assertRaisesRegex(RuntimeError,"ID1.*移动"):
            p.correct_home_axis(b,3,m.snapshot(b),Event(),p.time.monotonic()+3,Mock(),set())
        self.assertEqual(b.writes,[(3,2127)])

    def test_trim_never_enabled_for_slot_pose_or_unverified_axis(self):
        for stage,targets,axes in (("公共复位",m.HOME,{4}),("对齐",m.HOME,{3,4}),
                                   ("公共复位",m.HOME[:-1]+(1646,),{3,4})):
            b=self.StickyHomeBus()
            b.joints[5]["goal"]=targets[5]
            with patch.object(p.time,"monotonic",side_effect=itertools.count(0,.03)):
                with self.assertRaises(TimeoutError):
                    p.settle_pose(b,targets,Event(),stage,Mock(),trim_axes=axes)
            self.assertTrue(all(target==m.HOME[i-1] for i,target in b.writes))

    def test_load_refuses_trim_and_no_progress_restores_nominal(self):
        b=self.StickyHomeBus()
        b.load=140
        with self.assertRaisesRegex(RuntimeError,"负载原始值 140"):
            p.correct_home_axis(b,3,m.snapshot(b),Event(),p.time.monotonic()+3,Mock(),set())
        self.assertFalse(b.writes)
        b.load=36
        b.stuck=True
        with patch.object(p.time,"monotonic",side_effect=itertools.count(0,.02)):
            with self.assertRaisesRegex(TimeoutError,"最大向目标移动 0"):
                p.correct_home_axis(b,3,m.snapshot(b),Event(),10.,Mock(),set())
        self.assertEqual(b.writes,[(3,2127),(3,2133)])

    def test_no_correction_before_hardware_bound_receipt(self):
        with TemporaryDirectory() as tmp, patch.dict(os.environ,O6_WORKSTATION_RUNTIME=tmp):
            record=Path(tmp)/"left-home-trim.json"
            self.assertEqual(p.verified_home_axes(),set())
            valid={"release":Path(p.__file__).resolve().parents[1].name,
                   "policy":"left-home-trim-6-v1","controller":"5A7C122569","home":list(m.HOME),
                   "verified_axes":[3],"passed":True}
            for bad in ({"verified_axes":[6]},{"release":"earlier"},{"passed":False},{"controller":"different"}):
                record.write_text(json.dumps({**valid,**bad}),"utf8")
                self.assertEqual(p.verified_home_axes(),set())
            record.write_text(json.dumps(valid),"utf8")
            self.assertEqual(p.verified_home_axes(),{3})

    def test_held_near_home_is_not_relatched(self):
        bus=RecordingBus()
        for j in bus.joints:
            j["torque"]=1
        bus.joints[3].update(position=m.HOME[3]+8,goal=m.HOME[3]+6)
        p.move_pose(bus,m.HOME,Event(),"复位",Mock(),set())
        self.assertEqual(bus.writes,[(4,m.HOME[3])])

    def test_workstation_home_accepts_observed_static_elbow_wrist_errors(self):
        bus = self.StickyHomeBus()
        measured = p.move_pose(bus, m.HOME, Event(), "公共复位", Mock(), set())
        self.assertEqual([j["position"] - target for j, target in zip(measured, m.HOME)],
                         [0, 0, 14, 13, 0, 0])
        self.assertEqual(bus.writes, [])
        with patch.object(p.time, "monotonic", side_effect=itertools.count(0, .02)):
            with self.assertRaises(TimeoutError):
                p.move_pose(bus, m.HOME, Event(), "对齐点位 1 起姿", Mock(), set())

    def test_home_recovers_on_second_or_third_attempt(self):
        for travel, terminal in ((1, 0), (2, 0), (0, 1), (0, 2), (1, 1)):
            with self.subTest(travel=travel, terminal=terminal):
                bus = RetryElbowBus(travel, terminal)
                with patch.object(p.time, "monotonic", side_effect=itertools.count(0, .004)):
                    measured = p.move_pose(bus, m.HOME, Event(), "公共复位", Mock(), set())
                self.assertEqual([j["position"] for j in measured], list(m.HOME))
                self.assertEqual(len(bus.repeats), travel + terminal)
                self.assertEqual(bus.repeats.count(m.HOME[2]), terminal)
                self.assertTrue(all(abs(target - position) <= p.HOME_LEAD
                                    for _, old, position, target in bus.before_write if target != old))

    def test_home_shares_two_retries_across_the_entire_move(self):
        for travel, terminal in ((3, 0), (0, 3), (1, 2), (2, 1)):
            with self.subTest(travel=travel, terminal=terminal):
                bus = RetryElbowBus(travel, terminal)
                with patch.object(p.time, "monotonic", side_effect=itertools.count(0, .004)):
                    with self.assertRaises(TimeoutError):
                        p.move_pose(bus, m.HOME, Event(), "公共复位", Mock(), set())
                self.assertEqual(len(bus.repeats), 2)
                self.assertNotEqual(bus.joint(3)["position"], m.HOME[2])

    def test_home_does_not_retry_servo_fault_or_operator_stop(self):
        for fault in ("servo", "load", "stop"):
            with self.subTest(fault=fault):
                bus = RetryElbowBus(1, 0)
                stop = Event()
                if fault == "servo":
                    bus.status = 4
                    expected = RuntimeError
                else:
                    original = bus.read
                    def change_at_retry(axis, address, count):
                        packet = original(axis, address, count)
                        if count == 6:
                            if fault == "stop":
                                stop.set()
                            else:
                                return (135).to_bytes(2, "little") + packet[2:]
                        return packet
                    bus.read = change_at_retry
                    expected = p.ReplayStopped if fault == "stop" else RuntimeError
                with patch.object(p.time, "monotonic", side_effect=itertools.count(0, .004)):
                    with self.assertRaises(expected):
                        p.move_pose(bus, m.HOME, stop, "公共复位", Mock(), set())
                self.assertEqual(bus.repeats, [])

    def test_home_does_not_count_oscillating_encoder_as_three_still_samples(self):
        bus = RecordingBus()
        for joint in bus.joints:
            joint["torque"] = 1
        errors = itertools.cycle((4, 15))
        def sample(_):
            bus.joints[2]["position"] = m.HOME[2] + next(errors)
            return m.snapshot(bus)
        with patch.object(p, "snapshot", side_effect=sample), \
             patch.object(p.time, "monotonic", side_effect=itertools.count(0, .01)):
            with self.assertRaises(TimeoutError):
                p.settle_pose(bus, m.HOME, Event(), "公共复位", Mock(), attempts=p.PoseAttempts(Mock(), "公共复位"))
        self.assertFalse(bus.writes)

    def test_home_waits_for_delayed_measured_elbow_or_reports_stall(self):
        for recovers in (True,False):
            class DelayedElbow(RecordingBus):
                def __init__(self):
                    super().__init__()
                    self.limited_at=None
                    self.gaps=[]
                    for j in self.joints:
                        j["torque"]=1
                    self.joints[2].update(position=2159,goal=2153)

                def goal(self,axis,value):
                    pos=self.joints[axis-1]["position"]
                    self.gaps.append(abs(value-pos))
                    super().goal(axis,value)
                    if axis==3 and (not recovers or self.limited_at is None
                                    or p.time.monotonic()-self.limited_at<.35):
                        self.joints[2]["position"]=pos
                        if value<=2143 and self.limited_at is None:
                            self.limited_at=p.time.monotonic()

                def joint(self,axis):
                    if (axis==3 and recovers and self.limited_at is not None
                            and p.time.monotonic()-self.limited_at>=.35):
                        self.joints[2]["position"]=self.joints[2]["goal"]
                    return super().joint(axis)

                def read(self,axis,address,count):
                    assert (axis,address,count)==(3,60,11)
                    return bytes([36,0,126,38,0,0,0,0,0,2,0])

            b=DelayedElbow()
            with patch.object(p.time,"monotonic",side_effect=itertools.count(0,.009)):
                if recovers:
                    p.move_pose(b,m.HOME,Event(),"公共复位",Mock(),set(),trim_axes=())
                    self.assertTrue(p.pose_check(b,m.HOME,8))
                else:
                    with self.assertRaisesRegex(TimeoutError,"ID3.*实测 2159.*负载原始值 36.*电流原始值 2.*状态 0"):
                        p.move_pose(b,m.HOME,Event(),"公共复位",Mock(),set(),trim_axes=())
                    self.assertEqual(b.joints[2]["goal"],2143)
            self.assertLessEqual(max(b.gaps),16)
            written=[v for i,v in b.writes if i==3]
            self.assertEqual(written,sorted(set(written),reverse=True))

    def test_third_stationary_read_at_timeout_is_accepted(self):
        bus=RecordingBus()
        for j in bus.joints:
            j["torque"]=1
        with patch.object(p.time,"monotonic",side_effect=[0,2,4,5.1]):
            p.settle_pose(bus,m.HOME,Event(),"复位",Mock())
        self.assertEqual(bus.writes,[])

    def test_standalone_home_from_58_counts_with_actual_feedback(self):
        self.assertEqual(m.HOME, (2208, 1740, 2133, 2177, 1003, 1647))
        for follows in (True, False):
            with self.subTest(follows=follows):
                b = RecordingBus()
                for j in b.joints:
                    j["torque"] = 1
                b.joints[2].update(goal=2179, position=2191)
                write = b.goal
                def goal(axis, value):
                    position = b.joints[axis-1]["position"]
                    write(axis, value)
                    if not follows and axis == 3:
                        b.joints[axis-1]["position"] = position
                b.goal = goal
                with patch.object(p, "lease", return_value=m.lease("left", simulation=b, pick=True)) as locked, \
                     patch.object(p.time, "monotonic", side_effect=itertools.count(0,.008)):
                    if follows:
                        result = p.reset_left_home(Event(), Mock())
                        self.assertIn("2208、1740、2133、2177、1003、1647", result)
                        self.assertTrue(p.pose_check(b, m.HOME, 8))
                    else:
                        with self.assertRaisesRegex(TimeoutError, "ID3.*2191"):
                            p.reset_left_home(Event(), Mock())
                        self.assertEqual(b.joints[2]["goal"], 2191)
                        self.assertNotIn((3, 2133), b.writes)
                locked.assert_called_once_with("left", pick=True)

    def test_home_rechecks_group_sample_before_writing(self):
        bus=RecordingBus()
        for j in bus.joints:
            j["torque"]=1
        positions=itertools.cycle((m.HOME[1]+6,m.HOME[1]+7))
        read=bus.joint
        def flicker(axis):
            if axis==2:
                bus.joints[1]["position"]=next(positions)
            return read(axis)
        bus.joint=flicker
        proposed=list(m.HOME)
        proposed[1]-=10
        with patch.object(p.time,"monotonic",side_effect=itertools.count(0,.02)):
            with self.assertRaisesRegex(TimeoutError,"1.8 秒无有效进展。ID2"):
                p.advance_pose_step(bus,list(m.HOME),proposed,Event(),10,stage="公共复位",targets=m.HOME)
        self.assertEqual(bus.writes,[])

    def test_measured_shoulder_lag_pauses_the_group_until_feedback_recovers(self):
        for stage, next_shoulder in (("公共复位", 1750), ("对齐点位 1 起姿", 1755)):
            with self.subTest(stage=stage):
                bus = RecordingBus()
                for joint in bus.joints:
                    joint["torque"] = 1
                bus.joints[1].update(goal=1752, position=1735)
                expected = [j["goal"] for j in bus.joints]
                proposed = expected.copy()
                proposed[0] -= 2
                proposed[1] = next_shoulder
                targets = m.HOME if stage == "公共复位" else p.recorded_origins()["1"]
                samples = []
                read = bus.joint
                def measured(axis):
                    if axis == 2 and len(samples) < 5:
                        samples.append(list(bus.writes))
                        if len(samples) == 5:
                            bus.joints[1]["position"] = 1749
                    return read(axis)
                bus.joint = measured
                with patch.object(p.time, "monotonic", side_effect=itertools.count(0,.004)):
                    p.advance_pose_step(bus, expected, proposed, Event(), 10, stage=stage, targets=targets,
                                        attempts=p.PoseAttempts(Mock(), stage))
                self.assertEqual(samples, [[]]*5)
                self.assertEqual(expected, proposed)
                self.assertEqual(bus.writes, [(1, proposed[0]), (2, next_shoulder)])

    def test_shoulder_readback_faults_do_not_advance_either_trajectory(self):
        for stage in ("公共复位", "对齐点位 1 起姿"):
            for fault in ({"position": 1731}, {"goal": 1751}, {"torque": 0}, {"mode": 1}):
                with self.subTest(stage=stage, fault=fault):
                    bus = RecordingBus()
                    for joint in bus.joints:
                        joint["torque"] = 1
                    bus.joints[1].update(goal=1752, position=1735)
                    expected = [j["goal"] for j in bus.joints]
                    proposed = expected.copy()
                    proposed[0] -= 2
                    proposed[1] += 2
                    bus.joints[1].update(fault)
                    with self.assertRaises((RuntimeError, ValueError)):
                        p.advance_pose_step(bus, expected, proposed, Event(), p.time.monotonic()+5,
                                            stage=stage, targets=m.HOME, attempts=p.PoseAttempts(Mock(), stage))
                    self.assertFalse(bus.writes)

    def test_alignment_preflight_observes_original_hold_before_claiming_any_axis(self):
        for recovers in (True, False):
            with self.subTest(recovers=recovers):
                bus = RecordingBus()
                for joint in bus.joints:
                    joint["torque"] = 1
                bus.joints[1].update(goal=1752, position=1735)
                touched = set()
                read = bus.joint
                samples = []
                def measured(axis):
                    if axis == 2 and len(samples)<4:
                        samples.append((list(bus.writes), set(touched)))
                        if recovers and len(samples)==4:
                            bus.joints[1]["position"] = 1740
                    return read(axis)
                bus.joint = measured
                with patch.object(p.time, "monotonic", side_effect=itertools.count(0,.01)):
                    if recovers:
                        p.move_pose(bus, m.HOME, Event(), "对齐点位 1 起姿", Mock(), touched)
                        self.assertEqual([j["goal"] for j in bus.joints], list(m.HOME))
                    else:
                        with self.assertRaisesRegex(TimeoutError,"ID2.*1752.*1735"):
                            p.move_pose(bus, m.HOME, Event(), "对齐点位 1 起姿", Mock(), touched)
                        self.assertFalse(touched)
                        self.assertFalse(bus.writes)
                self.assertEqual(samples, [([], set())]*4)

    def test_still_servo_reissues_only_same_goal_once(self):
        bus=RecordingBus()
        for j in bus.joints:
            j["torque"]=1
        bus.joints[3]["position"]+=13
        with patch.object(p.time,"monotonic",side_effect=[0,.1,.8,1.4,2,2.5,3]):
            p.settle_pose(bus,m.HOME,Event(),"复位",Mock())
        self.assertEqual(bus.writes,[(4,m.HOME[3])])

    def test_persistent_static_error_reports_pre_cleanup_evidence(self):
        bus=RecordingBus()
        for j in bus.joints:
            j["torque"]=1
        bus.joints[3]["position"]+=13
        bus.goal=Mock()  # readback stays at the correct command but no motion
        with patch.object(p.time,"monotonic",side_effect=[0,.1,1.4,3,4.8,5.2]):
            with self.assertRaisesRegex(TimeoutError,"ID4.*实测 2190.*2177.*差 \\+13 格.*力矩 1.*运动 0"):
                p.settle_pose(bus,m.HOME,Event(),"复位",Mock())
        bus.goal.assert_called_once_with(4,m.HOME[3])

    def test_moving_servo_never_qualifies_or_reissues(self):
        bus=RecordingBus()
        for j in bus.joints:
            j["torque"]=1
        bus.joints[3]["moving"]=1
        with patch.object(p.time,"monotonic",side_effect=[0,.1,1.4,5.1]):
            with self.assertRaisesRegex(TimeoutError,"ID4.*运动 1"):
                p.settle_pose(bus,m.HOME,Event(),"复位",Mock())
        self.assertEqual(bus.writes,[])

    def test_converging_servo_can_finish_after_five_seconds(self):
        bus=RecordingBus()
        for j in bus.joints:
            j["torque"]=1
        positions=iter([18,16,14,10,8,7,7])
        def sample(_bus):
            bus.joints[3]["position"]=m.HOME[3]+next(positions)
            return [j.copy() for j in bus.joints]
        with patch.object(p,"snapshot",side_effect=sample), patch.object(p.time,"monotonic",
                side_effect=[0,1,2,4.1,5.3,6.0,6.2,6.4]):
            p.settle_pose(bus,m.HOME,Event(),"复位",Mock())
        self.assertEqual(bus.writes,[])

    def recorded_start(self, slot):
        bus = RecordingBus()
        first, _, _ = p.plan_for(slot)
        for joint, recorded in zip(bus.joints, first):
            joint.update(torque=1, position=recorded["position"], goal=recorded["goal"])
        return bus

    def test_package_forward_wait_caps_operator_pauses(self):
        self.assertEqual([p.recorded_pause(d) for d in (0., .02, .15, .30, 39.)],
                         [.06, .06, .15, .30, .30])

    def test_lagging_axis_can_catch_up_without_goal_retries(self):
        bus = self.recorded_start(2)
        joint = bus.joint(3)
        goal = joint["goal"]
        joint["position"] = goal + 45
        p.wait_for_recorded_step(bus, joint, goal, goal-8, Event())
        self.assertEqual(bus.writes, [])

    def test_stall_blocks_next_goal(self):
        bus = self.recorded_start(2)
        joint = bus.joints[2]
        goal = joint["goal"]
        joint["position"] = goal + 45
        with patch.object(p.time, "monotonic", side_effect=[0., 2.]):
            with self.assertRaisesRegex(TimeoutError, "ID3"):
                p.wait_for_recorded_step(bus, bus.joint(3), goal, goal-8, Event())
        self.assertEqual(bus.writes, [])

    def test_all_four_original_sequences(self):
        for slot, count in enumerate((304, 386, 346, 478), 1):
            with self.subTest(slot=slot):
                plan = p.plan_for(slot)
                bus = self.recorded_start(slot)
                check = Mock(side_effect=lambda: self.assertEqual(bus.writes, []))
                result = p.execute(bus, slot, plan, Event(), Mock(), check)
                check.assert_called_once()
                self.assertIn(str(count), result)
                self.assertEqual(bus.writes, [(axis, target) for _, axis, target, _ in plan[1]])
                self.assertEqual(len(bus.writes), count)

    def test_separate_origin_then_verified_pick_never_realigns(self):
        for slot in (1, 2, 3, 4):
            bus = RecordingBus()
            plan = p.plan_for(slot)
            with patch.object(p, "lease", return_value=m.lease("left", simulation=bus, pick=True)):
                p.prepare_slot_origin(slot, Event(), Mock())
            writes = len(bus.writes)
            p.execute(bus, slot, plan, Event(), Mock(), Mock())
            self.assertEqual(bus.writes[writes:], [(axis, target) for _, axis, target, _ in plan[1]])

    def test_wrong_origin_or_disabled_torque_sends_no_servo_writes(self):
        for change in ({"position": 2152, "goal": 2120}, {"torque": 0}, {"goal": 2000}):
            bus = self.recorded_start(1)
            bus.joints[2].update(change)
            verify = Mock()
            with self.assertRaisesRegex((ValueError, RuntimeError), "ID3"):
                p.execute(bus, 1, p.plan_for(1), Event(), Mock(), verify)
            self.assertEqual(bus.writes, [])
            verify.assert_not_called()

    def test_long_legal_start_and_finish_use_proportional_linear_steps(self):
        start = [1810, 2840, 1070, 2189, 2603, 2807]
        for stage in ("公共复位", "结束复位"):
            with self.subTest(stage=stage):
                bus = RecordingBus()
                for joint, value in zip(bus.joints, start):
                    joint.update(torque=1, goal=value, position=value)
                frames = []
                def step(*args, **kwargs):
                    frames.append(list(args[2]))
                    return original_step(*args, **kwargs)
                original_step = p.advance_pose_step
                with patch.object(p, "advance_pose_step", side_effect=step):
                    measured = p.move_pose(bus,m.HOME,Event(),stage,Mock(),set())
                self.assertEqual([j["position"] for j in measured], list(m.HOME))
                self.assertGreater(len(frames), 100)
                longest = max(range(6), key=lambda i:abs(m.HOME[i]-start[i]))
                distance = m.HOME[longest]-start[longest]
                for i, frame in enumerate(frames,1):
                    progress = (frame[longest]-start[longest])/distance
                    self.assertAlmostEqual(progress, i/len(frames), delta=.006)
                    for value, initial, goal in zip(frame,start,m.HOME):
                        self.assertAlmostEqual(value, initial+(goal-initial)*progress, delta=1.1)

    def test_stop_or_servo_fault_during_playback_never_starts_final_return(self):
        slot=2
        plan=p.plan_for(slot)
        for cause in ("stop","servo"):
            with self.subTest(cause=cause):
                bus=self.recorded_start(slot)
                stop=Event()
                stages=[]
                approved=[]
                def vision():
                    approved.append(len(bus.writes))
                write=bus.goal
                def interrupt(axis,target):
                    write(axis,target)
                    if approved and len(bus.writes)==approved[0]+3:
                        if cause=="stop":
                            stop.set()
                        else:
                            bus.joints[axis-1]["torque"]=0
                bus.goal=interrupt
                with self.assertRaises(p.ReplayStopped if cause=="stop" else RuntimeError):
                    p.execute(bus,slot,plan,stop,lambda s,_:stages.append(s),vision)
                self.assertEqual(len(approved),1)
                self.assertFalse(any("结束复位" in s for s in stages))

    def test_start_limit_zero_writes(self):
        for axis,pos in ((3,3057),(3,3053),(2,1010)):
            bus=RecordingBus()
            bus.joints[axis-1]["position"]=pos
            with self.assertRaises(ValueError):
                p.execute(bus,1,p.plan_for(1),Event(),Mock(),Mock())
            self.assertFalse(bus.writes)

    def test_vision_veto_and_late_confirmation(self):
        for fail in (True,False):
            stop,gate=Event(),Event()
            events=[]
            bus=self.recorded_start(1)
            def verify():
                if fail:
                    raise ValueError("wrong target")
                stop.set()
                gate.wait(2)
            with self.assertRaises((p.ReplayStopped,ValueError)):
                p.execute(bus,1,p.plan_for(1),stop,lambda m,n:events.append(n),verify)
            gate.set()
            self.assertEqual(bus.writes,[])
            self.assertFalse(any(events))

    def test_manual_recording_and_hold(self):
        from arm_episodes import ArmEpisodes
        with TemporaryDirectory() as temp, patch.dict(os.environ,O6_WORKSTATION_RUNTIME=temp):
            arms=m.ArmSessions()
            mode={"mode":"simulation"}
            for side in ("left","right"):
                arms.open(side,"simulation")
                with self.assertRaises(ValueError):
                    arms.action(side,"hold",**mode)
                arms.action(side,"hold",supported=True,**mode)
                arms.action(side,"record_start",**mode)
                with self.assertRaises(RuntimeError), m.lease(side,simulation=m.SimBus(),pick=True):
                    self.fail("recording lease bypassed")
                arms.action(side,"jog",joint=6,delta=8,**mode)
                arms.action(side,"record_stop",**mode)
                info=ArmEpisodes(Path(temp)).list(side)[0]
                self.assertEqual(info["action_count"],1)
                self.assertTrue(info["complete"])
                arms.action(side,"release",supported=True,**mode)
                self.assertEqual(arms.action(side,"close",**mode)["session"],"idle")

    def test_packet_echo_and_old_ack(self):
        bus=m.ServoBus.__new__(m.ServoBus)
        class Wire:
            def __init__(self):
                self.buffer=bytearray()
            def reset_input_buffer(self):
                self.buffer.clear()
            def write(self,packet):
                def reply(payload):
                    body=bytes((2,len(payload)+2,0))+payload
                    return b"\xff\xff"+body+bytes((~sum(body)&255,))
                self.buffer.extend(packet+reply(b"")+reply(bytes((53,7))))
                return len(packet)
            def read(self,n):
                out=self.buffer[:n]
                del self.buffer[:n]
                return bytes(out)
        bus.port=Wire()
        self.assertEqual(bus.number(2,56,2),7*256+53)
        with self.assertRaises(ValueError):
            bus.number(7,56,2)


if __name__=="__main__":
    unittest.main()

