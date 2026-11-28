import threading
import unittest
from unittest.mock import Mock

from native_motor import SimBus
from native_pick import plan_for
from package_replay import Interrupted, play


class RegisterSim(SimBus):
    def __init__(self, slot):
        super().__init__()
        self.sent = []
        for joint, start in zip(self.joints,plan_for(slot)[0]):
            joint.update(start)

    def read(self, axis, address, count):
        j = self.joints[axis-1]
        if address == 9:
            return j["min"].to_bytes(2,"little") + j["max"].to_bytes(2,"little")
        if address == 60:
            return bytes(count)
        key = {33:"mode", 40:"torque", 42:"goal", 56:"position"}[address]
        return j[key].to_bytes(count,"little")

    def number(self,axis,address,count):
        return int.from_bytes(self.read(axis,address,count),"little")

    def send(self, axis, opcode, address, packet):
        target = int.from_bytes(packet[1:3],"little")
        self.sent.append((axis,target))
        self.goal(axis,target)


class PackageReplayTests(unittest.TestCase):
    def test_original_script_and_mid_path_resume_all_four(self):
        for slot in range(1,5):
            with self.subTest(slot=slot):
                expected = [(a,t) for _,a,t,_ in plan_for(slot)[1]]
                bus, stop = RegisterSim(slot), threading.Event()
                # Split just after a gripper command where the recording has one.
                cut = next((i for i,(a,_) in enumerate(expected,1) if a == 6 and i<len(expected)), 3)
                def pause(msg,fraction):
                    if len(bus.sent)==cut:
                        stop.set()
                with self.assertRaises(Interrupted) as failure:
                    play(bus,slot,stop,pause,Mock())
                cursor = failure.exception.cursor
                self.assertEqual((cursor.done,bus.sent), (cut,expected[:cut]))
                stop.clear()
                play(bus,slot,stop,Mock(),Mock(),cursor)
                self.assertEqual(bus.sent,expected)

    def test_stopped_after_final_command_only_verifies(self):
        slot=3
        bus, stop=RegisterSim(slot),threading.Event()
        expected=len(plan_for(slot)[1])
        with self.assertRaises(Interrupted) as failure:
            play(bus,slot,stop,lambda *args: stop.set() if len(bus.sent)==expected else None,Mock())
        stop.clear()
        play(bus,slot,stop,Mock(),Mock(),failure.exception.cursor)
        self.assertEqual(len(bus.sent),expected)

    def test_lost_ack_does_not_duplicate_applied_write(self):
        bus, stop = RegisterSim(1), threading.Event()
        original = bus.read
        dropped = False
        def flaky(axis, address, count):
            nonlocal dropped
            if bus.sent and address == 42 and not dropped:
                dropped = True
                raise TimeoutError("lost register reply")
            return original(axis,address,count)
        bus.read = flaky
        with self.assertRaises(Interrupted) as failure:
            play(bus,1,stop,Mock(),Mock())
        self.assertEqual(failure.exception.cursor.done,1)
        play(bus,1,stop,Mock(),Mock(),failure.exception.cursor)
        self.assertEqual(bus.sent,[(a,t) for _,a,t,_ in plan_for(1)[1]])

    def test_unknown_write_disables_resume(self):
        bus = RegisterSim(1)
        original = bus.read
        def flaky(axis,address,count):
            if bus.sent and address == 42:
                raise TimeoutError("register unavailable")
            return original(axis,address,count)
        bus.read = flaky
        with self.assertRaisesRegex(RuntimeError, "无法确定断点"):
            play(bus,1,threading.Event(),Mock(),Mock())
        self.assertEqual(len(bus.sent),1)

    def test_pose_change_blocks_resume_before_write(self):
        bus, stop=RegisterSim(1),threading.Event()
        with self.assertRaises(Interrupted) as failure:
            play(bus,1,stop,lambda *args:stop.set(),Mock())
        stop.clear()
        bus.joints[0]["position"] += 80
        with self.assertRaises(Interrupted):
            play(bus,1,stop,Mock(),Mock(),failure.exception.cursor)
        self.assertEqual(len(bus.sent),1)

