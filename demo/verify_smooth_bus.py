"""Offline regressions for sustained torque and synchronized motion. No serial access."""
import threading
import unittest
from unittest.mock import patch

from jog_bus import _SimulatedLink, _SerialLink
from smooth_bus import PROFILE, SmoothBus
from smooth_tools import HoldCheck


class RecordingLink(_SimulatedLink):
    def __init__(self):
        super().__init__()
        self.writes = []
        self.frames = []

    def write(self, i, address, value):
        self.writes.append((i, address, value))
        super().write(i, address, value)

    def write_goals(self, goals):
        self.frames.append(dict(goals))
        super().write_goals(goals)


class TestSmoothBus(unittest.TestCase):
    def setUp(self):
        self.link = RecordingLink()
        self.patch = patch("jog_bus._SimulatedLink", return_value=self.link)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.bus = SmoothBus(simulate=True)
        self.bus.connect()
        self.stop = threading.Event()
        self.assertEqual(self.link.writes, [])

    def test_six_axis_large_motion_holds_and_release_restores(self):
        self.bus.enable_hold()
        starts = self.bus._goals.copy()
        offsets = [180, -145, 100, 65, -110, 32]
        targets = {i:v+offsets[i-1] for i,v in starts.items()}
        moved = self.bus.move_to(targets,30,self.stop)
        self.assertEqual(moved["state"], "holding")
        self.assertEqual(len(moved["rows"]), 6)
        for i,s in moved["rows"].items():
            self.assertTrue(s["held"])
            self.assertEqual(s["position"], targets[i])
        self.assertGreater(len(self.link.frames), 10)
        self.assertTrue(any(len(frame) > 1 for frame in self.link.frames))
        prev = starts.copy()
        for frame in self.link.frames:
            for i, v in frame.items():
                advance = v-prev[i]
                self.assertGreater(advance*offsets[i-1],0)
                self.assertLessEqual(abs(advance),17)
                prev[i] = v
        self.assertTrue(all(i <= 6 for i,_,_ in self.link.writes))
        self.assertEqual(self.bus.status()["rows"][5]["torque"],1)
        with self.assertRaises(RuntimeError):
            self.bus.close()
        original=self.bus._original.copy()
        self.bus.release()
        for i,saved in original.items():
            self.assertEqual(self.bus._number(i,40,1),0)
            for addr, v in saved.items():
                self.assertEqual(self.bus._number(i,addr,1 if addr == 41 else 2),v)

    def test_optional_base_outside_hard_limits_is_not_energized(self):
        self.link.put(1,56,2,1055)
        held=self.bus.enable_hold()
        self.assertFalse(held["rows"][1]["held"])
        self.assertFalse(any(i==1 for i,_,_ in self.link.writes))
        with self.assertRaises(ValueError):
            self.bus.move_to({1:1300},10,self.stop)
        self.assertTrue(held["rows"][4]["ready"])

    def test_shoulder_outside_limit_prevents_all_writes(self):
        self.link.put(2,56,2,PROFILE[2][0]-1)
        with self.assertRaises(ValueError):
            self.bus.enable_hold()
        self.assertEqual(self.link.writes,[])

    def test_hold_at_margin_only_moves_inward(self):
        low=PROFILE[3][0]
        self.link.put(3,56,2,low+4)
        s=self.bus.enable_hold()
        self.assertEqual(s["rows"][3]["minimum"],low+4)
        with self.assertRaises(ValueError):
            self.bus.move_to({3:low+1},15,self.stop)
        self.assertEqual(self.bus.move_to({3:low+35},30,self.stop)["rows"][3]["position"],low+35)

    def test_preload_checked_at_zero_output(self):
        original = self.link.write
        def reset_settings(i,a,v):
            original(i,a,v)
            if i==2 and a==40 and v==1:
                self.link.put(i,42,2,2500)
        self.link.write = reset_settings
        with self.assertRaises(RuntimeError):
            self.bus.enable_hold()
        self.assertEqual(self.bus.state,"fault")
        self.assertFalse(any(i==2 and a==48 and v>0 for i,a,v in self.link.writes))
        self.assertEqual(self.bus.release()["state"],"passive")

    def test_stop_during_large_trajectory_holds_and_does_not_resume(self):
        start=self.bus.enable_hold()["rows"][4]["position"]
        def progress(s):
            if s["rows"][4]["position"] >= start+12:
                self.stop.set()
        s=self.bus.move_to({4:start+200},30,self.stop,progress)
        row=s["rows"][4]
        self.assertTrue(row["held"])
        self.assertLess(row["target"],start+200)
        self.assertEqual(row["position"],row["target"])
        count=len(self.link.frames)
        self.bus.status()
        self.assertEqual(len(self.link.frames),count)

    def test_unmoved_small_request_does_not_claim_motion(self):
        start=self.bus.enable_hold()["rows"][6]["position"]
        original=self.link.write_goals
        def static_position(frame):
            original(frame)
            self.link.put(6,56,2,start)
        self.link.write_goals=static_position
        s=self.bus.move_to({6:start+4},20,self.stop)
        self.assertIn("未测得",s["reason"])
        self.assertEqual(s["rows"][6]["target"],start)

    def test_tripped_servo_is_never_rearmed_by_stop(self):
        start=self.bus.enable_hold()["rows"][4]["position"]
        after=[]
        def trip(s):
            if not after:
                self.link.put(4,40,1,0)
                after.append(len(self.link.writes))
        with self.assertRaises(RuntimeError):
            self.bus.move_to({4:start+180},30,self.stop,trip)
        self.assertEqual(self.bus.state,"fault")
        self.assertEqual(self.link.writes[after[0]:],[])

    def test_idle_stop_latches_measured_position(self):
        before=self.bus.enable_hold()["rows"][2]["target"]
        self.link.put(2,56,2,before-8)
        row=self.bus.stop_hold()["rows"][2]
        self.assertEqual((row["position"],row["target"],row["torque"]), (before-8,before-8,1))

    def test_excessive_sag_locks_without_continuing_previous_target(self):
        before=self.bus.enable_hold()["rows"][2]["target"]
        self.link.put(2,56,2,before-50)
        with self.assertRaises(RuntimeError):
            self.bus.status()
        self.assertEqual(self.bus.state,"fault")
        self.assertEqual(self.bus._number(2,42,2),before-50)
        count=len(self.link.frames)
        self.assertFalse(self.bus.status()["rows"][2]["ready"])
        self.assertEqual(len(self.link.frames),count)

    def test_unhealthy_temperature_and_read_failure_do_not_send_new_targets(self):
        self.bus.enable_hold()
        self.link.put(4,63,1,65)
        with self.assertRaises(RuntimeError):
            self.bus.status()
        self.assertEqual(self.link.frames,[])
        self.link.put(4,63,1,34)
        self.bus.release()
        self.bus.enable_hold()
        with patch.object(self.link,"read",side_effect=OSError("unplug")):
            with self.assertRaises(RuntimeError):
                self.bus.status()
            self.assertEqual(self.bus.state,"fault")
            with self.assertRaises(RuntimeError):
                self.bus.close()
        self.assertEqual(self.link.frames,[])

    def test_sync_write_has_known_encoding_and_rejects_wheel(self):
        class Port:
            data=b""
            def write(self, data):
                self.data=data
                return len(data)
        serial=_SerialLink.__new__(_SerialLink)
        serial.port=Port()
        serial.write_goals({1:1500,6:780})
        pkt=serial.port.data
        self.assertEqual(pkt[:13],bytes([255,255,254,10,131,42,2,1,220,5,6,12,3]))
        self.assertEqual(sum(pkt[2:]) % 256,255)
        with self.assertRaises(ValueError):
            serial.write_goals({7:1000})
        self.assertEqual(pkt,serial.port.data)


class TestHoldCheck(unittest.TestCase):
    def setUp(self):
        self.rows = {i: dict(torque=1, held=True, ready=True, position=1500, target=1500)
                     for i in range(1, 7)}
        self.snapshot = dict(state="holding", rows=self.rows)
        self.check = HoldCheck()

    def test_five_seconds_of_distinct_feedback_and_drift(self):
        for second in range(5):
            self.assertFalse(self.check.add(self.snapshot, now=second))
        self.assertFalse(self.check.result()["passed"])
        self.rows[3]["position"] -= 9
        self.assertTrue(self.check.add(self.snapshot, now=5))
        self.assertFalse(self.check.result()["passed"])
        self.assertEqual(self.check.result()["axes"][3]["span"], 9)

    def test_lost_samples_or_unowned_elbow_never_certify_hold(self):
        self.check.add(self.snapshot, now=0)
        with self.assertRaises(ValueError):
            self.check.add(self.snapshot, now=3)
        self.rows[3]["held"] = False
        with self.assertRaises(ValueError):
            HoldCheck().add(self.snapshot, now=0)


if __name__ == "__main__":
    unittest.main()
