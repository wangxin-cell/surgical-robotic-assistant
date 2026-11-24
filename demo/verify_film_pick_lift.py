"""A passive elbow may be out of travel only while mechanically fixed."""
import threading
import time
import unittest
from unittest.mock import patch

from film_pick_lift import ACTIVE, SupportedWristBus, read_pose, run_arc, teach_hover, validate_pair
from smooth_bus import PROFILE
from smooth_tools import TracedSmoothBus
from verify_smooth_bus import RecordingLink


class TestSupportedLift(unittest.TestCase):
    def setUp(self):
        self.link = RecordingLink()
        for i, p in {1: 987, 3: 851, 4: 1469, 6: 832}.items():
            self.link.put(i, 56, 2, p)
        hook = patch("jog_bus._SimulatedLink", return_value=self.link)
        hook.start()
        self.addCleanup(hook.stop)
        self.bus = TracedSmoothBus(simulate=True, bus_class=SupportedWristBus)
        self.bus.connect()
        self.stop = threading.Event()

    def assert_only_wrist_and_jaw_written(self):
        self.assertTrue(all(i in ACTIVE for i, _, _ in self.link.writes))
        self.assertTrue(all(set(frame).issubset(ACTIVE) for frame in self.link.frames))

    def test_full_arc_while_passives_are_outside_limits(self):
        high = read_pose(self.bus)
        low = {**high, 4: high[4] - 48, 6: high[6] - 30}
        report = {"encoder_stages": []}
        run_arc(self.bus, low, high, self.stop, report)
        self.assertTrue(report["encoder_cycle_completed"])
        self.assertEqual([s["id"] for s in report["encoder_stages"]], [4, 6, 4, 4, 6, 4])
        self.assertFalse(any(self.bus.status()["fixture_torques"].values()))
        with self.assertRaises(ValueError):
            self.bus.move_to({1: 1300}, 8, self.stop)
        self.bus.release()
        self.assert_only_wrist_and_jaw_written()
        self.bus.close()

    def test_fixture_slip_fault_and_release_preserve_scope(self):
        high = read_pose(self.bus)
        self.bus.bus.bind_fixture(high)
        self.bus.enable_hold()
        self.link.put(8, 56, 2, high[8] + 7)
        with self.assertRaisesRegex(RuntimeError, r"ID8 底盘驱动轮 \+7 格"):
            self.bus.status()
        self.assertEqual(self.bus.state, "fault")
        self.bus.release()
        self.assert_only_wrist_and_jaw_written()
        self.bus.close()

    def test_invalid_active_limit_torque_and_teaching_are_rejected(self):
        high = read_pose(self.bus)
        low = {**high, 4: high[4]-48, 6: high[6]-30}
        with self.assertRaises(ValueError):
            validate_pair({**low, 3: low[3]+7}, high)
        with self.assertRaises(ValueError):
            self.bus.enable_hold()  # No physical fixture pose has been taught.
        self.bus.bus.bind_fixture(high)
        self.link.put(7, 40, 1, 1)
        with self.assertRaises(ValueError):
            self.bus.enable_hold()
        self.link.put(7, 40, 1, 0)
        self.link.put(4, 56, 2, PROFILE[4][0]-1)
        with self.assertRaises(ValueError):
            self.bus.enable_hold()
        self.assertEqual(self.link.writes, [])
        self.bus.close()

    def test_retry_high_pose_retains_low_point_without_motor_writes(self):
        low = read_pose(self.bus)

        def keys():
            yield "capture"  # Both encoders still at the low point.
            self.link.put(4, 56, 2, low[4] - 72)
            self.link.put(6, 56, 2, low[6] + 30)
            yield None
            yield "capture"

        report = {"contact": dict(low)}
        high = teach_hover(self.bus, low, time.monotonic(), report, keys())
        self.assertEqual((high[4], high[6]), (low[4] - 72, low[6] + 30))
        self.assertEqual(report["contact"], low)
        self.assertEqual(report["hover_attempt"], high)
        self.assertFalse(self.link.writes or self.link.frames)
        self.bus.close()

    def test_high_pose_monitor_stops_on_fixture_slip(self):
        low = read_pose(self.bus)
        low[2], low[3] = 2415, 851
        self.link.put(2, 56, 2, 2407)
        self.link.put(3, 56, 2, 840)
        with self.assertRaises(ValueError) as failure:
            teach_hover(self.bus, low, time.monotonic(), {}, iter(["capture"]))
        details = str(failure.exception)
        self.assertIn("ID2 肩部俯仰 -8 格", details)
        self.assertIn("ID3 肘部屈伸 -11 格", details)
        self.assertIn("±6 格", details)
        self.assertNotIn("底盘驱动轮", details)
        self.assertFalse(self.link.writes or self.link.frames)
        self.bus.close()


if __name__ == "__main__":
    unittest.main()
