"""Offline register-level checks for a supported pickup of the blunt practice prop."""
import unittest
from unittest.mock import patch

import film_pick_lift as lift
from jog_bus import BUS_IDS
from smooth_tools import TracedSmoothBus
from verify_keyboard_lift import SimulationStop
from verify_smooth_bus import RecordingLink


class TestDirectPick(unittest.TestCase):
    def setUp(self):
        self.link = RecordingLink()
        # Actual 22:21 contact pose, with passive base and elbow outside their motion profiles.
        for axis, position in enumerate((1023, 2690, 626, 1394, 3185, 793, 3183, 3516, 1003), 1):
            self.link.put(axis, 56, 2, position)
        self.clock = SimulationStop()
        for hook in (patch("jog_bus._SimulatedLink", return_value=self.link),
                     patch("time.monotonic", side_effect=lambda: self.clock.seconds),
                     patch("time.sleep", side_effect=self.clock.sleep)):
            hook.start()
            self.addCleanup(hook.stop)
        self.bus = TracedSmoothBus(simulate=True, bus_class=lift.DirectPickBus)
        self.bus.connect()
        self.low = lift.read_pose(self.bus)
        self.report = {"actions": [], "encoder_stages": [], "encoder_cycle_completed": False,
                       "physical_outcome": "unverified"}
        self.frames = []
        self.jaw_goals = []
        send = self.link.write_goals

        def record(goals):
            send(goals)
            # Capture both positions after every command, including one-axis frames.
            self.frames.append((self.position(4), self.position(6)))
            self.jaw_goals.append(int.from_bytes(self.link.read(6, 42, 2), "little"))

        self.link.write_goals = record

    def position(self, axis):
        return int.from_bytes(self.link.read(axis, 56, 2), "little")

    def cycle(self, ask):
        return lift.direct_pick(self.bus, self.low, self.clock, self.report, ask)

    def tearDown(self):
        try:
            if self.bus.state != "passive":
                self.bus.release()
            for axis in BUS_IDS:
                self.assertEqual(self.link.read(axis, 40, 1), b"\0")
            for axis, _, _ in self.link.writes:
                self.assertIn(axis, (4, 6))
            for frame in self.link.frames:
                self.assertLessEqual(set(frame), {4, 6})
        finally:
            self.bus.close()

    def monitor_wheel(self, alter):
        """Modify ID9's full feedback words on the simulated read path only."""
        read = self.link.read
        calls = []

        def sample(axis, address, size):
            raw = read(axis, address, size)
            if (axis, address, size) != (9, 40, 27):
                return raw
            packet = bytearray(raw)
            calls.append(self.clock.seconds)
            alter(len(calls), packet)
            return bytes(packet)

        self.link.read = sample
        return calls

    def test_one_passive_wheel_moving_sample_settles_before_reporting_pose(self):
        reads = self.monitor_wheel(lambda count, packet: packet.__setitem__(26, count == 1))
        before = self.clock.seconds
        pose = lift.direct_read_pose(self.bus, self.low, timeout=3.0)
        self.assertEqual(pose, self.low)
        self.assertGreaterEqual(len(reads), 5)
        self.assertGreater(self.clock.seconds, before)
        self.assertEqual(self.link.writes, [])

    def test_persistent_passive_wheel_motion_expires_read_only(self):
        self.monitor_wheel(lambda _, packet: packet.__setitem__(26, 1))
        before = self.clock.seconds
        with self.assertRaisesRegex((TimeoutError, ValueError, RuntimeError), "ID9"):
            lift.direct_read_pose(self.bus, self.low, timeout=1.3)
        self.assertGreaterEqual(self.clock.seconds - before, 1.2)
        self.assertEqual(self.link.writes, [])

    def test_wheel_drift_during_settle_is_not_a_new_reference(self):
        def drift(count, packet):
            packet[26] = count == 1
            if count >= 2:
                packet[16:18] = (self.low[9] + 7).to_bytes(2, "little")

        self.monitor_wheel(drift)
        with self.assertRaisesRegex((ValueError, RuntimeError), "ID9"):
            lift.direct_read_pose(self.bus, self.low, timeout=2.0)
        self.assertEqual(self.link.writes, [])

    def test_unexpected_wheel_enable_during_settle_is_a_hard_failure(self):
        def enable_once(count, packet):
            packet[26] = count == 1
            packet[0] = count == 2

        reads = self.monitor_wheel(enable_once)
        with self.assertRaisesRegex((ValueError, RuntimeError), "ID9"):
            lift.direct_read_pose(self.bus, self.low, timeout=2.0)
        self.assertEqual(len(reads), 2)
        self.assertEqual(self.link.writes, [])

    def test_first_enable_gate_handles_one_transient_before_any_write(self):
        enable = self.bus.bus.enable_hold
        gate_reads = []

        def gated(*args, **kwargs):
            def first_moving(count, packet):
                if count <= 2:
                    self.assertFalse(self.link.writes)
                if count == 1:
                    packet[26] = 1
            gate_reads.append(self.monitor_wheel(first_moving))
            return enable(*args, **kwargs)

        keys = iter(["+", "U", "R"])
        with patch.object(self.bus.bus, "enable_hold", side_effect=gated) as attempts:
            self.cycle(lambda _p, _keys: next(keys))
        attempts.assert_called_once()
        self.assertGreaterEqual(len(gate_reads[0]), 5)
        self.assertTrue(self.report["released_at_support"])

    def test_one_io_error_after_enabling_is_not_retried(self):
        read = self.link.read
        faults = []

        def fail_once(axis, address, size):
            has_opened = any(self.low[6] + 4 < jaw for _, jaw in self.frames)
            if (axis, address, size) == (9, 40, 27) and has_opened and not faults:
                faults.append(True)
                raise OSError("lost powered feedback")
            return read(axis, address, size)

        with patch.object(self.link, "read", side_effect=fail_once), \
             patch.object(self.bus.bus, "enable_hold", wraps=self.bus.bus.enable_hold) as attempts:
            with self.assertRaisesRegex((RuntimeError, OSError, ValueError), "lost powered feedback"):
                self.cycle(lambda *_: self.fail("Reached direction prompt after a powered fault"))
        attempts.assert_called_once()
        self.assertEqual(faults, [True])
        self.assertFalse(self.report.get("released_at_support"))
        self.assertTrue(all(jaw >= self.low[6] for _, jaw in self.frames))

    def test_grip_stays_seated_through_visible_wrist_travel_and_return(self):
        confirmations = []

        def confirm(_prompt, allowed):
            key = ("+", "U", "R")[len(confirmations)]
            self.assertIn(key, allowed)
            confirmations.append((key, len(self.frames)))
            return key

        self.cycle(confirm)
        self.assertEqual([key for key, _ in confirmations], ["+", "U", "R"])
        _, (_, sampled), (_, at_support) = confirmations
        # The empty direction probe precedes any tightening. The subsequently
        # tightened jaws accompany the wrist above and back down to the taught low.
        loaded = self.frames[sampled:at_support]
        firm = min(jaw for _, jaw in loaded)
        self.assertLess(firm, self.low[6])
        first_seated = next(i for i, (_, jaw) in enumerate(loaded) if jaw == firm)
        seated = loaded[first_seated:]
        self.assertGreaterEqual(max(wrist for wrist, _ in seated) - self.low[4], 35)
        self.assertLessEqual(abs(seated[-1][0] - self.low[4]), 4)
        self.assertTrue(all(jaw == firm for _, jaw in seated))
        self.assertGreater(max(jaw for _, jaw in self.frames[at_support:]), self.low[6])
        self.assertTrue(self.report["released_at_support"])
        self.assertEqual(self.report["physical_outcome"], "unverified")

    def test_cancelling_support_confirmation_keeps_jaw_closed(self):
        asks = []

        def cancel(_prompt, allowed):
            if not asks:
                self.assertIn("+", allowed)
                asks.append("+")
                return "+"
            if asks == ["+"]:
                self.assertIn("U", allowed)
                asks.append("U")
                return "U"
            self.assertIn("R", allowed)
            asks.append("R")
            raise InterruptedError("model support not confirmed")

        with self.assertRaises(InterruptedError):
            self.cycle(cancel)
        self.assertEqual(asks, ["+", "U", "R"])
        self.assertLessEqual(abs(self.position(4) - self.low[4]), 4)
        self.assertLess(self.position(6), self.low[6])
        self.assertFalse(self.report.get("released_at_support", False))
        self.assertEqual(self.report["physical_outcome"], "unverified")

    def simulate_wrist_return_lag(self, *, follow_rising_immediately=False):
        write = self.link.write
        previous_wrist_goal = self.low[4]

        def return_lag(axis, address, value):
            nonlocal previous_wrist_goal
            previous = self.position(4)
            write(axis, address, value)
            if axis == 4 and address == 42:
                if value < previous_wrist_goal:
                    self.link.put(4, 56, 2, min(previous, value + 9))
                elif value > previous_wrist_goal and not follow_rising_immediately:
                    # Static friction holds the shaft until the rising target catches up.
                    self.link.put(4, 56, 2, max(previous, value))
                previous_wrist_goal = value

        self.link.write = return_lag

    def test_return_can_settle_nine_ticks_above_low_with_supported_release(self):
        self.simulate_wrist_return_lag()
        keys = iter(["+", "U", "R"])

        def confirm(_prompt, allowed):
            key = next(keys)
            self.assertIn(key, allowed)
            if key in ("U", "R"):
                self.assertEqual(self.position(4), self.low[4] + 9)
            return key

        self.cycle(confirm)
        self.assertTrue(self.report["released_at_support"])
        self.assertEqual(self.report["physical_outcome"], "unverified")

    def test_actual_reverse_motion_after_lag_stops_without_release(self):
        # At 1403 the low target is 1394. If the first rising target of 1397
        # actually moves the loaded wrist down six ticks, abort the lift.
        self.simulate_wrist_return_lag(follow_rising_immediately=True)
        keys = iter(["+", "U"])
        with self.assertRaisesRegex(RuntimeError, "反馈移出本次短行程"):
            self.cycle(lambda _p, _allowed: next(keys))
        self.assertFalse(self.report.get("released_at_support"))
        self.assertEqual(self.position(6), self.low[6] - 4)
        self.assertEqual(self.report["physical_outcome"], "unverified")

    def test_stiff_handle_blocks_extra_travel_but_preload_survives_lift_and_return(self):
        reference = self.low[6]
        rigid_stop = reference + 2
        preload_goal = reference - 4
        keys = iter(["+", "U", "R"])
        object_in_jaws = False
        returned = []
        original_write = self.link.write

        def handle_blocks_jaw(axis, address, value):
            original_write(axis, address, value)
            if axis == 6 and address == 42 and object_in_jaws and value < rigid_stop:
                self.link.put(6, 56, 2, rigid_stop)

        self.link.write = handle_blocks_jaw

        def confirm(_prompt, allowed):
            nonlocal object_in_jaws
            key = next(keys)
            self.assertIn(key, allowed)
            if key == "+":
                object_in_jaws = True
            if key == "U":
                self.assertEqual(self.position(6), rigid_stop)
                self.assertEqual(self.jaw_goals[-1], reference)
            if key == "R":
                returned.append(len(self.frames))
                self.assertLessEqual(abs(self.position(4) - self.low[4]), 4)
                self.assertEqual(self.position(6), rigid_stop)
                self.assertEqual(self.jaw_goals[-1], preload_goal)
            return key

        self.cycle(confirm)
        self.assertEqual(len(returned), 1)
        seated = self.jaw_goals.index(preload_goal)
        travel = self.frames[seated:returned[0]]
        self.assertTrue(all(goal == preload_goal for goal in self.jaw_goals[seated:returned[0]]))
        self.assertTrue(all(actual == rigid_stop for _, actual in travel))
        self.assertGreaterEqual(max(wrist for wrist, _ in travel) - self.low[4], 35)
        self.assertTrue(self.report["released_at_support"])
        self.assertEqual(self.report["physical_outcome"], "unverified")

    def test_recorded_seven_tick_passive_base_shift_stops_before_pick(self):
        send = self.link.write_goals

        def base_shifts_during_opening(goals):
            send(goals)
            if goals.get(6, self.low[6]) > self.low[6] + 4:
                # Real run stopped on this passive drift, 1023 to 1016 at ID1.
                self.link.put(1, 56, 2, 1016)

        self.link.write_goals = base_shifts_during_opening

        with self.assertRaisesRegex((RuntimeError, ValueError), "ID1"):
            self.cycle(lambda *_: self.fail("A displaced fixture reached the direction prompt"))
        self.assertEqual(self.position(4), self.low[4])
        self.assertTrue(all(jaw >= self.low[6] for _, jaw in self.frames))
        self.assertFalse(self.report.get("released_at_support", False))
        self.assertEqual(self.report["physical_outcome"], "unverified")

    def test_stationary_wrist_during_empty_probe_never_closes_on_model(self):
        send = self.link.write_goals

        def stalled(goals):
            send(goals)
            if 4 in goals:
                self.link.put(4, 56, 2, self.low[4])

        self.link.write_goals = stalled

        def authorize_empty_probe(_prompt, allowed):
            self.assertIn("+", allowed)
            return "+"

        with self.assertRaises((RuntimeError, ValueError, TimeoutError)):
            self.cycle(authorize_empty_probe)
        self.assertTrue(all(jaw >= self.low[6] for _, jaw in self.frames))
        self.assertFalse(self.report.get("released_at_support", False))


if __name__ == "__main__":
    unittest.main()

