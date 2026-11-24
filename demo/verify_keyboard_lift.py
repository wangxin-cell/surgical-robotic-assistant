"""Offline keyboard teaching regressions, using the right-bus register simulator only."""
import threading
import unittest
from contextlib import ExitStack, nullcontext
from types import SimpleNamespace
from unittest.mock import Mock, patch

import film_pick_lift as lift
from jog_bus import BUS_IDS, MARGIN, PROFILE
from smooth_tools import TracedSmoothBus
from verify_smooth_bus import RecordingLink


class SimulationStop(threading.Event):
    def __init__(self):
        super().__init__()
        self.seconds = 0.0

    def sleep(self, delay):
        self.seconds += delay
        if self.seconds > 120:
            raise AssertionError("No progress after two simulated minutes")

    def wait(self, timeout):
        self.sleep(timeout)
        return self.is_set()


class TestKeyboardLift(unittest.TestCase):
    def setUp(self):
        self.link = RecordingLink()
        # Most recent hardware low pose. Unpowered base and elbow are outside their profiles.
        for i, p in enumerate((996, 2417, 857, 1412, 3079, 840, 3183, 3517, 1003), 1):
            self.link.put(i, 56, 2, p)
        self.stop = SimulationStop()
        for hook in (patch("jog_bus._SimulatedLink", return_value=self.link),
                     patch("time.monotonic", side_effect=lambda: self.stop.seconds),
                     patch("time.sleep", side_effect=self.stop.sleep)):
            hook.start()
            self.addCleanup(hook.stop)
        self.original = {i: self.link.read(i, 40, 10) for i in lift.ACTIVE}
        self.bus = TracedSmoothBus(simulate=True, bus_class=lift.SupportedWristBus)
        self.bus.connect()
        self.contact = lift.read_pose(self.bus)
        self.report = {"encoder_stages": [], "encoder_cycle_completed": False}

    def position(self, axis):
        return int.from_bytes(self.link.read(axis, 56, 2), "little")

    def tearDown(self):
        if self.bus.state != "passive":
            self.bus.release()
        for axis in BUS_IDS:
            self.assertEqual(self.link.read(axis, 40, 1), b"\0")
        for axis, before in self.original.items():
            after = self.link.read(axis, 40, 10)
            for low, high in ((1, 2), (6, 10)):  # RAM acceleration, speed and effort.
                self.assertEqual(after[low:high], before[low:high])
        self.assertTrue(all(axis in lift.ACTIVE for axis, _, _ in self.link.writes))
        self.assertTrue(all(set(frame).issubset(lift.ACTIVE) for frame in self.link.frames))
        self.bus.close()

    def cycle(self, ask):
        return lift.keyboard_cycle(self.bus, self.contact, self.stop, self.report, ask)

    def test_wrong_direction_returns_before_teaching_the_opposite_and_replay(self):
        keys = iter(["+", "R", "-", "U", "G"])
        prompts = []

        def choose(_prompt, allowed):
            key = next(keys)
            self.assertIn(key, allowed)
            prompts.append((key, self.position(lift.WRIST), self.position(lift.JAW)))
            return key

        self.cycle(choose)
        low = self.contact[lift.WRIST]
        self.assertEqual(next(keys, None), None)
        self.assertEqual(prompts[0][1], low)
        self.assertGreater(prompts[1][1], low)
        self.assertEqual(prompts[2][1], low)
        self.assertLess(prompts[3][1], low)
        self.assertGreaterEqual(low - prompts[4][1], lift.WRIST_TRAVEL[0])
        self.assertLessEqual(low - prompts[4][1], lift.WRIST_TRAVEL[1])
        self.assertTrue(all(jaw - self.contact[lift.JAW] >= lift.JAW_OPENING[0]
                            for _, _, jaw in prompts))
        self.assertTrue(self.report["encoder_cycle_completed"])
        self.assertEqual([s["id"] for s in self.report["encoder_stages"]], [4, 6, 4, 4, 6, 4])

    def test_stale_low_pose_rejected_before_enabling(self):
        self.link.put(4, 56, 2, self.contact[4] + 12)
        with self.assertRaises((ValueError, RuntimeError)):
            self.cycle(lambda *_: self.fail("Asked for motion after stale start"))
        self.assertFalse(self.link.writes)

    def test_jaw_near_soft_stop_has_no_opening_budget(self):
        self.link.put(6, 56, 2, PROFILE[6][1] - MARGIN - 4)
        self.contact = lift.read_pose(self.bus)
        with self.assertRaises((ValueError, RuntimeError)):
            self.cycle(lambda *_: self.fail("Insufficient jaw travel"))
        self.assertFalse(self.link.frames)

    def test_roll_drift_between_commands_stops_before_wrist_probe(self):
        def disturb(_prompt, _allowed):
            self.link.put(5, 56, 2, self.contact[5] - 8)
            return "+"

        with self.assertRaisesRegex((ValueError, RuntimeError), "ID5"):
            self.cycle(disturb)
        self.assertEqual(self.position(4), self.contact[4])
        self.assertFalse(self.report["encoder_stages"])

    def test_operator_abort_before_replay_has_no_recorded_lift(self):
        keys = iter(["+", "U"])

        def choose(_prompt, allowed):
            key = next(keys, None)
            if key is None:
                self.assertIn("G", allowed)
                raise InterruptedError("operator quit")
            self.assertIn(key, allowed)
            return key

        with self.assertRaises(InterruptedError):
            self.cycle(choose)
        self.assertFalse(self.report["encoder_stages"])
        self.assertFalse(self.report["encoder_cycle_completed"])
        self.assertGreater(self.position(6), self.contact[6])

    def test_stalled_opening_never_probes_wrist(self):
        send = self.link.write_goals

        def stall_jaw(goals):
            send(goals)
            if 6 in goals:
                self.link.put(6, 56, 2, self.contact[6])

        self.link.write_goals = stall_jaw
        with self.assertRaises((RuntimeError, TimeoutError)):
            self.cycle(lambda *_: self.fail("Pilot requested with a stuck jaw"))
        self.assertEqual(self.position(4), self.contact[4])
        self.assertFalse(self.report["encoder_stages"])

    def pilot_from_camera_run(self, feedback):
        # Low-point wrist read 1359. It was 1358 immediately before the pilot.
        # Reproduce both readings and the separately measured, already-open jaw.
        self.link.put(4, 56, 2, 1359)
        self.link.put(6, 56, 2, 870)
        opened = lift.read_pose(self.bus)
        self.bus.bus.bind_fixture(opened)
        self.bus.enable_hold(stop_event=self.stop)
        self.link.put(4, 56, 2, 1358)
        send = self.link.write_goals

        def respond(goals):
            send(goals)
            if 4 in goals:
                self.link.put(4, 56, 2, feedback(goals[4]))

        self.link.write_goals = respond
        moves = self.report["teaching_moves"] = []
        return lift.wrist_direction_pilot(self.bus, opened, 1, self.stop, moves)

    def test_pilot_uses_actual_start_and_latches_the_observed_partial_stroke(self):
        pilot = self.pilot_from_camera_run(lambda goal: min(goal, 1366))
        wrist_goals = [frame[4] for frame in self.link.frames if 4 in frame]
        self.assertEqual(max(wrist_goals), 1374)
        self.assertEqual(wrist_goals[-1], 1366)
        self.assertEqual(pilot[4], 1366)
        self.assertEqual(self.bus.status()["rows"][4]["target"], 1366)
        self.assertEqual(self.report["teaching_moves"][-1]["actual"], 1366)
        self.assertFalse(self.report["encoder_stages"])

    def test_pilot_rejects_insufficient_stroke(self):
        with self.assertRaises(RuntimeError):
            self.pilot_from_camera_run(lambda goal: min(goal, 1365))  # Only +7.
        self.assertFalse(self.report["teaching_moves"])

    def test_pilot_rejects_stationary_wrist(self):
        with self.assertRaises(RuntimeError):
            self.pilot_from_camera_run(lambda _goal: 1358)
        self.assertFalse(self.report["teaching_moves"])

    def test_pilot_rejects_reversed_feedback(self):
        with self.assertRaises(RuntimeError):
            self.pilot_from_camera_run(lambda goal: 1358 - min(3, abs(goal - 1358)))
        self.assertFalse(self.report["teaching_moves"])

    def test_operator_stop_is_not_mistaken_for_the_pilot_latch(self):
        def interrupt(goal):
            if goal >= 1373:
                self.stop.set()
            return min(goal, 1366)

        with self.assertRaises(InterruptedError):
            self.pilot_from_camera_run(interrupt)
        self.assertFalse(self.report["teaching_moves"])
        self.assertFalse(self.report["encoder_stages"])

    def test_console_entry_routes_wait_timeout_to_supported_release(self):
        self.bus.close()
        self.bus = TracedSmoothBus(simulate=True, bus_class=lift.SupportedWristBus)
        # No key arrives during the monitored direction prompt. Model the local
        # operator acknowledging support with 's' only at the release prompt.
        console = SimpleNamespace(kbhit=lambda: False, getwch=lambda: "s")
        self.stop.native_check = lift.no_stop
        release = lift.confirm_supported

        def supported(bus, check):
            self.assertEqual(bus.state, "holding")
            console.kbhit = lambda: True
            return release(bus, check)

        with ExitStack() as stack:
            for hook in (patch.object(lift, "TracedSmoothBus", return_value=self.bus),
                         patch.object(lift, "Stop", return_value=self.stop),
                         patch.object(lift, "motion_stop", return_value=nullcontext(lift.no_stop)),
                         patch("sys.argv", ["film_pick_lift.py", "--live"]),
                         patch("sys.platform", "win32"),
                         patch("sys.stdin", Mock(isatty=Mock(return_value=True))),
                         patch("builtins.input", return_value="K"),
                         patch.dict("sys.modules", {"msvcrt": console})):
                stack.enter_context(hook)
            release_call = stack.enter_context(patch.object(lift, "confirm_supported",
                                                            side_effect=supported))
            with self.assertRaisesRegex(ValueError, "超时"):
                lift.main()
        release_call.assert_called_once()
        self.assertEqual(self.bus.state, "passive")
        self.assertEqual(self.position(4), self.contact[4])


if __name__ == "__main__":
    unittest.main()
