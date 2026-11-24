"""Exercise the demonstration endpoints and fault cleanup using simulated registers."""
import unittest
import itertools
from types import SimpleNamespace
from unittest.mock import patch

from film_grip import FilmingJaw, support_keys
from jog_bus import _SimulatedLink


class Clock:
    def __init__(self):
        self.t = 0.

    def monotonic(self):
        return self.t

    def sleep(self, duration):
        self.t += duration


class DemoTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.link = _SimulatedLink()
        self.writes = []
        self.bus = FilmingJaw(simulate=True)
        simulated = patch("jog_bus._SimulatedLink", return_value=self.link)
        simulated.start()
        self.addCleanup(simulated.stop)
        clock = patch("film_grip.time", self.clock)
        clock.start()
        self.addCleanup(clock.stop)
        bus_write = self.link.write

        def write(i, address, value):
            self.writes.append((i, address, value))
            bus_write(i, address, value)
        self.link.write = write
        self.bus.connect()
        self.addCleanup(self.bus.close)
        self.result = {}

    def test_cycle_records_contact_dwell_and_opens_then_disables(self):
        self.bus.run_cycle(self.result, lambda: None)
        self.assertTrue(self.result["encoder_cycle_completed"])
        self.assertTrue(self.result["jaw_torque_off"])
        self.assertEqual({i for i,a,v in self.writes}, {6})
        self.assertEqual(self.writes[0], (6,48,0))
        self.assertEqual(self.writes[-1], (6,48,500))
        events = self.result["events"]
        self.assertEqual([e["actual"] for e in events], [866,814,814,866])
        self.assertGreaterEqual(events[2]["elapsed"] - events[1]["elapsed"], 2.99)
        self.assertEqual(self.bus._feedback(6)["torque"], 0)
        self.assertGreaterEqual(min(v for _, a, v in self.writes if a == 42), 814)

    def test_abort_while_powered_restores_ram(self):
        before = self.bus._feedback(6)
        def stop():
            if self.bus._feedback(6)["position"] >= 832:
                raise InterruptedError("operator")
        with self.assertRaises(InterruptedError):
            self.bus.run_cycle(self.result, stop)
        after = self.bus._feedback(6)
        for key in ("torque","effort","accel","speed"):
            self.assertEqual(before[key],after[key])
        self.assertNotIn("encoder_cycle_completed", self.result)

    def test_changed_passive_axis_aborts_and_disables_gripper(self):
        def displace():
            if self.bus._feedback(6)["position"] >= 832:
                self.link.put(3,56,2,1200)
        with self.assertRaisesRegex(RuntimeError,"ID3"):
            self.bus.run_cycle(self.result, displace)
        self.assertTrue(self.result["jaw_torque_off"])
        self.assertEqual({i for i,a,v in self.writes}, {6})

    def test_insufficient_opening_does_not_enable(self):
        self.link.put(6,56,2,895)
        with self.assertRaises(ValueError):
            self.bus.run_cycle(self.result, lambda: None)
        self.assertEqual(self.writes, [])

    def test_bearing_requires_closing_room_before_any_write(self):
        self.link.put(6, 56, 2, 727)  # Soft limit is 725; the proposed preload would reach 723.
        with self.assertRaises(ValueError):
            self.bus.run_cycle(self.result, lambda: None, lambda _: self.fail("No usable preload"))
        self.assertEqual(self.writes, [])

    def test_setup_rechecks_soft_edge_and_reserve_without_writes(self):
        self.link.put(6, 56, 2, 915)  # Inside EEPROM limits, beyond the motion window.
        initial = self.bus.setup_snapshot(self.result)
        placements = iter((899, 839))

        def arrange(_):
            self.assertFalse(self.writes)
            self.link.put(6, 56, 2, next(placements))
            return ""

        approved = self.bus.prepare_contact(self.result, initial, arrange)
        self.assertEqual([s["position"] for s in self.result["setup_readings"]], [915, 899, 839])
        self.assertFalse(self.writes)
        self.assertEqual(approved[6]["position"], 839)
        self.bus.run_cycle(self.result, lambda: None, confirmed=approved)
        self.assertEqual(self.result["contact"], 839)

    def test_setup_annotates_hard_limit_and_identity_failures(self):
        self.link.put(6, 56, 2, 925)
        with self.assertRaisesRegex(ValueError, "925 格.*硬限"):
            self.bus.setup_snapshot(self.result)
        self.link.put(6, 56, 2, 814)
        self.link.put(6, 3, 2, 0)
        with self.assertRaisesRegex(ValueError, "814 格.*ID6 身份"):
            self.bus.setup_snapshot(self.result)
        self.assertEqual([s["position"] for s in self.result["setup_readings"]], [925, 814])
        self.assertFalse(self.writes)

    def test_setup_contact_cannot_change_after_confirmation(self):
        initial = self.bus.setup_snapshot(self.result)
        approved = self.bus.prepare_contact(self.result, initial, lambda _: "")
        self.link.put(6, 56, 2, approved[6]["position"] + 4)
        with self.assertRaisesRegex(ValueError, "确认后"):
            self.bus.run_cycle(self.result, lambda: None, confirmed=approved)
        self.assertFalse(self.writes)

    def test_setup_fails_on_enabled_passive_axis(self):
        self.link.put(8, 40, 1, 1)
        with self.assertRaisesRegex(ValueError, "ID8 状态异常"):
            self.bus.setup_snapshot(self.result)
        self.assertFalse(self.writes)

    def stop_opening_at(self, ceiling):
        write = self.link.write
        def capped(i, address, value):
            write(i, address, value)
            if address == 42 and self.bus._feedback(i)["position"] > ceiling:
                self.link.put(i, 56, 2, ceiling)
        self.link.write = capped

    def test_open_plateau_is_measured_then_used_for_release(self):
        self.stop_opening_at(844)
        self.bus.run_cycle(self.result, lambda: None)
        self.assertEqual(self.result["first_measured_open"], 844)
        self.assertEqual([e["actual"] for e in self.result["events"]], [844,814,814,844])
        self.assertIn("实测开位", self.result["events"][0]["stage"])
        self.assertEqual(self.result["events"][-1]["goal"],844)
        self.assertEqual(set(self.result["initial_feedback"]),set(range(1,10)))

    def test_insufficient_measured_open_fails_with_telemetry(self):
        self.stop_opening_at(824)
        with self.assertRaisesRegex(TimeoutError,"实测 824"):
            self.bus.run_cycle(self.result,lambda:None)
        self.assertEqual(self.result["events"], [])
        self.assertEqual(self.result["last_feedback"]["actual"],824)
        self.assertTrue(self.result["jaw_torque_off"])

    def test_failed_closure_never_counts_as_contact(self):
        write = self.link.write
        def blocked(i,address,value):
            previous = self.bus._feedback(6)["position"]
            write(i,address,value)
            if address == 42 and previous >= 830 and value < 830:
                self.link.put(i,56,2,830)
        self.link.write = blocked
        with self.assertRaisesRegex(TimeoutError,"合到剪柄"):
            self.bus.run_cycle(self.result,lambda:None)
        self.assertEqual(len(self.result["events"]),1)
        self.assertEqual(self.result["last_feedback"]["actual"],830)
        self.assertTrue(self.result["jaw_torque_off"])

    def test_bearing_waits_for_fresh_support_confirmation(self):
        def confirm():
            yield False  # Clear queued input.
            for _ in range(30):
                self.assertEqual(self.bus._feedback(6)["torque"], 1)
                yield False
            yield True
        self.bus.run_cycle(self.result, lambda: None, lambda key: confirm() if key == "R" else iter((False, True)))
        self.assertTrue(self.result["contact_confirmation_received"])
        self.assertTrue(self.result["support_restored_by_operator"])
        window = [e for e in self.result["events"] if "抽垫" in e["stage"] or "有承托" in e["stage"]]
        self.assertEqual(len(window), 2)
        self.assertGreater(window[1]["elapsed"] - window[0]["elapsed"], 2)
        self.assertTrue(self.result["encoder_cycle_completed"])
        self.assertNotIn("physical_outcome", self.result)
        self.assertTrue(self.result["jaw_torque_off"])
        self.assertEqual({i for i, _, _ in self.writes}, {6})

    def test_from_contact_only_opens_after_operator_restores_support(self):
        self.link.put(6, 56, 2, 793)
        stages = []

        def confirm(key):
            goals = [value for _, address, value in self.writes if address == 42]
            stages.append(key)
            if key == "C":
                self.assertEqual(set(goals), {793})
            elif key == "R":
                self.assertEqual(goals[-2:], [791, 789])
                self.assertLessEqual(max(goals), 793)
            yield False
            yield True

        self.bus.run_cycle(self.result, lambda: None, confirm, from_contact=True)
        self.assertEqual(stages, ["C", "R"])
        self.assertTrue(self.result["encoder_cycle_completed"])
        self.assertEqual(self.result["confirmed_touch_actual"], 793)
        self.assertEqual(self.result["released_position"], 845)
        self.assertTrue(self.result["jaw_torque_off"])
        self.assertEqual({i for i, _, _ in self.writes}, {6})

    def test_from_contact_base_drift_at_initial_hold_aborts(self):
        self.link.put(1, 56, 2, 1023)
        self.link.put(6, 56, 2, 793)

        def drift():
            if self.bus._feedback(6)["torque"]:
                self.link.put(1, 56, 2, 1016)

        with self.assertRaisesRegex(RuntimeError, "ID1.*1023.*1016.*-7"):
            self.bus.run_cycle(self.result, drift, lambda k: self.fail(k), from_contact=True)
        self.assertEqual({v for _, a, v in self.writes if a == 42}, {793})
        self.assertEqual({i for i, _, _ in self.writes}, {6})
        self.assertTrue(self.result["jaw_torque_off"])
        self.assertTrue(self.result["temporary_settings_restored"])

    def test_from_contact_requires_visual_confirmation_before_tightening(self):
        def no_response(key):
            self.assertEqual(key, "C")
            return itertools.repeat(False)

        with self.assertRaisesRegex(TimeoutError, "C"):
            self.bus.run_cycle(self.result, lambda: None, no_response, from_contact=True)
        self.assertEqual({v for _, a, v in self.writes if a == 42}, {self.result["contact"]})
        self.assertTrue(self.result["jaw_torque_off"])

    def test_from_contact_refuses_insufficient_closing_or_release_travel(self):
        for position in (727, 895):
            with self.subTest(position=position):
                self.link.put(6, 56, 2, position)
                with self.assertRaises(ValueError):
                    self.bus.run_cycle(self.result, lambda: None, lambda k: self.fail(k), from_contact=True)
                self.assertEqual(self.writes, [])

    def bearing_replay(self, closing_samples, confirm):
        """Reproduce measured jaw travel from 776 to 823 and back against the prop."""
        ref = 776
        self.link.put(6, 56, 2, ref)
        self.stop_opening_at(823)
        feedback = self.bus._feedback
        def sample(i):
            row = feedback(i)
            if (i == 6 and row["goal"] <= ref and row["torque"] and
                    "first_measured_open" in self.result):
                row["position"] = next(closing_samples)
            return row
        self.bus._feedback = sample
        self.bus.run_cycle(self.result, lambda: None, confirm)

    def test_observed_real_contact_six_ticks_before_reference(self):
        phases = []
        def confirm(k):
            phases.append(k)
            if k == "C":
                self.assertGreaterEqual(min(v for _, a, v in self.writes if a == 42), 776)
            return iter((False, False, True))
        self.bearing_replay(itertools.repeat(782), confirm)
        self.assertEqual(phases, ["C", "R"])
        self.assertEqual(self.result["candidate_contact_actual"], 782)
        self.assertEqual(self.result["first_measured_open"], 823)
        self.assertTrue(self.result["encoder_cycle_completed"])
        self.assertNotIn("physical_outcome", self.result)
        self.assertEqual(self.result["confirmed_touch_actual"], 782)
        self.assertEqual(self.result["firm_target"], 772)
        self.assertEqual(self.result["firm_actual"], 782)
        self.assertEqual([v for _, a, v in self.writes if a == 42 and v < 776], [774, 772])
        self.assertTrue(all(abs(s["requested"] - s["actual"]) <= 12 for s in self.result["samples"]))
        self.assertEqual({s["effort"] for s in self.result["samples"] if s["torque"]}, {0, 100})
        self.assertEqual(self.result["released_position"], 823)
        self.assertTrue(self.result["jaw_torque_off"])

    def test_loss_of_contact_during_preload_ends_before_shim_is_removed(self):
        def position():
            while True:
                last_goal = self.result.get("last_attempted_goal")
                yield 775 if last_goal == 774 else 782  # Seven ticks of sudden inward travel.

        def confirm(key):
            self.assertEqual(key, "C")
            return iter((False, True))

        with self.assertRaises(RuntimeError):
            self.bearing_replay(position(), confirm)
        self.assertEqual([v for _, a, v in self.writes if a == 42 and v < 776], [774])
        self.assertTrue(self.result["jaw_torque_off"])

    def test_elbow_drift_during_preload_aborts_without_more_squeezing(self):
        initial_elbow = self.bus._feedback(3)["position"]
        original_write = self.link.write

        def disturb_after_first_preload(i, address, value):
            original_write(i, address, value)
            if i == 6 and address == 42 and value == 774:
                self.link.put(3, 56, 2, initial_elbow + 7)

        self.link.write = disturb_after_first_preload
        def confirm(key):
            self.assertEqual(key, "C")
            return iter((False, True))

        with self.assertRaisesRegex(RuntimeError, "ID3"):
            self.bearing_replay(itertools.repeat(782), confirm)
        self.assertTrue(self.result["jaw_torque_off"])
        self.assertTrue(self.result["temporary_settings_restored"])
        self.assertEqual({i for i, _, _ in self.writes}, {6})
        self.assertEqual([v for _, a, v in self.writes if a == 42 and v < 776], [774])

    def test_unstable_contact_rejected_before_any_observation_prompt(self):
        with self.assertRaises(TimeoutError):
            self.bearing_replay(itertools.cycle([782, 778]), lambda _: self.fail("unstable"))
        self.assertNotIn("candidate_contact_actual", self.result)
        self.assertTrue(self.result["jaw_torque_off"])

    def test_stall_outside_hold_window_cannot_prompt_for_contact(self):
        with self.assertRaises(TimeoutError):
            self.bearing_replay(itertools.repeat(784), lambda _: self.fail("premature"))
        self.assertTrue(self.result["jaw_torque_off"])

    def test_unconfirmed_contact_times_out_without_shim_transfer(self):
        def confirm(key):
            self.assertEqual(key, "C")
            return itertools.repeat(False)
        with self.assertRaisesRegex(TimeoutError, "15 秒"):
            self.bearing_replay(itertools.repeat(782), confirm)
        self.assertNotIn("contact_confirmation_received", self.result)
        self.assertTrue(self.result["jaw_torque_off"])
        self.assertGreaterEqual(min(v for _, a, v in self.writes if a == 42), 776)

    def test_quit_at_c_never_applies_preload(self):
        def quit_at_contact(key):
            self.assertEqual(key, "C")
            yield False
            raise InterruptedError("operator quit")
        with self.assertRaises(InterruptedError):
            self.bearing_replay(itertools.repeat(782), quit_at_contact)
        self.assertTrue(self.result["jaw_torque_off"])
        self.assertGreaterEqual(min(v for _, a, v in self.writes if a == 42), 776)

    def test_live_support_key_requires_fresh_r_and_enter(self):
        keys = ["r", "\r"]
        console = SimpleNamespace(kbhit=lambda: bool(keys), getwch=lambda: keys.pop(0))
        with patch.dict("sys.modules", {"msvcrt": console}):
            commands = support_keys()
            self.assertFalse(next(commands))  # Queued input before the prompt is discarded.
            keys.extend(["r"])
            self.assertFalse(next(commands))
            keys.append("\r")
            self.assertTrue(next(commands))

    def test_bearing_timeout_unloads_without_commanding_open(self):
        with self.assertRaisesRegex(TimeoutError, "25 秒"):
            self.bus.run_cycle(self.result, lambda: None,
                               lambda key: itertools.repeat(False) if key == "R" else iter((False, True)))
        self.assertTrue(self.result["jaw_torque_off"])
        self.assertNotIn("encoder_cycle_completed", self.result)
        self.assertNotIn("support_restored_by_operator", self.result)
        self.assertEqual(self.bus._feedback(6)["torque"], 0)
        self.assertEqual(self.result["last_attempted_goal"], self.result["firm_target"])

    def test_stop_during_transfer_restores_ram(self):
        def stop_after_start():
            yield False
            raise InterruptedError("operator")
        before = self.bus._feedback(6)
        with self.assertRaises(InterruptedError):
            self.bus.run_cycle(self.result, lambda: None,
                               lambda key: stop_after_start() if key == "R" else iter((False, True)))
        self.assertTrue(self.result["jaw_torque_off"])
        after = self.bus._feedback(6)
        for k in ("accel", "speed", "effort", "torque"):
            self.assertEqual(after[k], before[k])


if __name__=="__main__":
    unittest.main()
