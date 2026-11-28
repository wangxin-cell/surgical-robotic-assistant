"""Regression checks for vision requests that could otherwise replay the wrong arm trajectory."""
import threading
import time
import unittest
from unittest.mock import Mock, patch

from native_pick import ReplayStopped
from pick_workflow import PickWorkflow


class ConfirmedTargetTests(unittest.TestCase):
    def test_only_confirmed_matches_count_as_pick_candidates(self):
        from native_vision import _summarize
        target = "straight_mayo_scissor"
        scissors = dict(number=1, class_name="ring_broad_split", evidence="found", alternatives=[])
        other = dict(number=3, class_name="ring_narrow_tip", evidence="occluded",
                     alternatives=["ring_broad_split"])
        for items, status, numbers, unique in [
            ([scissors, other], "found", [1], True),
            ([scissors, dict(other, class_name="ring_broad_split", evidence="uncertain")], "found", [1], True),
            ([scissors, dict(scissors, number=3)], "found", [1, 3], False),
            ([other], "occluded", [], False),
            ([], "absent", [], False),
        ]:
            with self.subTest(items=items):
                result = _summarize(target, items, set(), True)
                self.assertEqual((result["status"], result["numbers"], result["unique"]),
                                 (status, numbers, unique))


class PickWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.scene = {"source": "live", "views": {"head": "camera_1.jpg"}}
        self.decision = {"ready": True, "slot": 2, "center": [.4, .5], "class_name": "blade_shape",
                         "label": "刀形", "message": "点位 2"}
        self.read_camera = Mock(return_value=self.scene)
        self.motion = Mock(return_value="六轴结束姿态已核对")
        def simulated_cycle(slot, stop, report, verify):
            verify()
            return self.motion(slot)
        self.replay = Mock(side_effect=simulated_cycle)
        self.work = PickWorkflow("unused.json", threading.Lock(), observe=self.read_camera, replay=self.replay)
        self.work.locations = Mock()
        self.work.locations.match.return_value = self.decision
        self.work.locations.match_slot.return_value = self.decision
        self.work.locations.status.return_value = {"points": [[.2,.5],[.4,.5],[.6,.5],[.8,.5]],
                                                  "image_size": [1920,1080]}

    def wait_end(self):
        until = time.monotonic() + 3
        while self.work.active() and time.monotonic() < until:
            time.sleep(.005)
        self.assertFalse(self.work.active())
        return self.work.status()["state"]

    def test_recapture_then_dispatch_once(self):
        self.work.remember("request", self.scene, ["hemostat"])
        self.work.start("request")
        self.assertEqual(self.wait_end(), "completed")
        self.work.start("request")
        self.read_camera.assert_called_once_with(["hemostat"])
        self.assertEqual(self.replay.call_count, 1)
        self.assertEqual(self.replay.call_args.args[0], 2)
        self.motion.assert_called_once_with(2)

    def test_fixed_slot_is_manual_and_replays_once_without_camera(self):
        self.work.remember("voice", self.scene, ["hemostat"])
        ticket = self.work.fixed_request(2)["request_id"]
        for wrong in (ticket, "voice"):
            with self.assertRaises(ValueError):
                self.work.start(wrong)
        for slot, confirmed in [(1, True), (True, True), (2, False), (2, 1)]:
            with self.subTest(slot=slot, confirmed=confirmed), self.assertRaises(ValueError):
                self.work.start_fixed(ticket, slot, confirmed)
        job = self.work.start_fixed(ticket, 2, True)
        self.assertEqual((job["kind"], job["slot"]), ("fixed", 2))
        self.assertEqual(self.wait_end(), "completed")
        self.assertEqual(self.work.start_fixed(ticket, 2, True)["state"], "completed")
        self.motion.assert_called_once_with(2)
        self.read_camera.assert_not_called()

    def test_resume_ticket_is_single_use_and_target_bound(self):
        from package_replay import Cursor, Interrupted
        cursor = Cursor(2, [2000]*6)
        cursor.done = 5
        self.work.replay = Mock(side_effect=[Interrupted("暂停",cursor),"完成"])
        ticket = self.work.fixed_request(2)["request_id"]
        self.work.start_fixed(ticket,2,True)
        self.assertEqual(self.wait_end(),"failed")
        token = self.work.status()["resume_token"]
        with self.assertRaises(ValueError):
            self.work.resume("other",token)
        self.work.resume(ticket,token)
        with self.assertRaises(ValueError):
            self.work.resume(ticket,token)
        self.assertEqual(self.wait_end(),"completed")
        self.assertEqual(self.work.replay.call_args.kwargs["cursor"],cursor)
        self.assertNotIn("resume_token",self.work.status())

    def test_fixed_preview_is_scoped_to_latest_selection_or_lookup(self):
        for bad in (None, True, 2.0, 5):
            with self.subTest(slot=bad), self.assertRaises(ValueError):
                self.work.fixed_request(bad)
        a = self.work.fixed_request(1)["request_id"]
        b = self.work.fixed_request(2)["request_id"]
        with self.assertRaises(ValueError):
            self.work.start_fixed(a, 1, True)
        self.work.remember("new", self.scene, ["hemostat"])
        with self.assertRaises(ValueError):
            self.work.start_fixed(b, 2, True)
        with patch("pick_workflow.FIXED_TICKET_SECONDS", 0):
            c = self.work.fixed_request(2)["request_id"]
        with self.assertRaises(ValueError):
            self.work.start_fixed(c, 2, True)
        self.replay.assert_not_called()

    def test_fixed_stop_during_home_blocks_playback(self):
        entered = threading.Event()
        def cycle(slot, stop, update, verify):
            entered.set()
            self.assertTrue(stop.wait(1))
            verify()
            self.motion(slot)
        self.replay.side_effect = cycle
        ticket = self.work.fixed_request(1)["request_id"]
        self.work.start_fixed(ticket, 1, True)
        self.assertTrue(entered.wait(1))
        with self.assertRaises(ValueError):
            self.work.fixed_request(2)
        self.work.stop()
        self.assertEqual(self.wait_end(), "stopped")
        self.motion.assert_not_called()

    def test_click_replays_selected_file_once_without_new_camera_request(self):
        for slot in (1, 2, 3, 4):
            request = str(slot)
            self.work.locations.match.return_value = {**self.decision, "slot": slot}
            self.work.remember(request, self.scene, ["hemostat"])
            self.work.start(request, direct=True)
            self.assertEqual(self.wait_end(), "completed")
            self.work.start(request, direct=True)
            self.motion.assert_called_with(slot)
        self.assertEqual(self.motion.call_count, 4)
        self.read_camera.assert_not_called()

    def test_home_alone_consumes_ticket_and_invalidates_previous_pick(self):
        self.work.home = Mock(return_value="六轴实测已核对。")
        self.work.remember("old-pick", self.scene, ["hemostat"])
        ticket = self.work.home_request()
        self.assertEqual(self.work.home_request(), ticket)
        with self.assertRaises(ValueError):
            self.work.start_home("unknown")
        result = self.work.start_home(ticket)
        self.assertEqual((result["kind"], result["slot"]), ("home", None))
        self.assertEqual(self.wait_end(), "completed")
        self.assertEqual(self.work.start_home(ticket)["message"], "六轴实测已核对。")
        with self.assertRaises(ValueError):
            self.work.start("old-pick")
        self.work.home.assert_called_once()
        self.read_camera.assert_not_called()
        self.replay.assert_not_called()
        self.motion.assert_not_called()
        self.work.remember("next-pick", self.scene, ["hemostat"])
        self.work.start("next-pick")
        self.assertEqual(self.wait_end(), "completed")
        with self.assertRaises(ValueError):
            self.work.start_home(ticket)

    def test_home_shares_stop_and_excludes_pick(self):
        entered = threading.Event()
        def home(stop, update):
            entered.set()
            self.assertTrue(stop.wait(2))
            raise ReplayStopped("复位已停止。")
        self.work.home = home
        token = self.work.home_request()
        self.work.start_home(token)
        self.assertTrue(entered.wait(1))
        self.assertEqual(self.work.start_home(token)["state"], "homing")
        active_token = self.work.home_request()
        with self.assertRaises(ValueError):
            self.work.start("pick")
        self.work.stop()
        self.assertEqual(self.wait_end(), "stopped")
        with self.assertRaises(ValueError):
            self.work.start_home(active_token)

    def test_superseded_or_archived_request_cannot_start(self):
        self.work.remember("first", self.scene, ["hemostat"])
        self.work.remember("second", {**self.scene, "source":"archive"}, ["hemostat"])
        for request in ("first", "second"):
            with self.assertRaises(ValueError):
                self.work.start(request)
        self.replay.assert_not_called()

    def test_manual_slot_does_not_substitute_another_shape(self):
        self.work.remember_slot("manual", self.scene, 2)
        self.work.locations.match_slot.return_value = {**self.decision, "class_name":"tweezer_shape"}
        self.work.start("manual")
        self.assertEqual(self.wait_end(), "failed")
        self.read_camera.assert_called_once_with([])
        self.motion.assert_not_called()

    def test_target_moved_within_same_slot(self):
        self.work.remember("moving", self.scene, ["hemostat"])
        self.work.locations.match.return_value = {**self.decision, "center":[.45,.5]}
        self.work.start("moving")
        self.assertEqual(self.wait_end(), "failed")
        self.motion.assert_not_called()

    def test_homing_failure_prevents_recapture_and_recorded_motion(self):
        self.replay.side_effect = ValueError("ID3 超出限位")
        self.work.remember("limit", self.scene, ["hemostat"])
        self.work.start("limit")
        self.assertEqual(self.wait_end(), "failed")
        self.work.start("limit")
        self.replay.assert_called_once()
        self.read_camera.assert_not_called()
        self.motion.assert_not_called()

    def test_origin_ticket_preserves_recognition_and_needs_separate_pick(self):
        from unittest.mock import patch
        self.work.remember("one",self.scene,["hemostat"])
        refs=[None]*6
        with patch("pick_workflow.prepare_slot_origin",return_value=refs) as align:
            self.work.prepare_origin("one")
            self.assertEqual(self.wait_end(),"completed")
            self.assertTrue(self.work.status()["origin_ready"])
            self.work.prepare_origin("one")
            self.assertEqual(align.call_count,1)
            self.replay.assert_not_called()
            self.work.start("one")
            self.assertEqual(self.wait_end(),"completed")
            self.read_camera.assert_called_once()

    def test_stop_during_start_check_prevents_recapture(self):
        started = threading.Event()
        def home(slot, stop, report, verify):
            started.set()
            self.assertTrue(stop.wait(2))
            verify()
        self.replay.side_effect = home
        self.work.remember("home", self.scene, ["hemostat"])
        self.work.start("home")
        self.assertTrue(started.wait(1))
        self.assertEqual(self.work.status()["state"], "checking")
        self.work.stop()
        self.assertEqual(self.wait_end(), "stopped")
        self.read_camera.assert_not_called()

    def test_late_native_callback_cannot_authorize_motion(self):
        late = []
        def timeout(slot, stop, report, verify):
            late.append(verify)
            raise RuntimeError("复核超时")
        self.replay.side_effect = timeout
        self.work.remember("timeout", self.scene, ["hemostat"])
        self.work.start("timeout")
        self.assertEqual(self.wait_end(), "failed")
        with self.assertRaises(ReplayStopped):
            late[0]()
        self.read_camera.assert_not_called()

    def test_stop_during_camera_capture_prevents_motion(self):
        started, unblock = threading.Event(), threading.Event()
        def observe(ids):
            started.set()
            unblock.wait(2)
            return self.scene
        self.work.observe = observe
        self.work.remember("stop", self.scene, ["hemostat"])
        self.work.start("stop")
        self.assertTrue(started.wait(1))
        self.assertEqual(self.work.status()["state"], "checking")
        self.work.stop()
        unblock.set()
        self.assertEqual(self.wait_end(), "stopped")
        self.motion.assert_not_called()


if __name__ == "__main__":
    unittest.main()
