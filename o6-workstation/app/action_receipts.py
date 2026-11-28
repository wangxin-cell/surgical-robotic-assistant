"""Display actual controller acknowledgements for the current confirmed vision request."""
import json
from http.client import HTTPConnection, HTTPException
from pathlib import Path
from queue import SimpleQueue
from threading import Lock, Thread
from uuid import UUID


class ReceiptError(ValueError):
    """An event has no matching active target or cannot follow the existing event."""


_TRANSITIONS = {
    "awaiting_controller": {"grasp_started"},
    "grasp_started": {"awaiting_verification", "grasped", "failed", "unconfirmed"},
    "awaiting_verification": {"grasped", "failed", "unconfirmed"},
    "grasped": set(),
    "failed": set(),
    "unconfirmed": set(),
}


def _request_id(value):
    try:
        return str(UUID(str(value)))
    except (AttributeError, ValueError, TypeError) as exc:
        raise ReceiptError("无效的请求编号") from exc


class ReceiptTracker:
    """Call arm only for a current request; the caller authenticates controller receipts.

    State is per target because one spoken request can name several instruments.
    No method issues a motion command. Repeating a receipt is idempotent.
    """

    def __init__(self):
        self._lock = Lock()
        self._request = None
        self._states = {}

    def invalidate(self):
        with self._lock:
            self._request = None
            self._states.clear()

    def arm(self, request_id, target_ids, vision_status):
        """Replace the eligible targets with those confirmed found in this request."""
        request_id = _request_id(request_id)
        if isinstance(target_ids, str):
            target_ids = [target_ids]
        if not isinstance(target_ids, (list, tuple, set)) or any(
                not isinstance(target, str) or not target.strip() for target in target_ids):
            raise ReceiptError("无效的器具编号")
        eligible = dict.fromkeys(target_ids) if vision_status == "found" else {}
        with self._lock:
            previous = self._states if self._request == request_id else {}
            self._request = request_id
            self._states = {target: previous.get(target, "awaiting_controller") for target in eligible}

    def acknowledge(self, request_id, target_id, event):
        """Used exclusively by the authenticated controller input."""
        request_id = _request_id(request_id)
        if not isinstance(target_id, str) or event not in (
                "grasp_started", "awaiting_verification", "grasped", "failed", "unconfirmed"):
            raise ReceiptError("无效的动作回执")
        with self._lock:
            if request_id != self._request or target_id not in self._states:
                raise ReceiptError("该请求或器具尚未通过本轮视觉核对，或请求已经结束")
            current = self._states[target_id]
            if event != current and event not in _TRANSITIONS[current]:
                raise ReceiptError("动作回执的顺序与当前状态不符")
            self._states[target_id] = event
            return {"request_id": request_id, "target_id": target_id, "state": event}

    def current(self):
        with self._lock:
            if self._request is None:
                return None
            return {"request_id": self._request, "states": dict(self._states)}


class ControllerReceipts:
    """Laptop-side adapter for the real controller's feedback callbacks.

    current_request() returns an eligible request, or None. When the real
    controller associates its own job with that request, keep that exact dict
    with the job. Pass it to send_receipt() when hardware confirms motion
    started, grasp completed, or failed. Never look up the newest request to
    report an older job. The HTTP interface only reports these events.
    """

    def __init__(self, port=None, key_path=None, timeout=3):
        runtime = Path(__file__).parent / "runtime"
        if port is None:
            port = 8766
            try:
                saved = (runtime / "workstation-port.txt").read_text(encoding="ascii").strip()
                if saved in {"8766", "8767", "8768", "8769", "8770", "8771"}:
                    port = int(saved)
            except OSError:
                pass
        self.port = port
        key_name = "controller.key" if port == 8766 else f"controller-{port}.key"
        self.key_path = Path(key_path) if key_path is not None else runtime / key_name
        self.timeout = timeout

    def _call(self, method, path, payload=None):
        token = self.key_path.read_text(encoding="utf-8").strip()
        if not token:
            raise ReceiptError("控制器回执密钥为空")
        headers = {"X-O6-Controller": token}
        body = None
        if payload is not None:
            headers["Content-Type"] = "application/json"
            body = json.dumps(payload).encode("utf-8")
        connection = HTTPConnection("127.0.0.1", self.port, timeout=self.timeout)
        try:
            connection.request(method, path, body=body, headers=headers)
            response = connection.getresponse()
            data = response.read()
            if not 200 <= response.status < 300:
                raise ReceiptError(f"显示窗口拒绝该控制器请求，HTTP {response.status}")
            return json.loads(data) if data else {}
        finally:
            connection.close()

    def current_request(self):
        return self._call("GET", "/controller/current")["request"]

    def send_receipt(self, request, event):
        return self._call("POST", "/controller/receipt", {
            "request_id": request["request_id"],
            "target_id": request["target_id"],
            "event": event,
        })


class PracticeScissorFeedback:
    """Best-effort telemetry for one independently started straight-scissor demo."""

    @classmethod
    def bind(cls, client=None):
        # Called before torque. Freeze the request now, even if the window later
        # moves to another instrument or the operator replays an older session.
        client = client if client is not None else ControllerReceipts(timeout=.45)
        try:
            req = client.current_request()
            if not isinstance(req, dict) or req.get("target_id") != "straight_mayo_scissor":
                return None
            request = {"target_id": req["target_id"], "request_id": _request_id(req["request_id"])}
            return cls(client, request)
        except (OSError, HTTPException, ValueError, KeyError, RuntimeError):
            return None

    def __init__(self, client, request):
        self.events = SimpleQueue()
        self.started = False
        self.worker = Thread(target=self._send, args=(client, request), daemon=True)
        self.worker.start()

    def _send(self, client, request):
        for event in iter(self.events.get, None):
            try:
                try:
                    client.send_receipt(request, event)
                except (OSError, HTTPException):
                    client.send_receipt(request, event)  # A lost reply is safe to retry.
            except (OSError, HTTPException, ValueError, KeyError):
                return

    def closing_observed(self, start, actual):
        if not self.started and actual < start:
            self.started = True
            self.events.put("grasp_started")

    def awaiting_observation(self):
        if self.started:
            self.events.put("awaiting_verification")

    def finish(self, outcome):
        """Call after closing the serial bus. A missing physical result stays unknown."""
        if self.started:
            event = {"operator_observed_lift_and_return": "grasped",
                     "operator_observed_failure": "failed"}.get(outcome, "unconfirmed")
            self.events.put(event)
        self.events.put(None)
        self.worker.join(timeout=1.5)

