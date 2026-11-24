"""Take a read-only snapshot of the O6 right bus and the raised-table calibration."""
import json
import socket
import sys
from datetime import datetime, timezone
from pathlib import Path

import serial


ROOT = Path("/home/radxa/instrument_vision")
SERIAL_NUMBER = "5AAF288004"


def read_register(port, motor, address, size):
    # This is the Feetech READ instruction. The reply includes an error byte and checksum.
    body = bytes([motor, 4, 2, address, size])
    request = b"\xff\xff" + body + bytes([(~sum(body)) & 255])
    for _ in range(2):
        port.reset_input_buffer()
        port.write(request)
        reply = port.read(size + 6)
        if (len(reply) == size + 6 and reply[:5] == bytes([255, 255, motor, size + 2, 0])
                and sum(reply[2:]) % 256 == 255):
            return int.from_bytes(reply[5:-1], "little")
    raise OSError(f"ID {motor}, register {address}: invalid or missing reply")


def sample(port, motor):
    fields = [
        ("model", 3, 2), ("mode", 33, 1), ("torque", 40, 1),
        ("min", 9, 2), ("max", 11, 2), ("offset_raw", 31, 2),
        ("position_raw", 56, 2), ("voltage_raw", 62, 1), ("temperature_c", 63, 1),
    ]
    row = {key: read_register(port, motor, addr, size) for key, addr, size in fields}
    if motor <= 6 and row["mode"] == 0:
        row["in_register_limits"] = row["min"] <= row["position_raw"] <= row["max"]
    return row


def main():
    report = {"utc": datetime.now(timezone.utc).isoformat(), "host": socket.gethostname(),
              "devices": {}, "motors": {}, "bus_complete": False}
    try:
        for folder in (Path("/dev/serial/by-id"), Path("/dev/v4l/by-path")):
            report["devices"].update({str(p): str(p.resolve()) for p in sorted(folder.glob("*"))})
        calibration = ROOT / "o6-table-plane-raised.json"
        if calibration.is_file():
            report["table_calibration"] = json.loads(calibration.read_text())
        ports = [p for p in Path("/dev/serial/by-id").glob("*")
                 if f"_{SERIAL_NUMBER}-" in p.name and p.exists()]
        if len(ports) != 1:
            raise OSError(f"Expected one right controller with serial {SERIAL_NUMBER}, found {ports}")
        report["right_controller"] = str(ports[0].resolve())
        with serial.Serial(str(ports[0]), 1_000_000, timeout=.12,
                           write_timeout=.12, exclusive=True) as port:
            for motor in range(1, 10):
                try:
                    report["motors"][motor] = sample(port, motor)
                except OSError as exc:
                    report["motors"][motor] = {"error": str(exc)}
            report["bus_complete"] = all("error" not in row for row in report["motors"].values())
    except Exception as exc:
        report["error"] = str(exc)
    print(json.dumps(report, ensure_ascii=True, indent=2))
    return 0 if report["bus_complete"] else 1


if __name__ == "__main__":
    sys.exit(main())
