"""V4L2 exposure controls used by the three cameras."""
import struct

AUTO = 0x009A0901
ABSOLUTE = 0x009A0902
MANUAL = 1


def control(fd, cid):
    import fcntl

    data = bytearray(struct.pack("=II32s4i3I", cid, 0, b"", 0, 0, 0, 0, 0, 0, 0))
    fcntl.ioctl(fd, 0xC0445624, data)  # VIDIOC_QUERYCTRL
    _, _, name, lo, hi, step, default, flags, _, _ = struct.unpack("=II32s4i3I", data)
    if flags & 1:  # V4L2_CTRL_FLAG_DISABLED
        raise RuntimeError(f"Camera control {cid:#x} is disabled")
    value = bytearray(struct.pack("=Ii", cid, 0))
    fcntl.ioctl(fd, 0xC008561B, value)  # VIDIOC_G_CTRL
    return dict(name=name.rstrip(b"\0").decode(errors="replace"), min=lo, max=hi,
                step=step, default=default, value=struct.unpack("=Ii", value)[1])


def write_control(fd, cid, value):
    import fcntl

    fcntl.ioctl(fd, 0xC008561C, struct.pack("=Ii", cid, value))  # VIDIOC_S_CTRL
    result = control(fd, cid)["value"]
    if result != value:
        raise RuntimeError(f"Camera read back {result}, requested {value}")
