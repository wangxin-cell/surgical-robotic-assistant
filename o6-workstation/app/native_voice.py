"""Recognize browser microphone WAV data using Linux/arm64 models on the O6."""
from pathlib import Path
import sys
import threading

ASSETS = Path(__file__).resolve().parent / "voice_assets"
sys.path.insert(0, str(ASSETS / "kit"))
from command_intents import HOLD, SOUND_ALIKES, command_text, parse_transcript
from voice_server import BoardRecognizer, LABELS, MAX_WAV_BYTES, read_wav

_engines = {}
_engine_lock = threading.Lock()
_decode_lock = threading.Lock()


def _local(backend="sensevoice"):
    with _engine_lock:
        if backend not in _engines:
            model = ASSETS / ("sensevoice" if backend == "sensevoice" else "model")
            _engines[backend] = BoardRecognizer(backend, model)
        return _engines[backend]


def voice_health():
    try:
        _local()
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        return {"status": "unavailable", "backend": "sensevoice", "engine_location": "o6",
                "message": f"O6 离线语音模型未就绪：{exc}"}
    return {"status": "ready", "backend": "sensevoice", "engine_location": "o6",
            "message": "已加载 O6 本机普通话识别模型"}


def _prepare_audio(rate, pcm):
    """Measure sustained 20 ms AC levels, so a click or DC bias is not speech."""
    import numpy as np
    samples = np.frombuffer(pcm, dtype="<i2").astype(np.float32)
    width = max(1, round(rate * .02))
    blocks = samples[:len(samples) // width * width].reshape(-1, width)
    levels = np.std(blocks, axis=1)
    sustained = float(np.min(np.lib.stride_tricks.sliding_window_view(levels, 3), axis=1).max())
    if sustained < 20:
        return None
    # Light amplification for a quiet microphone; no gain on normal recordings.
    if sustained < 600:
        gain = min(8., 600 / sustained, 32767 / max(abs(samples).max(), 1.))
        if gain > 1:
            return (samples * gain).astype("<i2").tobytes()
    return pcm


def recognize_wav(data):
    if not isinstance(data, bytes) or not 44 <= len(data) <= MAX_WAV_BYTES:
        raise ValueError("请上传最长 16 秒的单声道 16 位 PCM WAV。")
    rate, pcm = read_wav(data)
    if not _decode_lock.acquire(blocking=False):
        raise RuntimeError("正在识别上一条语音，请稍后重试。")
    try:
        pcm = _prepare_audio(rate, pcm)
        info = {"engine_location": "o6", "backend": "sensevoice"}
        if pcm is None:
            return {**info, "status": "no_match", "text": "", "matches": [], "input_status": "too_quiet",
                    "message": "录音中没有足够的声音，请检查所选麦克风或靠近麦克风重试。"}
        transcript = _local().recognize(rate, pcm).get("text", "").strip()
        ids, message = parse_transcript(transcript)
        if (not ids and not HOLD.search(command_text(transcript))
                and any(pattern.search(transcript) for pattern, _ in SOUND_ALIKES.values())):
            acoustic = _local("kws").recognize(rate, pcm)
            ids, message = parse_transcript(transcript, [hit["id"] for hit in acoustic["matches"]])
        return {**info, "status": "transcribed" if ids else "no_match", "text": transcript,
                "matches": [{"id": item, "label": LABELS[item]} for item in ids],
                "message": message if transcript else "有录音，未辨出普通话词句，请再说一次器具名称。"}
    finally:
        _decode_lock.release()
