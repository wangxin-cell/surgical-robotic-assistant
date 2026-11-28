"""Receive WAV audio over an SSH tunnel and run recognition on the O6."""
import argparse
import io
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock
from urllib.parse import urlsplit
import wave
from command_intents import parse_transcript


ROOT = Path(__file__).resolve().parent
MAX_WAV_SECONDS = 16
MAX_SAMPLE_RATE = 96000
MAX_WAV_BYTES = 2 * MAX_SAMPLE_RATE * MAX_WAV_SECONDS + 4096
LABELS = {
    "straight_mayo_scissor": "直头手术剪", "hemostat": "止血钳",
    "tissue_forceps": "组织镊",
    "scalpel_10": "10 号手术刀", "scalpel_11": "11 号手术刀",
    "scalpel_15": "15 号手术刀", "scalpel_20": "20 号手术刀",
    "scalpel_handle_3_blade_10": "3 号刀柄 + 10 号刀片",
    "scalpel_handle_3_blade_11": "3 号刀柄 + 11 号刀片",
    "scalpel_handle_3_blade_15": "3 号刀柄 + 15 号刀片",
    "scalpel_handle_4_blade_20": "4 号刀柄 + 20 号刀片",
}


def read_wav(data):
    """Validate the WAV before handing samples to the board runtime."""
    try:
        with wave.open(io.BytesIO(data), "rb") as wav:
            rate, count = wav.getframerate(), wav.getnframes()
            if wav.getnchannels() != 1 or wav.getsampwidth() != 2 or wav.getcomptype() != "NONE":
                raise ValueError("需要单声道 16 位 PCM WAV。")
            if not 8000 <= rate <= MAX_SAMPLE_RATE or not rate * 0.15 <= count <= rate * MAX_WAV_SECONDS:
                raise ValueError("录音长度或采样率不在支持范围内。")
            samples = wav.readframes(count)
    except (wave.Error, EOFError, OverflowError) as error:
        raise ValueError("收到的音频不是有效 WAV。") from error
    if len(samples) != count * 2:
        raise ValueError("录音数据不完整。")
    return rate, samples


class BoardRecognizer:
    def __init__(self, backend, model_dir, keywords=None):
        import numpy as np
        import sherpa_onnx

        self.np = np
        self.backend = backend
        if backend == "kws":
            suffix = "-epoch-12-avg-2-chunk-16-left-64.int8.onnx"
            self.engine = sherpa_onnx.KeywordSpotter(
                encoder=str(model_dir / ("encoder" + suffix)),
                decoder=str(model_dir / ("decoder" + suffix)),
                joiner=str(model_dir / ("joiner" + suffix)),
                tokens=str(model_dir / "tokens.txt"),
                keywords_file=str(keywords or model_dir / "keywords.txt"),
                num_threads=2, provider="cpu",
            )
        elif backend == "sensevoice":
            self.engine = sherpa_onnx.OfflineRecognizer.from_sense_voice(
                model=str(model_dir / "model.int8.onnx"), tokens=str(model_dir / "tokens.txt"),
                language="zh", use_itn=True, num_threads=3, provider="cpu",
            )
        elif backend == "paraformer":
            self.engine = sherpa_onnx.OfflineRecognizer.from_paraformer(
                paraformer=str(model_dir / "model.int8.onnx"),
                tokens=str(model_dir / "tokens.txt"),
                num_threads=2, provider="cpu",
            )
        else:
            raise ValueError(f"未知语音引擎：{backend}")

    def recognize(self, rate, pcm):
        samples = self.np.frombuffer(pcm, dtype="<i2").astype(self.np.float32) / 32768.0
        stream = self.engine.create_stream()
        stream.accept_waveform(rate, samples)
        if self.backend != "kws":
            self.engine.decode_stream(stream)
            transcript = stream.result.text.strip()
            candidates, message = parse_transcript(transcript)
            return {"status": "transcribed" if transcript else "no_match", "text": transcript,
                    "matches": [{"id": item, "label": LABELS[item]} for item in candidates],
                    "message": message if transcript else ""}
        stream.accept_waveform(rate, self.np.zeros(rate // 2, dtype=self.np.float32))
        stream.input_finished()
        matches = {}
        while self.engine.is_ready(stream):
            self.engine.decode_stream(stream)
            result = self.engine.get_result(stream)
            if result:
                matches.setdefault(result, None)
                self.engine.reset_stream(stream)
        status = {0: "no_match", 1: "matched"}.get(len(matches), "ambiguous")
        return {"status": status, "text": "", "matches": [
            {"id": item, "label": LABELS.get(item, item)} for item in matches
        ]}


def handler_for(recognizer, allowed_origins=()):
    lock = Lock()
    pages = {"/": ("index.html", "text/html; charset=utf-8"),
             "/voice-input.js": ("voice-input.js", "text/javascript; charset=utf-8")}

    class Handler(BaseHTTPRequestHandler):
        def trusted_host(self):
            try:
                host = urlsplit("//" + self.headers.get("Host", "")).hostname
                return host in {"127.0.0.1", "localhost", "::1"}
            except ValueError:
                return False

        def origin_allowed(self):
            origin = self.headers.get("Origin")
            return not origin or origin == "http://" + self.headers.get("Host", "") or origin in allowed_origins

        def reply(self, status, body=b"", mime="application/json; charset=utf-8"):
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            origin = self.headers.get("Origin")
            if origin and self.origin_allowed():
                self.send_header("Access-Control-Allow-Origin", origin)
                self.send_header("Vary", "Origin")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()
            try:
                self.wfile.write(body)
            except OSError:
                pass

        def json(self, status, data):
            self.reply(status, json.dumps(data, ensure_ascii=False).encode("utf-8"))

        def guard(self):
            if self.trusted_host() and self.origin_allowed():
                return True
            self.json(403, {"status": "error", "message": "来源未授权。"})
            return False

        def do_OPTIONS(self):
            if self.guard():
                self.reply(204 if self.path in {"/api/voice", "/api/health"} else 404)

        def do_GET(self):
            if not self.guard():
                return
            if self.path == "/api/health":
                self.json(200, {"status": "ready", "service": "o6-voice", "backend": recognizer.backend})
            elif self.path in pages:
                filename, mime = pages[self.path]
                self.reply(200, (ROOT / filename).read_bytes(), mime)
            else:
                self.json(404, {"status": "error", "message": "接口不存在。"})

        def do_POST(self):
            if not self.guard():
                return
            if self.path != "/api/voice":
                self.json(404, {"status": "error", "message": "接口不存在。"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 44 <= size <= MAX_WAV_BYTES or self.headers.get_content_type() != "audio/wav":
                    raise ValueError("请上传最长 16 秒的单声道 16 位 PCM WAV。")
                self.connection.settimeout(5)
                data = self.rfile.read(size)
                if len(data) != size:
                    raise ValueError("录音上传不完整。")
                rate, pcm = read_wav(data)
            except (ValueError, OSError) as error:
                self.json(400, {"status": "error", "message": str(error)})
                return
            if not lock.acquire(blocking=False):
                self.json(409, {"status": "error", "message": "O6 正在处理上一条录音。"})
                return
            try:
                result = recognizer.recognize(rate, pcm)
                self.json(200, {"backend": recognizer.backend, **result})
            except Exception as error:
                print(f"Voice recognition failed: {error}", flush=True)
                self.json(500, {"status": "error", "message": "板端识别失败，请查看 O6 终端。"})
            finally:
                lock.release()

        def log_message(self, *_):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8772)
    parser.add_argument("--backend", choices=("kws", "paraformer", "sensevoice"), default="kws")
    parser.add_argument("--model-dir", type=Path)
    parser.add_argument("--keywords", type=Path)
    parser.add_argument("--allowed-origin", action="append", default=[])
    args = parser.parse_args()
    directory = args.model_dir or ROOT.parent / {
        "kws": "instrument-kws", "paraformer": "sherpa-onnx-paraformer-zh-small-2024-03-09",
        "sensevoice": "sensevoice",
    }[args.backend]
    keywords = args.keywords
    if args.backend == "kws" and keywords is None and (ROOT / "keywords.txt").is_file():
        keywords = ROOT / "keywords.txt"
    backend = BoardRecognizer(args.backend, directory, keywords)
    with ThreadingHTTPServer(("127.0.0.1", args.port), handler_for(backend, args.allowed_origin)) as server:
        print(f"O6 voice server ready on 127.0.0.1:{args.port} ({args.backend})", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
