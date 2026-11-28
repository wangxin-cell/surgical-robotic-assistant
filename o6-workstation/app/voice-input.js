// Portable microphone and O6 transport. This module does not depend on the UI.
export const MAX_RECORDING_MS = 15000;
const virtualMic = /tran\s*screen|virtual|虚拟/i;
const arrayMic = /麦克风阵列|麥克風陣列|microphone\s*array|array\s*microphone/i;

// Keep the ordinary mix when both microphone channels agree. Some stereo
// capture drivers duplicate speech with the opposite polarity on channel 2.
function monoSpeech(channels) {
    if (channels.length === 1) return channels[0];
    const mix = new Float32Array(channels[0].length);
    let strongest = channels[0], strongestEnergy = 0;
    for (const channel of channels) {
        let energy = 0;
        for (let i = 0; i < mix.length; i++) {
            mix[i] += channel[i] / channels.length;
            energy += channel[i] * channel[i];
        }
        if (energy > strongestEnergy) { strongest = channel; strongestEnergy = energy; }
    }
    let mixedEnergy = 0;
    for (const value of mix) mixedEnergy += value * value;
    return mixedEnergy <= strongestEnergy * 0.25 ? strongest : mix;
}

function meterInput(stream, onLevel) {
    const context = new AudioContext();
    let timer;
    try {
        const source = context.createMediaStreamSource(stream);
        const count = stream.getAudioTracks()[0].getSettings?.().channelCount || 2;
        const splitter = context.createChannelSplitter(count);
        source.connect(splitter);
        const analysers = Array.from({length:count}, (_,i) => {
            const analyser = context.createAnalyser();
            analyser.fftSize = 2048;
            splitter.connect(analyser,i);
            return analyser;
        });
        const channels = analysers.map(a => new Float32Array(a.fftSize));
        context.resume().catch(() => {});
        timer = setInterval(() => {
            analysers.forEach((a,i) => a.getFloatTimeDomainData(channels[i]));
            const values = monoSpeech(channels);
            let sum = 0;
            for (const v of values) sum += v * v;
            const db = 10 * Math.log10(sum / values.length || 1e-12);
            onLevel(Math.max(0, Math.min(100, Math.round((db + 60) * 2))));
        }, 100);
        return () => {
            clearInterval(timer);
            source.disconnect();
            context.close().catch(() => {});
        };
    } catch (error) {
        context.close().catch(() => {});
        throw error;
    }
}

export function encodeMonoWav(audio, checkSignal = false, virtual = false) {
    const count = audio.length;
    const channels = Array.from({length: audio.numberOfChannels}, (_, i) => audio.getChannelData(i));
    const mono = monoSpeech(channels);
    const buffer = new ArrayBuffer(44 + 2 * count);
    const view = new DataView(buffer);
    const ascii = (offset, value) => {
        for (let i = 0; i < value.length; i++) view.setUint8(offset + i, value.charCodeAt(i));
    };
    const word = (offset, value) => view.setUint16(offset, value, true);
    const uint = (offset, value) => view.setUint32(offset, value, true);
    ascii(0, "RIFF"); uint(4, buffer.byteLength - 8); ascii(8, "WAVE");
    ascii(12, "fmt "); uint(16, 16); word(20, 1); word(22, 1);
    uint(24, audio.sampleRate); uint(28, 2 * audio.sampleRate); word(32, 2); word(34, 16);
    ascii(36, "data"); uint(40, 2 * count);
    let peak = 0;
    for (let i = 0; i < count; i++) {
        const sample = Math.max(-1, Math.min(1, mono[i]));
        const pcm = Math.round(sample * (sample < 0 ? 32768 : 32767));
        peak = Math.max(peak, Math.abs(pcm));
        view.setInt16(44 + i * 2, pcm, true);
    }
    if (checkSignal && peak < 16) throw new Error(virtual ?
        "当前虚拟麦克风几乎没有声音，请在「麦克风」中选择实际录音设备。" :
        "几乎没有录到声音，请检查麦克风静音开关，或换一个输入设备。");
    return new Blob([buffer], {type: "audio/wav"});
}

export class VoiceInput {
    #active = null;
    #epoch = 0;
    #starting = false;

    constructor({baseUrl = "", onLimit = () => {}, onLevel = () => {}, deviceId = "", automatic = true} = {}) {
        this.baseUrl = baseUrl.replace(/\/$/, "");
        this.onLimit = onLimit;
        this.onLevel = onLevel;
        this.deviceId = deviceId;
        this.automatic = automatic;
        this.label = "";
    }

    async #openMic() {
        if (!window.isSecureContext) {
            throw new Error("请在 O6 本机或建立 SSH 端口转发后，打开 http://127.0.0.1:8876 使用麦克风。");
        }
        if (!navigator.mediaDevices?.getUserMedia || typeof MediaRecorder === "undefined") {
            throw new Error("当前浏览器不支持录音，请用支持麦克风录音的浏览器打开工作台。");
        }
        const audio = {channelCount: {ideal:2}, echoCancellation: true, noiseSuppression: true, autoGainControl: true};
        if (this.deviceId) {
            try {
                return await navigator.mediaDevices.getUserMedia({audio: {...audio, deviceId: {exact: this.deviceId}}});
            } catch (error) {
                if (!["NotFoundError", "OverconstrainedError"].includes(error.name)) throw error;
                this.deviceId = "";
                this.automatic = true;
            }
        }
        const stream = await navigator.mediaDevices.getUserMedia({audio});
        const current = stream.getAudioTracks()[0];
        if (this.automatic) {
            const inputs = await navigator.mediaDevices.enumerateDevices().catch(() => []);
            const fallback = inputs.find(d => d.kind === "audioinput" && d.deviceId === "default");
            if (!virtualMic.test(current?.label || "") && !virtualMic.test(fallback?.label || "")) return stream;
            const array = inputs.find(d => d.kind === "audioinput" && d.deviceId &&
                !["default","communications"].includes(d.deviceId) && arrayMic.test(d.label) && !virtualMic.test(d.label));
            if (array) {
                try {
                    const selected = await navigator.mediaDevices.getUserMedia({audio: {...audio, deviceId:{exact:array.deviceId}}});
                    stream.getTracks().forEach(t => t.stop());
                    return selected;
                } catch { /* The array may have disconnected. Keep the open microphone. */ }
            }
        }
        return stream;
    }

    async requestPermission() {
        const stream = await this.#openMic();
        const label = stream.getAudioTracks()[0]?.label || "";
        try {
            return {label, devices: (await navigator.mediaDevices.enumerateDevices())
                .filter(device => device.kind === "audioinput")};
        } finally {
            stream.getTracks().forEach(track => track.stop());
        }
    }

    async start() {
        if (this.#active || this.#starting) return false;
        const epoch = ++this.#epoch;
        this.#starting = true;
        let stream, closeMeter;
        try {
            stream = await this.#openMic();
            if (epoch !== this.#epoch) {
                stream.getTracks().forEach(track => track.stop());
                return false;
            }
            const track = stream.getAudioTracks()[0];
            if (!track || track.readyState === "ended") throw new Error("麦克风已断开，请重新选择输入设备。");
            this.label = track.label || "当前麦克风";
            this.virtual = virtualMic.test(this.label);
            const recorder = new MediaRecorder(stream);
            const chunks = [];
            let recordingError;
            recorder.ondataavailable = event => { if (event.data.size) chunks.push(event.data); };
            const done = new Promise(resolve => {
                recorder.onerror = event => { recordingError = event.error || new Error("录音中断。"); };
                recorder.onstop = () => resolve(new Blob(chunks, {type: recorder.mimeType}));
            });
            closeMeter = meterInput(stream, this.onLevel);
            recorder.start();
            const end = () => {
                if (this.#active?.recorder !== recorder) return;
                recordingError = new Error("麦克风已断开，请选好输入设备后重试。");
                this.onLimit();
            };
            track.addEventListener("ended", end, {once: true});
            const timer = setTimeout(() => {
                if (recorder.state !== "inactive") recorder.stop();
                this.onLimit();
            }, MAX_RECORDING_MS);
            this.#active = {stream, recorder, done, timer, closeMeter, error: () => recordingError};
            return true;
        } catch (error) {
            closeMeter?.();
            stream?.getTracks().forEach(track => track.stop());
            throw error;
        } finally {
            this.#starting = false;
        }
    }

    cancel() {
        this.#epoch++;
        const active = this.#active;
        this.#active = null;
        if (!active) return;
        clearTimeout(active.timer);
        active.closeMeter();
        if (active.recorder.state !== "inactive") active.recorder.stop();
        active.stream.getTracks().forEach(track => track.stop());
    }

    async stop() {
        this.#epoch++;
        const active = this.#active;
        this.#active = null;
        if (!active) return null;
        clearTimeout(active.timer);
        active.closeMeter();
        try {
            if (active.recorder.state !== "inactive") active.recorder.stop();
            const media = await active.done;
            const error = active.error();
            if (error) throw error;
            if (active.stream.getAudioTracks().some(t => t.readyState === "ended")) {
                throw new Error("麦克风已断开，本段录音不完整。请重新选择输入设备。");
            }
            if (!media.size) throw new Error("录音为空，请重新说一句。");
            const decoder = new OfflineAudioContext(1, 1, 16000);
            const audio = await decoder.decodeAudioData(await media.arrayBuffer());
            if (audio.duration < 0.15) throw new Error("录音太短，请等到显示「请说出所需器具」后再说。");
            return encodeMonoWav(audio, true, this.virtual);
        } finally {
            active.stream.getTracks().forEach(track => track.stop());
        }
    }

    async recognize(wav) {
        const response = await fetch(this.baseUrl + "/api/voice", {
            method: "POST", headers: {"Content-Type": "audio/wav"}, body: wav,
            signal: AbortSignal.timeout(12000), cache: "no-store"
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.message || ("O6 返回 HTTP " + response.status));
        return result;
    }

    async stopAndRecognize() {
        const wav = await this.stop();
        return wav ? this.recognize(wav) : null;
    }
}
