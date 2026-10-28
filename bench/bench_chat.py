#!/usr/bin/env python3
"""聊天延迟基准(在 X5 上跑):对真实链路逐环掐表。

A. ASR 环:用南瓜同款音色(edge-tts 晓伊+30Hz)合成问题语音 → 喂 paraformer 计时
   (模拟"用户说完一句话",顺带验证 TTS→ASR 回环识别质量)
B. 大脑环 + 出声环:POST /chat {"text":...} → 记 HTTP 回复耗时;
   同时盯 aplay 进程,记"从提问到音箱真出声"的耗时(用户感知的反映延迟)
C. TTS 分解环:对当轮回复文本分别计 edge-tts 云端合成 / ffmpeg 转 wav 耗时

会真的让南瓜开口说话(走 /say 队列)。跑 3 轮,逐轮+汇总打印。
"""
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request

import numpy as np
import sherpa_onnx

HOME = os.path.expanduser("~")
ASR_DIR = os.path.join(HOME, "magicbox", "sherpa-onnx-paraformer-zh-2023-09-14")
VOICE, PITCH = "zh-CN-XiaoyiNeural", "+30Hz"     # 同 pumpkin_voice
AGENT = "http://127.0.0.1:6186/chat"
TMP = "/tmp/bench_q"
QUESTIONS = ["你今天心情怎么样？", "给我讲一个很短的笑话", "你最喜欢吃什么呀？"]


def sh(cmd, timeout=60):
    subprocess.run(cmd, check=True, timeout=timeout,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class AplayWatch:
    """30ms 轮询 aplay 进程,记首次出现时刻(monotonic)。"""

    def __init__(self):
        self.first = None
        self._stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while not self._stop.is_set():
            if self.first is None and subprocess.run(
                    ["pgrep", "-x", "aplay"], capture_output=True).returncode == 0:
                self.first = time.monotonic()
            time.sleep(0.03)

    def reset(self):
        self.first = None

    def wait(self, deadline):
        while time.monotonic() < deadline and self.first is None:
            time.sleep(0.05)
        return self.first

    def wait_gone(self, deadline):
        """等 aplay 结束(轮与轮之间别把上一轮的尾巴算进来)。"""
        while time.monotonic() < deadline:
            if subprocess.run(["pgrep", "-x", "aplay"],
                              capture_output=True).returncode != 0:
                return True
            time.sleep(0.2)
        return False


def synth_question(text):
    """合成问题语音 → 返回 f32 采样(24k)。"""
    sh([sys.executable, "-m", "edge_tts", "--voice", VOICE, "--pitch", PITCH,
        "--text", text, "--write-media", TMP + ".mp3"])
    sh(["ffmpeg", "-y", "-loglevel", "quiet", "-i", TMP + ".mp3", TMP + ".wav"])
    raw = subprocess.run(["ffmpeg", "-loglevel", "quiet", "-i", TMP + ".wav",
                          "-f", "f32le", "-ac", "1", "-ar", "24000", "-"],
                         capture_output=True, check=True).stdout
    return np.frombuffer(raw, dtype=np.float32)


def main():
    print("载入 ASR 模型(int8 paraformer)...", flush=True)
    asr = sherpa_onnx.OfflineRecognizer.from_paraformer(
        paraformer=os.path.join(ASR_DIR, "model.int8.onnx"),
        tokens=os.path.join(ASR_DIR, "tokens.txt"), num_threads=2, provider="cpu")
    watch = AplayWatch()
    rows = []

    for rnd, q in enumerate(QUESTIONS, 1):
        print(f"\n===== 第 {rnd} 轮:{q} =====", flush=True)
        samples = synth_question(q)
        dur = len(samples) / 24000

        # A. ASR 计时
        t0 = time.perf_counter()
        st = asr.create_stream()
        st.accept_waveform(24000, samples)
        asr.decode_stream(st)
        t_asr = time.perf_counter() - t0
        heard = st.result.text.strip()
        print(f"A ASR: {t_asr*1000:.0f} ms(语音 {dur:.1f}s) → 「{heard}」", flush=True)

        # B. 大脑 + 出声
        payload = json.dumps({"text": heard or q, "timeout": 60}).encode()
        req = urllib.request.Request(AGENT, data=payload,
                                     headers={"Content-Type": "application/json"},
                                     method="POST")
        watch.reset()
        t0 = time.perf_counter()
        with urllib.request.urlopen(req, timeout=80) as r:
            resp = json.loads(r.read().decode())
        t_brain = time.perf_counter() - t0
        reply = str(resp.get("reply") or "").strip()
        t_out = None
        if watch.first is not None:
            t_out = watch.first - t0
        else:
            f = watch.wait(time.monotonic() + 25)      # HTTP 回来但还没出声,再等等
            t_out = f - t0 if f else None
        print(f"B 大脑HTTP: {t_brain:.2f}s | 出声: "
              f"{t_out:.2f}s" if t_out else "B 大脑HTTP: %.2fs | 出声: 25s内没等到" % t_brain,
              flush=True)
        print(f"  回复({len(reply)}字): {reply[:40]}{'…' if len(reply) > 40 else ''}", flush=True)

        # C. TTS 分解(不播放,免得和监听混)
        t0 = time.perf_counter()
        sh([sys.executable, "-m", "edge_tts", "--voice", VOICE, "--pitch", PITCH,
            "--text", reply, "--write-media", TMP + "c.mp3"], timeout=30)
        t_synth = time.perf_counter() - t0
        t0 = time.perf_counter()
        sh(["ffmpeg", "-y", "-loglevel", "quiet", "-i", TMP + "c.mp3", TMP + "c.wav"], timeout=10)
        t_ff = time.perf_counter() - t0
        print(f"C TTS合成: {t_synth:.2f}s | ffmpeg: {t_ff*1000:.0f} ms", flush=True)

        rows.append((t_asr, t_brain, t_out, t_synth, t_ff))
        watch.wait_gone(time.monotonic() + 30)          # 等这轮播完再下一轮

    print("\n===== 汇总(每轮一行:ASR / 大脑 / 出声 / TTS合成 / ffmpeg,单位秒)=====")
    for i, (a, b, o, s, f) in enumerate(rows, 1):
        print(f"  轮{i}: {a:.2f} / {b:.2f} / {o if o else -1:.2f} / {s:.2f} / {f:.3f}")
    med = lambda k: sorted(k)[len(k) // 2]
    outs = [o for *_, o, _, _ in rows if o]
    print(f"中位: ASR {med([r[0] for r in rows]):.2f}s | 大脑 {med([r[1] for r in rows]):.2f}s | "
          f"出声 {med(outs) if outs else -1:.2f}s | TTS {med([r[3] for r in rows]):.2f}s")
    print("(另:链路里还有固定的 静音判停1.0s + 唤醒应答,不随文本变化,不在上表)")


if __name__ == "__main__":
    main()
