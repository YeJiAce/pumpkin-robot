#!/usr/bin/env python3
"""本地 TTS 基准(X5):melo / kokoro 两个候选,测加载+合成耗时,对比 edge-tts 云端 ~3.0s。
顺带导出样本 wav 供人耳听音色质量(合成速度再快,难听也不能用)。
"""
import glob
import os
import time
import wave

import numpy as np
import sherpa_onnx

ROOT = os.path.expanduser("~/tts_local")
OUT = os.path.expanduser("~/tts_local/samples")
os.makedirs(OUT, exist_ok=True)

TEXTS = [
    ("开场白12字", "看好啦，我要跳啦！"),
    ("典型回复46字", "我今天心情特别好呀！刚才还转了个圈，有点小晕。你想让我干什么呀，我随时都可以陪你的！"),
]


def save(name, samples, sr):
    pcm = (np.clip(samples, -1, 1) * 32767).astype(np.int16)
    with wave.open(os.path.join(OUT, name), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(pcm.tobytes())


def run(tag, tts, sid, sids_extra=()):
    for label, text in TEXTS:
        for i in (1, 2):
            t0 = time.perf_counter()
            a = tts.generate(text, sid=sid, speed=1.0)
            dt = time.perf_counter() - t0
            dur = len(a.samples) / a.sample_rate
            print(f"{tag} | {label} #{i}: 合成 {dt:.2f}s → 音频 {dur:.1f}s (RTF {dt/dur:.2f})", flush=True)
            if i == 1:
                save(f"{tag}_{label[:3]}.wav", a.samples, a.sample_rate)
    for s in sids_extra:                      # 只出样本不计时,供挑音色
        a = tts.generate(TEXTS[0][1], sid=s, speed=1.0)
        save(f"{tag}_sid{s}.wav", a.samples, a.sample_rate)


import sys
ONLY = sys.argv[1] if len(sys.argv) > 1 else ""

if ONLY != "kokoro":
    print("===== melo-tts-zh_en =====", flush=True)
    t0 = time.perf_counter()
    melo = sherpa_onnx.OfflineTts(sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(
            vits=sherpa_onnx.OfflineTtsVitsModelConfig(
                model=f"{ROOT}/vits-melo-tts-zh_en/model.onnx",
                lexicon=f"{ROOT}/vits-melo-tts-zh_en/lexicon.txt",
                tokens=f"{ROOT}/vits-melo-tts-zh_en/tokens.txt",
                dict_dir=f"{ROOT}/vits-melo-tts-zh_en/dict"),
            num_threads=4)))
    print(f"加载 {time.perf_counter()-t0:.1f}s", flush=True)
    run("melo", melo, sid=0)

print("===== kokoro-multi-lang v1_1 =====", flush=True)
kok = f"{ROOT}/kokoro-int8-multi-lang-v1_1"
model = (glob.glob(f"{kok}/model.int8.onnx") or glob.glob(f"{kok}/model.onnx"))[0]
print(f"权重: {os.path.basename(model)}", flush=True)
t0 = time.perf_counter()
tts = sherpa_onnx.OfflineTts(sherpa_onnx.OfflineTtsConfig(
    model=sherpa_onnx.OfflineTtsModelConfig(
        kokoro=sherpa_onnx.OfflineTtsKokoroModelConfig(
            model=model, voices=f"{kok}/voices.bin", tokens=f"{kok}/tokens.txt",
            data_dir=f"{kok}/espeak-ng-data", lexicon=f"{kok}/lexicon-zh.txt",
            dict_dir=f"{kok}/dict"),
        num_threads=4),
    rule_fsts=f"{kok}/number-zh.fst,{kok}/phone-zh.fst,{kok}/date-zh.fst"))
print(f"加载 {time.perf_counter()-t0:.1f}s", flush=True)
run("kokoro", tts, sid=1, sids_extra=(35, 36, 37, 38, 39))   # 35~39 = 中文男女声

print(f"样本已存 {OUT}/,拖下来听听音色", flush=True)
