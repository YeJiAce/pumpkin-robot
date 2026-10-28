#!/usr/bin/env python3
"""拆解"大脑 4.5s":直连网关,逐步加料,看每样东西贵多少。
1 裸调用                    2 +完整人设(system)
3 +人设+10轮历史            4 +人设+20轮历史
模型=deepseek-flash + thinking disabled(当前大脑配置)。key 不落盘不打印。
"""
import json
import time
import urllib.request

BASE = "https://api.lan.x-lab.cc/v1/chat/completions"
cfg = json.load(open("/home/sunrise/astrbot_data/cmd_config.json", encoding="utf-8-sig"))
k = cfg["provider_sources"][0]["key"]
KEY = k[0] if isinstance(k, list) else k

persona = open("/home/sunrise/persona_for_bench.txt", encoding="utf-8").read().strip()
Q = "你今天心情怎么样？"


def history(n):
    msgs = []
    for i in range(n // 2):
        msgs.append({"role": "user", "content": f"第{i+1}轮:{Q}"})
        msgs.append({"role": "assistant", "content": "挺好的呀,今天精神满满,你想干什么呀?"})
    return msgs


def call(name, msgs):
    body = {"model": "deepseek-flash", "max_tokens": 200,
            "thinking": {"type": "disabled"}, "messages": msgs}
    req = urllib.request.Request(BASE, data=json.dumps(body).encode(),
                                 headers={"Authorization": "Bearer " + KEY,
                                          "Content-Type": "application/json"})
    t0 = time.time()
    with urllib.request.urlopen(req, timeout=120) as r:
        d = json.loads(r.read())
    dt = time.time() - t0
    u = d.get("usage", {})
    txt = (d["choices"][0]["message"].get("content") or "").strip()
    print(f"{name}: {dt:4.1f}s  in={u.get('prompt_tokens')}tok out={u.get('completion_tokens')}tok  {txt[:24]!r}")


for rnd in (1, 2):
    print(f"--- 第{rnd}遍 ---")
    call("1 裸调用         ", [{"role": "user", "content": Q}])
    call("2 +人设          ", [{"role": "system", "content": persona},
                               {"role": "user", "content": Q}])
    call("3 +人设+10轮历史 ", [{"role": "system", "content": persona}] + history(20)
         + [{"role": "user", "content": Q}])
    call("4 +人设+20轮历史 ", [{"role": "system", "content": persona}] + history(40)
         + [{"role": "user", "content": Q}])
