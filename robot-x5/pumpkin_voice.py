#!/usr/bin/env python3
"""南瓜机器人 语音前端(听见) + AstrBot 大脑接入(1026 改):
唤醒词「南瓜」(单声,-cn/-en 只切应答语言,耳朵双语都懂) → 录指令 → ASR 转文字 → POST 给本机 AstrBot 插件(127.0.0.1:6186) → 收回复文本。
回复文本用 edge-tts(晓伊+24Hz"曼波"音色)播报。
运动不再从回复文本解析——模型经 function calling 调 pumpkin_move/stop/follow 工具,
工具打到本进程里的运动桥(:8123) → 硬限幅 → pumpkin_chassis 执行。
旧 Dify 版备份在 ~/pumpkin_voice.py.bak_dify。

架构:
  麦克风(arecord, 48k mono 原生) ─► sherpa-onnx KeywordSpotter(唤醒词=南瓜)
        │ 唤醒后
        ▼
  录一段(静音1s断尾,最长20s) ─► sherpa-onnx OfflineRecognizer(paraformer-zh) ─► 文字
        │
        ▼
  POST 文字给 AstrBot 插件(blocking;人设/上下文/工具调用都在那边)
        └► 每条回复经 /say 推回本进程排队 → 播放线程逐条 TTS 播报(一场可多条:预告+汇报)
  运动桥(HTTP:8123,被动):AstrBot 工具 → clamp_motion 限幅 → chassis.run_sequence / FOLLOW
          /move 等动作真做完才回 → 模型的收尾汇报落在动作之后

依赖: pip3 install --user sherpa-onnx   (numpy 已有; arecord 系统自带, 不需 portaudio)
模型在 ~/magicbox/。
"""
import atexit
import base64
import hashlib
import json
import os
import random
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
import sherpa_onnx

import pumpkin_chassis as chassis

try:                                    # 视觉跟随:缺模型/缺摄像头时自动降级,不影响语音
    from pumpkin_follow import follow as FOLLOW
except Exception:
    FOLLOW = None

HOME = os.path.expanduser("~")
MAGIC = os.path.join(HOME, "magicbox")
KWS_DIR = os.path.join(MAGIC, "sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01")
ASR_DIR = os.path.join(MAGIC, "sherpa-onnx-paraformer-zh-2023-09-14")

SR = 48000            # 采样率:USB 麦原生 48k。ALSA 强制 16k 会低质重采样毁音频;
                      # sherpa-onnx 的 accept_waveform 内部自带重采样到 16k,直接喂 48k
CHUNK = 4800          # 100ms / 帧
SILENCE_DUR = 1.0     # 静音多久认为一句话结束
MAX_REC = 20.0        # 一句话最多录多少秒
SILENCE_RMS = 0.012   # 低于这个 RMS 视为静音(按麦克风调)
MIC = "plughw:0,0"    # USB 麦(card 0, UACDemoV1.0)

AGENT_URL = "http://127.0.0.1:6186/chat"   # AstrBot 南瓜插件的语音入口
AGENT_TIMEOUT = 90    # 等大脑(含工具调用)的时限

BRIDGE_PORT = 8123    # 运动桥:AstrBot 工具调用打到这,限幅后下发底盘
BRIDGE_DRY_RUN = False  # 2026-10-26 用户授权真车测试。回退安全阀:改回 True

CAM_DEV = "/dev/video0"   # USB 摄像头(MJPG);/snap 抓帧给 pumpkin_look 看图用
SNAP_FILE = os.path.join(HOME, "pumpkin_snap_latest.jpg")   # 每帧都存这,同名覆盖,方便翻看
SNAP_LOCK = threading.Lock()   # 抓帧串行化,ffmpeg 同时开同一摄像头会抖

# 运动限幅(运动桥硬夹,不信任模型工具参数)—— 10-27 起只有闭环定量段,开环段一律丢弃
MAX_SEGS = 12                  # 段数上限(方形舞 4转+4走+home=9,给足余量)
TOTAL_MAX = 20.0               # 动作总时长上限(秒,按保守估时计)
TURN_DEG_MAX = 360.0           # 闭环转角上限(度/段,左转为正)
MOVE_M_MAX = 0.8               # 闭环里程上限(米/段,move前进为正/strafe左移为正)

# TTS(edge-tts 微软云;X5 直连可通,无需代理)
TTS_VOICE = "zh-CN-XiaoyiNeural"
TTS_PITCH = "+30Hz"     # "曼波"味儿:晓伊升调(edge-tts 只收 ±NNHz 格式)
WAKE_ACKS_CN = ["哎，你说", "我听着呢", "嘿嘿，干嘛呀", "在呢在呢",
                "唔？叫我吗", "咋啦咋啦", "嘿嘿，南瓜在的"]          # 中文应答池
WAKE_ACKS_EN = ["Yes?", "I'm listening~", "Hehe, what's up?",
                "I'm all ears!", "Go ahead~", "Hmm? What is it?",
                "Pumpkin's here!", "Tell me~"]                      # 英文应答池
# 启动参数选池子:python3 pumpkin_voice.py -en → 英文应答;默认/-cn → 中文
WAKE_ACKS = WAKE_ACKS_EN if "-en" in sys.argv[1:] else WAKE_ACKS_CN
ACK_DIR = os.path.join(HOME, "pumpkin_acks")                     # 本地缓存,开机预合成
TTS_TMP = "/tmp/pumpkin_tts"
TTS_TIMEOUT = 15        # 云端合成超时(秒);失败/超时只打字幕不挡流程


def make_kws():
    return sherpa_onnx.KeywordSpotter(
        tokens=os.path.join(KWS_DIR, "tokens.txt"),
        encoder=os.path.join(KWS_DIR, "encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx"),
        decoder=os.path.join(KWS_DIR, "decoder-epoch-12-avg-2-chunk-16-left-64.onnx"),
        joiner=os.path.join(KWS_DIR, "joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx"),
        keywords_file=os.path.join(MAGIC, "kws_keywords.txt"),   # 自定义:单声「南瓜」
        num_threads=2,
        provider="cpu",
        keywords_threshold=0.3,   # 单声词只有0.3能醒(0.5实测漏);代价=提到南瓜就会醒
    )


def make_asr():
    return sherpa_onnx.OfflineRecognizer.from_paraformer(
        paraformer=os.path.join(ASR_DIR, "model.int8.onnx"),
        tokens=os.path.join(ASR_DIR, "tokens.txt"),
        num_threads=2,
        provider="cpu",
    )


class Mic:
    """arecord 流式 48k mono S16_LE。restart() 用于丢弃处理期间积压的旧音频。"""

    def __init__(self):
        self._cmd = ["arecord", "-D", MIC, "-r", str(SR), "-f", "S16_LE", "-c", "1",
                     "-t", "raw", "-q"]
        self._proc = None
        self._start()
        atexit.register(self._stop)   # 退出(含Ctrl-C)时杀掉arecord,防止占住麦

    def _start(self):
        self._proc = subprocess.Popen(self._cmd, stdout=subprocess.PIPE,
                                      stderr=subprocess.DEVNULL)
        time.sleep(0.3)
        if self._proc.poll() is not None:
            raise RuntimeError("arecord 起不来,多半麦克风被占,查残留: pgrep -x arecord")

    def _stop(self):
        try:
            self._proc.kill()
            self._proc.wait()
        except Exception:
            pass

    def restart(self):
        self._stop()
        self._start()

    def read(self, n):
        raw = b""
        while len(raw) < n * 2:
            b = self._proc.stdout.read(n * 2 - len(raw))
            if not b:
                return None
            raw += b
        return np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0


class AgentChat:
    """调本机 AstrBot 插件语音入口(blocking)。人设/上下文/工具都在 AstrBot 侧管。"""

    def ask(self, text):
        payload = json.dumps({"text": text, "timeout": AGENT_TIMEOUT}).encode("utf-8")
        req = urllib.request.Request(
            AGENT_URL, data=payload,
            headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=AGENT_TIMEOUT + 10) as r:
            resp = json.loads(r.read().decode("utf-8"))
        if not resp.get("ok"):
            raise RuntimeError(str(resp.get("reply") or "大脑返回失败"))
        return str(resp.get("reply") or "").strip()


def strip_fences(t):
    """剥掉 ```json ... ``` 围栏。"""
    t = t.strip()
    if t.startswith("```"):
        lines = t.split("\n")
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip().startswith("```"):
            lines = lines[:-1]
        t = "\n".join(lines).strip()
    return t


def clamp_motion(motion):
    """主控强制限幅:只认闭环段——turn|angle|<=360°、move/strafe|dist|<=0.8m、home;
    开环段(vx/vy/wz/duration、spin)10-27 起废除,一律丢弃;
    <=12段,总估时<=20s(home 不占预算,闭环段不能截短)。"""
    if not isinstance(motion, dict):
        return {"type": "none", "motions": [], "drops": 0}

    def num(s, k, lim):
        try:
            v = float(s.get(k) or 0.0)
        except (TypeError, ValueError):
            v = 0.0
        return max(-lim, min(lim, v))

    mtype = motion.get("type", "none")
    if mtype in ("stop", "follow"):
        return {"type": mtype, "motions": [], "drops": 0}
    segs = motion.get("motions")
    if mtype != "sequence" or not isinstance(segs, list):
        return {"type": "none", "motions": [], "drops": 0}
    out, total, drops = [], 0.0, 0
    for s in segs:
        if not isinstance(s, dict):
            drops += 1
            continue
        mode = str(s.get("mode") or "").lower()
        if mode == "home":                       # 回原位段:不占预算,自身超时兜底
            out.append({"mode": "home"})
            continue
        if mode == "turn":                       # 闭环段估时(保守,约90°/s含加减速)
            dur = abs(num(s, "angle", TURN_DEG_MAX)) / 90.0
        elif mode in ("move", "strafe"):         # 估时(保守,0.4m/s)
            dur = abs(num(s, "dist", MOVE_M_MAX)) / 0.4
        else:                                    # 开环段已废除
            drops += 1
            continue
        if total + dur > TOTAL_MAX:
            break                                # 闭环段不能截短,预算不够就到此为止
        if mode == "turn":
            out.append({"mode": "turn", "angle": round(num(s, "angle", TURN_DEG_MAX), 1)})
        elif mode == "move":
            out.append({"mode": "move", "dist": round(num(s, "dist", MOVE_M_MAX), 2)})
        else:
            out.append({"mode": "strafe", "dist": round(num(s, "dist", MOVE_M_MAX), 2)})
        total += dur
        if len(out) >= MAX_SEGS or total >= TOTAL_MAX - 1e-6:
            break
    if not out:
        return {"type": "none", "motions": [], "drops": drops}
    return {"type": "sequence", "motions": out, "drops": drops}


def make_chat():
    chat = AgentChat()
    import socket
    try:
        with socket.create_connection(("127.0.0.1", 6186), timeout=2):
            print("[大脑] AstrBot 语音入口在线", flush=True)
            return chat
    except OSError:
        print("[大脑] 127.0.0.1:6186 不通(AstrBot/南瓜插件没跑?),本次只做唤醒+ASR", flush=True)
        return None


_TTS_LOCK = threading.Lock()   # voice 线程和 follow 线程都可能说话,排队防撞车
ACKS = []                      # 开机时 prepare_acks() 填充;断网时为空=不播回应

# ---- 播报队列:AstrBot 每条消息 POST /say 入队,播放线程逐条播 ----
SAY_QUEUE = []                 # 待播文本(顺序保证)
SAY_COND = threading.Condition()
SPEAKING = threading.Event()   # 播报中;主循环看到就丢弃麦音频,防自己唤醒自己
SAY_PUSHED = 0                 # 本场交互经 /say 推来了几条;think() 据此决定要不要兜底


def _say_worker():
    """播报队列消费者:逐条合成+播放。播报期间 SPEAKING 置位。"""
    while True:
        with SAY_COND:
            while not SAY_QUEUE:
                SAY_COND.wait()
            text = SAY_QUEUE.pop(0)
        SPEAKING.set()
        try:
            speak(text)
        except Exception as e:
            print(f"[说] 播放失败跳过({type(e).__name__})", flush=True)
        finally:
            with SAY_COND:
                if not SAY_QUEUE:
                    SPEAKING.clear()


def _wait_say_done(timeout=25):
    """等队列播完(唤醒瞬间可能还有尾句在播,别掐也别录进自己声音)。"""
    t0 = time.time()
    while time.time() - t0 < timeout:
        with SAY_COND:
            if not SAY_QUEUE and not SPEAKING.is_set():
                return
        time.sleep(0.1)


def prepare_acks():
    """开机把固定回应合成好存本地,唤醒时秒播(现场等云端要2秒,太慢)。"""
    os.makedirs(ACK_DIR, exist_ok=True)
    paths = []
    for txt in WAKE_ACKS:
        tag = hashlib.md5(f"{TTS_VOICE}|{TTS_PITCH}|{txt}".encode()).hexdigest()[:10]
        p = os.path.join(ACK_DIR, f"ack_{tag}.wav")
        if not os.path.exists(p):
            try:
                subprocess.run(
                    [sys.executable, "-m", "edge_tts", "--voice", TTS_VOICE,
                     "--pitch", TTS_PITCH, "--text", txt,
                     "--write-media", p + ".mp3"],
                    check=True, timeout=TTS_TIMEOUT,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                subprocess.run(["ffmpeg", "-y", "-loglevel", "quiet",
                                "-i", p + ".mp3", p], check=True, timeout=10)
                os.remove(p + ".mp3")
            except Exception as e:
                print(f"[回应] 预合成失败,本次唤醒不播回应({e})", flush=True)
                return []
        paths.append(p)
    return paths


def play_ack(mic):
    """播一句随机回应,同时把麦热好:播放期音频(自己声音+抢话)读出来丢掉,
    播放一结束麦已在监听,没有"说完听不见你"的盲区。"""
    if not ACKS:
        mic.restart()
        return
    try:
        p = subprocess.Popen(["aplay", "-D", "plughw:0,0", "-q", random.choice(ACKS)])
    except Exception:
        p = None
    mic.restart()                     # 边播边重开麦(等播完才开会有半秒盲区)
    if p is None:
        return
    t0 = time.time()
    while p.poll() is None and time.time() - t0 < 6:
        if mic.read(CHUNK) is None:   # 丢弃播放期音频
            break
    try:
        p.wait(timeout=5)
    except Exception:
        p.kill()


def speak(text):
    """edge-tts 合成 → ffmpeg 转 wav → aplay 播到 USB 音箱(card 0)。失败只提示。"""
    with _TTS_LOCK:
        _speak_locked(text)


def _speak_locked(text):
    try:
        subprocess.run(
            [sys.executable, "-m", "edge_tts", "--voice", TTS_VOICE,
             "--pitch", TTS_PITCH, "--text", text,
             "--write-media", TTS_TMP + ".mp3"],
            check=True, timeout=TTS_TIMEOUT,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "quiet",
                        "-i", TTS_TMP + ".mp3", TTS_TMP + ".wav"],
                       check=True, timeout=10)
        subprocess.run(["aplay", "-D", "plughw:0,0", "-q", TTS_TMP + ".wav"],
                       check=True, timeout=30)
    except Exception as e:
        print(f"[TTS] 跳过({type(e).__name__}: {e})", flush=True)


def think(chat, text):
    """问 AstrBot。回复不再走这里播:AstrBot 每条消息经 /say 推来,已由播放线程
    逐条播过;这里只在一条都没推过来(推送链路挂了)时用 HTTP 应答文本兜底。"""
    global SAY_PUSHED
    SAY_PUSHED = 0
    try:
        reply = chat.ask(text)
    except Exception as e:
        print(f"[大脑] 调用失败: {e}", flush=True)
        speak("哎呀,我大脑好像断线了")
        return False
    print(f"[小南瓜·首条] {reply}", flush=True)
    if SAY_PUSHED == 0:
        print("[小南瓜] 推送链路没通,本地兜底播报", flush=True)
        speak(reply)
    return True


# ---------------- 运动桥:AstrBot 工具调用(127.0.0.1:8123) → 限幅 → 底盘 ----------------

class BridgeHandler(BaseHTTPRequestHandler):
    """AstrBot 插件的南瓜工具打到这里。限幅在本进程强制执行,不信任模型参数。"""

    def _reply(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        try:
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass   # 对方(插件)等超时先挂了电话:动作照跑,结果没人听就算了

    def do_POST(self):
        try:
            n = int(self.headers.get("Content-Length") or 0)
            req = json.loads(self.rfile.read(n).decode("utf-8")) if n else {}
        except Exception:
            return self._reply(400, {"ok": False, "error": "请求体不是JSON"})

        if self.path == "/say":                    # AstrBot 每条消息推来入队,立刻回
            txt = str(req.get("text") or "").strip()
            if not txt:
                return self._reply(400, {"ok": False, "error": "text为空"})
            global SAY_PUSHED
            with SAY_COND:
                SAY_QUEUE.append(txt)
                SAY_COND.notify()
            SAY_PUSHED += 1
            print(f"[说] {txt[:60]}", flush=True)
            return self._reply(200, {"ok": True})
        if self.path == "/move":
            return self._reply(200, self._do_move(req))
        if self.path == "/stop":
            chassis.stop()                       # 急停永远真执行(dry-run 也停)
            if FOLLOW is not None and FOLLOW.active:
                FOLLOW.stop()
            print("[桥] stop(急停)", flush=True)
            return self._reply(200, {"ok": True, "echo": "已急停"})
        if self.path == "/follow":
            return self._reply(200, self._do_follow(req))
        if self.path == "/snap":                   # 抓一帧摄像头画面(给看图工具)
            return self._reply(200, self._do_snap())
        return self._reply(404, {"ok": False, "error": "未知路径"})

    def _do_snap(self):
        """ffmpeg 从摄像头抓一帧 MJPG 转 base64 返回;同时存一份固定文件方便人看。"""
        with SNAP_LOCK:
            try:
                subprocess.run(
                    ["ffmpeg", "-y", "-loglevel", "error", "-f", "v4l2",
                     "-input_format", "mjpeg", "-i", CAM_DEV,
                     "-frames:v", "1", "-q:v", "4", SNAP_FILE],
                    check=True, timeout=6,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                with open(SNAP_FILE, "rb") as f:
                    raw = f.read()
            except Exception as e:
                return {"ok": False, "error": f"抓帧失败({type(e).__name__})"}
        if not raw:
            return {"ok": False, "error": "抓到空帧"}
        print(f"[snap] 一帧 {len(raw)}B", flush=True)
        return {"ok": True, "b64": base64.b64encode(raw).decode(),
                "bytes": len(raw)}

    def _do_move(self, req):
        segs = req.get("segments")
        if isinstance(segs, str):                # 工具传的是JSON字符串,宽容解析
            t = strip_fences(segs)
            try:
                segs = json.loads(t)
            except Exception:
                i, j = t.find("["), t.rfind("]")
                try:
                    segs = json.loads(t[i:j + 1]) if 0 <= i < j else None
                except Exception:
                    segs = None
        motion = clamp_motion({"type": "sequence", "motions": segs})
        if motion["type"] != "sequence":
            return {"ok": False, "error": "segments 不是合法的动作段数组(只支持 turn/move/strafe/home)"}
        seg = " ".join(
            f"[转{m['angle']:+.0f}°]" if m.get("mode") == "turn"
            else f"[走{m['dist']:+.2f}m]" if m.get("mode") == "move"
            else f"[移{m['dist']:+.2f}m]" if m.get("mode") == "strafe"
            else "[回原位]"
            for m in motion["motions"])
        if motion.get("drops"):
            seg += f"(丢弃{motion['drops']}个无效开环段)"
        if BRIDGE_DRY_RUN:
            print(f"[桥·DRY] {seg}", flush=True)
            return {"ok": True, "echo": f"DRY-RUN 未执行: {seg}"}
        print(f"[桥] {seg}", flush=True)
        chassis.run_sequence(motion["motions"])   # 等动作真做完才回:模型的收尾汇报落在动作之后
        return {"ok": True, "echo": f"已执行: {seg}"}

    def _do_follow(self, req):
        act = str(req.get("action") or "").lower()
        if act == "start":
            if FOLLOW is None or not FOLLOW.available():
                return {"ok": False, "error": "跟随不可用(模型或摄像头缺失)"}
            if BRIDGE_DRY_RUN:
                return {"ok": True, "echo": "DRY-RUN:跟随请求已收到但未开启"}
            chassis.stop()                       # 先停掉手头动作,再交给跟随线程
            FOLLOW.start()
            return {"ok": True, "echo": "已开始跟随"}
        if act == "stop":
            if FOLLOW is not None:
                FOLLOW.stop()
            chassis.stop()
            return {"ok": True, "echo": "已停止跟随"}
        return {"ok": False, "error": "action 必须是 start 或 stop"}

    def log_message(self, *a):                   # 关掉默认访问日志,print 足够
        pass


def start_bridge():
    srv = ThreadingHTTPServer(("127.0.0.1", BRIDGE_PORT), BridgeHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    threading.Thread(target=_say_worker, daemon=True).start()
    return srv


def main():
    for d in (KWS_DIR, ASR_DIR):
        if not os.path.isdir(d):
            sys.exit(f"模型目录不存在: {d}")
    kws = make_kws()
    asr = make_asr()
    chat = make_chat()
    start_bridge()
    print(f"[桥] 运动桥 127.0.0.1:{BRIDGE_PORT} "
          f"({'DRY-RUN 只打印不动车' if BRIDGE_DRY_RUN else '真车模式!限幅生效'})", flush=True)
    if FOLLOW is not None and FOLLOW.available():
        FOLLOW.say = speak              # 跟随丢人时借嘴喊"人呢"
        print("[跟随] 就绪(说「跟着我」开启)", flush=True)
    else:
        print("[跟随] 不可用,语音照常", flush=True)
    mic = Mic()
    global ACKS
    ACKS = prepare_acks()
    print(f"小南瓜已就绪(应答:{'英文' if '-en' in sys.argv[1:] else '中文'})。"
          f"喊「南瓜」后说一句指令。Ctrl-C 退出。", flush=True)

    kws_stream = kws.create_stream()
    state = "wake"          # wake = 监听唤醒词; cmd = 录指令
    rec_buf = []
    silence = 0.0
    spoke = False

    while True:
        chunk = mic.read(CHUNK)
        if chunk is None:
            sys.exit("arecord 结束/异常")
        if state == "wake":
            if SPEAKING.is_set():        # 自己正在播报,丢掉音频不喂唤醒检测(防自唤醒)
                continue
            kws_stream.accept_waveform(SR, chunk)
            while kws.is_ready(kws_stream):
                kws.decode_stream(kws_stream)
            r = kws.get_result(kws_stream)
            if r:
                print(f"[唤醒] {r}  -> 请说指令…", flush=True)
                kws.reset_stream(kws_stream)
                _wait_say_done()    # 上一场可能还有尾句在播,播完再进录音
                play_ack(mic)       # 播回应;边播边热麦,播完立即可说指令
                state = "cmd"
                rec_buf = []
                silence = 0.0
                spoke = False
        else:  # cmd
            rec_buf.append(chunk)
            rms = float(np.sqrt(np.mean(chunk * chunk)))
            if rms > SILENCE_RMS:
                spoke = True
                silence = 0.0
            else:
                silence += len(chunk) / SR
            dur = sum(len(c) for c in rec_buf) / SR
            if not ((spoke and silence >= SILENCE_DUR) or dur >= MAX_REC):
                continue
            # —— 跑 ASR ——
            audio = np.concatenate(rec_buf)
            st = asr.create_stream()
            st.accept_waveform(SR, audio)
            asr.decode_stream(st)
            text = st.result.text.strip()
            print(f"[指令] {text}", flush=True)
            if text and chat:
                think(chat, text)
            # ASR/LLM 处理期间麦里积压了旧音频,重开麦丢弃,避免误唤醒
            mic.restart()
            state = "wake"
            rec_buf = []
            silence = 0.0
            spoke = False


if __name__ == "__main__":
    import signal
    # 断开连接(SIGHUP)/被kill(SIGTERM)也走 atexit,防止 arecord 变孤儿占麦
    for _sig in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(_sig, lambda *_: sys.exit(1))
    try:
        main()
    except KeyboardInterrupt:
        print("\n再见~")
        sys.exit(0)
