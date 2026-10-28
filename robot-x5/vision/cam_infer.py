#!/usr/bin/env python3
"""X5 BPU 摄像头实时推理: 摄像头 → 逐帧 BPU + 6 输出后处理 → 画框 → HTTP MJPEG 视频流。

在 PC 浏览器打开 http://<X5的IP>:8080 实时看画框效果, 终端同时打 FPS。
用法: python3 cam_infer.py [conf] [port]     Ctrl-C 退出
"""
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from postprocess6 import decode

BIN = os.path.expanduser('~/pumpkin_vision/pumpkin_v2.bin')  # 家目录,重启不丢
SIZE = 640
PAD = 114
NAMES = ['person', 'feet']
COLORS = [(0, 0, 255), (0, 255, 0)]   # BGR 红=person 绿=feet

latest = None                 # 最新的 JPEG 字节, HTTP 线程读
lock = threading.Lock()


def letterbox(img, new=SIZE):
    h, w = img.shape[:2]
    r = min(new / h, new / w)
    nu = (int(round(w * r)), int(round(h * r)))
    rs = cv2.resize(img, nu, interpolation=cv2.INTER_LINEAR)
    dw, dh = (new - nu[0]) / 2, (new - nu[1]) / 2
    t, b = int(round(dh - 0.1)), int(round(dh + 0.1))
    l, r = int(round(dw - 0.1)), int(round(dw + 0.1))
    lb = cv2.copyMakeBorder(rs, t, b, l, r, cv2.BORDER_CONSTANT, value=(PAD,) * 3)
    return lb


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ('/', '/stream'):
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            with lock:
                buf = latest
            if buf is None:
                time.sleep(0.05)
                continue
            try:
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                 + str(len(buf)).encode() + b'\r\n\r\n' + buf + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                return   # 浏览器断开
            time.sleep(1 / 30)

    def log_message(self, *a):
        pass


def main():
    conf = float(sys.argv[1]) if len(sys.argv) > 1 else 0.25
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8080

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        print('打不开摄像头 /dev/video0')
        sys.exit(1)

    m = dnn.load(BIN)[0]

    srv = ThreadingHTTPServer(('0.0.0.0', port), StreamHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f'模型加载OK, 视频流: http://<X5的IP>:{port}  (conf={conf})  Ctrl-C 退出', flush=True)

    n = n_det = 0
    t0 = time.time()
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.1)
            continue
        lb = letterbox(frame)
        rgb = cv2.cvtColor(lb, cv2.COLOR_BGR2RGB).astype(np.uint8)
        data = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.uint8)
        outs = m.forward([data])
        npo = [np.frombuffer(o.buffer, dtype=np.float32) for o in outs]
        boxes = decode(npo, conf=conf)
        vis = lb.copy()
        for x1, y1, x2, y2, s, c in boxes:
            cv2.rectangle(vis, (int(x1), int(y1)), (int(x2), int(y2)), COLORS[c], 2)
            cv2.putText(vis, f'{NAMES[c]}{s:.2f}', (int(x1), max(int(y1) - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[c], 2)
        ok2, enc = cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok2:
            with lock:
                global latest
                latest = enc.tobytes()
        n += 1
        n_det += len(boxes)
        if n % 25 == 0:
            el = time.time() - t0
            print(f'  {n}帧  {n/el:.1f} FPS  每帧{((n_det)/n):.1f}框', flush=True)


if __name__ == '__main__':
    main()
