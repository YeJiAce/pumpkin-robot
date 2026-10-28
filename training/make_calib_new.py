#!/usr/bin/env python3
"""把新场景视频帧做成 hb_mapper 校准 bin:letterbox 640(114灰填充) → RGB → uint8 → CHW。

排布与既有 cal_*.bin 一致(实测平滑度验证: CHW 平面排布, 对应 NCHW 输入)。
用法: python3 make_calib_new.py [图片目录] [输出前缀] [步长]
"""
import sys
from pathlib import Path

import cv2
import numpy as np

SIZE, FILL = 640, 114


def letterbox_rgb_chw(img):
    h, w = img.shape[:2]
    r = min(SIZE / h, SIZE / w)
    nw, nh = round(w * r), round(h * r)
    im = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((SIZE, SIZE, 3), FILL, dtype=np.uint8)
    top, left = (SIZE - nh) // 2, (SIZE - nw) // 2
    canvas[top:top + nh, left:left + nw] = im
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
    return rgb.transpose(2, 0, 1).copy()          # HWC → CHW


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/home/yjy/pumpkin_robot/dataset_raw/approach_01/images")
    prefix = sys.argv[2] if len(sys.argv) > 2 else "cal_new"
    step = int(sys.argv[3]) if len(sys.argv) > 3 else 2
    out_dir = Path("/home/yjy/pumpkin_robot/rdk_convert/calib")
    frames = sorted(src.glob("*.jpg"))[::step]
    for i, f in enumerate(frames):
        (out_dir / f"{prefix}_{i:04d}.bin").write_bytes(letterbox_rgb_chw(cv2.imread(str(f))).tobytes())
    print(f"{len(frames)} 张 → {out_dir}/{prefix}_*.bin")


if __name__ == "__main__":
    main()
