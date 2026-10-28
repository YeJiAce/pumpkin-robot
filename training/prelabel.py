#!/usr/bin/env python3
"""预标注:用旧版权重(pumpkin_v2)给新录视频帧自动打 YOLO 标签,并画预览图供人工复核。

标签文件是标准 YOLO txt(不含 conf 列,可直接进训练);预览图画了框+置信度,
置信度故意放低到 0.15:宁可多标几个错的让人删,不让人肉从零补漏框。

用法: python3 prelabel.py <clip目录>   # 目录里要有 images/,标签进 labels/,预览进 vis/
"""
import sys
from pathlib import Path

import cv2
from ultralytics import YOLO

CONF = 0.15
WEIGHTS = "/home/yjy/pumpkin_robot/runs/pumpkin_v2/weights/best.pt"


def main():
    clip = Path(sys.argv[1])
    img_dir, lab_dir, vis_dir = clip / "images", clip / "labels", clip / "vis"
    lab_dir.mkdir(exist_ok=True)
    vis_dir.mkdir(exist_ok=True)

    model = YOLO(WEIGHTS)
    stats, confs, empties = {"person": 0, "feet": 0}, [], []
    for img in sorted(img_dir.glob("*.jpg")):
        r = model(str(img), verbose=False, conf=CONF, imgsz=640)[0]
        lines = []
        for b in r.boxes:
            cls = int(b.cls)
            x, y, w, h = b.xywhn[0].tolist()
            lines.append(f"{cls} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")
            stats[r.names[cls]] += 1
            confs.append(float(b.conf))
        (lab_dir / f"{img.stem}.txt").write_text("".join(l + "\n" for l in lines))
        cv2.imwrite(str(vis_dir / img.name), r.plot())   # 框+类别+置信度
        if not lines:
            empties.append(img.name)

    n = len(list(img_dir.glob("*.jpg")))
    lo = min(confs) if confs else 0.0
    hi = max(confs) if confs else 0.0
    avg = sum(confs) / len(confs) if confs else 0.0
    print(f"{n} 帧 | person 框 {stats['person']} 个,feet 框 {stats['feet']} 个"
          f" | conf min/avg/max = {lo:.2f}/{avg:.2f}/{hi:.2f}")
    print(f"空标签(模型一个框都没出,需人工看): {len(empties)} 帧"
          + (f" → {', '.join(empties[:8])}{'...' if len(empties) > 8 else ''}" if empties else ""))


if __name__ == "__main__":
    main()
