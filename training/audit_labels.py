#!/usr/bin/env python3
"""标签自动审计:COCO 预训练 yolov8s 找出 pumpkin_v2 漏标的人,补进标签。

背景:预标注漏检的坐姿人,标签里是空的——直接训练等于教模型"坐着的人不算人"。
COCO 检出 person(conf>=0.45)且与现有任意框 IoU<0.4(不是重复/不是已有 feet 的全身框)
→ 补为 person(class 0)。原标签备份在 labels_pre_audit/,可整体回滚。
vis 全部重画:原 person 绿框、原 feet 橙框、新补的蓝框标 ADD+置信度。
"""
import shutil
from pathlib import Path

import cv2
from ultralytics import YOLO

CONF, IOU_GATE = 0.45, 0.4
COCO_W = "/home/yjy/pumpkin_robot/dataset_raw/yolov8s.pt"   # 不存在会自动下载
CLIP = Path("/home/yjy/pumpkin_robot/dataset_raw/approach_01")
COLORS = {0: (0, 255, 0), 1: (0, 165, 255)}                  # person绿 feet橙
ADD_COLOR = (255, 0, 0)                                      # 补的蓝


def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    iw, ih = min(ax2, bx2) - max(ax1, bx1), min(ay2, by2) - max(ay1, by1)
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    return inter / ((ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter)


def main():
    img_dir, lab_dir, vis_dir = CLIP / "images", CLIP / "labels", CLIP / "vis"
    bak = CLIP / "labels_pre_audit"
    if not bak.exists():
        shutil.copytree(lab_dir, bak)
        print(f"原标签已备份 → {bak}")

    coco = YOLO(COCO_W)
    names = {0: "person", 1: "feet"}
    n_add, touched = 0, []
    for img in sorted(img_dir.glob("*.jpg")):
        im = cv2.imread(str(img))
        H, W = im.shape[:2]
        lab_file = lab_dir / f"{img.stem}.txt"
        rows = [l.split() for l in lab_file.read_text().splitlines() if l.strip()]
        boxes = [(int(r[0]), ((float(r[1]) - float(r[3]) / 2) * W,
                              (float(r[2]) - float(r[4]) / 2) * H,
                              (float(r[1]) + float(r[3]) / 2) * W,
                              (float(r[2]) + float(r[4]) / 2) * H)) for r in rows]

        r = coco(str(img), verbose=False, conf=CONF, classes=[0], imgsz=640)[0]
        added = []
        for b in r.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            if all(iou((x1, y1, x2, y2), bx) < IOU_GATE for _, bx in boxes):
                cx, cy, w, h = (x1 + x2) / 2 / W, (y1 + y2) / 2 / H, (x2 - x1) / W, (y2 - y1) / H
                boxes.append((0, (x1, y1, x2, y2)))
                rows.append(["0", f"{cx:.6f}", f"{cy:.6f}", f"{w:.6f}", f"{h:.6f}"])
                added.append((x1, y1, x2, y2, float(b.conf)))
        if added:
            lab_file.write_text("".join(" ".join(r_) + "\n" for r_ in rows))
            n_add += len(added)
            touched.append(f"{img.stem}(+{len(added)})")

        # 重画 vis
        for c, (x1, y1, x2, y2) in boxes:
            cv2.rectangle(im, (int(x1), int(y1)), (int(x2), int(y2)), COLORS[c], 2)
            cv2.putText(im, names[c], (int(x1), int(y1) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[c], 1)
        for x1, y1, x2, y2, cf in added:
            cv2.rectangle(im, (int(x1), int(y1)), (int(x2), int(y2)), ADD_COLOR, 2)
            cv2.putText(im, f"ADD {cf:.2f}", (int(x1), int(y2) + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, ADD_COLOR, 1)
        cv2.imwrite(str(vis_dir / img.name), im)

    print(f"共补 {n_add} 个 person 框,涉及 {len(touched)} 帧:")
    print(" ".join(touched))


if __name__ == "__main__":
    main()
