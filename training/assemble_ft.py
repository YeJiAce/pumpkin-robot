#!/usr/bin/env python3
"""装配微调训练集:
- ft_train.txt  = 原 train 全部 64115 张 + 新 105 张×20 过采样(绝对路径清单)
- val_new.txt   = 新数据均匀留 15 张(独立验证集,评"真实目标场景")
- data_ft.yaml / data_valnew.yaml 供 ultralytics 用;原 val 2693 张原样不动(指标可比)
"""
from pathlib import Path

ROOT = Path("/home/yjy/pumpkin_robot")
NEW = ROOT / "dataset_raw/approach_01/images"
DERIVED = ROOT / "data/coco/derived"
OUT = ROOT / "dataset_raw"
OVERSAMPLE = 20

frames = sorted(NEW.glob("*.jpg"))
assert len(frames) == 120, f"帧数不对: {len(frames)}"
val_new = frames[7::8]                     # 每 8 张取 1 → 15 张,覆盖头中尾(含近距段)
val_set = set(val_new)
train_new = [f for f in frames if f not in val_set]
assert len(val_new) == 15 and len(train_new) == 105

old = [str(p) for p in sorted((DERIVED / "train/images").glob("*.jpg"))]
lines = old + [str(p) for p in train_new for _ in range(OVERSAMPLE)]
(OUT / "ft_train.txt").write_text("\n".join(lines) + "\n")
(OUT / "val_new.txt").write_text("\n".join(str(p) for p in val_new) + "\n")

names = "names: [person, feet]\nnc: 2\n"
(OUT / "data_ft.yaml").write_text(
    f"path: {DERIVED}\ntrain: {OUT}/ft_train.txt\nval: val/images\n{names}")
(OUT / "data_valnew.yaml").write_text(
    f"path: {DERIVED}\nval: {OUT}/val_new.txt\n{names}")

print(f"train: 原 {len(old)} + 新 {len(train_new)}×{OVERSAMPLE} = {len(lines)} 行")
print(f"val_new: {[p.name for p in val_new]}")
