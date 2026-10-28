#!/usr/bin/env python3
"""YOLOv8 6 输出后处理(BPU 友好导出的配套): DFL 解码 + sigmoid + grid 解码 + NMS。

输入 outs: list/tuple, 顺序 = [reg0, cls0, reg1, cls1, reg2, cls2]
  reg_i: (1, 4*reg_max=64, A_i)  原始 DFL 输入(未解码)
  cls_i: (1, nc=2, A_i)          原始 cls logit(未 sigmoid)
A_i: 6400(stride8) + 1600(stride16) + 400(stride32) = 8400

输出: [(x1,y1,x2,y2,score,cls)] 框, 坐标在 640 letterbox 空间。
"""
import numpy as np

REG_MAX = 16
STRIDES = [8, 16, 32]
GRIDS = [(80, 80), (40, 40), (20, 20)]   # (h,w) per scale
NC = 2
_ARANGE = np.arange(REG_MAX, dtype=np.float32)   # DFL 权重 [0..15]
# 预计算每尺度网格坐标(cell 单位, +0.5), 避免每帧 meshgrid
_GRID_X = []
_GRID_Y = []
for _h, _w in GRIDS:
    _sy, _sx = np.meshgrid(np.arange(_h) + 0.5, np.arange(_w) + 0.5, indexing='ij')
    _GRID_X.append(_sx.reshape(-1).astype(np.float32))
    _GRID_Y.append(_sy.reshape(-1).astype(np.float32))


def _softmax(x, axis=-1):
    x = x - x.max(axis=axis, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=axis, keepdims=True)


def _dfl(reg):
    """reg: (64, A) → (4, A) 距离(单位: stride cell)。
    view(4,16,A).transpose(1,0,2) → (16,4,A) softmax over 16 × arange(16) → (4,A)。"""
    a = reg.shape[-1]
    x = reg.reshape(4, REG_MAX, a)        # (4,16,A)
    x = x.transpose(1, 0, 2)             # (16,4,A)
    p = _softmax(x, axis=0)              # softmax over 16 bins
    ar = np.arange(REG_MAX, dtype=np.float32)   # (16,)
    dist = (p * ar[:, None, None]).sum(0)       # (4,A)
    return dist


def _grid(h, w):
    ys, xs = np.meshgrid(np.arange(h) + 0.5, np.arange(w) + 0.5, indexing='ij')
    return xs.reshape(-1), ys.reshape(-1)   # (A,),(A,)  cell 坐标


def nms(boxes, iou=0.45):
    if not boxes:
        return []
    boxes = sorted(boxes, key=lambda b: -b[4])
    keep = []
    for b in boxes:
        x1, y1, x2, y2, s, c = b
        ok = True
        for k in keep:
            kx1, ky1, kx2, ky2, _, kc = k
            if kc != c:
                continue
            ix1, iy1 = max(x1, kx1), max(y1, ky1)
            ix2, iy2 = min(x2, kx2), min(y2, ky2)
            iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
            inter = iw * ih
            union = (x2 - x1) * (y2 - y1) + (kx2 - kx1) * (ky2 - ky1) - inter
            if union > 0 and inter / union > iou:
                ok = False
                break
        if ok:
            keep.append(b)
    return keep


def decode(outs, conf=0.25, iou=0.45):
    """outs: [reg0,cls0,reg1,cls1,reg2,cls2], 每个是 (1,C,A) 或 (C,A) → 框列表(640空间)。
    全向量化 + 预计算网格, 无逐锚框循环。"""
    boxes = []
    for si in range(len(GRIDS)):
        reg = np.asarray(outs[si * 2]).reshape(4 * REG_MAX, -1)     # (64,A)
        cls = np.asarray(outs[si * 2 + 1]).reshape(NC, -1)          # (2,A)
        a = reg.shape[-1]
        # DFL: softmax over 16 bins, 加权和 → 距离(4,A)
        x = reg.reshape(4, REG_MAX, a).transpose(1, 0, 2)          # (16,4,A)
        x = x - x.max(0, keepdims=True)
        p = np.exp(x)
        p /= p.sum(0, keepdims=True)                              # softmax over 16
        dist = (p * _ARANGE[:, None, None]).sum(0)               # (4,A)
        scores = 1.0 / (1.0 + np.exp(-cls))                       # sigmoid (2,A)
        gx = _GRID_X[si]
        gy = _GRID_Y[si]
        s = STRIDES[si]
        x1 = (gx - dist[0]) * s
        y1 = (gy - dist[1]) * s
        x2 = (gx + dist[2]) * s
        y2 = (gy + dist[3]) * s
        best = scores.argmax(0)
        best_s = scores.max(0)
        mask = best_s >= conf
        if not mask.any():
            continue
        idx = np.where(mask)[0]
        for j in idx:                                              # 只遍历幸存锚框(通常几个)
            boxes.append((float(x1[j]), float(y1[j]), float(x2[j]), float(y2[j]),
                          float(best_s[j]), int(best[j])))
    return nms(boxes, iou)
