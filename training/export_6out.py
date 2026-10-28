#!/usr/bin/env python3
"""导出 YOLOv8 → 6 输出 ONNX(RDK BPU 官方 yolov8 格式):

  每个尺度(3个, stride 8/16/32)输出两个张量:
    reg_raw : (1, 4*reg_max=64, A_i)   原始 box 回归(DFL 输入, 未解码)
    cls_raw : (1, nc=2, A_i)            原始 cls logit(未 sigmoid)
  共 6 个输出。DFL/sigmoid/NMS 留给 CPU 后处理(或 dnn_node yolov8 parser)。

为什么不用 ultralytics 默认单输出: BPU PTQ 对 cls 偏态分布量化会把正 logit
压没(已实测 default/max/kl 三种校准 cls 全塌)。6 输出按尺度拆开、各自量化,
是 D-Robotics 文档里 YOLOv8→RDK 的已知可行路径。

用法: python3 export_6out.py [权重.pt] [输出.onnx]
默认 v2; 10-27 起支持传 v3 等新权重。
"""
import os
import sys
import types
import numpy as np
import torch
from ultralytics import YOLO

PT = sys.argv[1] if len(sys.argv) > 1 else '/home/yjy/pumpkin_robot/runs/pumpkin_v2/weights/best.pt'
DST = sys.argv[2] if len(sys.argv) > 2 else '/home/yjy/pumpkin_robot/rdk_convert/best_6out.onnx'
SIZE = 640


def forward_6out(self, x):
    """替代 Detect.forward: 返回每尺度 (reg_raw, cls_raw) 共 6 个张量。"""
    bs = x[0].shape[0]
    outs = []
    for i in range(self.nl):
        outs.append(self.cv2[i](x[i]).view(bs, 4 * self.reg_max, -1))  # reg_raw [bs,64,A]
        outs.append(self.cv3[i](x[i]).view(bs, self.nc, -1))           # cls_logit [bs,2,A]
    return tuple(outs)


def main():
    m = YOLO(PT)
    mdl = m.model
    det = next(mm for mm in mdl.modules() if mm.__class__.__name__ == 'Detect')
    det.export = True                       # 导出模式: 关掉后处理分支
    det.forward = types.MethodType(forward_6out, det)
    mdl.eval()
    try:
        mdl.fuse()                          # 融合 Conv+BN(纯 conv, BPU 友好)
    except Exception as e:
        print('fuse warn:', e)
    dummy = torch.zeros(1, 3, SIZE, SIZE)
    torch.onnx.export(
        mdl, dummy, DST,
        opset_version=11,
        input_names=['images'],
        output_names=[f'out_{i}' for i in range(6)],
        dynamic_axes=None,
        do_constant_folding=True,
        dynamo=False,           # legacy 导出器: opset11/单文件/IR兼容(hb_mapper 要求 opset<=11)
    )
    print(f'导出完成 -> {DST}')
    # 自检: 跑一下, 看输出形状(3 尺度各 reg+cls)
    import onnxruntime as ort
    sess = ort.InferenceSession(DST, providers=['CPUExecutionProvider'])
    outs = sess.run(None, {'images': np.zeros((1, 3, SIZE, SIZE), np.float32)})
    for i, o in enumerate(outs):
        print(f'  out{i}: {o.shape}  dtype={o.dtype}')


if __name__ == '__main__':
    main()
