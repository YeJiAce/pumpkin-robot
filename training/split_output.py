#!/usr/bin/env python3
"""把 YOLOv8 best.onnx 的单输出(1,6,8400)改造成两个 BPU 友好输出:
   - output_box:    /model.22/Mul_2_output_0  (1,4,8400)  cx,cy,w,h (0~640)
   - output_clslogit: /model.22/Concat_1_output_0 (1,2,8400) 原始 cls logit

两处改造(根因 = BPU PTQ):
  1) 删最终 Concat: 原本 box(0~640) 和 cls(0~1) 拼成一个张量, 共用一个量化
     scale, box 把 scale 撑大, cls 被压成 0。拆开后各自量化。
  2) 删最终 Sigmoid: BPU 用 LUT 实现 Sigmoid, 对 cls(大部分锚框是背景 logit
     很负→0, 少数目标 logit 正→0.7) 这种极端分布量化后输出塌成 ~0/1。
     改成输出原始 logit(范围大、量化无压力), sigmoid 拿到 CPU float 上做。

后处理(CPU): cls = sigmoid(clslogit); 其余同 YOLOv8。
"""
import onnx
from onnx import shape_inference, TensorProto, helper

SRC = '/home/yjy/pumpkin_robot/runs/pumpkin_v2/weights/best.onnx'
DST = '/home/yjy/pumpkin_robot/rdk_convert/best_split.onnx'

BOX = '/model.22/Mul_2_output_0'        # (1,4,8400)
CLS_LOGIT = '/model.22/Concat_1_output_0'  # (1,2,8400)  Sigmoid 之前
SIGMOID_OUT = '/model.22/Sigmoid_output_0'
CONCAT_OUT = 'output0'


def main():
    m = onnx.load(SRC)
    g = m.graph
    # 1. 删掉最终 Concat(产出 output0) 和最终 Sigmoid(产出 Sigmoid_output_0)
    keep = [n for n in g.node
            if CONCAT_OUT not in n.output and SIGMOID_OUT not in n.output]
    assert len(keep) == len(g.node) - 2, f'应删2个节点, 实删{len(g.node)-len(keep)}'
    del g.node[:]
    g.node.extend(keep)
    # 2. 清掉旧单一输出 output0
    while g.output:
        g.output.pop()
    # 3. 推断形状, 拿 box/logit 的 ValueInfo
    m_inf = shape_inference.infer_shapes(m)
    vi = {v.name: v for v in m_inf.graph.value_info}
    assert BOX in vi and CLS_LOGIT in vi, 'box 或 logit 张量在 value_info 里找不到'

    def mk_out(name):
        d = [x.dim_value for x in vi[name].type.tensor_type.shape.dim]
        return helper.make_tensor_value_info(name, TensorProto.FLOAT, d)

    g.output.append(mk_out(BOX))
    g.output.append(mk_out(CLS_LOGIT))
    onnx.checker.check_model(m)
    onnx.save(m, DST)
    print(f'拆分完成 -> {DST}')
    print(f'输出1 BOX:    {BOX} {[x.dim_value for x in vi[BOX].type.tensor_type.shape.dim]}')
    print(f'输出2 CLSLOGIT: {CLS_LOGIT} {[x.dim_value for x in vi[CLS_LOGIT].type.tensor_type.shape.dim]}')


if __name__ == '__main__':
    main()
