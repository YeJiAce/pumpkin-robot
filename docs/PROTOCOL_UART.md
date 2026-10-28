# ⚠️ 本文档已废弃（2026-10-25）

这是早期我方拟的 15 字节简化协议，**已被 MCU 侧完整协议（v1.1）取代，切勿按本文实现**。

现行协议要点（以用户/Codex 提供的完整文档为准）：
- 帧：`AA 55 | VER=01 | TYPE | SEQ:u16 | LEN:u16 | TIME_MS:u32 | PAYLOAD | CRC16-CCITT-FALSE(小端)`
- CRC：poly 0x1021 / init 0xFFFF / 不反射 / 无异或；范围 VERSION..PAYLOAD
- 上位机下发：HELLO(0x01) → TIME_SYNC(0x04) → 等 IMU 就绪 → ENABLE(0x02) → CMD_VEL(0x10) 20Hz → STOP(0x03)
- 速度：vx/vy i16 mm/s(±800)、wz i16 mrad/s(0x13 可到 ±10000)；0x13 小陀螺模式进入时先发约 1 秒零速
- 反馈：STATUS(0x20) 5Hz / WHEEL_FEEDBACK(0x21) 50Hz / IMU_FEEDBACK(0x22) 100Hz / ACK(0x80) 失败才回

X5 侧实现：`pumpkin_serial_node.py`（ROS2 节点，含握手/20Hz 指令流/遥测）；只读监听：`serial_sniff.py`。
