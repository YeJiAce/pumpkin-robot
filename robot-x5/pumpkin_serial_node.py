#!/usr/bin/env python3
"""X5 底盘 ROS2 节点: /cmd_vel ⇄ 底盘串口协议 v1 (MCU 侧规范)。

订阅: /cmd_vel (Twist, m/s|rad/s) → 握手后 20Hz 发 CMD_VEL(mm/s, mrad/s 定点)
      /pumpkin/stop (Empty) → 发 STOP 帧取消使能(硬急停)
      /pumpkin/spin (Float64MultiArray [wz,duration]) → 小陀螺 0x13:
          先约1秒零速 0x13(进入时序) → 0x13 带 wz 到 duration 结束 → 回 0x10
发布: /odom (编码器反馈里的 body_velocity 积分)
      /imu/data_raw (μ 单位换 SI)
      STATUS → 异常时打日志(不带 diagnostics 话题)

用法: source /opt/tros/setup.bash
      python3 pumpkin_serial_node.py [串口路径]     (默认自动探测,PUMPKIN_PORT 环境变量亦可)
协议: 见 ~/pumpkin_robot/PROTOCOL_UART.md 及 MCU 侧文档
      AA 55 | VER TYPE SEQ LEN TIME_MS | PAYLOAD | CRC16-CCITT-FALSE(小端)
"""
import math
import os
import random
import struct
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float64MultiArray

import serial

# ---- 协议常量 ----
VER = 1
T_HELLO, T_ENABLE, T_STOP, T_TIME_SYNC = 0x01, 0x02, 0x03, 0x04
T_CMD_VEL, T_SET_HEADING, T_IMU_CAL, T_SPIN = 0x10, 0x11, 0x12, 0x13
T_STATUS, T_WHEEL, T_IMU = 0x20, 0x21, 0x22
T_ACK, T_HELLO_REPLY, T_TIME_SYNC_REPLY = 0x80, 0x81, 0x82

VX_LIM, VY_LIM, WZ_LIM = 800, 800, 10000     # mm/s, mm/s, mrad/s(协议v1.1)
                                             # 实测(10-25):MCU硬拒收>800的平移速度,轮子不动
IMU_READY = 0x08                             # status_flags bit3 = IMU 就绪
RATE = 20.0                                  # CMD_VEL 发送频率
CMD_TIMEOUT = 1.0                            # 超这时间没来新 /cmd_vel 且目标非零 → 自动归零
                                             # (防语音进程崩溃后最后速度被一直重发)
# 注: 0x13 小陀螺模式若启用,进入时须先发约1秒零速 0x13,再给 vx(固件当前约定)


def crc16(data):
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, 不反射, 异或0。"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(ftype, seq, payload):
    body = struct.pack("<BBHHI", VER, ftype, seq & 0xFFFF, len(payload), 0) + payload
    c = crc16(body)
    return b"\xaa\x55" + body + struct.pack("<H", c)


class RxParser:
    """字节流 → (TYPE, SEQ, TIME_MS, payload) 帧列表;容错找帧头。"""

    def __init__(self):
        self.buf = b""

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(b"\xaa\x55")
            if i < 0:
                self.buf = b"" if len(self.buf) > 512 else self.buf
                return out
            self.buf = self.buf[i:]
            if len(self.buf) < 14:
                return out
            ver, ftype, seq, ln = struct.unpack_from("<BBHH", self.buf, 2)
            if ver != VER or ln > 64:
                self.buf = self.buf[2:]          # 假帧头,跳过
                continue
            if len(self.buf) < 14 + ln + 2:
                return out
            body = self.buf[2:12 + ln]
            (c,) = struct.unpack_from("<H", self.buf, 12 + ln)
            if crc16(body) != c:
                self.buf = self.buf[2:]
                continue
            (tms,) = struct.unpack_from("<I", self.buf, 8)
            out.append((ftype, seq, tms, self.buf[12:12 + ln]))
            self.buf = self.buf[14 + ln:]


class PumpkinSerialNode(Node):
    def __init__(self):
        super().__init__("pumpkin_serial")
        port = (sys.argv[1] if len(sys.argv) > 1 else os.environ.get("PUMPKIN_PORT", ""))
        self.port = port or self.autodetect()
        self.get_logger().info(f"串口: {self.port or '(未找到,持续探测)'}")
        self.ser = None
        self.seq = 0
        self.session = 0
        self.enabled = False
        self.lock = threading.Lock()
        self.target = (0, 0, 0)                # mm/s, mm/s, mrad/s(必须int,struct.pack不收float)
        self.need_enable = False               # STOP 后来了新速度 → 重新使能
        self.hard_stop = False
        self.spin_req = None                   # 小陀螺请求 {prime_until,end_at,wz(mrad/s)}
        self.last_cmd = time.monotonic()       # 最近一次 /cmd_vel(心跳超时用)
        self.cmd_count = 0                     # /cmd_vel 消息计数:spin 只被"新来的"消息抢占
        self.rxparser = RxParser()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Empty, "/pumpkin/stop", self.on_hard_stop, 10)
        self.create_subscription(Float64MultiArray, "/pumpkin/spin", self.on_spin, 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 20)
        self.pub_imu = self.create_publisher(Imu, "/imu/data_raw", 50)
        self.odom_x = self.odom_y = self.odom_th = 0.0
        self.t_wheel = time.monotonic()
        self.last_fault = self.last_stop_reason = 0
        threading.Thread(target=self.manager, daemon=True).start()

    # ---------- 工具 ----------
    def autodetect(self):
        # ttyS0 是 X5 调试控制台,不能抢;引脚 UART(ttyS*)等用户报端口号再指定
        for p in ("/dev/ttyUSB0", "/dev/ttyACM0"):
            if os.path.exists(p):
                return p
        return ""

    def next_seq(self):
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq

    def send(self, ftype, payload):
        frame = build_frame(ftype, self.next_seq(), payload)
        self.ser.write(frame)

    def wait_reply(self, want_type, timeout=0.6):
        """从串口收字节喂解析器,等指定类型帧。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            n = self.ser.in_waiting
            if n:
                for ftype, seq, tms, pl in self.rxparser.feed(self.ser.read(n)):
                    if ftype == want_type:
                        return pl
        return None

    # ---------- 回调 ----------
    def on_cmd_vel(self, msg):
        vx = max(-VX_LIM, min(VX_LIM, int(round(msg.linear.x * 1000))))
        vy = max(-VY_LIM, min(VY_LIM, int(round(msg.linear.y * 1000))))
        wz = max(-WZ_LIM, min(WZ_LIM, int(round(msg.angular.z * 1000))))
        clipped = (abs(msg.linear.x) * 1000 > VX_LIM or
                   abs(msg.linear.y) * 1000 > VY_LIM or
                   abs(msg.angular.z) * 1000 > WZ_LIM)
        with self.lock:
            self.last_cmd = time.monotonic()
            self.cmd_count += 1
            if self.target != (vx, vy, wz):
                note = "(超协议上限已截断)" if clipped else ""
                self.get_logger().info(f"目标: vx={vx}mm/s vy={vy}mm/s wz={wz}mrad/s{note}")
            self.target = (vx, vy, wz)
            if (vx, vy, wz) != (0, 0, 0) and not self.enabled:
                self.need_enable = True

    def on_spin(self, msg):
        """小陀螺请求 [wz(rad/s), dur(s)] 或 [wz,dur,vx,vy](m/s,平移固定在进入时坐标系)。
        已在小陀螺中→原位更新速度并续期(不重排引子,键盘连续变速用);首次进入排 1 秒零速引子。"""
        if len(msg.data) < 2:
            return
        wz = max(-WZ_LIM, min(WZ_LIM, int(round(msg.data[0] * 1000))))
        dur = max(0.1, min(10.0, float(msg.data[1])))
        vx = max(-VX_LIM, min(VX_LIM, int(round(float(msg.data[2]) * 1000)))) if len(msg.data) > 2 else 0
        vy = max(-VY_LIM, min(VY_LIM, int(round(float(msg.data[3]) * 1000)))) if len(msg.data) > 3 else 0
        now = time.monotonic()
        with self.lock:
            sp = self.spin_req
            if (sp is not None and now < sp["end_at"]
                    and self.cmd_count == sp["cmd_count"]):
                sp.update(wz=wz, vx=vx, vy=vy, end_at=now + dur)   # 原位续期变速
                return
            # 记下当前 /cmd_vel 编号:只有比它新的消息才算抢占(旧残留目标不算);
            # 进小陀螺即抛弃上一段的残留速度
            self.spin_req = {"prime_until": now + 1.0, "end_at": now + 1.0 + dur,
                             "wz": wz, "vx": vx, "vy": vy, "cmd_count": self.cmd_count}
            self.target = (0, 0, 0)
            if not self.enabled:                 # 纯 spin 不经过 /cmd_vel,这里补触发重新使能
                self.need_enable = True
        self.get_logger().info(f"小陀螺: wz={wz}mrad/s vx={vx} vy={vy} {dur:.1f}s")

    def on_hard_stop(self, _):
        with self.lock:
            self.hard_stop = True
        self.get_logger().warn("收到硬急停 /pumpkin/stop")

    # ---------- 遥测 ----------
    def on_wheel(self, pl):
        if len(pl) < 40:
            return
        *_, vx, vy, wz = struct.unpack("<I3I3h3h3h3h", pl)   # 16值:跳过session/编码器/转速/PWM
        now = time.monotonic()
        dt = now - self.t_wheel
        self.t_wheel = now
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "odom"
        m.child_frame_id = "base_link"
        c, s = math.cos(self.odom_th), math.sin(self.odom_th)
        bvx, bvy, bwz = vx / 1000.0, vy / 1000.0, wz / 1000.0
        self.odom_x += (bvx * c - bvy * s) * dt
        self.odom_y += (bvx * s + bvy * c) * dt
        self.odom_th += bwz * dt
        m.pose.pose.position.x, m.pose.pose.position.y = self.odom_x, self.odom_y
        m.pose.pose.orientation.z = math.sin(self.odom_th / 2.0)   # 补:yaw→四元数(纯偏航)
        m.pose.pose.orientation.w = math.cos(self.odom_th / 2.0)
        m.twist.twist.linear.x, m.twist.twist.linear.y = bvx, bvy
        m.twist.twist.angular.z = bwz
        self.pub_odom.publish(m)

    def on_imu(self, pl):
        if len(pl) < 36:
            return
        _, flags, period, ax, ay, az, gx, gy, gz, yaw = struct.unpack("<IHH3i3ii", pl)
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "imu_link"
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = \
            ax / 1e6, ay / 1e6, az / 1e6
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = \
            gx / 1e6, gy / 1e6, gz / 1e6
        m.orientation_covariance[0] = -1          # 无完整姿态(Foxy/Humble 惯例)
        if flags & 1:
            self.pub_imu.publish(m)

    def on_status(self, pl):
        if len(pl) < 24:
            return
        _, sfl, ffl, last_seq, cmd_age, cpu, imu_age, heap, crc_e, drop_e, stop_r, mode = \
            struct.unpack("<I9H2B", pl)
        if ffl != self.last_fault or stop_r != self.last_stop_reason:
            self.last_fault, self.last_stop_reason = ffl, stop_r
            if ffl or stop_r:
                self.get_logger().warn(f"STATUS fault=0x{ffl:04x} stop_reason={stop_r} "
                                       f"cmd_age={cmd_age}ms mode={mode}")

    # ---------- 主管理线程 ----------
    def manager(self):
        parser = RxParser()
        while True:
            # 1) 开串口
            try:
                self.ser = serial.Serial(self.port, 115200, timeout=0.02)
                self.get_logger().info(f"串口已打开 {self.port}@115200")
            except Exception as e:
                self.get_logger().warning(f"串口打开失败({e}),2s后重试", throttle_duration_sec=10)
                time.sleep(2)
                continue
            # 2) 握手
            if not self.handshake(parser):
                self.close_ser()
                time.sleep(1)
                continue
            # 3) 指令循环 20Hz
            period = 1.0 / RATE
            next_t = time.monotonic()
            while True:
                if self.ser is None:
                    break
                try:
                    n = self.ser.in_waiting
                    if n:
                        for ftype, seq, tms, pl in parser.feed(self.ser.read(n)):
                            if ftype == T_WHEEL:
                                self.on_wheel(pl)
                            elif ftype == T_IMU:
                                self.on_imu(pl)
                            elif ftype == T_STATUS:
                                self.on_status(pl)
                    now = time.monotonic()
                    with self.lock:
                        tgt, hard, need_en = self.target, self.hard_stop, self.need_enable
                        self.hard_stop = False
                        self.need_enable = False
                        sp = self.spin_req
                        if sp is not None and now >= sp["end_at"]:      # 到时,退出小陀螺
                            self.spin_req = None
                            self.target = (0, 0, 0)                     # 不复活旧速度
                            sp = None
                            self.get_logger().info("小陀螺结束,回普通模式")
                        if sp is not None and self.cmd_count != sp["cmd_count"]:
                            # spin 发起之后又来了新 /cmd_vel(跟随/遥控/新动作) → 真抢占
                            self.spin_req = None
                            sp = None
                            self.get_logger().info("新 /cmd_vel 抢占,退出小陀螺")
                    if hard:
                        with self.lock:
                            self.spin_req = None
                            self.target = (0, 0, 0)
                        tgt, sp = (0, 0, 0), None
                        self.send(T_STOP, struct.pack("<I", self.session))
                        self.enabled = False
                        self.get_logger().warn("已发 STOP(取消使能)")
                    elif need_en and not self.enabled:
                        self.send(T_ENABLE, struct.pack("<IB", self.session, 1))
                        self.enabled = True            # ACK 异常会由 STATUS/日志暴露
                        self.get_logger().info("已重新使能")
                    if sp is None and tgt != (0, 0, 0) and now - self.last_cmd > CMD_TIMEOUT:
                        with self.lock:
                            self.target = (0, 0, 0)
                        tgt = (0, 0, 0)
                        self.get_logger().warn("超1s无 /cmd_vel 心跳,自动归零停车")
                    if sp is not None:
                        if now < sp["prime_until"]:    # 进入时序:先约1秒零速 0x13
                            self.send(T_SPIN, struct.pack("<I3h", self.session, 0, 0, 0))
                        else:
                            self.send(T_SPIN, struct.pack(
                                "<I3h", self.session,
                                sp.get("vx", 0), sp.get("vy", 0), sp["wz"]))
                    else:
                        self.send(T_CMD_VEL, struct.pack("<I3h", self.session, *tgt))
                except Exception as e:
                    self.get_logger().error(f"串口异常({e}),重连")
                    self.close_ser()
                    break
                next_t += period
                d = next_t - time.monotonic()
                if d > 0:
                    time.sleep(d)
                else:
                    next_t = time.monotonic()

    def handshake(self, parser):
        self.rxparser = parser
        self.session = random.getrandbits(32) or 1
        for step in range(3):
            try:
                self.send(T_HELLO, struct.pack("<I", self.session))
                pl = self.wait_reply(T_HELLO_REPLY)
                if pl is None:
                    continue
                sess, fw, caps = struct.unpack("<III", pl)
                self.get_logger().info(f"HELLO_REPLY 固件 {fw >> 16}.{(fw >> 8) & 0xFF}.{fw & 0xFF} "
                                       f"caps=0x{caps:x}")
                token = random.getrandbits(31)
                self.send(T_TIME_SYNC, struct.pack("<II", self.session, token))
                if self.wait_reply(T_TIME_SYNC_REPLY) is None:
                    continue
                # 协议v1.1: ENABLE 前必须等状态反馈显示 IMU 就绪(status_flags bit3)
                imu_ok = self.wait_imu_ready(parser, timeout=8.0)
                if not imu_ok:
                    self.get_logger().warning("8s内未见 IMU 就绪,放弃本轮握手重来")
                    continue
                self.send(T_ENABLE, struct.pack("<IB", self.session, 1))
                pl = self.wait_reply(T_ACK)
                if pl is None:
                    continue
                _, rseq, rtype, result = struct.unpack("<IHBB", pl)
                if result != 0:
                    self.get_logger().error(f"ENABLE 被拒 result={result}")
                    return False
                self.enabled = True
                self.get_logger().info("握手完成,已使能,20Hz 指令流启动")
                return True
            except Exception as e:
                self.get_logger().warning(f"握手异常: {e}")
        self.get_logger().error("握手3次失败,重开串口")
        return False

    def wait_imu_ready(self, parser, timeout):
        """收 STATUS 直到 status_flags bit3(IMU就绪)。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            n = self.ser.in_waiting
            if n:
                for ftype, seq, tms, pl in parser.feed(self.ser.read(n)):
                    if ftype == T_STATUS and len(pl) >= 6:
                        (sfl,) = struct.unpack_from("<H", pl, 4)
                        if sfl & IMU_READY:
                            self.get_logger().info("IMU 已就绪")
                            return True
            time.sleep(0.02)
        return False

    def close_ser(self):
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.enabled = False


def main():
    rclpy.init()
    node = PumpkinSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
