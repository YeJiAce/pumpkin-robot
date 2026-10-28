#!/usr/bin/env python3
"""底盘接口 —— 全项目唯一发运动指令的入口(voice/follow/teleop 都只调它)。

内部走 ROS2: drive() → 发布 /cmd_vel, stop() → 零速 + /pumpkin/stop(硬急停)。
真正的串口协议(握手/CMD_VEL 20Hz/看门狗)在 pumpkin_serial_node.py(单独进程跑)。

没 source ROS(rclpy 导入失败)时自动退回打印模式:语音/跟随流程照跑,方便无底盘调试。
  drive(vx, vy, wz)          设定期望速度(普通模式 0x10),持续生效到下次调用(跟随用)
  stop()                     急停(零速 + 硬急停;串口侧 STOP 取消使能)
  run_sequence(motions)      逐段执行 LLM 动作序列,全部闭环(靠 /odom 反馈走到位):
                            turn:相对航向转 angle 度;move:沿航向走;strafe:横移;
                            home:回到本序列起点位姿(舞蹈收尾用)
                            (开环速度×时长段和小陀螺 0x13 已于 10-27 废除,收到即跳过)
"""
import math
import os
import socket
import threading
import time

ABORT = threading.Event()   # stop() 置位,run_sequence 每段检查

# ---- 闭环段(turn/move)参数:P 控制 + 减速曲线 + 到位判据 + 段超时 ----
# 10-27 提速到 wz=10 后实测:满速甩过去刹不住,±100° 过冲来回震荡到超时(转圈"抽搐"的根因)。
# 10-28 修正:角速度上限 10→3,并加减速曲线 v≤√(2·a·err)(临近目标自动收敛,保证刹得住)。
TURN_K, TURN_WZ_MAX, TURN_WZ_MIN = 2.5, 3.0, 0.40   # 转角:wz=min(K×误差, 上限, 减速曲线)
TURN_DECEL = 4.0                                      # 刹车能力假设(rad/s²),减速曲线用
TURN_TOL = math.radians(4.0)                          # 到位:残差<4°,连续两拍
MOVE_K, MOVE_VX_MAX, MOVE_VX_MIN = 2.0, 0.8, 0.06   # 里程:vx 同样走减速曲线
MOVE_DECEL = 1.2                                      # 平移刹车能力假设(m/s²)
MOVE_TOL = 0.03                                       # 到位:残差<3cm,连续两拍

_odom = {"t": 0.0, "x": 0.0, "y": 0.0, "th": None}   # /odom 最新值(th=None=从未收到)


def _on_odom(msg):
    q = msg.pose.pose.orientation
    _odom.update(t=time.monotonic(),
                 x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                 th=math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))
_epoch = 0                   # 动作代数:起新序列/stop() 都会+1,旧序列发现代数变了立即退出
                             # (防止两条 run_sequence 线程并发互踩抢发速度)

_node = None
_pub_cmd = None
_pub_stop = None
_lock = threading.Lock()
_printed = (None, None, None)

try:
    # 保险丝:PC 与车同 WiFi 会被 DDS 互相发现(/cmd_vel 能穿过去!),非车端主机一律打印模式。
    # 白名单外要强制开:环境变量 PUMPKIN_CHASSIS_ROS=1(自己承担后果)。
    if socket.gethostname() not in ("ubuntu",) and not os.environ.get("PUMPKIN_CHASSIS_ROS"):
        raise RuntimeError(f"主机 {socket.gethostname()} 不是车(防 PC 误发 /cmd_vel;PUMPKIN_CHASSIS_ROS=1 可强制)")
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Empty

    rclpy.init()
    _node = rclpy.create_node("pumpkin_chassis")
    _pub_cmd = _node.create_publisher(Twist, "/cmd_vel", 10)
    _pub_stop = _node.create_publisher(Empty, "/pumpkin/stop", 10)
    _node.create_subscription(Odometry, "/odom", _on_odom, 20)   # 闭环反馈
    threading.Thread(target=lambda: rclpy.spin(_node), daemon=True).start()
    # 注:不做 atexit 清理——rclpy.shutdown 在 spin 线程活跃时会偶发死锁,
    # 宁可退出时报一句 "terminate called" 噪音,也不能挂死进程。
    print("[底盘] ROS 模式: /cmd_vel /pumpkin/stop /pumpkin/spin (串口由 pumpkin_serial_node 负责)", flush=True)
except Exception as _e:
    print(f"[底盘] ROS 不可用({_e}),退回打印模式(只打不动)", flush=True)
    _node = None


def drive(vx, vy, wz):
    """设定期望速度 (m/s, m/s, rad/s)。全向轮:x前 y左 w逆时针。"""
    if _node is not None:
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(vx), float(vy), float(wz)
        _pub_cmd.publish(m)
    else:
        global _printed
        with _lock:
            if (vx, vy, wz) != _printed:
                _printed = (vx, vy, wz)
                print(f"[底盘·打印] vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}", flush=True)


def stop():
    """急停:打断动作序列 + 零速 + 硬急停(串口侧 STOP,取消使能)。"""
    global _epoch
    _epoch += 1
    ABORT.set()
    drive(0, 0, 0)
    if _pub_stop is not None:
        _pub_stop.publish(Empty())


def _alive(my):
    return not ABORT.is_set() and _epoch == my


def _exec_turn(angle_deg, my):
    """转角闭环:相对当前航向转 angle_deg 度(左转/逆时针为正,同 wz 约定)。
    注意必须按「累计转角」算:±360° 的目标角 wrap 后误差恒为 0,会一拍就"到位"不转。"""
    total = math.radians(angle_deg)
    prev = _odom["th"]
    done, ref_done = 0.0, 0.0                    # 已累计转过的弧度 / 卡住检测基准
    deadline = time.monotonic() + 1.5 * abs(total) / 2.0 + 1.5
    hits, err, stuck = 0, total, False
    last_t = time.monotonic()
    while _alive(my) and time.monotonic() < deadline:
        done += _wrap(_odom["th"] - prev)        # 逐拍增量(短拍不会绕回,wrap 安全)
        prev = _odom["th"]
        err = total - done
        hits = hits + 1 if abs(err) < TURN_TOL else 0
        if hits >= 2:
            break
        if abs(done - ref_done) > math.radians(2.0):
            ref_done, last_t = done, time.monotonic()         # 在转,刷新卡住计时
        elif time.monotonic() - last_t > 1.5:                 # 1.5秒没转2° → 顶住了
            stuck = True
            break
        wz = min(TURN_K * abs(err), TURN_WZ_MAX,
                 math.sqrt(2.0 * TURN_DECEL * abs(err)))   # 减速曲线:临近目标自动收速
        wz = math.copysign(max(wz, TURN_WZ_MIN), err)
        drive(0.0, 0.0, wz)
        time.sleep(0.05)
    drive(0, 0, 0)
    if _alive(my) and (stuck or time.monotonic() >= deadline):
        why = "卡住放弃" if stuck else "超时"
        print(f"[底盘·闭环] 转{angle_deg:.0f}° {why},残差 {math.degrees(err):.0f}°", flush=True)


def _exec_move(dist, my, axis="x"):
    """里程闭环:axis="x" 沿当前航向(前进为正),axis="y" 横移(左为正)。单位米。"""
    th = _odom["th"]
    c, s = (math.cos(th), math.sin(th)) if axis == "x" else (-math.sin(th), math.cos(th))
    v_min = MOVE_VX_MIN if axis == "x" else 0.08   # 横移摩擦大,门槛略高
    est_v = 0.40 if axis == "x" else 0.35          # 10-27 提速后按新速度估段超时
    x0, y0 = _odom["x"], _odom["y"]
    deadline = time.monotonic() + 1.5 * abs(dist) / est_v + 2.0
    hits, rem, stuck = 0, dist, False
    lx, ly, last_t = x0, y0, time.monotonic()
    while _alive(my) and time.monotonic() < deadline:
        rem = dist - ((_odom["x"] - x0) * c + (_odom["y"] - y0) * s)
        hits = hits + 1 if abs(rem) < MOVE_TOL else 0
        if hits >= 2:
            break
        if math.hypot(_odom["x"] - lx, _odom["y"] - ly) > 0.01:
            lx, ly, last_t = _odom["x"], _odom["y"], time.monotonic()   # 在走,刷新
        elif time.monotonic() - last_t > 1.5:                           # 1.5秒没挪1cm → 顶住了
            stuck = True
            break
        v = min(MOVE_K * abs(rem), MOVE_VX_MAX,
                math.sqrt(2.0 * MOVE_DECEL * abs(rem)))    # 平移同样走减速曲线
        v = math.copysign(max(v, v_min), rem)
        drive(v, 0.0, 0.0) if axis == "x" else drive(0.0, v, 0.0)
        time.sleep(0.05)
    drive(0, 0, 0)
    if _alive(my) and (stuck or time.monotonic() >= deadline):
        why = "卡住放弃" if stuck else "超时"
        print(f"[底盘·闭环] 走{dist:.2f}m {why},残差 {rem * 100:.0f}cm", flush=True)


def _exec_home(my, x0, y0, th0):
    """回原位闭环:最多两轮「转向起点→走回」,最后转回原航向(th0)。"""
    for _ in range(2):
        dx, dy = x0 - _odom["x"], y0 - _odom["y"]
        dist = math.hypot(dx, dy)
        if dist <= MOVE_TOL:
            break
        _exec_turn(math.degrees(_wrap(math.atan2(dy, dx) - _odom["th"])), my)
        if not _alive(my):
            return
        _exec_move(dist, my)
    back = math.degrees(_wrap(th0 - _odom["th"]))
    if abs(back) > 2.0:
        _exec_turn(back, my)
    if _alive(my):
        r = math.hypot(x0 - _odom["x"], y0 - _odom["y"])
        if r > 0.05:
            print(f"[底盘·闭环] 回原位完成,残余偏差 {r * 100:.0f}cm", flush=True)


def run_sequence(motions):
    """逐段执行闭环动作序列(仅 turn/move/strafe/home 四种段),期间 stop() 可打断。
    阻塞调用,建议放线程里;起新序列会自动顶掉仍在跑的旧序列(代数令牌)。"""
    global _epoch
    my = _epoch + 1
    _epoch = my
    ABORT.clear()
    x0, y0, t0 = ((_odom["x"], _odom["y"], _odom["th"])
                  if _odom["th"] is not None else (0.0, 0.0, None))   # home 段的"原位"
    for m in motions:
        if ABORT.is_set() or _epoch != my:
            break
        mode = m.get("mode")
        if mode not in ("turn", "move", "strafe", "home"):
            print(f"[底盘] 未知动作段(已跳过,开环段已废除): {m}", flush=True)
            continue
        if _node is None:
            print(f"[底盘·打印] 闭环段 {m}(无ROS,只列出不执行)", flush=True)
        elif _odom["th"] is None or time.monotonic() - _odom["t"] > 0.5:
            print("[底盘·闭环] 没有 /odom 反馈(串口节点没跑?),跳过该段", flush=True)
            time.sleep(0.3)
        elif mode == "turn":
            _exec_turn(m["angle"], my)
        elif mode == "strafe":
            _exec_move(m["dist"], my, axis="y")
        elif mode == "home":
            _exec_home(my, x0, y0, t0)
        else:
            _exec_move(m["dist"], my)
    drive(0, 0, 0)
