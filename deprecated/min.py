#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
min.py —— 最小终端监控视图（不依赖服务端、不依赖数据库）

把原本只通过 UDP 发往 PC:9090 的三类数据直接画在终端上。
数据契约与 smp.py 完全一致：

    TD1  电池电压/剩余电量 + 烟雾状态     对应 smp.py:556
    TD2  毫米波人数 + 簇中心坐标          对应 smp.py:557
    TD3  热成像整帧温度矩阵               对应 smp.py:558

用法：
    python3 min.py                  # 自动：有硬件就用硬件，否则仿真
    python3 min.py --source sim     # 强制仿真（无需任何外设/第三方库）
    python3 min.py --source hw      # 强制硬件（缺库/缺设备时明确报错退出）
    python3 min.py --frames 3       # 只画 3 帧后退出（无 TTY 时便于验证）
    python3 min.py --color          # 热图使用 256 色
    python3 min.py --sim-size 96x40 # 仿真热图尺寸 WxH

设计约束（刻意为之）：
  * 只用标准库。仿真模式在任何机器上可直接运行，不需要 numpy/senxor/RPi.GPIO。
  * 热图尺寸完全动态，从 frame 的实际形状推导；代码里不存在任何写死的分辨率。
  * 不使用 os.system('clear')，改用 ANSI 原地刷新，避免闪屏与子进程开销。
  * 本文件不连数据库、不开 UDP 端口，因此不需要任何服务端。

验证状态（重要，请勿当成全部已验证）：
  [已验证] 仿真数据源、DBSCAN、终端渲染、CLI、非 TTY 降级、颜色合并
           —— 在 Python 3.14.7 / 无外设环境下实际运行通过。
           中文标签按显示宽度手工补齐到 7 列，未引入宽度计算依赖。
  [未验证] Hw* 三个类逐行对应 smp.py 的既有实现，但本机没有树莓派外设
           （无 /dev/i2c-*、无 /dev/radar_uart_*，且缺 RPi.GPIO/smbus2/serial），
           无法运行验证；逻辑以 smp.py 为准，真机使用前请先在设备上试跑。
  [未实现] 热成像的真实采集需要 senxor + MI48，其初始化约 110 行
           （smp.py:122-241），与"最小"目标冲突。硬件模式下热图显示"未接入"，
           真实热图请沿用 smp.py 的采集循环。
"""

from __future__ import annotations

import argparse
import atexit
import math
import os
import random
import shutil
import struct
import sys
import time

# ============================================================ 1. 渲染工具

RAMP = " .:-=+*#%@"                     # 冷 -> 热
_COLOR_256 = (21, 27, 33, 39, 45, 82, 118, 154, 190, 226, 220, 214, 208, 202, 196)
_YELLOW, _RED, _DIM, _RESET = (
    "\033[33m", "\033[31m", "\033[2m", "\033[0m")


def ramp_char(v: float, lo: float, span: float) -> str:
    """温度 -> 灰度字符（RAMP 索引由归一化温度决定）。"""
    t = (v - lo) / span
    return RAMP[min(len(RAMP) - 1, max(0, int(t * (len(RAMP) - 1) + 0.5)))]


def color_index(v: float, lo: float, span: float) -> int:
    """温度 -> 256 色索引。"""
    t = (v - lo) / span
    return int(max(0.0, min(1.0, t)) * (len(_COLOR_256) - 1))


# ============================================================ 2. 数据处理
# 以下两个函数是 smp.py:380-448 / 484-500 的等价最小实现，
# 参数名与语义保持一致，便于与 smp.py 对照。

def dbscan_mmwave(points, epsilon, weight, min_neighbors):
    """对 [x, y, z, doppler] 点云做 DBSCAN，返回簇中心 [[x, y, 0.0], ...]。

    距离度量与 smp.py:380-390 一致：dx^2 + dy^2 + weight * dv^2。
    min_neighbors 的语义也与 smp.py:417 一致：比较的是 len(neighbors) - 1。
    """
    n = len(points)
    labels = [0] * n
    visited = [False] * n
    cluster_id = 0

    def region_query(i):
        xi, yi, vi = points[i][0], points[i][1], points[i][3]
        out = []
        for j in range(n):
            dx = points[j][0] - xi
            dy = points[j][1] - yi
            dv = points[j][3] - vi
            if dx * dx + dy * dy + weight * dv * dv <= epsilon:
                out.append(j)
        return out

    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        neighbors = region_query(i)
        if len(neighbors) - 1 < min_neighbors:
            labels[i] = 0
            continue
        cluster_id += 1
        labels[i] = cluster_id
        seeds = list(neighbors)
        si = 0
        while si < len(seeds):
            p = seeds[si]
            if not visited[p]:
                visited[p] = True
                neigh_p = region_query(p)
                if len(neigh_p) - 1 >= min_neighbors:
                    for nb in neigh_p:
                        if nb not in seeds:
                            seeds.append(nb)
            if labels[p] == 0:
                labels[p] = cluster_id
            si += 1

    centers = []
    for cid in range(1, cluster_id + 1):
        idxs = [i for i in range(n) if labels[i] == cid]
        if not idxs:
            continue
        centers.append([sum(points[i][0] for i in idxs) / len(idxs),
                        sum(points[i][1] for i in idxs) / len(idxs),
                        0.0])
    return centers


def cluster_people(points, epsilon, weight):
    """复刻 smp.py:484-500：邻居数随点云规模切换，并按视场范围过滤。"""
    min_neighbors = 1 if len(points) <= 10 else 2
    centers = dbscan_mmwave(points, epsilon, weight, min_neighbors)
    # smp.py:492 的过滤条件，原样保留
    return [c for c in centers if c[0] < 10 and c[0] > -10 and c[1] < 10]


def downsample(frame, rows, cols):
    """块平均降采样，纯 Python；尺寸完全由入参决定，不假设分辨率。"""
    h = len(frame)
    w = len(frame[0]) if h else 0
    if h == 0 or w == 0 or rows <= 0 or cols <= 0:
        return []
    out = []
    for r in range(rows):
        y0 = r * h // rows
        y1 = max(y0 + 1, (r + 1) * h // rows)
        line = []
        for c in range(cols):
            x0 = c * w // cols
            x1 = max(x0 + 1, (c + 1) * w // cols)
            total = 0.0
            count = 0
            for y in range(y0, min(y1, h)):
                row = frame[y]
                for x in range(x0, min(x1, w)):
                    total += row[x]
                    count += 1
            line.append(total / count if count else 0.0)
        out.append(line)
    return out


def thermal_stats(frame):
    """返回 (最低温, 最高温, 均温, (最热点x, 最热点y))。"""
    lo = math.inf
    hi = -math.inf
    total = 0.0
    count = 0
    hot_xy = (0, 0)
    for y, row in enumerate(frame):
        for x, v in enumerate(row):
            if v < lo:
                lo = v
            if v > hi:
                hi = v
                hot_xy = (x, y)
            total += v
            count += 1
    if count == 0:
        return 0.0, 0.0, 0.0, (0, 0)
    return lo, hi, total / count, hot_xy


# ============================================================ 3. 数据源

class SimSource:
    """仿真数据源：不需要任何外设或第三方库。

    注意：仿真热图的尺寸是 --sim-size 给定的任意值，**不代表真实 MI48 的分辨率**。
    渲染路径完全按实际形状处理，所以真机换任何尺寸都无需改代码。
    """

    def __init__(self, device_id, size=(64, 32), seed=None):
        self.device_id = device_id
        self.cols, self.rows = size
        self.rnd = random.Random(seed)
        self.voltage = 4.05
        self.frame_no = 0
        self._alarm_left = 0
        self._hot = [
            [self.rnd.uniform(0.2, 0.8) * self.cols,
             self.rnd.uniform(0.2, 0.8) * self.rows,
             self.rnd.uniform(9.0, 15.0),
             self.rnd.uniform(2.5, 5.0)]
            for _ in range(2)
        ]

    def battery(self):
        self.voltage = max(3.0, self.voltage - self.rnd.uniform(0.0004, 0.0016))
        pct = (self.voltage - 3.0) / (4.2 - 3.0) * 100.0
        return self.voltage, max(0.0, min(100.0, pct))

    def smoke(self):
        """GPIO 低有效：0 = 报警，1 = 正常（对应 smp.py:502-503）。"""
        if self._alarm_left > 0:
            self._alarm_left -= 1
            return 0
        if self.rnd.random() < 0.02:
            self._alarm_left = self.rnd.randint(2, 4)
            return 0
        return 1

    def radar(self):
        points = []
        for _ in range(self.rnd.choice([0, 1, 1, 2, 2, 3])):
            cx = self.rnd.uniform(-3.0, 3.0)
            cy = self.rnd.uniform(1.0, 6.0)
            for _ in range(self.rnd.randint(4, 9)):
                points.append([cx + self.rnd.gauss(0, 0.15),
                               cy + self.rnd.gauss(0, 0.15),
                               0.0,
                               self.rnd.gauss(0, 0.1)])
        return points

    def thermal(self):
        self.frame_no += 1
        base = 22.0 + 3.0 * math.sin(self.frame_no / 40.0)
        frame = []
        for y in range(self.rows):
            row = []
            for x in range(self.cols):
                v = base + self.rnd.gauss(0, 0.35)
                for hx, hy, amp, sig in self._hot:
                    d2 = (x - hx) ** 2 + (y - hy) ** 2
                    v += amp * math.exp(-d2 / (2.0 * sig * sig))
                row.append(v)
            frame.append(row)
        # 热斑缓慢漂移
        for h in self._hot:
            h[0] = min(self.cols - 1, max(0, h[0] + self.rnd.uniform(-0.6, 0.6)))
            h[1] = min(self.rows - 1, max(0, h[1] + self.rnd.uniform(-0.6, 0.6)))
        return frame


class HwBattery:
    """真实电量计：I2C 地址 0x36，寄存器 2/4。

    逐行对应 smp.py:99-116（含 struct 字节交换与 1.25/1000/16 换算）。
    [未验证] 本机无 /dev/i2c-*，无法运行。
    """

    def __init__(self, bus=1, addr=0x36):
        try:
            import smbus2 as _busmod
        except ImportError:
            import smbus as _busmod
        self.addr = addr
        self.bus = _busmod.SMBus(bus)

    def battery(self):
        read = self.bus.read_word_data(self.addr, 2)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        voltage = swapped * 1.25 / 1000 / 16
        read = self.bus.read_word_data(self.addr, 4)
        swapped = struct.unpack("<H", struct.pack(">H", read))[0]
        capacity = max(0.0, min(100.0, swapped / 256))
        return voltage, capacity


class HwSmoke:
    """真实烟雾输入：GPIO BCM17，上拉输入，低有效。

    对应 smp.py:248 与 smp.py:502。去抖计数(>=30)属于业务逻辑，此处只读电平。
    [未验证] 本机无 RPi.GPIO。
    """

    def __init__(self, pin=17):
        import RPi.GPIO as GPIO
        self._gpio = GPIO
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def smoke(self):
        return self._gpio.input(self.pin)


class HwRadar:
    """真实毫米波点云：解析数据口 TLV type=1。

    字段解析逐行对应 smp.py:686-755：
      * 帧头 32 字节，长度字段在 header[12:16]，小端，再减 32
      * TLV 头 8 字节，type/len 均为小端；type > 9 或 len >= 帧长则视为填充
      * type == 1 时每 16 字节一个点：x, y, z, doppler 四个 float

    字节序说明（易错点）：smp.py:722-726 先把 4 字节拼成【小端】整数，
    再按【大端】重新解释为 float。这一"双重交换"与 struct.unpack_from('<f')
    完全等价（若写成 '!f' 则数值错误），此处直接用 '<f'。

    采集前 smp.py 会先向控制口下发 36 条配置命令(smp.py:254-290)。
    为避免把那套硬编码命令表复制进来，这里从外部文件读取：
        --radar-profile <文件>   每行一条命令
    不提供该文件时只尝试解析已有数据流，通常不会有输出。
    [未验证] 本机无 /dev/radar_uart_* 且缺 pyserial。
    """

    HEADER_LEN = 32

    def __init__(self, data_port="/dev/radar_uart_1", control_port="/dev/radar_uart_0",
                 baud=921600, ctrl_baud=115200, profile_path=None):
        import serial
        self._serial = serial
        self.ser = serial.Serial(data_port, baud, timeout=0.5)
        self.ctrl = None
        if profile_path:
            with open(profile_path, "r", encoding="utf-8") as fh:
                cmds = [ln.rstrip("\r\n") for ln in fh if ln.strip()]
            self.ctrl = serial.Serial(control_port, ctrl_baud, timeout=0.5)
            for cmd in cmds:
                self.ctrl.write((cmd + "\r").encode())
                time.sleep(0.05)

    def _read_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                break
            buf += chunk
        return buf

    def radar(self):
        header = self._read_exact(self.HEADER_LEN)
        if len(header) < self.HEADER_LEN:
            return []
        data_len = struct.unpack_from("<I", header, 12)[0] - self.HEADER_LEN
        if data_len <= 0:
            return []
        data = self._read_exact(data_len)
        return self._parse_tlv(data)

    def _parse_tlv(self, data):
        points = []
        i = 0
        n = len(data)
        while i < n:
            if n - i <= 8:
                break
            tlv_type = struct.unpack_from("<I", data, i)[0]
            tlv_len = struct.unpack_from("<I", data, i + 4)[0]
            if tlv_type > 9 or tlv_len >= n:
                break
            i += 8
            end = i + tlv_len
            if tlv_type == 1:
                while i + 16 <= end:
                    x, y, z, v = struct.unpack_from("<ffff", data, i)
                    points.append([x, y, z, v])
                    i += 16
            i = end
        return points

    def close(self):
        for port in (self.ser, self.ctrl):
            if port is not None:
                try:
                    port.close()
                except Exception:
                    pass


class HwSource:
    """把可用的硬件子模块组合起来；每个子模块独立降级，缺失则该项返回 None。"""

    def __init__(self, notes, battery=None, smoke=None, radar=None):
        self._notes = notes
        self._battery = battery
        self._smoke = smoke
        self._radar = radar

    def battery(self):
        if self._battery is None:
            return None, None
        try:
            return self._battery.battery()
        except Exception as exc:
            self._notes.append("电量读取失败: %s" % exc)
            self._battery = None
            return None, None

    def smoke(self):
        if self._smoke is None:
            return None
        try:
            return self._smoke.smoke()
        except Exception as exc:
            self._notes.append("烟雾读取失败: %s" % exc)
            self._smoke = None
            return None

    def radar(self):
        if self._radar is None:
            return None
        try:
            return self._radar.radar()
        except Exception as exc:
            self._notes.append("雷达读取失败: %s" % exc)
            self._radar = None
            return None

    def thermal(self):
        return None                     # 见文件头 [未实现] 说明

    def close(self):
        if self._radar is not None:
            self._radar.close()


def read_device_id(explicit=None):
    """设备编号。注意 .strip()：run.py:54 少了它，会让 res[0] == dev_num 永不成立。"""
    if explicit:
        return explicit.strip()
    for path in ("/home/xzt/pysenxor/dev_num.txt", "dev_num.txt"):
        try:
            with open(path, "r", encoding="utf-8", errors="replace") as fh:
                value = fh.read().strip()
            if value:
                return value
        except OSError:
            continue
    return "RP-SIM-00000000"


def _try_sensor_hub(args, notes):
    """尝试用 hw_sensors.SensorHub 采集真实数据；不可用则返回 None。

    min.py 自带的 Hw* 类是可独立运行的最小实现（不含热成像），
    hw_sensors 才是完整硬件层（含 MI48），因此优先使用后者。
    radar_retries=2：避免串口缺失时按 smp.py 的无限重试挂住终端视图。
    """
    try:
        import hw_sensors
    except Exception as exc:
        notes.append("hw_sensors 模块不可用: %s" % exc)
        return None
    hub = hw_sensors.SensorHub(radar_profile_path=args.radar_profile,
                               radar_retries=2)
    try:
        hub.start()
    except Exception as exc:
        notes.append("hw_sensors 启动失败: %s" % exc)
        try:
            hub.close()
        except Exception:
            pass
        return None
    # 一个子系统都没起来时，视为硬件不可用，交回上层走仿真/报错。
    if not any((hub.thermal_cam, hub.radar_dev, hub.battery_dev, hub.smoke_dev)):
        notes.extend(hub.notes)
        try:
            hub.close()
        except Exception:
            pass
        return None
    hub.notes[:0] = notes
    hub._notes = hub.notes
    return hub


def open_source(args):
    """返回 (source, mode_note)。hardware 模式下按子模块尽力而为。"""
    if args.source == "sim":
        return SimSource(args.device_id, _parse_size(args.sim_size), args.seed), "仿真"

    notes = []

    hub = _try_sensor_hub(args, notes)
    if hub is not None:
        return hub, "硬件"

    battery = smoke = radar = None
    try:
        battery = HwBattery()
    except Exception as exc:
        notes.append("电量计不可用: %s" % exc)
    try:
        smoke = HwSmoke()
    except Exception as exc:
        notes.append("烟雾 GPIO 不可用: %s" % exc)
    try:
        radar = HwRadar(profile_path=args.radar_profile)
    except Exception as exc:
        notes.append("雷达串口不可用: %s" % exc)

    if battery is None and smoke is None and radar is None:
        if args.source == "hw":
            sys.stderr.write("--source hw 失败：没有任何可用硬件。\n")
            for note in notes:
                sys.stderr.write("  - %s\n" % note)
            sys.stderr.write("可改用 --source sim 查看终端视图。\n")
            raise SystemExit(2)
        sim = SimSource(args.device_id, _parse_size(args.sim_size), args.seed)
        sim._notes = notes
        return sim, "仿真（硬件不可用）"

    return HwSource(notes, battery, smoke, radar), "硬件"


def _parse_size(text):
    try:
        w, h = text.lower().split("x")
        return max(4, int(w)), max(3, int(h))
    except Exception:
        sys.stderr.write("--sim-size 需要 WxH 格式，例如 64x32\n")
        raise SystemExit(2)


# ============================================================ 4. 视图

class View:
    """ANSI 原地刷新；非 TTY 时自动退化为逐帧换行输出，避免转义码污染日志。"""

    def __init__(self, ansi=True, color=False):
        self.ansi = ansi
        self.color = color
        self._closed = False
        if ansi:
            sys.stdout.write("\033[?25l")          # 隐藏光标
            atexit.register(self.close)

    def close(self):
        if self.ansi and not self._closed:
            self._closed = True
            sys.stdout.write("\033[?25h\033[0m\n")
            sys.stdout.flush()

    def draw(self, lines):
        if self.ansi:
            # 每行补 \033[K：\033[J 只能清掉最后一行的残余。中间行若比上一帧短，
            # 旧字符会留在行尾（曾出现 df/free 残影，如 "... - ext4"、"10.3/28"）。
            sys.stdout.write("\033[H"
                             + "\n".join(line + "\033[K" for line in lines)
                             + "\033[J")
        else:
            sys.stdout.write("\n".join(lines) + "\n" + "-" * 60 + "\n")
        sys.stdout.flush()


def xy_map(xs, ys, cols, rows=11, xrng=10.0, yrng=10.0):
    """俯视示意图：x 向右，y 向上（与 smp.py:492 的视场范围一致）。"""
    cols = max(8, cols)
    grid = [[" "] * cols for _ in range(rows)]
    for x, y in zip(xs, ys):
        if not (-xrng <= x <= xrng and 0 <= y <= yrng):
            continue
        c = int((x + xrng) / (2 * xrng) * (cols - 1))
        r = int((yrng - y) / yrng * (rows - 1))
        grid[max(0, min(rows - 1, r))][max(0, min(cols - 1, c))] = "o"
    top = "+" + "-" * cols + "+"
    return [top] + ["|" + "".join(r) + "|" for r in grid] + [top]


def build_frame(source, mode_note, device_id, frame_no, width, color, ansi, notes):
    """组装一屏文本。所有数值来自 source，不做任何虚构。"""
    def paint(text, code):
        return "%s%s%s" % (code, text, _RESET) if ansi else text

    def safe(fn, default):
        """单次传感器读取失败不应终止整个视图：记录后按缺省值继续。"""
        try:
            return fn()
        except Exception as exc:
            notes.append("读取失败: %s" % exc)
            return default

    sep = "-" * max(40, width - 1)
    out = []

    out.append("智能船舶舱室监控 %s" % paint("· 终端视图", _DIM))
    out.append("设备 %s   来源 %s   帧 %d   %s"
               % (device_id, mode_note, frame_no, time.strftime("%H:%M:%S")))
    if mode_note.startswith("仿真"):
        out.append(paint("注意：以下为仿真数据，不是真实传感器读数。", _YELLOW))
    out.append(sep)

    # ---- TD1：电池 + 烟雾 ----
    voltage, capacity = safe(source.battery, (None, None))
    smoke = safe(source.smoke, None)
    if voltage is None:
        out.append("电池   未接入")
    else:
        filled = int(round(capacity / 100.0 * 24))
        bar = "[" + "=" * filled + "-" * (24 - filled) + "]"
        out.append("电池   %5.2f V   %5.1f%%   %s" % (voltage, capacity, bar))
    if smoke is None:
        out.append("烟雾   未接入")
    elif smoke == 0:
        out.append("烟雾   " + paint("报警（GPIO17 低电平）", _RED))
    else:
        out.append("烟雾   正常")
    out.append(sep)

    # ---- TD2：毫米波 ----
    points = safe(source.radar, None)
    if points is None:
        out.append("毫米波 未接入")
    else:
        centers = cluster_people(points, EPSILON, WEIGHT)
        xs = [c[0] for c in centers]
        ys = [c[1] for c in centers]
        coord = ", ".join("(%.2f, %.2f)" % (c[0], c[1]) for c in centers)
        out.append("毫米波 点数 %d   人数 %d   簇中心 %s"
                   % (len(points), len(centers), coord or "-"))
        out.extend(xy_map(xs, ys, min(width - 4, 60)))
    out.append(sep)

    # ---- TD3：热成像 ----
    frame = safe(source.thermal, None)
    if frame is None:
        out.append("热成像 未接入（真实采集需 senxor + MI48，见文件头说明）")
    else:
        lo, hi, mean, hot = thermal_stats(frame)
        out.append("热成像 尺寸 %d行x%d列   最低 %.1f°C   最高 %.1f°C   均温 %.1f°C   最热点 %s"
                   % (len(frame), len(frame[0]) if frame else 0, lo, hi, mean, hot))
        use_color = color and ansi
        out.append("       灰度 '%s'=冷  ->  '%s'=热%s"
                   % (RAMP[0], RAMP[-1], "（已启用 --color）" if use_color else ""))
        rows = min(len(frame), 18)
        cols = max(8, min(len(frame[0]), width - 6))
        span = (hi - lo) or 1.0
        for line in downsample(frame, rows, cols):
            if not use_color:
                out.append("       " + "".join(ramp_char(v, lo, span) for v in line))
                continue
            # 同色连续段合并：逐字符转义会为每格产生 2 个转义序列，
            # 合并后每帧可减少约一半输出量，避免慢速终端撕裂。
            parts = []
            run_idx = None
            run = []
            for v in line:
                idx = color_index(v, lo, span)
                if idx != run_idx and run:
                    parts.append("\033[38;5;%dm%s" % (_COLOR_256[run_idx], "".join(run)))
                    run = []
                run_idx = idx
                run.append(ramp_char(v, lo, span))
            if run:
                parts.append("\033[38;5;%dm%s" % (_COLOR_256[run_idx], "".join(run)))
            out.append("       " + "".join(parts) + _RESET)
    out.append(sep)

    seen = set()
    for note in notes:
        if note in seen:          # hw_sensors 与内置 Hw* 可能报同一条缺失原因
            continue
        seen.add(note)
        out.append(paint("! " + note, _DIM))
    out.append(paint("Ctrl+C 退出   （--frames N 可限定帧数）", _DIM))
    return out


# ============================================================ 5. 入口

# DBSCAN 参数：与 smp.py:28-32 一致。这是场景相关常量，
# 真机上应外置为配置项，此处为保持"最小"沿用同一取值。
R = 0.3
Vs = 0.5
WEIGHT = 1.0
EPSILON = R ** 2 + WEIGHT * Vs ** 2


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="最小终端监控视图（不依赖服务端/数据库）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="数据契约与 smp.py:556-558 的 TD1/TD2/TD3 一致。")
    parser.add_argument("--source", choices=("auto", "sim", "hw"), default="auto",
                        help="数据来源；auto=有硬件用硬件，否则仿真")
    parser.add_argument("--device-id", default=None, help="设备编号（默认读取 dev_num.txt）")
    parser.add_argument("--frames", type=int, default=0, help="渲染多少帧后退出，0=无限")
    parser.add_argument("--interval", type=float, default=0.75,
                        help="帧间隔秒；smp.py 的采集循环约 0.75s/轮")
    parser.add_argument("--sim-size", default="64x32",
                        help="仿真热图尺寸 WxH（任意值，不代表真实分辨率）")
    parser.add_argument("--seed", type=int, default=None, help="仿真随机种子，便于复现")
    parser.add_argument("--color", action="store_true", help="热图使用 256 色")
    parser.add_argument("--no-ansi", action="store_true", help="禁用 ANSI（非 TTY 自动启用）")
    parser.add_argument("--radar-profile", default=None,
                        help="雷达配置命令文件（每行一条），供 --source hw 使用")
    args = parser.parse_args(argv)

    args.device_id = read_device_id(args.device_id)
    source, mode_note = open_source(args)
    notes = list(getattr(source, "_notes", []))
    width = max(60, min(shutil.get_terminal_size((100, 40)).columns, 160))
    ansi = sys.stdout.isatty() and not args.no_ansi
    view = View(ansi=ansi, color=args.color)

    frame_no = 0
    try:
        while True:
            frame_no += 1
            view.draw(build_frame(source, mode_note, args.device_id,
                                  frame_no, width, args.color, ansi, notes))
            if args.frames and frame_no >= args.frames:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        close = getattr(source, "close", None)
        if close is not None:
            close()
        view.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
