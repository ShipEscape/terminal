import atexit
import struct
import time
import serial

CLI_PORT, DATA_PORT = "/dev/radar_uart_0", "/dev/radar_uart_1"
EPSILON, WEIGHT = 0.4, 1.0
MAGIC_WORD = b"\x02\x01\x04\x03\x06\x05\x08\x07"

RADAR_CONFIG = """sensorStop
flushCfg
dfeDataOutputMode 1
channelCfg 15 7 0
adcCfg 2 1
adcbufCfg -1 0 1 1 1
profileCfg 0 60 359 7 57.14 0 0 70 1 256 5209 0 0 158
chirpCfg 0 0 0 0 0 0 0 1
chirpCfg 1 1 0 0 0 0 0 2
chirpCfg 2 2 0 0 0 0 0 4
frameCfg 0 2 16 0 100 1 0
lowPower 0 0
guiMonitor -1 1 1 0 0 0 1
cfarCfg -1 0 2 8 4 3 0 15 1
cfarCfg -1 1 0 4 2 3 1 15 1
multiObjBeamForming -1 1 0.5
clutterRemoval -1 1
calibDcRangeSig -1 0 -5 8 256
extendedMaxVelocity -1 0
lvdsStreamCfg -1 0 0 0
compRangeBiasAndRxChanPhase 0.0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0 1 0 -1 0
measureRangeBiasAndRxChanPhase 0 1.5 0.2
CQRxSatMonitor 0 3 5 121 0
CQSigImgMonitor 0 127 4
analogMonitor 0 0
aoaFovCfg -1 -90 90 -90 90
cfarFovCfg -1 0 0 8.92
cfarFovCfg -1 1 -1 1.00
calibData 0 0 0
sensorStart"""


def cluster_people(pts, eps=EPSILON, w=WEIGHT):
    if not pts:
        return []
    n, min_nb = len(pts), 1 if len(pts) <= 10 else 2
    adj = [
        [
            j
            for j, q in enumerate(pts)
            if (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + w * (p[3] - q[3]) ** 2
            <= eps
        ]
        for p in pts
    ]

    visited, labels, cid = set(), [0] * n, 0
    for i in range(n):
        if i in visited:
            continue
        visited.add(i)
        if len(adj[i]) - 1 < min_nb:
            continue
        cid += 1
        labels[i], seeds, seeds_set = cid, list(adj[i]), set(adj[i])
        for p in seeds:
            if p not in visited:
                visited.add(p)
                if len(adj[p]) - 1 >= min_nb:
                    for nb in adj[p]:
                        if nb not in seeds_set:
                            seeds.append(nb)
                            seeds_set.add(nb)
            if labels[p] == 0:
                labels[p] = cid

    centers = []
    for c in range(1, cid + 1):
        c_pts = [pts[i] for i, l in enumerate(labels) if l == c]
        cx = round(sum(p[0] for p in c_pts) / len(c_pts), 3)
        cy = round(sum(p[1] for p in c_pts) / len(c_pts), 3)
        if -10 < cx < 10 and cy < 10:
            centers.append([cx, cy, 0.0])
    return centers


def read_frame(ser):
    if not ser.read_until(MAGIC_WORD).endswith(MAGIC_WORD):
        return None
    head = ser.read(32)
    if len(head) < 32:
        return None

    payload_len = struct.unpack_from("<I", head, 4)[0] - 40
    if payload_len <= 0:
        return []
    payload = ser.read(payload_len)
    if len(payload) < payload_len:
        return []

    points, idx = [], 0
    while idx + 8 <= payload_len:
        t, l = struct.unpack_from("<II", payload, idx)
        if t == 1:
            points.extend(
                list(p)
                for p in struct.iter_unpack("<ffff", payload[idx + 8 : idx + 8 + l])
            )
        idx += 8 + l
    return points


data_ser=None
cli_ser=None

"""初始化雷达"""
def init():
    global data_ser,cli_ser
    cli_ser = serial.Serial(CLI_PORT, 115200, timeout=0.1)
    data_ser = serial.Serial(DATA_PORT, 921600, timeout=0.2)

    cli_ser.reset_input_buffer()
    data_ser.reset_input_buffer()

    # 3. 命令间隔压缩至 10ms，启动耗时从 1.5s 降到 0.3s
    for cmd in RADAR_CONFIG.splitlines():
        cli_ser.write(f"{cmd}\r".encode())
        time.sleep(0.01)

    data_ser.reset_input_buffer()  # 再次清空配置期间产生的无用数据

    atexit.register(cleanup)

"""读取雷达数据"""
def read():
    global data_ser
    points = read_frame(data_ser)
    if points is not None:
        people = cluster_people(points)
        info = (
            f"人员坐标: {people}" if people else "无人"
        )
        print(
            f"【当前人数: {len(people)}】 {info} (原始点数: {len(points)})"
        )

"""清理"""
def cleanup():
    global data_ser,cli_ser
    try:
        cli_ser.write(b"sensorStop\r")
        time.sleep(0.05)  # 等待雷达硬件射频关闭
    except:
        pass
    data_ser.reset_input_buffer()  # 强制丢弃驱动积压数据，避免底层 close() 阻塞
    cli_ser.close()
    data_ser.close()
