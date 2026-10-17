import struct
import threading
import time
import traceback
import os,random

import numpy as np
import spidev
from gpiozero import DigitalInputDevice, DigitalOutputDevice
from senxor.interfaces import I2C_Interface, SPI_Interface
from senxor.mi48 import MI48
from smbus2 import SMBus

H, W = 62, 80
PATROL_S, LIVE_S = 0.5, 0.1  # 读帧间隔：日常巡检 / live（网页查看或告警）
STALL_S = 10  # 超过这么久没有有效帧，整体重开传感器


class ThermalCamera:
    """MI48 热成像：常开。后台线程按节奏读帧、每帧回调 on_frame，并监护硬件
    （STALL_S 秒没有有效帧就完全释放后重新开机）。"""

    def __init__(self, fps: int = 15, on_frame=None):
        self._fps, self._on_frame = fps, on_frame
        self._frame, self._last_ok = None, 0.0
        self._running, self._thread = False, None
        self._cam = self._cs = None
        self._res = []  # 释放函数，按创建顺序登记、反向执行
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._period = PATROL_S

    # ---------- 生命周期 ----------
    def start(self):
        if self._thread:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._wake.set()
        if self._thread:
            self._thread.join()
            self._thread = None

    def set_live(self, live: bool):
        """切换读帧节奏，立刻生效"""
        self._period = LIVE_S if live else PATROL_S
        self._wake.set()

    def _run(self):
        while self._running:
            try:
                self._open()
                while self._running and time.time() - self._last_ok < STALL_S:
                    self._grab()
                    self._wake.wait(self._period)
                    self._wake.clear()
                if self._running:
                    print(f"热成像 {STALL_S} 秒没有有效帧，重新开机")
            except Exception:
                traceback.print_exc()
            finally:
                self._release()
            if self._running:
                time.sleep(5)  # 退避后整体重开

    def _open(self):
        bus = SMBus(1)
        self._res.append(bus.close)
        raw_spi = spidev.SpiDev()
        raw_spi.open(0, 0)
        self._res.append(raw_spi.close)
        raw_spi.mode = 0
        raw_spi.max_speed_hz = 31200000
        self._cs = DigitalOutputDevice("BCM7", active_high=False, initial_value=False)
        self._res.append(self._cs.close)
        rst = DigitalOutputDevice("BCM23", active_high=False, initial_value=True)
        self._res.append(rst.close)
        drdy = DigitalInputDevice("BCM24", pull_up=False)
        self._res.append(drdy.close)

        def reset():
            rst.on()
            time.sleep(0.0001)
            rst.off()
            time.sleep(0.05)

        self._cam = MI48([I2C_Interface(bus, 0x40), SPI_Interface(raw_spi, 160)],
                         data_ready=drdy, reset_handler=reset)
        self._cam.set_fps(self._fps)  # 传感器帧率保持高于读取速率，已验证稳定
        self._res.append(self._cam.stop)
        self._cam.start(stream=True, with_header=True)
        self._frame, self._last_ok = None, time.time()  # 给开机预热留出 STALL_S 的时间

    def _release(self):
        for fn in reversed(self._res):
            try:
                fn()
            except Exception:
                pass
        self._res, self._cam, self._cs = [], None, None

    def _grab(self):
        data = None
        try:
            self._cs.on()
            time.sleep(0.0001)
            data, _ = self._cam.read()
            time.sleep(0.0001)
        except Exception:
            pass  # 偶发读错忽略，持续出错由 STALL_S 看门狗兜底
        finally:
            self._cs.off()
        if data is None or data.max() <= -200:  # 过滤开机废帧
            return
        f = data.reshape((H, W)).astype(np.float32)



        # ================= 模拟火灾 =================
        if os.path.exists("/tmp/sim_fire"):
            y, x = np.ogrid[:H, :W]
            # 1. 动态中心位置微动 (模拟气流扰动)
            cx = (W // 2) + random.uniform(-0.5, 0.5)
            cy = (H // 2) + random.uniform(-0.5, 0.5)
            # 2. 核心温度闪烁 (220℃ ~ 260℃ 动态跳动)
            t_core = 70.0 + random.uniform(-20.0, 20.0)
            # 3. 羽流形态抖动
            sigma_x = random.uniform(0.5, 1.2)
            sigma_y = random.uniform(0.5, 2.3)

            dist = ((x - cx) ** 2) / (2 * sigma_x**2) + ((y - cy) ** 2) / (
                2 * sigma_y**2
            )
            fire = (t_core * np.exp(-dist)).astype(np.float32)

            # 叠加到原始画面中
            f = np.maximum(f, fire)
        # =============================================



        with self._lock:
            self._frame = f
        self._last_ok = time.time()
        if self._on_frame:
            try:
                self._on_frame(f)
            except Exception:
                traceback.print_exc()

    # ---------- 读取 ----------
    def age(self) -> float:
        """距上一帧有效数据的秒数"""
        return time.time() - self._last_ok

    def frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def stats(self) -> dict:
        f = self.frame()
        if f is None:
            return {}
        return {"thermalAvg": round(float(f.mean()), 2),
                "thermalMax": round(float(f.max()), 2),
                "thermalMin": round(float(f.min()), 2),
                "thermalCenter": round(float(f[H // 2, W // 2]), 2)}

    def packed(self) -> bytes | None:
        """WebSocket 推流用的二进制帧；数据过期（传感器卡住）时不发送"""
        f = self.frame()
        return None if f is None or self.age() > 2 else pack_frame(f)

    def jpeg(self) -> bytes | None:
        f = self.frame()
        return None if f is None else to_jpeg(f)


def pack_frame(f: np.ndarray) -> bytes:
    """<HHff 头(宽,高,最低温,最高温) + uint8 像素，约 5 KB"""
    lo, hi = float(f.min()), float(f.max())
    q = ((f - lo) / max(hi - lo, 0.01) * 255).astype(np.uint8)
    return struct.pack("<HHff", f.shape[1], f.shape[0], lo, hi) + q.tobytes()


def to_jpeg(f: np.ndarray, scale: int = 1) -> bytes:
    """伪彩色 JPEG。默认不放大（80×62，约 2 KB），由网页端平滑放大"""
    import cv2  # 延迟导入：不抓拍就不占这部分内存
    norm = cv2.normalize(f, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    img = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    if scale > 1:
        img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    return cv2.imencode(".jpg", img)[1].tobytes()
