import struct
import threading
import time

import cv2
import numpy as np
import spidev
from gpiozero import DigitalInputDevice, DigitalOutputDevice
from smbus2 import SMBus
from senxor.interfaces import I2C_Interface, SPI_Interface
from senxor.mi48 import MI48

H, W = 62, 80   # 62 行 × 80 列


class ThermalCamera:
    """MI48 热成像：后台线程持续取帧，随时读取最新一帧"""

    def __init__(self, fps: int = 15):
        i2c = I2C_Interface(SMBus(1), 0x40)
        self._raw_spi = spidev.SpiDev()
        self._raw_spi.open(0, 0)
        self._raw_spi.mode = 0
        self._raw_spi.max_speed_hz = 31200000
        spi = SPI_Interface(self._raw_spi, 160)
        self._cs = DigitalOutputDevice("BCM7", active_high=False, initial_value=False)
        self._rst = DigitalOutputDevice("BCM23", active_high=False, initial_value=True)
        drdy = DigitalInputDevice("BCM24", pull_up=False)

        self._cam = MI48([i2c, spi], data_ready=drdy, reset_handler=self._reset)
        self._cam.set_fps(fps)
        self._cam.start(stream=True, with_header=True)

        self._frame = None
        self._lock = threading.Lock()
        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _reset(self):
        self._rst.on()
        time.sleep(0.0001)
        self._rst.off()
        time.sleep(0.05)

    def _loop(self):
        while self._running:
            try:
                self._cs.on()
                time.sleep(0.0001)
                data, _ = self._cam.read()
                time.sleep(0.0001)
                self._cs.off()
                if data is not None and data.max() > -200:      # 过滤开机废帧
                    with self._lock:
                        self._frame = data.reshape((H, W)).astype(np.float32)
                time.sleep(0.01)
            except Exception:
                time.sleep(0.1)

    def frame(self):
        """最新一帧 (62, 80) float32 ℃，还没有有效帧时返回 None"""
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def stats(self) -> dict:
        """热成像统计值，字段名与服务端一致，可直接并入上报数据"""
        f = self.frame()
        if f is None:
            return {}
        return {"thermalMax": round(float(f.max()), 2),
                "thermalMin": round(float(f.min()), 2),
                "thermalCenter": round(float(f[H // 2, W // 2]), 2)}

    def close(self):
        self._running = False
        self._cam.stop()
        self._raw_spi.close()


def pack_frame(f: np.ndarray) -> bytes:
    """WebSocket 帧：<HHff 头(宽,高,最低温,最高温) + uint8 像素，约 5 KB"""
    lo, hi = float(f.min()), float(f.max())
    q = ((f - lo) / max(hi - lo, 0.01) * 255).astype(np.uint8)
    return struct.pack("<HHff", f.shape[1], f.shape[0], lo, hi) + q.tobytes()


def to_jpeg(f: np.ndarray, scale: int = 8) -> bytes:
    """伪彩色热力图 JPEG 字节（80×62 放大 8 倍），只在内存里生成"""
    norm = cv2.normalize(f, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    img = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
    img = cv2.resize(img, (f.shape[1] * scale, f.shape[0] * scale), interpolation=cv2.INTER_CUBIC)
    return cv2.imencode(".jpg", img)[1].tobytes()