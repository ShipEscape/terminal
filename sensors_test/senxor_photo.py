#!/usr/bin/env python3
import time
import cv2
import spidev
from smbus2 import SMBus
from gpiozero import DigitalInputDevice, DigitalOutputDevice
from senxor.interfaces import I2C_Interface, SPI_Interface
from senxor.mi48 import MI48

# 1. 硬件初始化
i2c = I2C_Interface(SMBus(1), 0x40)

raw_spi = spidev.SpiDev()
raw_spi.open(0, 0)
raw_spi.mode = 0
raw_spi.max_speed_hz = 31200000
spi = SPI_Interface(raw_spi, 160)

cs = DigitalOutputDevice("BCM7", active_high=False, initial_value=False)
rst = DigitalOutputDevice("BCM23", active_high=False, initial_value=True)
drdy = DigitalInputDevice("BCM24", pull_up=False)

def reset():
    rst.on()
    time.sleep(0.0001)
    rst.off()
    time.sleep(0.05)

# 2. 启动传感器并预热
cam = MI48([i2c, spi], data_ready=drdy, reset_handler=reset)
cam.set_fps(15)
cam.start(stream=True, with_header=True)

# 3. 核心优化：动态快速过滤废帧，拿到第一帧真数据秒退
data = None
timeout = time.time() + 2.0  # 2秒超时防卡死

while time.time() < timeout:
    cs.on()
    time.sleep(0.0001)
    data, _ = cam.read()
    time.sleep(0.0001)
    cs.off()

    # 只要最高温大于 -200°C，说明有效数据已到达，立即跳出！
    if data is not None and data.max() > -200:
        break
    
    time.sleep(0.03)  # 等待下一帧刷新 (25fps 下每帧约 40ms)

cam.stop()
raw_spi.close()

frame = data.reshape((62, 80))

# 4. 生成热力图图片 (归一化 -> 伪彩色 -> 放大到 800x620)
norm = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
color_img = cv2.applyColorMap(norm, cv2.COLORMAP_JET)

# 5. 保存到当前目录
filename = "thermal_photo.jpg"
cv2.imwrite(filename, color_img)
print(f"拍照成功！已保存为: {filename} (最高温: {frame.max():.2f}°C, 最低温: {frame.min():.2f}°C)")