#!/usr/bin/env python3
import time
import spidev
from smbus2 import SMBus
from gpiozero import DigitalInputDevice, DigitalOutputDevice
from senxor.interfaces import I2C_Interface, SPI_Interface
from senxor.mi48 import MI48

# 1. 硬件接口初始化
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

# 2. 启动传感器
cam = MI48([i2c, spi], data_ready=drdy, reset_handler=reset)
cam.set_fps(15)
cam.start(stream=True, with_header=True)
time.sleep(1.5)

def get_frame():
    cs.on()
    time.sleep(0.0001)
    data, _ = cam.read()
    time.sleep(0.0001)
    cs.off()
    return data.reshape((62, 80))

# 3. 读取循环
try:
    while True:
        frame = get_frame()
        print(f"Max: {frame.max():.2f}°C | Min: {frame.min():.2f}°C | Center: {frame[31, 40]:.2f}°C")
        time.sleep(0.1)
finally:
    cam.stop()
    raw_spi.close()