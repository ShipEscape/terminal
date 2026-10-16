import gpiod
from gpiod.line import Direction, Value
from smbus2 import SMBus

I2C_ADDR = 0x36

def read_word(bus, reg):
    w = bus.read_word_data(I2C_ADDR, reg)
    return ((w & 0xFF) << 8) | (w >> 8)


def read():
    # 读取电压与电量百分比
    with SMBus(1) as bus:
        voltage = round((read_word(bus, 0x02) >> 4) * 0.00125, 2)
        percent = round(min(100.0, read_word(bus, 0x04) / 256.0), 1)

    # 读取供电状态
    with gpiod.request_lines(
        "/dev/gpiochip4",
        consumer="x120x",
        config={6: gpiod.LineSettings(direction=Direction.INPUT)}
    ) as pld:
        is_ac = (pld.get_value(6) == Value.ACTIVE)


    data = {
        "is_ac": is_ac,
        "voltage": voltage,
        "battery_percent": percent
    }

    return data