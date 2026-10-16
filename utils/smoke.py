from gpiozero import DigitalInputDevice, DigitalOutputDevice

# 引脚定义
SMOKE_ENABLE_PIN = 26  # 烟雾传感器电源
SMOKE_OUT_PIN = 17  # 传感器信号输出端(无烟为高电平)


def read() -> bool:
    with DigitalOutputDevice(SMOKE_ENABLE_PIN, active_high=True, initial_value=True) as smoke_enable, DigitalInputDevice(SMOKE_OUT_PIN, pull_up=True) as smoke_sensor:
        return bool(smoke_sensor.value)