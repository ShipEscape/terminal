import time
from gpiozero import DigitalInputDevice, DigitalOutputDevice

# 引脚定义 (BCM 编码)
SMOKE_EN_PIN = 26  # 传感器电源使能端 (常开高电平)
SMOKE_OUT_PIN = 17  # 传感器信号输出端 (读状态)

# 1. 传感器使能端：拉高以给传感器供电/激活
smoke_en = DigitalOutputDevice(SMOKE_EN_PIN, active_high=True, initial_value=True)

# 2. 传感器输出端：配置内部上拉输入
smoke_sensor = DigitalInputDevice(SMOKE_OUT_PIN, pull_up=True)


def read_smoke() -> int:
    """读取烟雾传感器状态

    返回:
        1: 正常 (无烟雾)
        0: 报警 (检测到烟雾)
    """
    return smoke_sensor.value


# ==================== 测试与使用 ====================
if __name__ == "__main__":
    print("烟雾传感器已初始化并通电...")

    try:
        while True:
            state = read_smoke()
            # 在这里写你自己的处理逻辑：
            # state == 1 为正常，state == 0 为有烟雾
            print(f"当前烟雾状态: {state}")
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        smoke_en.off()  # 退出时关闭传感器供电
        smoke_en.close()
        smoke_sensor.close()