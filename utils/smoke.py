from gpiozero import DigitalInputDevice, DigitalOutputDevice

# 引脚定义
SMOKE_ENABLE_PIN = 26  # 烟雾传感器电源
SMOKE_OUT_PIN = 17  # 传感器信号输出端(无烟为高电平)

_smoke_enable = None
_cnt=0
_init_finished=False # 刚启动时可能错误获取低电平，导致火灾误报，故增加初始化判断

def init():
    global _smoke_enable,_cnt,_init_finished
    _smoke_enable=DigitalOutputDevice(SMOKE_ENABLE_PIN, active_high=True, initial_value=True)
    _cnt=0
    _init_finished=False


def read():
    global _cnt,_init_finished
    result=True
    with DigitalInputDevice(SMOKE_OUT_PIN, pull_up=True) as smoke_sensor:
        result= bool(smoke_sensor.value)
    if _init_finished: return result

    if result or _cnt>30: _init_finished=True
    else: 
        _cnt+=1
        result = True
    return result



def cleanup():
    global _smoke_enable
    _smoke_enable.off()
    _smoke_enable.close()
    _smoke_enable = None