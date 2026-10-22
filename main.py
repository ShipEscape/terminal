import numpy as np

from utils import battery, mm_wave, smoke, thermal_camera
from utils.fire_detector import FireDetector
from utils.hub import Hub
from utils.server import Server

device_id = ""
server_ip_port = "127.0.0.1:8080"

# 采样间隔(日常, live)
SMOKE_S = (2, 1)
BATTERY_S = (60, 60)
MMWAVE_S = (60, 1)  # 日常：开机,测一次,关机；live：保持开机，每秒测一次
THERMAL_S = (60, 1)  # 摄像头常开；这里只切换读帧节奏、上报统计值、告警抓拍


def read_file(path: str) -> str:
    try:
        with open(path) as f:
            return f.read().strip("\x00\n ")
    except OSError:
        return ""


def init():
    global device_id, server_ip_port
    device_id = read_file("data/device_id.txt") or read_file("/proc/device-tree/serial-number")
    server_ip_port = read_file("data/server_ip_port.txt") or server_ip_port


def main():
    init()
    server = Server(server_ip_port, device_id)
    hub = Hub(server)
    detector = FireDetector("data/fire_model.onnx")
    # smoke.init()
    state = {"mm_on": False, "snapped": False}

    def on_frame(f):  # 在读帧线程里调用，必须很快
        alert = detector.update(f)
        if alert != hub.alert:
            hub.put({"fireAlert": alert})
            hub.set_alert(alert)  # 立刻上报，并切到 live 节奏（毫米波开机数人）

    cam = thermal_camera.ThermalCamera(on_frame=on_frame)

    def sample_smoke(live):
        alert = not smoke.read()
        print(f'[烟雾] {alert}')
        hub.put({"smokeAlert": alert})
        hub.set_alert(alert)  # 告警变化：立刻上报，并切到 live 节奏
        return {}

    def sample_battery(live):
        bat = battery.read()
        return {"isAc": bat["is_ac"], "voltage": bat["voltage"], "batteryPercent": bat["battery_percent"]}

    def sample_mmwave(live):
        try:
            if not state["mm_on"]:
                mm_wave.init()
                state["mm_on"] = True
                mm_wave.read()  # 刚开机的第一次读数丢弃
            mm = mm_wave.read()
            if not live:
                mm_wave.cleanup()  # 日常：测完就关
                state["mm_on"] = False
        except Exception:
            state["mm_on"] = False  # 出错就关掉，下个周期重新开机
            try:
                mm_wave.cleanup()
            except Exception:
                pass
            raise
        return {"personCnt": mm["person_cnt"], "dotCnt": mm["dot_cnt"], "persons": mm["persons"]}

    def sample_thermal(live):
        cam.set_live(live)  # 切换读帧节奏
        if hub.alert and not state["snapped"]:
            server.snapshot(cam.jpeg())  # 告警上升沿抓拍一次
        state["snapped"] = hub.alert
        return {**cam.stats(), "fireAlert": hub.alert}

    server.start_stream(cam.packed)
    # hub.add(sample_smoke, *SMOKE_S)
    cam.start()
    hub.add(sample_battery, *BATTERY_S)
    hub.add(sample_mmwave, *MMWAVE_S)
    hub.add(sample_thermal, *THERMAL_S)
    try:
        hub.run()
    finally:
        cam.stop()
        if state["mm_on"]:
            mm_wave.cleanup()
        # smoke.cleanup()


if __name__ == "__main__":
    main()
