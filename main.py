from time import sleep,time

import requests
import threading
import json

from utils import battery
from utils import smoke
from utils import mm_wave
from utils import thermal_camera

device_id=""
server_ip_port="127.0.0.1"


def init():
    global device_id,server_ip_port
    try:
        with open("data/device_id.txt","r") as f:
            device_id=f.read()
    except:
        pass

    if len(device_id)==0:
        try:
            with open('/proc/device-tree/serial-number', 'r') as f:
                serial = f.read().strip('\x00\n')
                device_id= serial
        except:
            pass

    try:
        with open("data/server_ip_port.txt","r") as f:
            server_ip_port=f.read()
    except:
        pass



class TerminalClient:
    def __init__(self, server_ip_port: str, device_id: str, cam: thermal_camera.ThermalCamera):
        self.id, self.cam = device_id, cam
        self.http = f"http://{server_ip_port}/api/terminal"
        self.ws_url = f"ws://{server_ip_port}/api/terminal/ws/{device_id}"

    def report(self, **fields) -> bool:
        """上报状态，只带有变化或需要更新的 key 即可；失败返回 False"""
        try:
            r = requests.post(f"{self.http}/report", json={"id": self.id, **fields}, timeout=3)
            return r.ok
        except requests.RequestException:
            return False

    def snapshot(self) -> bool:
        """上传一张热成像抓拍"""
        f = self.cam.frame()
        if f is None:
            return False
        try:
            r = requests.post(f"{self.http}/snapshot/{self.id}", params={"kind": "thermal"},
                              files={"file": ("t.jpg", thermal_camera.to_jpeg(f), "image/jpeg")}, timeout=5)
            return r.ok
        except requests.RequestException:
            return False

    def start_stream(self):
        threading.Thread(target=self._stream, daemon=True).start()

    def _stream(self):
        """WebSocket 推帧：默认 1 fps，按服务端 {"fps","ttl"} 指令调速，租约过期自动回落，断线重连"""
        while True:
            try:
                ws = thermal_camera.create_connection(self.ws_url, timeout=5)
                ws.settimeout(0.05)                   # recv 最多等 50 ms，同时作为发送节拍
                fps, lease, nxt = 1, time() + 30, 0.0
                while True:
                    try:
                        cmd = json.loads(ws.recv())
                        fps, lease = cmd["fps"], time() + cmd.get("ttl", 30)
                    except thermal_camera.WebSocketTimeoutException:
                        pass
                    now =time()
                    if now > lease:
                        fps = 1                       # 没收到续约，回落低帧率
                    f = self.cam.frame()
                    if f is not None and now >= nxt:
                        ws.send_binary(thermal_camera.pack_frame(f))
                        nxt = now + 1 / fps
            except Exception:
                sleep(3)


def main():
    init()
    connect_failure_cnt=0
    # mm_wave.init()
    # mm_wave.read()
    # mm_wave.cleanup()

    cam=thermal_camera.ThermalCamera(fps=1)
    client=TerminalClient(server_ip_port,device_id,cam)
    client.start_stream()
    input()
    # while True:
    #     bat=battery.read()
    #     smo=smoke.read()
    #     try:
    #         requests.post(f"http://{server_ip_port}/api/terminal/report",json={
    #             "id":device_id,
    #     "smokeAlert": not smo,
    #     "batteryPercent": bat["battery_percent"],
    #     })
    #         connect_failure_cnt=0
    #     except Exception as e:
    #         connect_failure_cnt=1
    #         print(e)
    #     sleep(1)
    #     sleep(4*connect_failure_cnt)

if __name__ == '__main__':
    main()