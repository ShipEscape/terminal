import json
import threading
import time

import requests
from websocket import WebSocketTimeoutException, create_connection

LIVE_FPS = 8    # 网页查看时的热成像帧率
PING_S = 25     # 日常保活间隔：空闲时线程几乎不醒来


class Server:
    """与服务端的所有通信：状态上报、抓拍上传、WebSocket 长连接（推帧 + live 指令）"""

    def __init__(self, ip_port: str, device_id: str):
        self.id = device_id
        self.http = f"http://{ip_port}/api/terminal"
        self.ws_url = f"ws://{ip_port}/api/terminal/ws/{device_id}"
        self.session = requests.Session()
        self.lock = threading.Lock()        # 多个采样线程共用同一个 HTTP 连接
        self.ok = True
        self.on_live = lambda live: None    # 服务端 live 指令变化时的回调

    def _post(self, path: str, **kw) -> bool:
        with self.lock:
            try:
                self.ok = self.session.post(f"{self.http}/{path}", timeout=5, **kw).ok
            except requests.RequestException as e:
                self.ok = False
                print(e)
            return self.ok

    def report(self, **fields) -> bool:
        return self._post("report", json={"id": self.id, **fields})

    def snapshot(self, jpeg: bytes | None, kind: str = "thermal") -> bool:
        return bool(jpeg) and self._post(f"snapshot/{self.id}", params={"kind": kind},
                                         files={"file": ("snap.jpg", jpeg, "image/jpeg")})

    def start_stream(self, get_frame):
        threading.Thread(target=self._stream, args=(get_frame,), daemon=True).start()

    def _stream(self, get_frame):
        """长连接：空闲时只睡眠并保活；服务端下发 live=true 后才按 LIVE_FPS 推帧"""
        while True:
            try:
                ws = create_connection(self.ws_url, timeout=5)
                live, lease = False, 0.0
                while True:
                    ws.settimeout(1 / LIVE_FPS if live else PING_S)   # 超时即帧节拍 / 保活节拍
                    try:
                        cmd = json.loads(ws.recv())                   # {"live": bool, "ttl": 秒}
                        live, lease = cmd["live"], time.time() + cmd.get("ttl", 30)
                        self.on_live(live)
                    except WebSocketTimeoutException:
                        if not live:
                            ws.ping()
                    if live and time.time() > lease:                  # 没收到续约，自动回落
                        live = False
                        self.on_live(False)
                    if live and (data := get_frame()):
                        ws.send_binary(data)
            except Exception as e:
                print(e)
                self.on_live(False)
                time.sleep(3)
