import threading

IDLE_REPORT_S, LIVE_REPORT_S = 60, 1  # 合并上报间隔


class Hub:
    """采样调度：每类设备一个线程，各有日常和live两档间隔，数据合并后统一上报"""

    def __init__(self, server):
        self.server, self.viewing, self.alert = server, False, False
        self._data, self._lock = {}, threading.Lock()
        self._wakes: list[threading.Event] = []
        self._tick = threading.Event()
        server.on_live = self.set_viewing

    @property
    def live(self) -> bool:
        return self.viewing or self.alert

    def add(self, sample, idle_s: float, live_s: float):
        wake = threading.Event()
        self._wakes.append(wake)

        def run():
            while True:
                try:
                    self.put(sample(self.live))
                except Exception as e:
                    print(e)
                wake.wait(live_s if self.live else idle_s)
                wake.clear()

        threading.Thread(target=run, daemon=True).start()

    def put(self, data: dict):
        with self._lock:
            self._data.update(data)

    def set_viewing(self, v: bool):
        self._change(viewing=v)

    def set_alert(self, a: bool):
        self._change(alert=a)

    def _change(self, **kw):
        changed = any(getattr(self, k) != v for k, v in kw.items())
        before = self.live
        self.__dict__.update(kw)
        if self.live != before:
            for w in self._wakes:
                w.set()  # 立刻切换各设备的采样节奏
        if changed:
            self._tick.set()  # 立刻上报

    def flush(self):
        with self._lock:
            data, self._data = self._data, {}
        if data and not self.server.report(**data):
            with self._lock:
                self._data = {**data, **self._data}  # 失败则保留，下次补发

    def run(self):
        """主线程：定时合并上报"""
        self._tick.wait(8)  # 开机先等各传感器出第一批数据
        while True:
            self.flush()
            wait = (LIVE_REPORT_S if self.live else IDLE_REPORT_S) if self.server.ok else 5
            self._tick.wait(wait)
            self._tick.clear()
