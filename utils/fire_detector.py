import time
from collections import deque

import numpy as np

MAX_TEMP = 300.0  # 与训练脚本的归一化基准一致


class FireDetector:
    def __init__(self, model_path: str, t_abs=60.0, delta=15.0, votes=(3, 5), hold_s=30.0, eval_hz=2.0):
        self.t_abs, self.delta, self.hold_s = t_abs, delta, hold_s
        self.need, window = votes
        self._gap = 1 / eval_hz
        self._hist = deque(maxlen=window)
        self._bg = None
        self._last_eval = self._last_hit = 0.0
        self.alert = False
        self._sess = None
        try:
            import onnxruntime as ort  # 延迟导入，并限制为单线程：模型很小，省电
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 1
            self._sess = ort.InferenceSession(model_path, opts, providers=["CPUExecutionProvider"])
        except Exception as e:
            print(f"火情模型不可用：{e}")

    def _cnn(self, f: np.ndarray) -> bool:
        x = (np.clip(f, 0, MAX_TEMP) / MAX_TEMP).astype(np.float32)[None, None]
        return bool(self._sess.run(None, {"input": x})[0][0, 0] > 0)  # logit > 0 即 sigmoid > 0.5

    def update(self, f: np.ndarray) -> bool:
        now = time.monotonic()
        if now - self._last_eval < self._gap:
            return self.alert
        self._last_eval = now
        if self._bg is None:
            self._bg = f.copy()
        hot = f > self.t_abs
        cand = hot & (f - self._bg > self.delta)  # 又热、又比背景明显升温
        cool = ~hot
        self._bg[cool] += 0.01 * (f[cool] - self._bg[cool])  # 背景只从不热的像素学习，慢慢跟随环境温度
        hit = bool(cand.any()) and (self._sess is None or self._cnn(f))  # 没有候选点就不跑 CNN
        self._hist.append(hit)
        if sum(self._hist) >= self.need:
            self._last_hit, self.alert = now, True
        elif self.alert and now - self._last_hit > self.hold_s:
            self.alert = False
        return self.alert
