"""
Етап 4 — конвеєр обробки: камера -> YOLO -> анотований кадр.

Архітектурне рішення, заради якого це окремий модуль:

    ESP32 ──> [ОДИН читач] ──> [ОДИН прохід YOLO] ──> останній анотований кадр
                                                            │
                                              ┌─────────────┼─────────────┐
                                          браузер        телефон       API

Спокуса зробити інакше — дати кожному HTTP-клієнту свій MjpegCamera і свій
виклик моделі. Це працює, але масштабується найгіршим можливим чином:
два глядачі = два потоки з камери (а вони ділять її пропускну здатність,
див. docs/benchmarks.md) і подвійна робота моделі над тим самим відео.

Тут навпаки: конвеєр крутиться в одному потоці незалежно від того, скільки
глядачів підключено, і навіть якщо їх нуль. Глядачі лише забирають останній
готовий кадр. Ціна одного глядача — тільки віддача байтів у мережу.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field

import cv2

from camera import MjpegCamera


@dataclass
class PipelineStats:
    recv_fps: float = 0.0
    process_fps: float = 0.0
    infer_ms: float = 0.0
    detections: dict[str, int] = field(default_factory=dict)
    frames_processed: int = 0
    viewers: int = 0
    camera_connected: bool = False
    camera_error: str = ""

    def as_dict(self) -> dict:
        return {
            "recv_fps": round(self.recv_fps, 1),
            "process_fps": round(self.process_fps, 1),
            "infer_ms": round(self.infer_ms, 1),
            "detections": self.detections,
            "frames_processed": self.frames_processed,
            "viewers": self.viewers,
            "camera_connected": self.camera_connected,
            "camera_error": self.camera_error,
        }


class DetectionPipeline:
    """
    Крутить камеру й модель в окремому потоці, тримає останній анотований кадр
    уже закодований у JPEG — щоб кодувати один раз на кадр, а не один раз
    на кожного глядача.
    """

    def __init__(
        self,
        url: str,
        model,
        *,
        conf: float = 0.35,
        iou: float = 0.45,
        imgsz: int = 640,
        classes: list[int] | None = None,
        device: str | None = None,
        jpeg_quality: int = 80,
        draw_fn=None,
        summarize_fn=None,
    ) -> None:
        self.cam = MjpegCamera(url)
        self.model = model
        self.names = model.names
        self.conf = conf
        self.iou = iou
        self.imgsz = imgsz
        self.classes = classes
        self.device = device
        self.jpeg_quality = jpeg_quality
        self._draw = draw_fn
        self._summarize = summarize_fn

        self.stats = PipelineStats()

        self._jpeg: bytes | None = None
        self._generation = 0
        self._cond = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._viewer_lock = threading.Lock()

    # -- життєвий цикл ------------------------------------------------------

    def start(self) -> "DetectionPipeline":
        self.cam.start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="pipeline", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5.0)
            self._thread = None
        self.cam.stop()

    # -- для глядачів -------------------------------------------------------

    def wait_for_frame(self, last_seen: int, timeout: float = 5.0):
        """
        Блокується, доки не з'явиться кадр, новіший за last_seen.
        Повертає (jpeg_bytes, generation) або (None, last_seen) при таймауті.

        Глядач передає номер останнього побаченого кадру, тому повільний
        глядач просто пропускає кадри, а не гальмує конвеєр.
        """
        with self._cond:
            if self._generation == last_seen:
                self._cond.wait(timeout)
            if self._jpeg is None or self._generation == last_seen:
                return None, last_seen
            return self._jpeg, self._generation

    def snapshot(self) -> bytes | None:
        with self._cond:
            return self._jpeg

    def viewer_joined(self) -> None:
        with self._viewer_lock:
            self.stats.viewers += 1

    def viewer_left(self) -> None:
        with self._viewer_lock:
            self.stats.viewers = max(0, self.stats.viewers - 1)

    # -- внутрішнє ----------------------------------------------------------

    def _publish(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._generation += 1
            self._cond.notify_all()

    def _loop(self) -> None:
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        fps_t0, fps_n = time.monotonic(), 0

        while not self._stop.is_set():
            frame = self.cam.read(timeout=1.0)

            self.stats.camera_connected = self.cam.stats.connected
            self.stats.camera_error = self.cam.stats.last_error
            self.stats.recv_fps = self.cam.stats.recv_fps

            if frame is None:
                continue

            r = self.model.predict(
                frame,
                conf=self.conf,
                iou=self.iou,
                imgsz=self.imgsz,
                classes=self.classes,
                device=self.device,
                verbose=False,
            )[0]

            annotated = frame.copy()
            if self._draw is not None:
                self._draw(annotated, r.boxes, self.names)

            ok, buf = cv2.imencode(".jpg", annotated, encode_params)
            if ok:
                self._publish(buf.tobytes())

            self.stats.infer_ms = r.speed.get("inference", 0.0)
            self.stats.frames_processed += 1
            if self._summarize is not None:
                counts: dict[str, int] = {}
                for box in r.boxes:
                    name = self.names.get(int(box.cls[0]), "?")
                    counts[name] = counts.get(name, 0) + 1
                self.stats.detections = counts

            fps_n += 1
            elapsed = time.monotonic() - fps_t0
            if elapsed >= 1.0:
                self.stats.process_fps = fps_n / elapsed
                fps_n, fps_t0 = 0, time.monotonic()

        self.stats.process_fps = 0.0
