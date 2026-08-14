"""
Етап 2 — прийом MJPEG-потоку з ESP32-S3 CAM.

Чому не просто `cv2.VideoCapture(url)`?
Можна й так, у два рядки. Але VideoCapture тримає всередині чергу кадрів:
якщо твій код обробляє кадри повільніше, ніж камера їх шле (а YOLO саме такий),
черга росте, і ти дивишся дедалі старіше відео. Через хвилину затримка
буде 10+ секунд. Це та сама проблема, яку на боці ESP32 вирішує
CAMERA_GRAB_LATEST, і тут її треба вирішити ще раз.

Рішення: окремий потік безперервно читає мережу на повній швидкості й тримає
ТІЛЬКИ останній кадр. Основний цикл забирає найсвіжіше, а все, що він не встиг
обробити, просто перезаписується. Затримка тоді не накопичується ніколи.

Заодно розбираємо MJPEG-протокол вручну, а не ховаємо його за VideoCapture —
це рівно те, що прошивка віддає на іншому кінці дроту.
"""

from __future__ import annotations

import io
import threading
import time
from dataclasses import dataclass, field

import cv2
import numpy as np
import requests


def status_url_for(stream_url: str) -> str:
    """http://host:81/stream  ->  http://host/status"""
    from urllib.parse import urlparse, urlunparse

    p = urlparse(stream_url)
    return urlunparse((p.scheme, p.hostname or p.netloc, "/status", "", "", ""))


def probe_status(stream_url: str, timeout: float = 3.0) -> dict | None:
    """
    Питає /status на порту 80 — він живий навіть тоді, коли порт 81 недоступний.

    Навіщо: esp_http_server обслуговує всі сокети однією задачею FreeRTOS,
    а наш обробник стріму блокується назавжди. Тому камера віддає потік
    РІВНО ОДНОМУ клієнту: поки відкрита вкладка браузера з /, Python
    не підключиться і отримає таймаут. Цей пробник дозволяє відрізнити
    "камера зайнята" від "камера мертва".
    """
    try:
        return requests.get(status_url_for(stream_url), timeout=timeout).json()
    except Exception:
        return None


@dataclass
class CameraStats:
    """Телеметрія прийому. Оновлюється в потоці читача."""

    frames_received: int = 0
    frames_dropped: int = 0  # кадри, які прийшли, але споживач їх не забрав
    bytes_received: int = 0
    recv_fps: float = 0.0
    last_frame_kb: float = 0.0
    connected: bool = False
    reconnects: int = 0
    last_error: str = ""

    def snapshot(self) -> dict:
        return dict(self.__dict__)


class MjpegCamera:
    """
    Читач MJPEG-потоку з авто-перепідключенням.

    Використання:
        cam = MjpegCamera("http://192.168.1.50:81/stream")
        cam.start()
        frame = cam.read(timeout=5.0)   # numpy-масив BGR або None
        ...
        cam.stop()
    """

    def __init__(
        self,
        url: str,
        connect_timeout: float = 5.0,
        read_timeout: float = 5.0,
        reconnect_delay: float = 2.0,
    ) -> None:
        self.url = url
        self.connect_timeout = connect_timeout
        self.read_timeout = read_timeout
        self.reconnect_delay = reconnect_delay

        self.stats = CameraStats()

        self._frame: np.ndarray | None = None
        self._frame_id = 0
        self._consumed_id = 0
        self._lock = threading.Lock()
        self._new_frame = threading.Condition(self._lock)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # -- публічний API ------------------------------------------------------

    def start(self) -> "MjpegCamera":
        if self._thread is not None:
            raise RuntimeError("камера вже запущена")
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._reader_loop, name="mjpeg-reader", daemon=True)
        self._thread.start()
        return self

    def read(self, timeout: float | None = None) -> np.ndarray | None:
        """
        Повертає найсвіжіший кадр. Блокується, доки не з'явиться НОВИЙ кадр
        (той самий двічі не віддається), або поки не мине timeout.
        """
        with self._new_frame:
            if self._frame_id == self._consumed_id:
                self._new_frame.wait(timeout)
            if self._frame is None or self._frame_id == self._consumed_id:
                return None
            self._consumed_id = self._frame_id
            return self._frame

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=self.connect_timeout + 1.0)
            self._thread = None

    def __enter__(self) -> "MjpegCamera":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()

    # -- внутрішня кухня ----------------------------------------------------

    def _publish(self, jpeg: bytes) -> None:
        """Декодує JPEG і кладе кадр у слот, витісняючи попередній."""
        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return  # побитий кадр — просто пропускаємо, потік не рвемо

        with self._new_frame:
            # Якщо попередній кадр так і не забрали — він щойно згорів.
            # Це не помилка, а сенс усієї конструкції; рахуємо для статистики.
            if self._frame_id != self._consumed_id:
                self.stats.frames_dropped += 1
            self._frame = frame
            self._frame_id += 1
            self.stats.frames_received += 1
            self.stats.bytes_received += len(jpeg)
            self.stats.last_frame_kb = len(jpeg) / 1024
            self._new_frame.notify()

    def _reader_loop(self) -> None:
        session = requests.Session()

        while not self._stop_event.is_set():
            try:
                resp = session.get(
                    self.url,
                    stream=True,
                    timeout=(self.connect_timeout, self.read_timeout),
                    headers={"Accept": "multipart/x-mixed-replace"},
                )
                resp.raise_for_status()

                self.stats.connected = True
                self.stats.last_error = ""
                self._consume_stream(resp)

            except Exception as exc:  # мережа, таймаут, розрив — усе лікується однаково
                self.stats.last_error = f"{type(exc).__name__}: {exc}"
            finally:
                if self.stats.connected:
                    self.stats.connected = False
                    self.stats.recv_fps = 0.0
                    self.stats.reconnects += 1

            if not self._stop_event.is_set():
                self._stop_event.wait(self.reconnect_delay)

        session.close()

    def _consume_stream(self, resp: requests.Response) -> None:
        """
        Розбирає multipart-потік.

        На дроті це виглядає так, і саме це шле наша прошивка:

            \\r\\n--frameboundary\\r\\n
            Content-Type: image/jpeg\\r\\n
            Content-Length: 24576\\r\\n
            \\r\\n
            <24576 байт JPEG>
            \\r\\n--frameboundary\\r\\n
            ...

        Читаємо заголовки построково, з них беремо точну довжину тіла —
        і рівно стільки байтів забираємо. Це надійніше, ніж шукати в потоці
        маркери кінця JPEG (0xFFD9): такий байт може випадково трапитись
        і всередині стиснених даних.
        """
        # decode_content=False: жодних gzip-перетворень, нам потрібні сирі байти
        resp.raw.decode_content = False
        stream = io.BufferedReader(resp.raw, buffer_size=65536)

        fps_window_start = time.monotonic()
        fps_window_frames = 0

        while not self._stop_event.is_set():
            # 1. Дочекатися рядка-роздільника. Порожні рядки перед ним
            #    (той самий \r\n, з якого починається кожна частина) пропускаємо.
            while True:
                line = stream.readline()
                if not line:
                    return  # з'єднання закрилось
                if line.strip().startswith(b"--"):
                    break

            # 2. Заголовки частини: читаємо до порожнього рядка
            content_length: int | None = None
            while True:
                line = stream.readline()
                if not line:
                    return
                line = line.strip()
                if not line:
                    break  # порожній рядок = кінець заголовків, далі тіло
                if line.lower().startswith(b"content-length:"):
                    content_length = int(line.split(b":", 1)[1])

            if content_length is None:
                # Прошивка завжди шле Content-Length. Якщо його нема —
                # це чужий сервер, і надійно розпарсити ми не можемо.
                raise ValueError("у частині потоку немає Content-Length")

            jpeg = stream.read(content_length)
            if jpeg is None or len(jpeg) < content_length:
                return  # потік обірвався посеред кадру

            self._publish(jpeg)

            # FPS прийому — усереднення по вікну в 1 секунду
            fps_window_frames += 1
            elapsed = time.monotonic() - fps_window_start
            if elapsed >= 1.0:
                self.stats.recv_fps = fps_window_frames / elapsed
                fps_window_frames = 0
                fps_window_start = time.monotonic()
