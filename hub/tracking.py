"""
Етап 5 — трекінг: від окремих детекцій до траєкторій.

Детекція не має пам'яті: кожен кадр обробляється з нуля, і модель не знає,
що людина в цьому кадрі — та сама, що була в попередньому. Тому порахувати
людей через саму детекцію неможливо: за хвилину нарахуєш півтори тисячі.

Трекер додає час. Фільтр Калмана передбачає, де об'єкт має опинитись
у наступному кадрі, передбачення зіставляється з реальними детекціями,
і з них зшиваються треки зі стабільними ID.

Трюк ByteTrack закладений у назві (BYTE = Bring Every detecTion). Звичайні
трекери відкидають рамки з низькою впевненістю ще до зіставлення. ByteTrack
робить два проходи: спершу зіставляє впевнені детекції, а потім рештою —
слабкими — рятує треки, які інакше загубились би. Саме слабкі детекції дає
людина, яку частково затулили або яка розмилась у русі.

Що трекер НЕ вміє: якщо людина зникла надовго й повернулась, вона отримає
новий ID. ByteTrack зіставляє за геометрією й рухом, а не за зовнішністю.
Для «це той самий Олег, що виходив 10 хвилин тому» потрібен re-ID —
окрема задача, у наш scope вона не входить.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field

import cv2

# Скільки точок траєкторії тримати на трек. 64 при ~25 fps ≈ 2.5 секунди хвоста —
# достатньо, щоб побачити напрямок руху, і не настільки багато, щоб екран
# перетворився на макарони.
TRAIL_LEN = 64

# Через скільки секунд без оновлень забути трек. Трохи більше, ніж
# track_buffer у ByteTrack, щоб не викинути те, що трекер ще пам'ятає.
TRACK_TTL = 5.0


@dataclass
class Track:
    """Історія одного об'єкта."""

    track_id: int
    cls_name: str
    first_seen: float
    last_seen: float
    trail: deque = field(default_factory=lambda: deque(maxlen=TRAIL_LEN))
    frames: int = 0

    @property
    def dwell_s(self) -> float:
        """Скільки секунд об'єкт у кадрі. Основа для правил «затримався в зоні»."""
        return self.last_seen - self.first_seen

    def speed_px_s(self) -> float:
        """
        Швидкість у пікселях за секунду по останніх точках хвоста.
        У пікселях, а не в метрах: без калібрування камери перевести не можна.
        Для правил «біжить / стоїть» відносної величини вистачає.
        """
        if len(self.trail) < 2:
            return 0.0
        (t0, x0, y0), (t1, x1, y1) = self.trail[0], self.trail[-1]
        dt = t1 - t0
        if dt <= 0:
            return 0.0
        return ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5 / dt


class TrackHistory:
    """Накопичує траєкторії по ID і прибирає ті, що зникли."""

    def __init__(self, ttl: float = TRACK_TTL) -> None:
        self.ttl = ttl
        self.tracks: dict[int, Track] = {}
        self.total_seen = 0  # скільки унікальних об'єктів пройшло за весь час

    def update(self, boxes, names) -> None:
        now = time.monotonic()

        for box in boxes:
            if box.id is None:  # детекція без треку — трекер її ще не прийняв
                continue
            tid = int(box.id[0])
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

            track = self.tracks.get(tid)
            if track is None:
                track = Track(
                    track_id=tid,
                    cls_name=names.get(int(box.cls[0]), "?"),
                    first_seen=now,
                    last_seen=now,
                )
                self.tracks[tid] = track
                self.total_seen += 1

            track.last_seen = now
            track.frames += 1
            track.trail.append((now, cx, cy))

        stale = [tid for tid, t in self.tracks.items() if now - t.last_seen > self.ttl]
        for tid in stale:
            del self.tracks[tid]

    @property
    def active(self) -> int:
        return len(self.tracks)


def track_color(track_id: int):
    """
    Свій колір кожному треку — так одразу видно підміну ID.
    Якщо людина йде через кадр і колір рамки раптом змінився, значить
    трекер її загубив і завів новий трек. Це найважливіший діагностичний
    сигнал у всьому етапі, тому колір саме за ID, а не за класом.
    """
    import numpy as np

    hue = (track_id * 67) % 180  # 67 взаємно просте з 180 -> сусідні ID далеко по відтінку
    hsv = np.uint8([[[hue, 190, 245]]])
    b, g, r = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
    return int(b), int(g), int(r)


def draw_tracks(frame, boxes, names, history: TrackHistory, *, show_trail: bool = True) -> int:
    """Малює рамки з ID, хвостом траєкторії і часом перебування в кадрі."""
    n = 0
    for box in boxes:
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
        conf = float(box.conf[0])
        cls_name = names.get(int(box.cls[0]), "?")

        if box.id is None:
            # Детекція є, треку ще немає: трекер вимагає кількох кадрів
            # підтвердження, перш ніж завести ID. Малюємо тьмяно й тонко.
            cv2.rectangle(frame, (x1, y1), (x2, y2), (120, 120, 120), 1)
            n += 1
            continue

        tid = int(box.id[0])
        color = track_color(tid)
        track = history.tracks.get(tid)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        if show_trail and track and len(track.trail) > 1:
            pts = [(int(x), int(y)) for _, x, y in track.trail]
            for i in range(1, len(pts)):
                # Хвіст тоншає до хвоста: видно напрямок руху без стрілок
                thickness = max(1, int(3 * i / len(pts)))
                cv2.line(frame, pts[i - 1], pts[i], color, thickness, cv2.LINE_AA)

        label = f"#{tid} {cls_name} {conf:.2f}"
        if track:
            label += f"  {track.dwell_s:.0f}s"

        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
        cv2.putText(frame, label, (x1 + 3, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 1, cv2.LINE_AA)
        n += 1

    return n
