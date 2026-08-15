"""
Етап 5 — зони, лінії та події поверх треків.

Три рішення, які визначають, чи буде це працювати в реальності.

1. ЯКА ТОЧКА ПРЕДСТАВЛЯЄ ЛЮДИНУ

   Здається, що центр рамки. Насправді для зон на підлозі це неправильно:
   людина стоїть ногами, а центр рамки — десь на рівні грудей. Зона біля
   дверей спрацює, коли в неї зазирне торс, тобто на метр раніше.
   Тому типова точка прив'язки — НИЖНЯ СЕРЕДИНА рамки.

   Коли людина підходить упритул і обрізається краєм кадру, ноги з кадру
   зникають, і точка прив'язки стрибає вгору. Це відома вада підходу;
   лікується або вужчим кутом, або anchor="center" для конкретної зони.

2. ДЕБАУНС

   Людина, що стоїть на межі зони, за секунду видасть двадцять подій
   «зайшов/вийшов»: рамка коливається на пару пікселів. Тому подія
   підтверджується кількома кадрами поспіль, а не одним.

3. ПЕРЕТИН ЛІНІЇ — ЦЕ ПОДІЯ, А НЕ СТАН

   Важливо не «людина по інший бік лінії», а сам момент переходу і його
   напрямок. Ловимо перетин відрізка траєкторії (попередня точка -> поточна)
   з відрізком лінії. Напрямок беремо зі знака векторного добутку.

Координати в конфігу — НОРМОВАНІ (0..1) відносно розміру кадру. Так зони
переживають зміну роздільності камери: перемкнув VGA -> HD, і нічого
не поїхало.
"""

from __future__ import annotations

import json
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

# Скільки кадрів поспіль підтверджують вхід/вихід. При ~22 fps три кадри —
# це ~0.14 с: достатньо, щоб прибити тремтіння рамки, і непомітно для людини.
DEFAULT_DEBOUNCE = 3

# Скільки секунд чекати, перш ніж вважати зниклий трек справді зниклим.
#
# Не можна забувати трек одразу, щойно його немає в поточному кадрі: 62%
# провалів детекції тривають 1-3 кадри (docs/benchmarks.md). Людина нікуди
# не дівається, просто модель моргнула — а система встигає видати фальшиву
# пару «вийшов» + «зайшов». Це та сама проблема, що й дебаунс на межі зони,
# тільки з боку зникнення.
DEFAULT_FORGET_AFTER = 1.5

EVENT_LOG_SIZE = 200


# ---------------------------------------------------------------------------
# Геометрія
# ---------------------------------------------------------------------------

def anchor_point(box_xyxy, mode: str = "bottom") -> tuple[float, float]:
    """Точка, якою трек «стоїть» у кадрі."""
    x1, y1, x2, y2 = box_xyxy
    if mode == "center":
        return (x1 + x2) / 2, (y1 + y2) / 2
    return (x1 + x2) / 2, y2  # bottom: ноги


def _orient(ax, ay, bx, by, cx, cy) -> float:
    """Знак векторного добутку: з якого боку від AB лежить C."""
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def segments_cross(p1, p2, p3, p4) -> bool:
    """
    Чи перетинаються відрізки p1p2 і p3p4.

    Саме відрізки, а не прямі. Якщо перевіряти лише зміну боку відносно
    ПРЯМОЇ, спрацює будь-який рух за продовженням лінії — людина, що пройшла
    повз двері за три метри від них, зарахується як «увійшла».
    """
    d1 = _orient(*p3, *p4, *p1)
    d2 = _orient(*p3, *p4, *p2)
    d3 = _orient(*p1, *p2, *p3)
    d4 = _orient(*p1, *p2, *p4)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


# ---------------------------------------------------------------------------
# Конфігурація
# ---------------------------------------------------------------------------

@dataclass
class Zone:
    name: str
    points: list[list[float]]          # нормовані 0..1
    dwell_alert_s: float = 0.0         # 0 = не стежити за затримкою
    anchor: str = "bottom"
    color: tuple[int, int, int] = (60, 180, 255)

    def polygon_px(self, w: int, h: int) -> np.ndarray:
        return np.array([[int(x * w), int(y * h)] for x, y in self.points], dtype=np.int32)

    def contains(self, pt, w: int, h: int) -> bool:
        poly = self.polygon_px(w, h)
        return cv2.pointPolygonTest(poly, (float(pt[0]), float(pt[1])), False) >= 0


@dataclass
class Line:
    name: str
    a: list[float]                     # нормовані 0..1
    b: list[float]
    positive: str = "→"                # як називати перетин у один бік
    negative: str = "←"                # і в інший
    anchor: str = "bottom"
    color: tuple[int, int, int] = (80, 220, 120)

    def px(self, w: int, h: int):
        return ((int(self.a[0] * w), int(self.a[1] * h)),
                (int(self.b[0] * w), int(self.b[1] * h)))


@dataclass
class Event:
    ts: float            # монотонний час: для логіки й різниць
    kind: str            # zone_enter | zone_exit | zone_dwell | line_cross
    track_id: int
    target: str          # назва зони або лінії
    detail: str = ""
    # Монотонний лічильник не прив'язаний до годинника й не годиться, щоб
    # показати «о 21:14». Тримаємо обидва: ts для логіки, wall для людей.
    wall: float = field(default_factory=time.time)

    def as_dict(self) -> dict:
        return {
            "ts": self.ts,
            "wall": self.wall,
            "kind": self.kind,
            "track_id": self.track_id,
            "target": self.target,
            "detail": self.detail,
        }

    def human(self) -> str:
        label = {
            "zone_enter": "увійшов у",
            "zone_exit": "вийшов з",
            "zone_dwell": "затримався в",
            "line_cross": "перетнув",
        }.get(self.kind, self.kind)
        tail = f" ({self.detail})" if self.detail else ""
        return f"#{self.track_id} {label} «{self.target}»{tail}"


# ---------------------------------------------------------------------------
# Рушій правил
# ---------------------------------------------------------------------------

@dataclass
class _TrackZoneState:
    inside: bool = False
    pending: int = 0        # скільки кадрів поспіль стан відрізняється від підтвердженого
    since: float = 0.0      # коли підтверджено вхід
    dwell_fired: bool = False


class RuleEngine:
    """
    Тримає стан «хто де» і видає події.

    Стан веде по track_id, тому все тримається на трекінгу: без стабільних ID
    «людина зайшла в зону» неможливо відрізнити від «та сама людина досі
    стоїть у зоні».
    """

    def __init__(self, zones: list[Zone], lines: list[Line],
                 debounce: int = DEFAULT_DEBOUNCE,
                 forget_after: float = DEFAULT_FORGET_AFTER) -> None:
        self.zones = zones
        self.lines = lines
        self.debounce = debounce
        self.forget_after = forget_after

        self.events: deque[Event] = deque(maxlen=EVENT_LOG_SIZE)
        self.counters: dict[str, int] = {}

        self._zone_state: dict[tuple[int, str], _TrackZoneState] = {}
        self._last_pt: dict[tuple[int, str], tuple[float, float]] = {}
        self._last_seen: dict[int, float] = {}

    # -- публічне -----------------------------------------------------------

    def occupancy(self) -> dict[str, int]:
        """Скільки треків зараз усередині кожної зони."""
        out = {z.name: 0 for z in self.zones}
        for (_, zname), st in self._zone_state.items():
            if st.inside and zname in out:
                out[zname] += 1
        return out

    def update(self, boxes, frame_shape, now: float | None = None) -> list[Event]:
        """Прогоняє поточні рамки через правила. Повертає НОВІ події."""
        if now is None:
            now = time.monotonic()

        h, w = frame_shape[:2]
        fresh: list[Event] = []
        seen_ids: set[int] = set()

        for box in boxes:
            if box.id is None:
                continue  # без ID правила не мають сенсу
            tid = int(box.id[0])
            seen_ids.add(tid)
            self._last_seen[tid] = now
            xyxy = [float(v) for v in box.xyxy[0]]

            for zone in self.zones:
                fresh += self._check_zone(tid, zone, xyxy, w, h, now)
            for line in self.lines:
                fresh += self._check_line(tid, line, xyxy, w, h, now)

        self._forget_missing(seen_ids, now, fresh)

        for e in fresh:
            self.events.append(e)
        return fresh

    # -- внутрішнє ----------------------------------------------------------

    def _check_zone(self, tid, zone, xyxy, w, h, now) -> list[Event]:
        pt = anchor_point(xyxy, zone.anchor)
        now_inside = zone.contains(pt, w, h)

        key = (tid, zone.name)
        st = self._zone_state.setdefault(key, _TrackZoneState())
        out: list[Event] = []

        if now_inside != st.inside:
            # Стан змінився — але ще не віримо. Дебаунс проти тремтіння
            # рамки на межі зони.
            st.pending += 1
            if st.pending >= self.debounce:
                st.inside = now_inside
                st.pending = 0
                if now_inside:
                    st.since = now
                    st.dwell_fired = False
                    self.counters[f"{zone.name}:enter"] = \
                        self.counters.get(f"{zone.name}:enter", 0) + 1
                    out.append(Event(now, "zone_enter", tid, zone.name))
                else:
                    held = now - st.since
                    self.counters[f"{zone.name}:exit"] = \
                        self.counters.get(f"{zone.name}:exit", 0) + 1
                    out.append(Event(now, "zone_exit", tid, zone.name, f"{held:.1f} с"))
        else:
            st.pending = 0

        if (st.inside and zone.dwell_alert_s > 0 and not st.dwell_fired
                and now - st.since >= zone.dwell_alert_s):
            st.dwell_fired = True
            out.append(Event(now, "zone_dwell", tid, zone.name,
                             f"{zone.dwell_alert_s:.0f}+ с"))
        return out

    def _check_line(self, tid, line, xyxy, w, h, now) -> list[Event]:
        pt = anchor_point(xyxy, line.anchor)
        key = (tid, line.name)
        prev = self._last_pt.get(key)
        self._last_pt[key] = pt

        if prev is None:
            return []

        a, b = line.px(w, h)
        if not segments_cross(prev, pt, a, b):
            return []

        # Знак векторного добутку напрямку лінії на вектор руху:
        # він і каже, в який бік людина перетнула
        side = _orient(*a, *b, *pt)
        direction = line.positive if side > 0 else line.negative

        counter_key = f"{line.name}:{direction}"
        self.counters[counter_key] = self.counters.get(counter_key, 0) + 1
        return [Event(now, "line_cross", tid, line.name, direction)]

    def _forget_missing(self, seen_ids, now, fresh) -> None:
        """
        Трек справді зник — людина вийшла з кадру або трекер її втратив назовсім.

        Забуваємо не одразу, а через forget_after: короткі провали детекції
        інакше давали б фальшиву пару «вийшов» + «зайшов» на тому самому треку.

        Якщо трек числився в зоні — закриваємо подією виходу, інакше
        лічильник «усередині» зростатиме вічно.
        """
        gone = {tid for tid, seen in self._last_seen.items()
                if tid not in seen_ids and now - seen > self.forget_after}
        if not gone:
            return

        for key in [k for k in self._zone_state if k[0] in gone]:
            st = self._zone_state.pop(key)
            if st.inside:
                tid, zname = key
                held = now - st.since
                self.counters[f"{zname}:exit"] = self.counters.get(f"{zname}:exit", 0) + 1
                fresh.append(Event(now, "zone_exit", tid, zname,
                                   f"{held:.1f} с, трек зник"))

        for key in [k for k in self._last_pt if k[0] in gone]:
            del self._last_pt[key]
        for tid in gone:
            del self._last_seen[tid]


# ---------------------------------------------------------------------------
# Конфіг і малювання
# ---------------------------------------------------------------------------

def load_config(path: str | Path) -> tuple[list[Zone], list[Line]]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    zones = [Zone(**z) for z in data.get("zones", [])]
    lines = [Line(**l) for l in data.get("lines", [])]
    return zones, lines


def save_config(path: str | Path, zones: list[Zone], lines: list[Line]) -> None:
    def clean(obj: dict) -> dict:
        return {k: v for k, v in obj.items() if k != "color"}

    data = {
        "zones": [clean(z.__dict__) for z in zones],
        "lines": [clean(l.__dict__) for l in lines],
    }
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def draw_overlay(frame, zones: list[Zone], lines: list[Line],
                 engine: RuleEngine | None = None) -> None:
    """Малює зони (напівпрозорою заливкою) і лінії зі стрілкою напрямку."""
    h, w = frame.shape[:2]
    occ = engine.occupancy() if engine else {}

    if zones:
        # Заливка по окремому шару, щоб прозорість не накладалась двічі
        # на перетинах зон
        layer = frame.copy()
        for z in zones:
            cv2.fillPoly(layer, [z.polygon_px(w, h)], z.color)
        cv2.addWeighted(layer, 0.18, frame, 0.82, 0, frame)

    for z in zones:
        poly = z.polygon_px(w, h)
        cv2.polylines(frame, [poly], True, z.color, 2, cv2.LINE_AA)
        label = z.name
        if z.name in occ:
            label += f" [{occ[z.name]}]"
        x, y = poly[0]
        cv2.putText(frame, label, (int(x) + 4, int(y) + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, z.color, 2, cv2.LINE_AA)

    for ln in lines:
        a, b = ln.px(w, h)
        cv2.line(frame, a, b, ln.color, 2, cv2.LINE_AA)

        # Стрілка перпендикулярно лінії показує «додатний» напрямок
        mx, my = (a[0] + b[0]) // 2, (a[1] + b[1]) // 2
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = max(1.0, (dx * dx + dy * dy) ** 0.5)
        nx, ny = -dy / length, dx / length     # нормаль
        cv2.arrowedLine(frame, (mx, my), (int(mx + nx * 34), int(my + ny * 34)),
                        ln.color, 2, cv2.LINE_AA, tipLength=0.35)
        cv2.putText(frame, f"{ln.name}: {ln.positive}",
                    (int(mx + nx * 40), int(my + ny * 40)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, ln.color, 1, cv2.LINE_AA)
