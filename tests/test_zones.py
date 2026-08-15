"""
Перевірка геометрії зон і ліній на синтетичних треках.

Без камери й без моделі: підставляємо підроблені рамки з відомими
координатами і звіряємо, які події має видати рушій. Цей код легко зламати
непомітно — помилка в знаку векторного добутку не падає, вона просто
переплутує «увійшов» і «вийшов».

  python tests/test_zones.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "hub"))
from zones import Line, RuleEngine, Zone, anchor_point, segments_cross  # noqa: E402

W, H = 640, 480
SHAPE = (H, W, 3)


class Box:
    """Мінімальна підробка ultralytics-боксу."""
    def __init__(self, tid, x1, y1, x2, y2):
        self.id = [tid]
        self.xyxy = [[x1, y1, x2, y2]]


def person_at(tid, cx, foot_y, w=60, h=160):
    return Box(tid, cx - w / 2, foot_y - h, cx + w / 2, foot_y)


ok = True


def check(name, got, want):
    global ok
    mark = "OK " if got == want else "FAIL"
    if got != want:
        ok = False
    print(f"  [{mark}] {name}: {got}" + ("" if got == want else f"  (очікували {want})"))


print("1. Точка прив'язки")
check("bottom = ноги", anchor_point([100, 100, 200, 400], "bottom"), (150.0, 400.0))
check("center = центр", anchor_point([100, 100, 200, 400], "center"), (150.0, 250.0))

print("\n2. Перетин відрізків (а не прямих)")
check("перетинаються", segments_cross((0, 0), (10, 10), (0, 10), (10, 0)), True)
check("паралельні", segments_cross((0, 0), (10, 0), (0, 5), (10, 5)), False)
check("рух ЗА продовженням лінії не рахується",
      segments_cross((100, 0), (100, 10), (0, 5), (50, 5)), False)

print("\n3. Зона: вхід і вихід із дебаунсом 3")
zone = Zone(name="кухня", points=[[0.4, 0.5], [0.9, 0.5], [0.9, 0.95], [0.4, 0.95]])
eng = RuleEngine([zone], [], debounce=3)

events = []
# ззовні (ноги вище зони по y)
for t in range(5):
    events += eng.update([person_at(1, 320, 200)], SHAPE, now=t * 0.1)
check("поки ззовні — подій немає", len(events), 0)

# заходимо: ноги в зоні
for t in range(2):
    events += eng.update([person_at(1, 400, 400)], SHAPE, now=1.0 + t * 0.1)
check("2 кадри всередині — ще мовчить (дебаунс 3)", len(events), 0)

events += eng.update([person_at(1, 400, 400)], SHAPE, now=1.3)
check("3-й кадр — подія входу", [e.kind for e in events], ["zone_enter"])

# виходимо
for t in range(3):
    events += eng.update([person_at(1, 100, 400)], SHAPE, now=2.0 + t * 0.1)
check("вихід підтверджено", [e.kind for e in events], ["zone_enter", "zone_exit"])

print("\n4. Зона: торс над зоною не рахується як вхід")
eng2 = RuleEngine([zone], [], debounce=1)
# ноги на y=460 (нижче зони немає), центр рамки потрапив би в зону, ноги — ні
outside = [person_at(1, 400, 200)]          # ноги на y=200, зона з y=240
got = eng2.update(outside, SHAPE, now=0.0)
check("людина попереду зони — тиша", len(got), 0)

print("\n5. Лінія: напрямок перетину")
line = Line(name="двері", a=[0.5, 0.0], b=[0.5, 1.0],
            positive="увійшов", negative="вийшов")
eng3 = RuleEngine([], [line], debounce=1)

eng3.update([person_at(1, 200, 400)], SHAPE, now=0.0)          # зліва
ev = eng3.update([person_at(1, 440, 400)], SHAPE, now=0.1)     # перейшов праворуч
check("перетин зліва направо", [(e.kind, e.detail) for e in ev],
      [("line_cross", "вийшов")])

ev = eng3.update([person_at(1, 200, 400)], SHAPE, now=0.2)     # назад ліворуч
check("перетин справа наліво", [(e.kind, e.detail) for e in ev],
      [("line_cross", "увійшов")])

ev = eng3.update([person_at(1, 100, 400)], SHAPE, now=0.3)     # рух без перетину
check("рух без перетину — тиша", len(ev), 0)

print("\n6. Трек зник, поки був у зоні")
eng4 = RuleEngine([zone], [], debounce=1, forget_after=1.5)
eng4.update([person_at(7, 400, 400)], SHAPE, now=0.0)
ev = eng4.update([], SHAPE, now=3.0)
check("закривається подією виходу", [e.kind for e in ev], ["zone_exit"])
check("зона більше нікого не рахує", eng4.occupancy(), {"кухня": 0})

print("\n6b. Короткий провал детекції НЕ рве подію")
# 62% провалів детекції тривають 1-3 кадри: людина в кадрі, модель моргнула.
# Без витримки це давало фальшиву пару «вийшов» + «зайшов» на тому ж треку.
eng5 = RuleEngine([zone], [], debounce=1, forget_after=1.5)
eng5.update([person_at(9, 400, 400)], SHAPE, now=0.0)
gap = eng5.update([], SHAPE, now=0.08)                    # моргнув 2 кадри
gap += eng5.update([], SHAPE, now=0.16)
check("під час провалу — тиша", len(gap), 0)
back = eng5.update([person_at(9, 400, 400)], SHAPE, now=0.24)
check("після повернення теж тиша (він і не виходив)", len(back), 0)
check("трек досі числиться в зоні", eng5.occupancy(), {"кухня": 1})

print("\n7. Лічильники")
print("   ", eng3.counters)
check("двері: два різні напрямки", sorted(eng3.counters),
      ["двері:вийшов", "двері:увійшов"])

print("\n" + ("УСЕ ЗІЙШЛОСЬ" if ok else "Є РОЗБІЖНОСТІ"))
sys.exit(0 if ok else 1)
