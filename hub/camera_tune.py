"""
Підбір налаштувань сенсора: XCLK × якість JPEG -> FPS, розмір кадру, смуги,
а також стан автоекспозиції — для калібрування адаптивного XCLK.

Навіщо окремий інструмент. Після заміни модуля OV3660 на OV5640 з'ясувалось,
що оптимальні налаштування в сенсорів різні, і що інтуїція тут бреше:
  - вертикальні смуги зникали зі зниженням XCLK, але ЛИШЕ в темряві;
    зі світлом їх не було навіть на 20 МГц;
  - FPS майже не залежав від якості JPEG, хоча логічно було чекати, що менші
    кадри швидше пройдуть через Wi-Fi. Вузьке місце — такт сенсора.
Тобто вибір XCLK — компроміс «смуги проти FPS», і його треба міряти.

Що міряється для кожної конфігурації:
  fps          — скільки кадрів реально прийшло за вікно (лише ті, що
                 декодувались: побиті кадри в FPS не потрапляють)
  КБ/кадр      — середній розмір JPEG (байти / кадри за вікно)
  смуги        — оцінка вертикальних смуг: беремо середню яскравість кожного
                 СТОВПЦЯ кадру, віднімаємо згладжену версію і рахуємо розкид
                 залишку. Смуги — це різкі стрибки між сусідніми стовпцями.
                 Краї предметів у сцені теж дають внесок, але однаковий для
                 всіх конфігурацій, якщо сцена нерухома — тому число корисне
                 для ПОРІВНЯННЯ, а не як абсолютна величина.
  експ/підс    — стан автоекспозиції сенсора з /status (якщо прошивка вміє)
  light_need   — те, що рахує прошивка: експозиція × підсилення
  норм         — експозиція × підсилення / XCLK, див. нижче

КАЛІБРУВАННЯ АВТО-XCLK І ЧОМУ «НОРМ».

Прошивка перемикає XCLK між «світлом» і «темрявою». Сигнал для цього мусить
не лише розрізняти світло й темряву, а й після ПЕРЕМИКАННЯ частоти зсуватися
від порогу повернення, а не до нього. Інакше перемикання саме себе скасовує
і режим смикається.

Калібрування перевірило три кандидати (docs/benchmarks.md):

  експ × підс        — зі світлом розкид між 16 і 8 МГц 77%, при переході
                       16 -> 8 падає вдвічі, НАЗУСТРІЧ порогу повернення
  експ × підс / XCLK — гіпотеза «регістр рахує рядки, а не час»: зі світлом
                       розкид упав до 12%, але в темряві виріс до 67%
  підсилення         — світло 21-28, темрява 248; при переході 16 -> 8 на
                       світлі РОСТЕ (від порогу повернення), у темряві
                       не змінюється. Цей і обрано.

У темряві експозиція (885) і підсилення (248) однакові на обох частотах —
автоекспозиція вперлась у стелю обох параметрів.

Колонки експ/підс/light_need/норм лишаються — вони знадобляться при
калібруванні іншого модуля або інших умов освітлення. Пороги беруться між
значеннями ПІДСИЛЕННЯ зі світлом і в темряві:
  python hub/camera_tune.py --xclk 16 8 --quality 12    # зі світлом
  python hub/camera_tune.py --xclk 16 8 --quality 12    # без світла

ВИМОГИ ДО ПРОГОНУ: сцена нерухома (рух змінює середні по стовпцях і псує
оцінку смуг) і до потоку не підключено інших глядачів.

Після прогону повертаються налаштування, що були до нього, включно з авторежимом.

  python hub/camera_tune.py
  python hub/camera_tune.py --xclk 12 10 8 --quality 12 20 --seconds 6
  python hub/camera_tune.py --save-frames captures/tune
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import requests

from camera import MjpegCamera

DEFAULT_HOST = os.environ.get("CAM_HOST", "esp32cam.local")


def stripe_score(frame: np.ndarray) -> float:
    """Розкид високочастотної складової середньої яскравості по стовпцях."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
    cols = gray.mean(axis=0)
    k = 9
    smooth = np.convolve(cols, np.ones(k, dtype=np.float32) / k, mode="same")
    resid = (cols - smooth)[k:-k]      # краї згортки некоректні — відкидаємо
    return float(resid.std())


def control(host: str, var: str, val: int) -> bool:
    try:
        r = requests.get(f"http://{host}/control", params={"var": var, "val": val}, timeout=5)
        return r.ok
    except requests.RequestException:
        return False


def status(host: str) -> dict:
    try:
        return requests.get(f"http://{host}/status", timeout=3).json()
    except (requests.RequestException, ValueError):
        return {}


def median_or_none(values: list[float]) -> float | None:
    clean = [v for v in values if v is not None and v >= 0]
    return statistics.median(clean) if clean else None


def measure(cam: MjpegCamera, host: str, seconds: float, settle: float,
            save_to: Path | None, label: str) -> dict:
    # Після зміни XCLK сенсору потрібно кілька кадрів, щоб перебудувати такти,
    # а автоекспозиції — щоб зійтись. Кадри з цього проміжку в замір не беремо.
    t_end_settle = time.monotonic() + settle
    while time.monotonic() < t_end_settle:
        cam.read(timeout=0.5)

    f0, b0 = cam.stats.frames_received, cam.stats.bytes_received
    r0 = cam.stats.reconnects
    scores: list[float] = []
    exposures: list[float] = []
    gains: list[float] = []
    needs: list[float] = []
    last = None
    next_status = 0.0

    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        now = time.monotonic()
        if now >= next_status:
            # Прошивка оновлює стан експозиції раз на секунду — частіше питати марно
            st = status(host)
            exposures.append(st.get("aec_exposure"))
            gains.append(st.get("aec_gain"))
            needs.append(st.get("light_need"))
            next_status = now + 1.0

        frame = cam.read(timeout=1.0)
        if frame is None:
            continue
        last = frame
        if len(scores) < 30:           # 30 кадрів на оцінку смуг достатньо
            scores.append(stripe_score(frame))
    elapsed = time.monotonic() - t0

    frames = cam.stats.frames_received - f0
    nbytes = cam.stats.bytes_received - b0

    if save_to is not None and last is not None:
        save_to.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(save_to / f"{label}.jpg"), last)

    return {
        "fps": frames / elapsed if elapsed else 0.0,
        "kb": (nbytes / frames / 1024) if frames else 0.0,
        "stripes": statistics.median(scores) if scores else float("nan"),
        "reconnects": cam.stats.reconnects - r0,
        "exposure": median_or_none(exposures),
        "gain": median_or_none(gains),
        "light_need": median_or_none(needs),
    }


def fmt(v: float | None, spec: str = ".0f") -> str:
    return "—" if v is None else format(v, spec)


def spread(values: list[float]) -> float:
    """Розкид як частка від медіани: (max - min) / median."""
    return (max(values) - min(values)) / max(1e-9, statistics.median(values))


def main() -> int:
    ap = argparse.ArgumentParser(description="Підбір XCLK і якості JPEG")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--xclk", type=int, nargs="+", default=[20, 16, 12, 10, 8],
                    help="частоти XCLK для перебору, МГц")
    ap.add_argument("--xclk-quality", type=int, default=12,
                    help="якість JPEG, на якій перебирається XCLK")
    ap.add_argument("--quality", type=int, nargs="+", default=[10, 12, 16, 20],
                    help="значення якості для перебору")
    ap.add_argument("--quality-xclk", type=int, default=None,
                    help="XCLK, на якому перебирається якість "
                         "(типово — найменша з --xclk)")
    ap.add_argument("--seconds", type=float, default=8.0, help="вікно заміру")
    ap.add_argument("--settle", type=float, default=4.0,
                    help="пауза після зміни: сенсор перебудовує такти, "
                         "автоекспозиція сходиться")
    ap.add_argument("--save-frames", type=Path, default=None,
                    help="зберегти по кадру на конфігурацію, щоб подивитись очима")
    args = ap.parse_args()

    before = status(args.host)
    if not before:
        print(f"[tune] камера не відповідає: http://{args.host}/status")
        return 1

    print(f"[tune] сенсор {before.get('sensor', '?')} ({before.get('pid', '?')}), "
          f"профіль: {before.get('profile', '?')}")
    print(f"[tune] зараз: XCLK {before.get('xclk_mhz')} МГц, якість {before.get('quality')}, "
          f"авто-XCLK {'увімкнено' if before.get('auto_xclk') else 'вимкнено'}")
    if before.get("clients", 0) > 0:
        print(f"[tune] УВАГА: до потоку підключено {before['clients']} глядачів — "
              "закрий їх. Спостерігалось, що другий клієнт може не отримати жодного кадру")
    if "light_need" not in before:
        print("[tune] прошивка не віддає стан автоекспозиції — колонки експ/підс/потреба будуть порожні")

    quality_xclk = args.quality_xclk or min(args.xclk)
    plan = [(x, args.xclk_quality) for x in args.xclk]
    plan += [(quality_xclk, q) for q in args.quality if (quality_xclk, q) not in plan]

    total = len(plan) * (args.seconds + args.settle)
    print(f"[tune] конфігурацій {len(plan)}, ~{total:.0f} с. Сцена має бути нерухомою.\n")

    rows = []
    cam = MjpegCamera(f"http://{args.host}:81/stream")
    try:
        with cam:
            for xclk, quality in plan:
                # Ручна зміна xclk у прошивці вимикає авторежим — тож він
                # не воюватиме з перебором
                ok = control(args.host, "xclk", xclk) and control(args.host, "quality", quality)
                if not ok:
                    print(f"  XCLK {xclk:2d} / якість {quality:2d}: не вдалося застосувати")
                    continue
                res = measure(cam, args.host, args.seconds, args.settle, args.save_frames,
                              f"xclk{xclk}_q{quality}")
                # Експозиція в регістрі — у рядках; ділення на XCLK переводить
                # її в одиниці, пропорційні реальному часу (див. docstring)
                res["norm"] = (res["exposure"] * res["gain"] / xclk
                               if res["exposure"] is not None and res["gain"] is not None
                               else None)
                rows.append((xclk, quality, res))
                print(f"  XCLK {xclk:2d} МГц / якість {quality:2d}: "
                      f"{res['fps']:5.1f} fps  {res['kb']:5.1f} КБ/кадр  "
                      f"смуги {res['stripes']:5.2f}  "
                      f"експ {fmt(res['exposure'])}  підс {fmt(res['gain'])}  "
                      f"light_need {fmt(res['light_need'])}  норм {fmt(res['norm'])}"
                      + (f"  (перепідключень {res['reconnects']})" if res["reconnects"] else ""))
    finally:
        # Повертаємо як було: прогін не повинен лишати камеру в чужих налаштуваннях.
        # Порядок важливий: спершу частота (вона вимикає авто), потім авто назад
        control(args.host, "xclk", int(before.get("xclk_mhz", 20)))
        control(args.host, "quality", int(before.get("quality", 12)))
        if before.get("auto_xclk"):
            control(args.host, "auto", 1)
        print(f"\n[tune] відновлено: XCLK {before.get('xclk_mhz')} МГц, "
              f"якість {before.get('quality')}"
              + (", авто-XCLK увімкнено" if before.get("auto_xclk") else ""))

    if not rows:
        return 1

    print("\n| XCLK, МГц | якість | fps | КБ/кадр | смуги | експозиція | підсилення "
          "| light_need | норм (експ×підс/XCLK) |")
    print("|----------:|-------:|----:|--------:|------:|-----------:|-----------:"
          "|-----------:|----------------------:|")
    for xclk, quality, r in rows:
        print(f"| {xclk} | {quality} | {r['fps']:.1f} | {r['kb']:.1f} | {r['stripes']:.2f} | "
              f"{fmt(r['exposure'])} | {fmt(r['gain'])} | {fmt(r['light_need'])} | "
              f"{fmt(r['norm'])} |")

    by_xclk = {}
    for xclk, _q, r in rows:
        if r["light_need"] is not None and r["norm"] is not None:
            by_xclk.setdefault(xclk, r)   # по одному рядку на частоту
    if len(by_xclk) >= 2:
        raw = [r["light_need"] for r in by_xclk.values()]
        norm = [r["norm"] for r in by_xclk.values()]
        print(f"\nРозкид на різних XCLK, та сама сцена:")
        print(f"  light_need (сирий добуток)  {spread(raw):5.0%}")
        print(f"  норм (поділено на XCLK)     {spread(norm):5.0%}")
        print("Для авто-XCLK розкид має бути малим порівняно з різницею між світлом")
        print("і темрявою — інакше перемикання частоти само змінює сигнал рішення.")

    print("\nОцінка смуг — ВІДНОСНА: порівнюй рядки між собою, а не з нулем.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
