"""
Етап 5 — редактор зон і ліній.

Малюєш мишею по справжньому кадру з камери, зберігається JSON
з НОРМОВАНИМИ координатами (0..1). Завдяки нормуванню конфіг переживає
зміну роздільності: перемкнув VGA -> HD, і зони лишились на місці.

Керування:
  ЛКМ        додати точку
  u          прибрати останню точку
  z          замкнути точки в ЗОНУ (треба >= 3)
  l          зробити з точок ЛІНІЮ (треба рівно 2)
  d          видалити останню створену зону/лінію
  f          новий кадр з камери (людина заважає — прибери її й онови)
  s          зберегти
  q          вийти без збереження

Назву питає в консолі, тому під час введення вікно виглядає замерзлим —
це нормально, друкуй у терміналі й тисни Enter.

  python hub/zone_editor.py
  python hub/zone_editor.py --source docs/track_test.mp4 --out docs/zones.json
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import cv2

from camera import MjpegCamera
from zones import Line, Zone, draw_overlay, load_config, save_config

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT = PROJECT_ROOT / "docs" / "zones.json"
DEFAULT_URL = os.environ.get("CAM_URL", "http://esp32cam.local:81/stream")

points: list[tuple[int, int]] = []


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append((x, y))


def grab_frame(args):
    if args.source:
        cap = cv2.VideoCapture(args.source)
        # середній кадр: на першому часто ще не встановилась експозиція
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        if n > 1:
            cap.set(cv2.CAP_PROP_POS_FRAMES, n // 2)
        ok, frame = cap.read()
        cap.release()
        if not ok:
            raise SystemExit(f"[zones] не читається: {args.source}")
        return frame

    cam = MjpegCamera(args.url)
    with cam:
        for _ in range(60):
            frame = cam.read(timeout=1.0)
            if frame is not None:
                return frame
    raise SystemExit("[zones] не вдалось узяти кадр з камери")


def ask(prompt: str, default: str) -> str:
    try:
        value = input(f"{prompt} [{default}]: ").strip()
    except EOFError:
        value = ""
    return value or default


def main() -> int:
    ap = argparse.ArgumentParser(description="Редактор зон і ліній")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--source", help="кадр з відеофайлу замість камери")
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    args = ap.parse_args()

    frame = grab_frame(args)
    h, w = frame.shape[:2]
    print(f"[zones] кадр {w}x{h}")

    zones: list[Zone] = []
    lines: list[Line] = []
    if Path(args.out).exists():
        zones, lines = load_config(args.out)
        print(f"[zones] завантажено наявних: зон {len(zones)}, ліній {len(lines)}")

    cv2.namedWindow("zones")
    cv2.setMouseCallback("zones", on_mouse)
    print(__doc__.split("Керування:")[1].split("Назву питає")[0])

    while True:
        canvas = frame.copy()
        draw_overlay(canvas, zones, lines)

        for i, p in enumerate(points):
            cv2.circle(canvas, p, 4, (0, 255, 255), -1)
            if i:
                cv2.line(canvas, points[i - 1], p, (0, 255, 255), 1, cv2.LINE_AA)
        if len(points) > 2:
            cv2.line(canvas, points[-1], points[0], (0, 200, 200), 1, cv2.LINE_AA)

        hint = f"точок {len(points)} | зон {len(zones)} | ліній {len(lines)}"
        cv2.putText(canvas, hint, (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow("zones", canvas)
        key = cv2.waitKey(30) & 0xFF

        if key in (ord("q"), 27):
            print("[zones] вихід без збереження")
            break

        if key == ord("u") and points:
            points.pop()

        if key == ord("f") and not args.source:
            frame = grab_frame(args)
            h, w = frame.shape[:2]

        if key == ord("z"):
            if len(points) < 3:
                print("[zones] для зони треба щонайменше 3 точки")
                continue
            name = ask("назва зони", f"зона {len(zones) + 1}")
            dwell = ask("тривога, якщо затримався довше N с (0 = не стежити)", "0")
            anchor = ask("точка прив'язки: bottom (ноги) або center", "bottom")
            zones.append(Zone(
                name=name,
                points=[[x / w, y / h] for x, y in points],
                dwell_alert_s=float(dwell or 0),
                anchor=anchor if anchor in ("bottom", "center") else "bottom",
            ))
            points.clear()
            print(f"[zones] додано зону «{name}»")

        if key == ord("l"):
            if len(points) != 2:
                print("[zones] для лінії потрібні рівно 2 точки")
                continue
            name = ask("назва лінії", f"лінія {len(lines) + 1}")
            pos = ask("як називати перетин у бік стрілки", "увійшов")
            neg = ask("як називати перетин у зворотний бік", "вийшов")
            (x1, y1), (x2, y2) = points
            lines.append(Line(
                name=name,
                a=[x1 / w, y1 / h],
                b=[x2 / w, y2 / h],
                positive=pos,
                negative=neg,
            ))
            points.clear()
            print(f"[zones] додано лінію «{name}». Стрілка на екрані показує "
                  f"напрямок «{pos}» — якщо навпаки, перестав точки місцями")

        if key == ord("d"):
            if lines:
                print(f"[zones] видалено лінію «{lines.pop().name}»")
            elif zones:
                print(f"[zones] видалено зону «{zones.pop().name}»")

        if key == ord("s"):
            save_config(args.out, zones, lines)
            print(f"[zones] збережено: {args.out} "
                  f"(зон {len(zones)}, ліній {len(lines)})")
            break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
