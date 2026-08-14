"""
Етап 5 — вимірювання якості трекінгу.

«На вигляд стабільніше» — не метрика. Тут рахуємо те, що можна порівняти
між налаштуваннями:

  ID switches  — скільки НОВИХ ID заведено за прогін. В ідеалі дорівнює
                 кількості людей, які реально проходили перед камерою.
                 Якщо перед камерою одна людина, а ID заведено 9 — трекер
                 губить трек і заводить новий дев'ять разів.

  ID/людину    — головне число. Скільки ID припадає на одну людину в кадрі.
                 1.0 = ідеал, 2.0 = кожен трек рветься навпіл.

  середня      — скільки секунд живе трек до розриву. Для правил
  тривалість     «затримався в зоні на 30 с» це критично: якщо треки
  треку          рвуться кожні 3 секунди, таке правило не спрацює ніколи.

Приклад порівняння:
  python hub/track_quality.py --conf 0.35 --seconds 60
  python hub/track_quality.py --conf 0.10 --seconds 60

Прогін треба робити з людиною в кадрі, інакше міряти нічого.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

from camera import MjpegCamera
from detect import resolve_model
from tracking import TrackHistory

DEFAULT_URL = os.environ.get("CAM_URL", "http://esp32cam.local:81/stream")


def main() -> int:
    ap = argparse.ArgumentParser(description="Якість трекінгу в числах")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--model", default="640")
    ap.add_argument("--device", default="intel:gpu")
    ap.add_argument("--conf", type=float, default=0.10)
    ap.add_argument("--imgsz", type=int, default=None)
    ap.add_argument("--tracker", default="bytetrack.yaml")
    ap.add_argument("--seconds", type=float, default=60.0)
    args = ap.parse_args()

    if args.imgsz is None:
        args.imgsz = int(args.model) if args.model.isdigit() else 640

    from ultralytics import YOLO

    model_path = resolve_model(args.model)
    is_ov = model_path.rstrip("/\\").endswith("_openvino_model")
    model = YOLO(model_path, task="detect") if is_ov else YOLO(model_path)

    history = TrackHistory()
    cam = MjpegCamera(args.url)

    # Тривалість життя треків, які встигли зникнути за час прогону
    finished: list[float] = []
    seen_before: dict[int, float] = {}

    print(f"[track] conf={args.conf} tracker={args.tracker} imgsz={args.imgsz}")
    print(f"[track] {args.seconds:.0f} с — стань перед камерою і рухайся як зазвичай\n")

    frames = frames_with_person = 0
    persons_per_frame: list[int] = []
    t_end = time.monotonic() + args.seconds
    last_print = 0.0

    with cam:
        while time.monotonic() < t_end:
            frame = cam.read(timeout=1.0)
            if frame is None:
                continue

            r = model.track(frame, persist=True, tracker=args.tracker,
                            conf=args.conf, imgsz=args.imgsz, classes=[0],
                            device=args.device, verbose=False)[0]

            known = set(history.tracks)
            history.update(r.boxes, model.names)
            for tid in known - set(history.tracks):
                # трек зник — фіксуємо, скільки він прожив
                if tid in seen_before:
                    finished.append(seen_before.pop(tid))
            for tid, t in history.tracks.items():
                seen_before[tid] = t.dwell_s

            frames += 1
            n = len(r.boxes)
            persons_per_frame.append(n)
            if n:
                frames_with_person += 1

            now = time.monotonic()
            if now - last_print >= 2.0:
                left = t_end - now
                print(f"\r  залишилось {left:4.0f} с | активних {history.active} | "
                      f"усього ID {history.total_seen}", end="", flush=True)
                last_print = now

    finished.extend(seen_before.values())
    avg_persons = statistics.mean(persons_per_frame) if persons_per_frame else 0.0
    coverage = frames_with_person / frames if frames else 0.0

    print("\n" + "-" * 54)
    print(f"кадрів оброблено          {frames}")
    print(f"кадрів з людиною          {frames_with_person}  ({coverage:.0%})")
    print(f"людей у кадрі в середньому {avg_persons:.2f}")
    print(f"унікальних ID за прогін   {history.total_seen}")
    if avg_persons > 0.05:
        print(f"ID на людину              {history.total_seen / max(avg_persons, 0.01):.1f}"
              "   (менше = краще, 1.0 = ідеал)")
    if finished:
        print(f"середня тривалість треку  {statistics.mean(finished):.1f} с"
              f"   (медіана {statistics.median(finished):.1f} с)")
    print("-" * 54)
    return 0


if __name__ == "__main__":
    sys.exit(main())
