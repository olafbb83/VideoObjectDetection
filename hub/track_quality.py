"""
Етап 5 — вимірювання якості трекінгу.

«На вигляд стабільніше» — не метрика. Тут рахуємо те, що можна порівняти:

  унікальних ID   — скільки треків заведено за прогін. В ідеалі дорівнює
                    кількості людей, які реально проходили перед камерою.
                    9 ID на одну людину = трекер губить її дев'ять разів.

  тривалість      — скільки секунд живе трек до розриву. Для правил
  треку             «затримався в зоні 30 с» це критично: якщо треки
                    рвуться кожні 3 секунди, таке правило не спрацює ніколи.

ВАЖЛИВО про метод. Порівнювати конфігурації на живій камері майже безглуздо:
в кожному прогоні людина рухається інакше, і різниця налаштувань тоне в цій
різниці. Тому спочатку записуємо один ролик, а потім ганяємо на ньому всі
конфігурації — вхід ідентичний, і різниця в числах означає саме налаштування.

  1) записати ролик (60 с, походи перед камерою):
     python hub/view.py --record docs/track_test.mp4 --seconds 60

  2) прогнати на ньому кілька конфігурацій:
     python hub/track_quality.py --source docs/track_test.mp4 --sweep

  3) або одну конкретну:
     python hub/track_quality.py --source docs/track_test.mp4 --conf 0.35
     python hub/track_quality.py --conf 0.35 --seconds 60      # наживо
"""

from __future__ import annotations

import argparse
import copy
import os
import statistics
import sys
import tempfile
import time
from pathlib import Path

import cv2
import yaml

from camera import MjpegCamera
from detect import TUNED_TRACKER, resolve_model
from tracking import TrackHistory

DEFAULT_URL = os.environ.get("CAM_URL", "http://esp32cam.local:81/stream")
PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Для нормування метрики. Збігається з тим, що пише view.py --record.
FPS_ASSUMED = 25.0


def frames_from_file(path: str):
    """Кадри з файлу разом із часом, на який вони припадають у записі."""
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise SystemExit(f"[track] не відкривається: {path}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    i = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        yield frame, i / fps
        i += 1
    cap.release()


def frames_from_camera(url: str, seconds: float):
    cam = MjpegCamera(url)
    with cam:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            frame = cam.read(timeout=1.0)
            if frame is not None:
                yield frame, time.monotonic() - t0


def tracker_config(base: str, overrides: dict) -> str:
    """
    Створює тимчасовий YAML трекера з підміненими параметрами.
    ultralytics приймає лише шлях до файлу, тож інакше параметри не підсунеш.
    """
    if not overrides:
        return base

    import ultralytics

    # base може бути як назвою стокового конфіга, так і шляхом до нашого
    base_path = Path(base)
    if not base_path.exists():
        base_path = Path(ultralytics.__file__).parent / "cfg" / "trackers" / base

    cfg = yaml.safe_load(base_path.read_text(encoding="utf-8"))
    cfg = copy.deepcopy(cfg)
    cfg.update(overrides)

    tmp = Path(tempfile.gettempdir()) / f"tracker_{abs(hash(str(sorted(overrides.items()))))}.yaml"
    tmp.write_text(yaml.safe_dump(cfg), encoding="utf-8")
    return str(tmp)


def run(model, frames, *, conf, imgsz, device, tracker_path, label):
    history = TrackHistory()
    finished: list[float] = []
    lifetimes: dict[int, float] = {}

    n = n_with = 0
    first = True

    for frame, ts in frames:
        # persist=False на першому кадрі скидає стан трекера — інакше
        # наступна конфігурація успадкує треки від попередньої
        r = model.track(frame, persist=not first, tracker=tracker_path,
                        conf=conf, imgsz=imgsz, classes=[0],
                        device=device, verbose=False)[0]
        first = False

        known = set(history.tracks)
        history.update(r.boxes, model.names, now=ts)
        for tid in known - set(history.tracks):
            if tid in lifetimes:
                finished.append(lifetimes.pop(tid))
        for tid, t in history.tracks.items():
            lifetimes[tid] = t.dwell_s

        n += 1
        if len(r.boxes):
            n_with += 1

    finished.extend(lifetimes.values())
    mean_life = statistics.mean(finished) if finished else 0.0
    med_life = statistics.median(finished) if finished else 0.0

    coverage = n_with / max(n, 1)

    # Головна метрика — розривів на секунду ПРИСУТНОСТІ людини, а не всього.
    #
    # Просто мінімізувати кількість ID не можна: за такою логікою найкраща
    # конфігурація та, що взагалі нічого не детектує (нуль детекцій — нуль
    # розривів). Нормування на час, коли людина реально була в кадрі, прибирає
    # цю лазівку: конфігурація, яка «виграє» пропусканням детекцій, втрачає
    # знаменник разом із чисельником і нічого не виграє.
    person_seconds = n_with / FPS_ASSUMED
    ids_per_s = history.total_seen / person_seconds if person_seconds > 0 else 0.0

    print(f"{label:<34s} {n:5d} {coverage:6.0%} "
          f"{history.total_seen:6d} {mean_life:8.1f} {med_life:8.1f} {ids_per_s:9.2f}")
    return {
        "ids": history.total_seen,
        "mean": mean_life,
        "median": med_life,
        "coverage": coverage,
        "ids_per_s": ids_per_s,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Якість трекінгу в числах")
    ap.add_argument("--source", help="файл із записом; без нього — жива камера")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--seconds", type=float, default=60.0, help="тільки для живої камери")
    ap.add_argument("--model", default="640")
    ap.add_argument("--device", default="intel:gpu")
    ap.add_argument("--imgsz", type=int, default=None)
    ap.add_argument("--conf", type=float, default=0.45)
    ap.add_argument("--tracker", default=str(TUNED_TRACKER))
    ap.add_argument("--sweep", action="store_true",
                    help="прогнати набір конфігурацій на тому самому вході")
    ap.add_argument("--sweep-set", default="base", choices=["base", "combo"],
                    help="base — окремі параметри; combo — комбінації переможців")
    ap.add_argument("--sweep-base", default="bytetrack.yaml",
                    help="від якого конфіга відштовхуються перебори. Типово "
                         "СТОКОВИЙ bytetrack.yaml, щоб результати лишались "
                         "відтворюваними після підкрутки нашого власного")
    args = ap.parse_args()

    if args.imgsz is None:
        args.imgsz = int(args.model) if args.model.isdigit() else 640

    if args.sweep and not args.source:
        print("[track] --sweep потребує --source: на живій камері прогони "
              "непорівнювані, бо вхід щоразу інший")
        return 1

    from ultralytics import YOLO

    model_path = resolve_model(args.model)
    is_ov = model_path.rstrip("/\\").endswith("_openvino_model")
    model = YOLO(model_path, task="detect") if is_ov else YOLO(model_path)

    def make_frames():
        if args.source:
            return frames_from_file(args.source)
        return frames_from_camera(args.url, args.seconds)

    print(f"\n{'конфігурація':<34s} {'кадрів':>5s} {'з люд.':>6s} "
          f"{'ID':>6s} {'сер.с':>8s} {'мед.с':>8s} {'ID/с':>9s}")
    print("-" * 82)

    if not args.sweep:
        if not args.source:
            print(f"[track] {args.seconds:.0f} с наживо — стань перед камерою\n")
        run(model, make_frames(), conf=args.conf, imgsz=args.imgsz,
            device=args.device, tracker_path=args.tracker,
            label=f"conf {args.conf} {args.tracker}")
        return 0

    # Набір гіпотез. track_buffer — скільки кадрів пам'ятати втрачений трек
    # (30 ≈ 1.2 с при 25 fps). match_thresh — наскільки СУВОРО зіставляти:
    # більше значення = трекер приймає гірший збіг замість заведення нового ID.
    SWEEPS = {
        "base": [
            ("базова (conf 0.35)", 0.35, {}),
            ("conf 0.25", 0.25, {}),
            ("conf 0.50", 0.50, {}),
            ("buffer 60 (2.4 c)", 0.35, {"track_buffer": 60}),
            ("buffer 90 (3.6 c)", 0.35, {"track_buffer": 90}),
            ("buffer 90 + match 0.9", 0.35, {"track_buffer": 90, "match_thresh": 0.9}),
            ("buffer 90 + new_thresh 0.5", 0.35, {"track_buffer": 90, "new_track_thresh": 0.5}),
        ],
        # Комбінації переможців першого раунду: conf 0.50 дав найдовші треки,
        # match_thresh 0.9 — найкращу медіану. Разом їх ще не перевіряли.
        "combo": [
            ("базова (conf 0.35)", 0.35, {}),
            ("conf 0.50", 0.50, {}),
            ("match 0.9", 0.35, {"match_thresh": 0.9}),
            ("conf 0.50 + match 0.9", 0.50, {"match_thresh": 0.9}),
            ("conf 0.50 + match 0.95", 0.50, {"match_thresh": 0.95}),
            ("conf 0.50 + match 0.9 + buf 90", 0.50, {"match_thresh": 0.9, "track_buffer": 90}),
            ("conf 0.45 + match 0.9 + new 0.5", 0.45,
             {"match_thresh": 0.9, "new_track_thresh": 0.5}),
            ("conf 0.60 + match 0.9", 0.60, {"match_thresh": 0.9}),
        ],
    }
    configs = SWEEPS[args.sweep_set]

    results = []
    for label, conf, overrides in configs:
        path = tracker_config(args.sweep_base, overrides)
        res = run(model, make_frames(), conf=conf, imgsz=args.imgsz,
                  device=args.device, tracker_path=path, label=label)
        results.append((label, res))

    print("-" * 82)
    best = min(results, key=lambda kv: kv[1]["ids_per_s"])
    b = best[1]
    print(f"\nнайменше розривів на секунду присутності: {best[0]}")
    print(f"   {b['ids_per_s']:.2f} ID/с | покриття {b['coverage']:.0%} | "
          f"треки живуть у середньому {b['mean']:.1f} с (медіана {b['median']:.1f} с)")
    print("\nЗвіряй з покриттям: конфігурація може «виграти» просто тим, що")
    print("детектує менше людей. Падіння покриття — це втрачені кадри.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
