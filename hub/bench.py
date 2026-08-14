"""
Етап 3 — бенчмарк бекендів інференсу.

Питання, на яке відповідає цей скрипт: скільки коштує один кадр на різному
залізі ноутбука. Core Ultra 7 155H має три різні обчислювачі, і YOLO можна
запустити на кожному:

  CPU  — універсально, працює всюди, найповільніше
  iGPU — вбудована Arc, зазвичай найшвидша для згорткових мереж
  NPU  — окремий прискорювач, повільніший за iGPU, але майже не їсть енергію
         і не заважає решті системи

PyTorch використовує тільки CPU. Щоб задіяти iGPU та NPU, модель треба
експортувати в OpenVINO — це проміжний формат Intel, у який конвертується
граф мережі. Експорт робиться один раз, далі вантажиться готова тека.

Міряємо на РЕАЛЬНИХ кадрах з нашої камери, а не на випадковому шумі:
час інференсу залежить від кількості знайдених об'єктів (постобробка й NMS),
тож синтетика збрехала б.

  python hub/bench.py                 # зібрати кадри з камери і прогнати все
  python hub/bench.py --frames 100
  python hub/bench.py --imgsz 320 640
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

import cv2

from camera import MjpegCamera

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = PROJECT_ROOT / "models"
CAPTURES_DIR = PROJECT_ROOT / "captures"

DEFAULT_URL = "http://esp32cam.local:81/stream"


def collect_frames(url: str, n: int) -> list:
    """Набирає n кадрів з камери. Якщо камера недоступна — бере з captures/."""
    frames = []
    cam = MjpegCamera(url)
    try:
        with cam:
            deadline = time.monotonic() + n / 8.0 + 15.0
            while len(frames) < n and time.monotonic() < deadline:
                f = cam.read(timeout=1.0)
                if f is not None:
                    frames.append(f)
    except Exception as exc:
        print(f"[bench] камера недоступна: {exc}")

    if len(frames) < n:
        files = sorted(CAPTURES_DIR.glob("*.jpg"))[: n - len(frames)]
        if files:
            print(f"[bench] додаю {len(files)} кадрів з captures/")
            frames += [cv2.imread(str(p)) for p in files]

    return [f for f in frames if f is not None]


def bench(model, frames, imgsz, device, warmup=5) -> dict:
    """Прогін по кадрах. Повертає медіану й p95 часу інференсу в мс."""
    for f in frames[:warmup]:
        model.predict(f, imgsz=imgsz, device=device, classes=[0], verbose=False)

    times, dets = [], 0
    for f in frames:
        t0 = time.perf_counter()
        r = model.predict(f, imgsz=imgsz, device=device, classes=[0],
                          conf=0.35, verbose=False)[0]
        times.append((time.perf_counter() - t0) * 1000)
        dets += len(r.boxes)

    times.sort()
    return {
        "median_ms": statistics.median(times),
        "p95_ms": times[int(len(times) * 0.95)],
        "fps": 1000.0 / statistics.median(times),
        "dets": dets,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Бенчмарк бекендів YOLO")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--weights", default=str(MODELS_DIR / "yolo11n.pt"))
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--imgsz", type=int, nargs="+", default=[320, 640])
    ap.add_argument("--devices", nargs="+",
                    default=["cpu", "intel:cpu", "intel:gpu", "intel:npu"])
    args = ap.parse_args()

    from ultralytics import YOLO

    MODELS_DIR.mkdir(parents=True, exist_ok=True)

    print(f"[bench] набираю {args.frames} кадрів...")
    frames = collect_frames(args.url, args.frames)
    if not frames:
        print("[bench] немає жодного кадру — ні з камери, ні з captures/")
        return 1
    h, w = frames[0].shape[:2]
    print(f"[bench] {len(frames)} кадрів {w}x{h}\n")

    pt_model = YOLO(args.weights)

    # Експорт в OpenVINO робиться під КОНКРЕТНИЙ imgsz: граф фіксує розмір
    # входу, тож для 320 і 640 потрібні дві різні теки. Експорт кешуємо —
    # він займає десятки секунд, а результат не змінюється.
    ov_models: dict[int, object] = {}
    if any(d.startswith("intel") for d in args.devices):
        import shutil

        for size in args.imgsz:
            target = MODELS_DIR / f"yolo11n_ov_{size}"
            if not target.exists():
                print(f"[bench] експорт в OpenVINO для imgsz={size}...")
                exported = Path(YOLO(args.weights).export(format="openvino", imgsz=size))
                shutil.move(str(exported), str(target))
            ov_models[size] = YOLO(str(target))

    rows = []
    for size in args.imgsz:
        for device in args.devices:
            model = pt_model if device == "cpu" else ov_models.get(size)
            backend = "PyTorch" if device == "cpu" else "OpenVINO"
            if model is None:
                continue
            try:
                res = bench(model, frames, size, device)
                rows.append((backend, device, size, res))
                print(f"  {backend:9s} {device:10s} imgsz {size}: "
                      f"{res['median_ms']:6.1f} ms  ({res['fps']:5.1f} fps)")
            except Exception as exc:
                print(f"  {backend:9s} {device:10s} imgsz {size}: НЕДОСТУПНО "
                      f"({type(exc).__name__}: {str(exc)[:70]})")

    print("\n" + "-" * 62)
    print(f"{'бекенд':10s} {'пристрій':11s} {'imgsz':>6s} {'median':>9s} {'p95':>8s} {'fps':>7s}")
    for backend, device, size, r in rows:
        print(f"{backend:10s} {device:11s} {size:6d} "
              f"{r['median_ms']:8.1f}м {r['p95_ms']:7.1f}м {r['fps']:7.1f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
