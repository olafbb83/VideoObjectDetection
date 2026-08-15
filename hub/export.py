"""
Експорт моделей у формати прискорювачів.

Навіщо окремий скрипт: експорт робиться під КОНКРЕТНИЙ розмір входу — граф
зашиває його всередину. Для 320 і 640 потрібні дві різні теки, і плутанина
тут дає помилки, які виглядають як «модель раптом погано бачить».

Ultralytics визначає формат за СУФІКСОМ назви теки (`_openvino_model`,
`_ncnn_model`), а не за вмістом. Тому свою частину імені — розмір входу —
додаємо на початок: yolo11n_640_openvino_model.

  python hub/export.py                          # detect + pose, 320 і 640
  python hub/export.py --task pose --imgsz 640
  python hub/export.py --format ncnn            # для RPi5 (етап 7)
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = PROJECT_ROOT / "models"

WEIGHTS = {
    "detect": "yolo11n.pt",
    "pose": "yolo11n-pose.pt",
}


def export_one(task: str, imgsz: int, fmt: str, force: bool) -> Path | None:
    from ultralytics import YOLO

    weights = MODELS_DIR / WEIGHTS[task]
    stem = weights.stem                       # yolo11n або yolo11n-pose
    target = MODELS_DIR / f"{stem}_{imgsz}_{fmt}_model"

    if target.exists() and not force:
        print(f"  [=] {target.name} уже є")
        return target

    if target.exists():
        shutil.rmtree(target)

    print(f"  [>] {task} @ {imgsz} -> {fmt} ...")
    model = YOLO(str(weights))                # завантажиться/скачається сюди ж
    produced = Path(model.export(format=fmt, imgsz=imgsz))

    # export кладе результат поруч із вагами під власною назвою,
    # яка не містить розміру входу — переносимо під нашу
    shutil.move(str(produced), str(target))
    print(f"  [+] {target.name}")
    return target


def main() -> int:
    ap = argparse.ArgumentParser(description="Експорт моделей")
    ap.add_argument("--task", nargs="+", default=["detect", "pose"],
                    choices=["detect", "pose"])
    ap.add_argument("--imgsz", nargs="+", type=int, default=[320, 640])
    ap.add_argument("--format", default="openvino",
                    help="openvino (ноут) | ncnn (RPi5) | onnx")
    ap.add_argument("--force", action="store_true", help="перезаписати наявні")
    args = ap.parse_args()

    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"[export] формат {args.format}, теки в {MODELS_DIR}")

    for task in args.task:
        for size in args.imgsz:
            try:
                export_one(task, size, args.format, args.force)
            except Exception as exc:
                print(f"  [!] {task}@{size}: {type(exc).__name__}: {str(exc)[:100]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
