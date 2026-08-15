"""
Етап 3 — детекція людей у живому потоці з ESP32-S3 CAM.

Три параметри, які визначають майже все, що ти побачиш:

  conf  — поріг впевненості. Модель дає кожній рамці число 0..1, усе нижче
          порогу відкидається. Нижчий поріг = більше знайдених людей, але й
          більше хибних спрацювань. Вищий = чисто, але модель губить людей
          у півоберта, частково перекритих чи далеко.

  iou   — поріг для NMS (Non-Maximum Suppression). Модель видає СОТНІ
          перекритих рамок на одну людину. NMS лишає найвпевненішу і викидає
          ті, що перекриваються з нею більше ніж на iou. Дві людини поруч
          злились в одну рамку — iou замалий. Одна людина обведена трьома
          рамками — завеликий.

  imgsz — до якого розміру кадр масштабується перед подачею в модель.
          Найдорожчий параметр: час інференсу росте приблизно квадратично.
          640 — стандарт, 320 вчетверо швидше, але дрібні фігури зникають.

Клас 0 у COCO — це `person`. Модель уміє ще 79 класів, але ми одразу
фільтруємо, щоб не витрачати час на малювання котів і стільців.

Шлях до моделі можна не писати повністю: `--model 640` розкривається в
models/yolo11n_640_openvino_model і сам виставляє відповідний imgsz.
Відносні шляхи рахуються від кореня проекту, а не від поточної теки —
скрипт можна запускати звідки завгодно.

Приклади:
  python hub/detect.py                                  # PyTorch CPU, imgsz 640
  python hub/detect.py --model 640 --device intel:gpu   # найшвидше
  python hub/detect.py --model 320 --device intel:npu   # найстабільніше
  python hub/detect.py --model 640 --device intel:gpu --conf 0.1
  python hub/detect.py --no-window --seconds 30
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import cv2

from camera import MjpegCamera, probe_status
from tracking import TrackHistory, draw_tracks
from view import _poll_key, save_frame

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = PROJECT_ROOT / "models"
TUNED_TRACKER = Path(__file__).resolve().parent / "trackers" / "bytetrack_tuned.yaml"

DEFAULT_URL = os.environ.get("CAM_URL", "http://esp32cam.local:81/stream")

# Ultralytics качає ваги в поточну теку; хочемо, щоб вони лежали в models/
os.environ.setdefault("YOLO_CONFIG_DIR", str(MODELS_DIR / ".ultralytics"))

PERSON_COLOR = (80, 220, 80)
TEXT_COLOR = (20, 20, 20)


def class_color(cls_id: int):
    """
    Свій колір кожному класу, щоб у кадрі з різними об'єктами було видно,
    де що. person лишаємо зеленим — він для нас головний.
    """
    if cls_id == 0:
        return PERSON_COLOR
    # детермінований розкид по відтінках: той самий клас завжди того ж кольору
    h = (cls_id * 47) % 180
    import numpy as np

    hsv = np.uint8([[[h, 200, 240]]])
    b, g, r = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
    return int(b), int(g), int(r)


def resolve_model(path: str) -> str:
    """
    Дозволяє передавати шлях до моделі відносно кореня проекту, а не поточної
    теки. Інакше `--model models/...` працює тільки якщо запускати скрипт
    саме з C:\\ESP_dev\\projects\\VideoDetection, що неочевидно і легко забути.

    Також приймає скорочення: `--model 640` -> models/yolo11n_640_openvino_model
    """
    if path.isdigit():
        return str(MODELS_DIR / f"yolo11n_{path}_openvino_model")

    p = Path(path)
    if p.exists():
        return str(p)

    candidate = PROJECT_ROOT / path
    if candidate.exists():
        return str(candidate)

    return path  # хай ultralytics сам скаже, чого саме не вистачає


def draw_detections(frame, boxes, names) -> int:
    """
    Малює рамки. Повертає кількість намальованих.

    boxes — це results[0].boxes від ultralytics: .xyxy (координати кутів),
    .conf (впевненість), .cls (номер класу).
    names — словник {номер класу: назва} з самої моделі.
    """
    n = 0
    for box in boxes:
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        color = class_color(cls_id)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        label = f"{names.get(cls_id, cls_id)} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        # підкладка під текст, щоб він читався на будь-якому фоні
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
        cv2.putText(frame, label, (x1 + 3, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, TEXT_COLOR, 1, cv2.LINE_AA)
        n += 1
    return n


def summarize(boxes, names) -> str:
    """'person x1, chair x2' — що саме зараз у кадрі."""
    counts: dict[str, int] = {}
    for box in boxes:
        name = names.get(int(box.cls[0]), "?")
        counts[name] = counts.get(name, 0) + 1
    if not counts:
        return "-"
    top = sorted(counts.items(), key=lambda kv: -kv[1])[:4]
    return ", ".join(f"{k} x{v}" for k, v in top)


def draw_hud(frame, *, recv_fps, show_fps, speed, summary, conf, imgsz, tracks=None):
    """Накладка: окремо FPS прийому, FPS показу і розклад часу інференсу."""
    pre = speed.get("preprocess", 0.0)
    inf = speed.get("inference", 0.0)
    post = speed.get("postprocess", 0.0)

    lines = [
        f"recv {recv_fps:5.1f} fps   show {show_fps:5.1f} fps",
        f"infer {inf:5.1f} ms  (pre {pre:.1f} / post {post:.1f})",
        f"conf {conf}   imgsz {imgsz}",
        summary[:44],
    ]
    if tracks is not None:
        # total росте тільки коли з'являється НОВИЙ ID. Якщо він біжить угору
        # при одній людині в кадрі — трекер губить трек і заводить новий
        lines.append(f"треків {tracks.active}   унікальних за сеанс {tracks.total_seen}")

    pad, line_h, w = 8, 20, 340
    h = pad * 2 + line_h * len(lines)
    roi = frame[0:h, 0:w]
    cv2.rectangle(roi, (0, 0), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(roi, 0.55, frame[0:h, 0:w], 0.45, 0, frame[0:h, 0:w])

    for i, text in enumerate(lines):
        y = pad + line_h * (i + 1) - 5
        cv2.putText(frame, text, (pad, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (230, 230, 230), 1, cv2.LINE_AA)
    return frame


def main() -> int:
    ap = argparse.ArgumentParser(description="Детекція людей у потоці з ESP32-S3 CAM")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--model", default=str(MODELS_DIR / "yolo11n.pt"),
                    help="файл ваг .pt, тека *_openvino_model, "
                         "або просто розмір входу: 320 / 640")
    ap.add_argument("--device", default=None,
                    help="cpu | intel:cpu | intel:gpu | intel:npu (для OpenVINO)")
    ap.add_argument("--conf", type=float, default=None,
                    help="поріг впевненості (типово 0.35; з --track 0.1, див. нижче)")
    ap.add_argument("--iou", type=float, default=0.45, help="поріг NMS")
    ap.add_argument("--imgsz", type=int, default=None,
                    help="розмір входу моделі (типово 640, або той, під який "
                         "експортовано модель)")
    ap.add_argument("--classes", type=int, nargs="*", default=[0],
                    help="номери класів COCO (0 = person). "
                         "Без значень (--classes) = показувати всі 80")
    ap.add_argument("--list-classes", action="store_true",
                    help="показати всі класи, які модель уміє розпізнавати, і вийти")
    ap.add_argument("--track", action="store_true",
                    help="увімкнути трекінг: стабільні ID, хвости траєкторій, час у кадрі")
    ap.add_argument("--tracker", default=str(TUNED_TRACKER),
                    help="типово наш підібраний bytetrack_tuned.yaml. "
                         "Для порівняння: bytetrack.yaml (стоковий) або "
                         "botsort.yaml (точніший, але важчий: re-ID + "
                         "компенсація руху камери)")
    ap.add_argument("--no-trail", action="store_true", help="не малювати хвости траєкторій")
    ap.add_argument("--save-detections", action="store_true",
                    help="зберігати кадри, де знайдено людей, у captures/")
    ap.add_argument("--no-window", action="store_true")
    ap.add_argument("--seconds", type=float, default=0.0)
    args = ap.parse_args()

    # OpenVINO-модель експортується під ФІКСОВАНИЙ розмір входу — граф його
    # зашиває. Якщо модель задана скороченням (--model 320), imgsz має збігтися,
    # інакше ultralytics подасть у мережу кадр не того розміру.
    if args.imgsz is None:
        args.imgsz = int(args.model) if args.model.isdigit() else 640

    # Поріг 0.35 і в режимі трекінгу — це результат заміру, а не замовчування
    # «щоб було однаково».
    #
    # Теорія підказувала протилежне. Трекінг виконується в
    # on_predict_postprocess_end, тобто ПІСЛЯ NMS: усе, що не пройшло conf,
    # трекер не побачить. А bytetrack.yaml побудований навколо слабких
    # детекцій (track_low_thresh 0.1) — здавалося б, поріг треба знизити,
    # щоб другий прохід зіставлення взагалі отримав вхід.
    #
    # Замір на записі docs/track_test.mp4 (883 кадри, docs/benchmarks.md):
    #   conf 0.10 -> 14 ID (наживо), треки рвуться щосекунди
    #   conf 0.35 -> 18 ID, медіана треку 0.7 с
    #   conf 0.45 + tuned tracker -> 6 ID, медіана 3.9 с
    #
    # Причина, чому низький поріг шкодить: слабкі детекції тут переважно
    # сміття, а не затулена людина. Трек зповзає на сміття, справжня людина
    # перестає з ним зіставлятись, заводиться новий ID. У bytetrack.yaml це
    # названо «recovery vs drift»: на чистому відео виграє recovery,
    # на шумному — drift.
    #
    # Для трекінгу поріг вищий (0.45), бо там ціна хибної детекції — розрив
    # треку. Для чистої детекції лишаємо 0.35: краще покриття кадрів.
    if args.conf is None:
        args.conf = 0.45 if args.track else 0.35

    from ultralytics import YOLO  # імпорт тут: він важкий, ~3 с

    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    model_path = resolve_model(args.model)
    print(f"[detect] завантажую модель {model_path}")
    # У теці OpenVINO немає метаданих про задачу — без явного task ultralytics
    # лише вгадує її й сипле попередженням
    is_ov = model_path.rstrip("/\\").endswith("_openvino_model")
    model = YOLO(model_path, task="detect") if is_ov else YOLO(model_path)
    names = model.names

    if args.list_classes:
        print(f"\nМодель уміє розпізнавати {len(names)} класів:\n")
        for i in range(0, len(names), 4):
            row = "  ".join(f"{k:2d} {names[k]:<16s}" for k in range(i, min(i + 4, len(names))))
            print("  " + row)
        return 0

    # Порожній список від argparse (--classes без значень) означає "усі класи".
    # Для ultralytics "усі" — це None, а НЕ порожній список.
    classes = args.classes if args.classes else None
    print(f"[detect] класи: {'усі 80' if classes is None else [names[c] for c in classes]}")

    st = probe_status(args.url)
    if st and st.get("clients", 0) > 0:
        print(f"[detect] УВАГА: до потоку вже підключено {st['clients']} глядачів — "
              "вони ділять пропускну здатність")

    history = TrackHistory() if args.track else None
    if history is not None:
        print(f"[detect] трекінг: {args.tracker}")

    cam = MjpegCamera(args.url)
    show_fps, fps_t0, fps_n = 0.0, time.monotonic(), 0
    last_report, started = 0.0, time.monotonic()
    infer_ms_sum, infer_n = 0.0, 0

    print("[detect] q — вихід, s — зберегти кадр")

    try:
        with cam:
            while True:
                now = time.monotonic()
                if args.seconds and now - started >= args.seconds:
                    break

                frame = cam.read(timeout=1.0)
                if frame is None:
                    continue

                # verbose=False — інакше ultralytics друкує рядок на КОЖЕН кадр
                common = dict(conf=args.conf, iou=args.iou, imgsz=args.imgsz,
                              classes=classes, device=args.device, verbose=False)

                if history is not None:
                    # persist=True критично: без нього трекер скидає стан на
                    # кожному виклику, і кожен кадр отримує нові ID з нуля
                    r = model.track(frame, persist=True, tracker=args.tracker, **common)[0]
                    history.update(r.boxes, names)
                else:
                    r = model.predict(frame, **common)[0]

                # Малюємо по копії: у captures/ мають лежати чисті кадри,
                # інакше ми навчимо майбутню модель на власних рамках
                display = frame.copy()
                if history is not None:
                    n_det = draw_tracks(display, r.boxes, names, history,
                                        show_trail=not args.no_trail)
                else:
                    n_det = draw_detections(display, r.boxes, names)

                infer_ms_sum += r.speed.get("inference", 0.0)
                infer_n += 1

                if args.save_detections and n_det:
                    save_frame(frame, f"det{n_det}")

                fps_n += 1
                if now - fps_t0 >= 1.0:
                    show_fps = fps_n / (now - fps_t0)
                    fps_n, fps_t0 = 0, now

                if args.no_window:
                    if now - last_report >= 1.0:
                        extra = (f" | треків {history.active}/{history.total_seen}"
                                 if history is not None else "")
                        print(f"\rrecv {cam.stats.recv_fps:5.1f} | show {show_fps:5.1f} | "
                              f"infer {r.speed.get('inference', 0):5.1f} ms | "
                              f"{summarize(r.boxes, names)}{extra}", end="", flush=True)
                        last_report = now
                else:
                    draw_hud(display, recv_fps=cam.stats.recv_fps, show_fps=show_fps,
                             speed=r.speed, summary=summarize(r.boxes, names),
                             conf=args.conf, imgsz=args.imgsz, tracks=history)
                    cv2.imshow("YOLO — person detection", display)

                    key = _poll_key() & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key == ord("s"):
                        print(f"\n[detect] збережено {save_frame(frame, 'manual').name}")

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

    if infer_n:
        print(f"\n[detect] середній інференс: {infer_ms_sum / infer_n:.1f} мс "
              f"({1000 * infer_n / infer_ms_sum:.1f} fps стеля моделі)")
    print(f"[detect] кадрів прийнято {cam.stats.frames_received}, "
          f"викинуто {cam.stats.frames_dropped}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
