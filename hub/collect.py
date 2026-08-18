"""
Етап 6 — збір кадрів для власного датасету.

ЩО МИ ЗБИРАЄМО: кадри для донавчання ДЕТЕКТОРА людей — моделі, яка відповідає
«чи є тут людина і де саме». Не «хто це»: ідентифікація конкретних людей —
інша задача (re-ID / face recognition) і в наш scope не входить.

Тому важлива не кількість людей, а ПОКРИТТЯ ВАРІАЦІЙ. За спаданням важливості:
масштаб (близько/далеко) -> освітлення -> поза й ракурс -> перекриття ->
позиція в кадрі -> одяг -> різні люди. Одна людина у двадцяти ситуаціях
цінніша за двадцять людей в одній.

ЧОМУ СЕСІЇ, А НЕ ПРОСТО КАДРИ

Найважливіша деталь усього етапу. Сусідні кадри відео майже однакові. Якщо
потім поділити датасет на train/val ВИПАДКОВО, у валідацію потраплять
майже-копії тренувальних кадрів. Метрики покажуть чудовий mAP, а модель
нічого нового не навчиться — вона просто впізнає бачене. Це витік даних,
і виглядає він як успіх.

Тому кадри складаються по сесіях, кожна сесія має метадані, а ділити датасет
треба ПО СЕСІЯХ: одна сесія цілком іде у валідацію і в тренуванні не бере
участі.

ПОРОЖНІ КАДРИ ПОТРІБНІ

Кадри без людей — не марна трата, а те, що вчить модель НЕ спрацьовувати.
Халат на кріслі, тіні, пересвічене вікно — класичні джерела хибних спрацювань.
Плануй ~20% датасету на порожню кімнату.

Використання:
  python hub/collect.py --session day_backlit --note "яскравий день, штори відкриті"
  python hub/collect.py --session night --interval 3 --seconds 300
  python hub/collect.py --session empty_room --note "порожня кімната" --no-model

У вікні видно, чи бачить тебе поточна модель. Це і є зворотний зв'язок:
коли рамка зникає — ти щойно створив саме той кадр, якого моделі бракує.

  q / Esc — завершити сесію
  s       — зберегти кадр негайно. ТИСНИ ЙОГО, КОЛИ РАМКА ЗНИКЛА, А ТИ В КАДРІ:
            це пропущена детекція, найцінніший тип кадру для донавчання.
            Автоматично такі кадри не зловити — код не знає, чи ти в кадрі,
            і порожня кімната для нього виглядає так само.
  space   — пауза/продовження автозбору
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2

from camera import MjpegCamera, probe_status
from view import _poll_key

PROJECT_ROOT = Path(__file__).resolve().parent.parent
RAW_DIR = PROJECT_ROOT / "datasets" / "raw"
DEFAULT_URL = os.environ.get("CAM_URL", "http://esp32cam.local:81/stream")

# Нижче цієї впевненості вважаємо, що модель «не впевнена» — такі кадри
# найцінніші для донавчання, бо саме там вона зараз помиляється.
HARD_CONF = 0.5

# Щоб складні кадри не залили датасет однаковими: не частіше ніж раз на стільки
HARD_MIN_GAP = 1.0


def utc_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")


def draw_hud(frame, *, saved, hard, elapsed, paused, boxes_info, session):
    lines = [
        f"сесія: {session}",
        f"збережено {saved}   складних {hard}   {elapsed:5.0f} с",
        boxes_info,
    ]
    if paused:
        lines.append("ПАУЗА (space)")

    pad, line_h, w = 8, 20, 330
    h = pad * 2 + line_h * len(lines)
    roi = frame[0:h, 0:w]
    cv2.rectangle(roi, (0, 0), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(roi, 0.55, frame[0:h, 0:w], 0.45, 0, frame[0:h, 0:w])

    for i, text in enumerate(lines):
        color = (80, 200, 255) if paused and i == len(lines) - 1 else (230, 230, 230)
        cv2.putText(frame, text, (pad, pad + line_h * (i + 1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    return frame


def main() -> int:
    ap = argparse.ArgumentParser(description="Збір кадрів для датасету")
    ap.add_argument("--session", required=True,
                    help="коротка назва сесії, латиницею: day_backlit, night, empty_room")
    ap.add_argument("--note", default="",
                    help="умови зйомки словами: освітлення, одяг, що робив у кадрі")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--interval", type=float, default=2.0,
                    help="секунд між автозбереженнями (типово 2)")
    ap.add_argument("--seconds", type=float, default=0.0, help="автостоп через N с")
    ap.add_argument("--model", default="640")
    ap.add_argument("--device", default="intel:gpu")
    ap.add_argument("--imgsz", type=int, default=None)
    ap.add_argument("--no-model", action="store_true",
                    help="без моделі: швидше, але в манифесті не буде впевненості "
                         "й не працюватиме дозбір складних кадрів")
    ap.add_argument("--no-hard", action="store_true",
                    help="не дозбирувати кадри, де модель невпевнена")
    ap.add_argument("--no-window", action="store_true")
    args = ap.parse_args()

    session_dir = RAW_DIR / args.session
    frames_dir = session_dir / "frames"
    if session_dir.exists():
        print(f"[collect] сесія «{args.session}» уже існує: {session_dir}")
        print("[collect] візьми іншу назву — інакше поділ по сесіях зіпсується")
        return 1
    frames_dir.mkdir(parents=True)

    model = None
    if not args.no_model:
        from detect import resolve_model
        from ultralytics import YOLO

        if args.imgsz is None:
            args.imgsz = int(args.model) if args.model.isdigit() else 640
        model_path = resolve_model(args.model)
        print(f"[collect] модель для підказок: {Path(model_path).name}")
        is_ov = model_path.rstrip("/\\").endswith("_openvino_model")
        model = YOLO(model_path, task="detect") if is_ov else YOLO(model_path)

    cam_status = probe_status(args.url)
    if cam_status and cam_status.get("clients", 0) > 0:
        print(f"[collect] УВАГА: до потоку вже підключено {cam_status['clients']} "
              "глядачів — закрий їх, інакше кадрів буде вдвічі менше")

    manifest_path = session_dir / "manifest.jsonl"
    manifest = manifest_path.open("w", encoding="utf-8")

    cam = MjpegCamera(args.url)
    saved = hard_saved = with_person = 0
    idx = 0
    paused = False
    started = time.monotonic()
    last_auto = 0.0
    last_hard = 0.0
    last_report = 0.0

    print(f"[collect] сесія «{args.session}» -> {session_dir}")
    print(f"[collect] кадр кожні {args.interval:.0f} с. "
          "q — завершити, s — зберегти зараз, space — пауза\n")

    def store(frame, reason: str, n_boxes: int, max_conf: float) -> None:
        nonlocal saved, idx, hard_saved, with_person
        idx += 1
        name = f"{args.session}_{idx:05d}.jpg"
        # ЧИСТИЙ кадр без накладок: інакше модель навчиться на наших же рамках
        cv2.imwrite(str(frames_dir / name), frame)
        manifest.write(json.dumps({
            "file": name,
            "ts": utc_now(),
            "reason": reason,           # interval | manual | hard
            "boxes": n_boxes,
            "max_conf": round(max_conf, 3) if max_conf else None,
        }, ensure_ascii=False) + "\n")
        manifest.flush()
        saved += 1
        if reason == "hard":
            hard_saved += 1
        if n_boxes:
            with_person += 1

    try:
        with cam:
            while True:
                now = time.monotonic()
                elapsed = now - started
                if args.seconds and elapsed >= args.seconds:
                    break

                frame = cam.read(timeout=1.0)
                if frame is None:
                    continue

                n_boxes, max_conf = 0, 0.0
                display = frame.copy()

                if model is not None:
                    r = model.predict(frame, conf=0.25, imgsz=args.imgsz, classes=[0],
                                      device=args.device, verbose=False)[0]
                    n_boxes = len(r.boxes)
                    for b in r.boxes:
                        c = float(b.conf[0])
                        max_conf = max(max_conf, c)
                        x1, y1, x2, y2 = (int(v) for v in b.xyxy[0])
                        color = (80, 220, 80) if c >= HARD_CONF else (60, 160, 255)
                        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(display, f"{c:.2f}", (x1 + 3, y1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

                if not paused and now - last_auto >= args.interval:
                    store(frame, "interval", n_boxes, max_conf)
                    last_auto = now

                # Дозбір складних кадрів: модель КОГОСЬ бачить, але сумнівається.
                #
                # Умова саме така, і це важливо. Спокусливо додати сюди ще
                # «модель нікого не бачить» — мовляв, пропустила людину. Але
                # порожня кімната для коду виглядає точно так само: він не знає,
                # чи є людина в кадрі.
                #
                # Наскільки легко переплутати ці два випадки — перевірено на собі:
                # у першому ж тесті модель не бачила нікого на 10 кадрах із 16,
                # і це було витлумачено як «кімната порожня». Насправді людина
                # сиділа впритул до камери, заповнюючи кадр торсом, і детектор
                # її не розпізнавав. Тобто це були найцінніші кадри, а не сміття.
                #
                # Висновок для коду: автоматично ці випадки не розділити.
                # Рівномірний збір по інтервалу однаково збереже і порожню
                # кімнату, і пропущену людину — а хто з них хто, вирішиться
                # на розмітці. Свідомо позначити пропуск може лише людина:
                # бачиш, що рамка зникла, хоча ти в кадрі — тиснеш `s`.
                if (model is not None and not args.no_hard and not paused
                        and now - last_hard >= HARD_MIN_GAP
                        and n_boxes > 0 and max_conf < HARD_CONF):
                    store(frame, "hard", n_boxes, max_conf)
                    last_hard = now

                if args.no_window:
                    if now - last_report >= 5.0:
                        last_report = now
                        print(f"  {elapsed:5.0f} с | збережено {saved} "
                              f"(складних {hard_saved})", flush=True)
                    continue

                info = (f"модель бачить: {n_boxes}, впевненість {max_conf:.2f}"
                        if model is not None else "модель вимкнена")
                draw_hud(display, saved=saved, hard=hard_saved, elapsed=elapsed,
                         paused=paused, boxes_info=info, session=args.session)
                cv2.imshow("collect — датасет", display)

                key = _poll_key() & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    store(frame, "manual", n_boxes, max_conf)
                    print(f"[collect] збережено вручну ({saved})")
                if key == ord(" "):
                    paused = not paused
                    print(f"[collect] {'пауза' if paused else 'продовжую'}")

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        manifest.close()

    duration = time.monotonic() - started
    meta = {
        "session": args.session,
        "note": args.note,
        "started": utc_now(),
        "duration_s": round(duration, 1),
        "camera_url": args.url,
        # Налаштування камери фіксуємо: якщо між сесіями вони різні, це
        # додаткова варіація, про яку потім треба знати
        "camera_status_at_start": cam_status,
        "interval_s": args.interval,
        "model": args.model if model is not None else None,
        "hard_conf_threshold": HARD_CONF if (model and not args.no_hard) else None,
        "frames_total": saved,
        "frames_hard": hard_saved,
        "frames_with_person_predicted": with_person,
    }
    (session_dir / "session.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"\n[collect] сесія «{args.session}» завершена")
    print(f"[collect] кадрів {saved} (складних {hard_saved}), "
          f"{duration:.0f} с -> {session_dir}")
    if saved and model is not None:
        print(f"[collect] модель бачила людину на {with_person}/{saved} кадрах "
              f"({with_person / saved:.0%})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
