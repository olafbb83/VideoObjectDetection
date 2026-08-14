"""
Етап 2 — переглядач потоку з ESP32-S3 CAM.

Що тут можна побачити й навіщо це потрібно далі:

  * FPS прийому (recv) vs FPS показу (show). Зараз вони майже рівні.
    На етапі 3, коли між ними стане YOLO, show просяде — і різниця
    recv - show покаже, скільки кадрів ми викидаємо, щоб не накопичувати
    затримку. Це нормальна робота системи, а не втрата даних.

  * drop — лічильник тих самих викинутих кадрів (див. camera.py).

Гарячі клавіші:
  q / Esc — вихід
  s       — зберегти поточний кадр у captures/
  c       — увімкнути/вимкнути автозбір кадрів (для датасету на етапі 6)
  i       — сховати/показати накладку зі статистикою

Приклади:
  python hub/view.py --url http://192.168.1.50:81/stream
  python hub/view.py --capture-interval 2      # кадр кожні 2 с у captures/
  python hub/view.py --record out.mp4
  python hub/view.py --no-window --seconds 30  # тільки заміряти, без вікна
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2

from camera import MjpegCamera, probe_status

# cv2.waitKey(1) на Windows коштує НЕ 1 мс, а ~15 мс: він реалізований через
# системний таймер із гранулярністю 15.6 мс, тож мінімальна пауза — один тік.
# Це саме по собі опускає стелю показу до ~64 FPS, а на практиці різало
# нам показ з 22 до 10 FPS. cv2.pollKey() робить те саме без сну (~0.4 мс).
_poll_key = getattr(cv2, "pollKey", None) or (lambda: cv2.waitKey(1))

PROJECT_ROOT = Path(__file__).resolve().parent.parent
CAPTURES_DIR = PROJECT_ROOT / "captures"

DEFAULT_URL = os.environ.get("CAM_URL", "http://esp32cam.local:81/stream")


def draw_overlay(frame, cam: MjpegCamera, show_fps: float, capturing: bool):
    """Малює напівпрозору панель зі статистикою у верхньому лівому куті."""
    s = cam.stats
    lines = [
        f"recv {s.recv_fps:5.1f} fps   show {show_fps:5.1f} fps",
        f"frame {s.last_frame_kb:5.1f} KB   drop {s.frames_dropped}",
        f"{'ONLINE' if s.connected else 'OFFLINE'}   reconnects {s.reconnects}",
    ]
    if capturing:
        lines.append("CAPTURING")
    if s.last_error:
        lines.append(s.last_error[:60])

    pad, line_h = 8, 20
    h = pad * 2 + line_h * len(lines)
    w = 330

    # Підкладка: копіюємо ділянку, заливаємо, змішуємо — так текст читається
    # і на світлому, і на темному фоні
    roi = frame[0:h, 0:w]
    cv2.rectangle(roi, (0, 0), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(roi, 0.55, frame[0:h, 0:w], 0.45, 0, frame[0:h, 0:w])

    for i, text in enumerate(lines):
        y = pad + line_h * (i + 1) - 5
        color = (80, 220, 80) if s.connected else (80, 80, 240)
        cv2.putText(frame, text, (pad, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    return frame


def diagnose(cam: MjpegCamera) -> None:
    """
    Пояснює, ЧОМУ немає потоку. Найчастіша причина не в мережі:
    ESP32 віддає потік лише одному клієнту, і його зазвичай тримає
    відкрита вкладка браузера.
    """
    print(f"\n[view] немає потоку: {cam.stats.last_error[:80]}")

    st = probe_status(cam.url)
    if st is None:
        print("[view] камера не відповідає і на /status — перевір живлення й Wi-Fi")
    elif st.get("streaming"):
        print(f"[view] камера жива (uptime {st.get('uptime_s')} с, {st.get('fps')} fps), "
              "але потік ЗАЙНЯТИЙ іншим клієнтом.")
        print("[view] закрий вкладку браузера з відео — ESP32 тримає лише одного глядача")
    else:
        print(f"[view] камера жива й вільна: {st} — схоже на проблему мережі до порту 81")


def save_frame(frame, reason: str) -> Path:
    CAPTURES_DIR.mkdir(parents=True, exist_ok=True)
    name = f"{datetime.now():%Y%m%d_%H%M%S_%f}_{reason}.jpg"
    path = CAPTURES_DIR / name
    cv2.imwrite(str(path), frame)
    return path


def main() -> int:
    ap = argparse.ArgumentParser(description="Переглядач MJPEG-потоку з ESP32-S3 CAM")
    ap.add_argument("--url", default=DEFAULT_URL, help=f"URL потоку (типово {DEFAULT_URL})")
    ap.add_argument("--capture-interval", type=float, default=0.0,
                    help="автозбір кадрів у captures/ кожні N секунд (0 = вимкнено)")
    ap.add_argument("--record", metavar="FILE.mp4", help="писати відео у файл")
    ap.add_argument("--no-window", action="store_true", help="без вікна, тільки статистика в консоль")
    ap.add_argument("--seconds", type=float, default=0.0, help="автовихід через N секунд (0 = без ліміту)")
    args = ap.parse_args()

    print(f"[view] підключаюсь до {args.url}")
    print("[view] q — вихід, s — зберегти кадр, c — автозбір, i — накладка")

    cam = MjpegCamera(args.url)
    writer = None
    capturing = args.capture_interval > 0
    show_overlay = True

    show_fps = 0.0
    fps_t0, fps_n = time.monotonic(), 0
    last_capture = 0.0
    last_report = 0.0
    started = time.monotonic()

    try:
        with cam:
            while True:
                # Ліміт часу перевіряємо ПЕРЕД читанням: інакше, якщо кадри
                # взагалі не йдуть, --seconds ніколи не спрацює і скрипт зависне
                now = time.monotonic()
                if args.seconds and now - started >= args.seconds:
                    break

                frame = cam.read(timeout=1.0)

                if frame is None:
                    if not cam.stats.connected and now - last_report >= 3.0:
                        last_report = now
                        diagnose(cam)
                    continue

                # FPS показу — рахуємо тут, бо він принципово відрізняється
                # від FPS прийому, щойно між ними з'явиться обробка
                fps_n += 1
                if now - fps_t0 >= 1.0:
                    show_fps = fps_n / (now - fps_t0)
                    fps_n, fps_t0 = 0, now

                if capturing and now - last_capture >= args.capture_interval:
                    save_frame(frame, "auto")
                    last_capture = now

                if args.record:
                    if writer is None:
                        h, w = frame.shape[:2]
                        writer = cv2.VideoWriter(
                            args.record, cv2.VideoWriter_fourcc(*"mp4v"), 25.0, (w, h)
                        )
                        print(f"\n[view] пишу відео у {args.record} ({w}x{h})")
                    writer.write(frame)

                if args.no_window:
                    if now - last_report >= 1.0:
                        s = cam.stats
                        print(f"\rrecv {s.recv_fps:5.1f} fps | show {show_fps:5.1f} fps | "
                              f"{s.last_frame_kb:5.1f} KB | drop {s.frames_dropped} | "
                              f"reconnects {s.reconnects}", end="", flush=True)
                        last_report = now
                else:
                    # copy(): накладка не повинна потрапити ні в запис, ні в датасет
                    display = draw_overlay(frame.copy(), cam, show_fps, capturing) if show_overlay else frame
                    cv2.imshow("ESP32-S3 CAM", display)

                    key = _poll_key() & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key == ord("s"):
                        print(f"\n[view] збережено {save_frame(frame, 'manual').name}")
                    if key == ord("c"):
                        capturing = not capturing
                        if capturing and args.capture_interval <= 0:
                            args.capture_interval = 1.0
                        print(f"\n[view] автозбір: {'увімкнено' if capturing else 'вимкнено'}")
                    if key == ord("i"):
                        show_overlay = not show_overlay

                if args.seconds and now - started >= args.seconds:
                    break

    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()

    s = cam.stats
    total_mb = s.bytes_received / (1024 * 1024)
    print(f"\n[view] підсумок: {s.frames_received} кадрів, {s.frames_dropped} викинуто, "
          f"{total_mb:.1f} МБ, перепідключень {s.reconnects}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
