# VideoDetection

Детекція людей і аналіз поведінки на відео з Freenove ESP32-S3-WROOM CAM.
Повний план — [docs/PLAN.md](docs/PLAN.md).

## Головне про цей проект

**Це навчальний проект.** Основна ціль власника — самому вивчити ML/OpenCV/YOLO,
сфера для нього нова. Тому: пояснювати концепції перед кодом, називати терміни
правильно й розшифровувати при першій появі, не пропускати етапи заради швидкого
результату. Спілкування українською.

Власник має солідний досвід з ESP32 та сенсорами — ембедед-базу пояснювати не треба.

## Структура

```
firmware/esp32s3_cam/   PlatformIO-проект прошивки ESP32-S3 (MJPEG-сервер)
hub/                    Python: прийом потоку, YOLO, веб-віддача, тренування
docs/                   план, нотатки, результати бенчмарків
models/                 ваги (у git не потрапляють)
datasets/               датасети (у git не потрапляють)
```

## Залізо

- **Freenove ESP32-S3-WROOM CAM**, OV2640. Пінаут камери = профіль
  `CAMERA_MODEL_ESP32S3_EYE`, зафіксований у `firmware/esp32s3_cam/src/camera_pins.h`.
  Референс-приклади Freenove: `C:\ESP_dev\Freenove_Ultimate_Starter_Kit_for_ESP32_S3-main\C\Sketches\`
- **Ноут** Intel Core Ultra 7 155H, 32GB — розробка й навчання моделей (OpenVINO: iGPU Arc + NPU).
  Є локальна Ollama.
- **RPi 5 8GB** — цільова платформа для цілодобового деплою.

## Важливі обмеження

- **Глядачі потоку ділять спільну пропускну здатність.** Камера обслуговує
  кількох клієнтів, але сумарний FPS не росте: два клієнти дали 13.3 + 7.4 там,
  де один отримував 21. Відкрита вкладка браузера з `http://esp32cam.local/`
  вдвічі ріже FPS у Python-клієнта — під час замірів її треба закривати.
  `/status` на порту 80 живий завжди, поле `clients` показує кількість глядачів.
- Камера доступна як `esp32cam.local` (mDNS) або `192.168.1.48`.

## Оточення

- Python 3.14 (колеса cp314 є для torch/ultralytics/opencv/openvino — перевірено)
- venv у `.venv/`, залежності: `hub/requirements.txt` (легкі) і `hub/requirements-ml.txt` (torch і Ко)
- PlatformIO не в PATH: `C:\Users\Lenovo\.platformio\penv\Scripts\pio.exe`

## Команди

```bash
# зібрати й залити прошивку
C:\Users\Lenovo\.platformio\penv\Scripts\pio.exe run -d firmware/esp32s3_cam -t upload

# монітор порту
C:\Users\Lenovo\.platformio\penv\Scripts\pio.exe device monitor -d firmware/esp32s3_cam

# хаб з детекцією по HTTPS (токен друкується при старті)
.venv\Scripts\python.exe hub\server.py --model 640 --device intel:gpu

# просто подивитись потік / детекцію локально
.venv\Scripts\python.exe hub\view.py
.venv\Scripts\python.exe hub\detect.py --model 640 --device intel:gpu --classes
```

## Хаб (етап 4)

- `hub/pipeline.py` — ЄДИНИЙ споживач камери: один читач + один прохід YOLO,
  глядачі забирають готовий кадр. Не давати кожному HTTP-клієнту свій
  MjpegCamera: перевірено, 3 глядачі не просідають (19.7/19.9/19.8 fps).
- `hub/server.py` — FastAPI на HTTPS, порт 8443. Токен обов'язковий скрізь,
  крім `/health`. Приймається як `Authorization: Bearer` або `?token=`
  (тег `<img>` не вміє слати заголовки, а MJPEG живе саме в `<img>`).
- `secrets/` — токен, сертифікат, ключ. У git не потрапляє.
- Сертифікат самопідписаний (`hub/make_cert.py`). Браузери дивляться **тільки
  в SAN**, тому адресу треба вписати туди: `--extra <ім'я або IP> --force`.
  Коли з'явиться домен — просто підмінити файли на Let's Encrypt, код той самий.

## Конвенції

- Секрети Wi-Fi — у `firmware/esp32s3_cam/src/config.h` (gitignored),
  шаблон поруч у `config.example.h`.
- Коментарі в коді — українською, пояснювальні, а не описові
  («чому саме так», а не «збільшуємо лічильник»).
- Результати бенчмарків (FPS/точність) складаємо таблицями в `docs/`.
