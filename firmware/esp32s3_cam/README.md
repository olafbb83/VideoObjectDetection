# ESP32-S3 CAM firmware

MJPEG streaming firmware for the **Freenove ESP32-S3-WROOM CAM** (OV2640),
built with PlatformIO and the Arduino framework.

The board does one job: capture frames and push them over Wi-Fi. All inference
happens on the hub — see the [project README](../../README.md).

---

## Endpoints

| URL | Port | Description |
|-----|-----:|-------------|
| `http://<ip>/` | 80 | debug page: live video, resolution buttons, live `/status` |
| `http://<ip>:81/stream` | 81 | **MJPEG stream** — this is what the hub reads |
| `http://<ip>/jpg` | 80 | single JPEG frame |
| `http://<ip>/status` | 80 | JSON telemetry |
| `http://<ip>/control?var=<name>&val=<n>` | 80 | change sensor settings at runtime |

The board also announces itself over mDNS, so `http://esp32cam.local/` works
without knowing the IP.

### Why two HTTP servers

The stream handler blocks forever in a `while(true)` loop. If it lived on
port 80, `/status` and `/control` would stop responding for as long as anything
was streaming. Splitting the stream onto its own server on port 81 keeps
telemetry alive — which is exactly what made it possible to diagnose stream
problems from the Python side.

### `/status` fields

```json
{
  "uptime_s": 383, "reset_reason": "POWERON", "rssi": -40,
  "ip": "192.168.1.48", "heap_free": 235752, "psram_free": 8247743,
  "streaming": false, "clients": 0, "fps": 0.0, "frame_kb": 14,
  "framesize": 8, "quality": 12, "xclk_mhz": 20
}
```

* **`reset_reason`** separates two entirely different classes of problem.
  `BROWNOUT` means the power supply sagged — fix the PSU or the cable.
  `PANIC` / `TASK_WDT` means the firmware crashed or hung — fix the code.
  `POWERON` / `EXT_PIN` is a normal start or the reset button.
* **`clients`** is the number of viewers currently attached to the stream.
  They share the available bandwidth: two clients measured 13.3 + 7.4 fps where
  a single one got 21. An open browser tab therefore halves what the hub gets.
* **`fps`** is the aggregate across all clients. It is computed in `loop()`
  rather than inside the stream handler, because several handler instances run
  concurrently and each would otherwise overwrite the others' value.

### `/control` variables

`framesize`, `quality`, `brightness`, `contrast`, `ae_level`, `hmirror`,
`vflip`. Useful `framesize` values: 5 = QVGA, 8 = VGA, 9 = SVGA, 11 = HD.

```bash
curl "http://esp32cam.local/control?var=framesize&val=8"
curl "http://esp32cam.local/control?var=quality&val=12"
```

Note that `quality` is inverted: **lower value means better quality** and larger
frames (valid range 0–63; 10–12 is the practical band for streaming).

---

## Build and flash

PlatformIO is not on `PATH` in this setup:

```bash
# build
C:\Users\Lenovo\.platformio\penv\Scripts\pio.exe run -d firmware/esp32s3_cam

# build, upload and open the serial monitor
C:\Users\Lenovo\.platformio\penv\Scripts\pio.exe run -d firmware/esp32s3_cam -t upload -t monitor

# serial monitor only
C:\Users\Lenovo\.platformio\penv\Scripts\pio.exe device monitor -d firmware/esp32s3_cam
```

Before the first build, create `src/config.h`:

```bash
cp src/config.example.h src/config.h
```

and fill in your **2.4 GHz** Wi-Fi credentials — the ESP32 has no 5 GHz radio.
`config.h` is git-ignored, so credentials never reach the repository. The build
fails with an explicit `#error` if the file is missing.

If the board does not enter download mode: hold `BOOT`, tap `RST`, release
`BOOT`.

### Expected boot log

```
=== VideoDetection / ESP32-S3 CAM ===
[sys] причина старту: POWERON
[sys] flash=8 МБ  psram=7 МБ  cpu=240 МГц
[cam] сенсор PID=0x26
[wifi] OK  ip=192.168.1.48  rssi=-40 dBm
[mdns] також доступна як http://esp32cam.local
[http] сторінка : http://192.168.1.48/
[http] потік    : http://192.168.1.48:81/stream
```

**Check `psram=7 МБ`.** The 8 MB module reports ~7 MB free because the camera
driver reserves the rest for frame buffers. If it reports 0, the PSRAM type is
wrong — change `board_build.psram_type` in `platformio.ini` from `opi` to `qio`.
Without PSRAM the firmware falls back to QVGA with a single frame buffer and
prints a warning; the stream still works, but VGA does not.

---

## Configuration

### `platformio.ini`

The board has no PlatformIO definition of its own, so a generic
`esp32-s3-devkitc-1` is used with the memory layout of the ESP32-S3-WROOM-1 N8R8
module spelled out: 8 MB flash (QIO) plus 8 MB PSRAM (OPI = octal).

`board_build.partitions = huge_app.csv` is required — `esp_camera` plus Wi-Fi
plus the HTTP server do not fit into the default 1.3 MB application partition.
Current usage: RAM 15.1%, flash 25.6% of 3 MB.

`upload_port` / `monitor_port` are pinned to `COM3`. The board's UART connector
is a CH343 bridge (`USB VID:PID=1A86:55D3`); the machine also has Bluetooth COM
ports that auto-detection can pick by mistake. Change these if your port
differs.

**There is deliberately no `lib_deps` entry for `esp32-camera`.** The Arduino
ESP32 core already ships that driver inside its SDK
(`tools/sdk/esp32s3/include/esp32-camera`). Adding it through `lib_deps` as
well risks compiling two different versions of the driver into one image.

### Camera pinout — `src/camera_pins.h`

Matches the `CAMERA_MODEL_ESP32S3_EYE` profile from Freenove's own example
(`Sketch_32.1_CameraWebServer`) and the board pinout in
[docs/S3CamWroom1.png](../../docs/S3CamWroom1.png).

| Signal | GPIO | | Signal | GPIO |
|--------|-----:|-|--------|-----:|
| XCLK | 15 | | Y2 (D0) | 11 |
| PCLK | 13 | | Y3 (D1) | 9 |
| VSYNC | 6 | | Y4 (D2) | 8 |
| HREF | 7 | | Y5 (D3) | 10 |
| SIOD (SDA) | 4 | | Y6 (D4) | 12 |
| SIOC (SCL) | 5 | | Y7 (D5) | 18 |
| PWDN | −1 | | Y8 (D6) | 17 |
| RESET | −1 | | Y9 (D7) | 16 |

`PWDN` and `RESET` are not routed on this board, hence −1.

### Tunables at the top of `src/main.cpp`

| Constant | Default | Notes |
|----------|---------|-------|
| `XCLK_FREQ_HZ` | 20 MHz | drop to 10 MHz if the image shows banding artefacts |
| `START_FRAMESIZE` | `FRAMESIZE_VGA` | 640×480 — the measured sweet spot |
| `START_JPEG_QUALITY` | 12 | lower = better quality, bigger frames |

The camera is configured with `fb_count = 2` and `CAMERA_GRAB_LATEST`: while one
frame is being sent over the network the next is already being captured, and
stale frames are discarded rather than queued. Without this, latency accumulates
indefinitely. The Python client repeats the same pattern on its side.

---

## Measured performance

VGA, JPEG quality 12, XCLK 20 MHz, single client:

| Mode | Resolution | fps | frame size |
|------|------------|----:|-----------:|
| VGA | 640×480 | 29–34 | ~9–14 KB |
| SVGA | 800×600 | 29 | — |
| HD | 1280×720 | 16–18 | — |

VGA and SVGA both hit the OV2640's ~30 fps ceiling, so neither the ESP32 nor
Wi-Fi is the limit there. HD halves it because the larger frame no longer fits
through the sensor's DVP bus and the network in time.

Frame size varies with scene content — the same VGA/quality-12 setting produced
9.6 KB on one scene and 14 KB on another. At 30 fps that is roughly
1.7–3.4 Mbit/s per camera.

---

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| Random reboots under load | power supply — needs a stable 5 V at ≥1 A and a decent cable. Check `reset_reason` for `BROWNOUT` |
| `psram=0 МБ` in the boot log | wrong `board_build.psram_type`; try `qio` |
| Camera init fails | ribbon cable seating; also check power |
| Python client times out on port 81 | a browser tab is probably holding the stream — check `clients` in `/status` |
| Banding or wavy image | lower `XCLK_FREQ_HZ` to 10 MHz |
| Board not found on upload | hold `BOOT`, tap `RST`, release `BOOT`; verify the port with `pio device list` |
| Overheating during long HD runs | expected; reduce resolution or add airflow for 24/7 operation |

---

## Reference

Freenove's own examples are a useful cross-check for the pinout and sensor
setup:

```
C:\ESP_dev\Freenove_Ultimate_Starter_Kit_for_ESP32_S3-main\C\Sketches\
```
