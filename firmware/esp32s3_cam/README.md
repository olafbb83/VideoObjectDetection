# ESP32-S3 CAM firmware

MJPEG streaming firmware for the **Freenove ESP32-S3-WROOM CAM**, built with
PlatformIO and the Arduino framework. The camera module is interchangeable
(OV2640, OV3660, OV5640): the sensor is detected at boot and configured from a
per-sensor profile. The board currently runs an **OV5640**.

The board does one job: capture frames and push them over Wi-Fi. All inference
happens on the hub — see the [project README](../../README.md).

---

## Endpoints

| URL | Port | Description |
|-----|-----:|-------------|
| `http://<ip>/` | 80 | debug page: live video, buttons for resolution, XCLK, quality, orientation, adaptive mode, save/reset; live `/status` |
| `http://<ip>:81/stream` | 81 | **MJPEG stream** — this is what the hub reads |
| `http://<ip>/jpg` | 80 | single JPEG frame |
| `http://<ip>/status` | 80 | JSON telemetry, sensor, profile, auto-exposure state |
| `http://<ip>/control?var=<name>&val=<n>` | 80 | change sensor settings at runtime (lost on reboot) |
| `http://<ip>/save` | 80 | persist the current settings to flash **for the detected sensor** |
| `http://<ip>/reset` | 80 | forget saved settings for this sensor, fall back to its profile |

The board also announces itself over mDNS, so `http://esp32cam.local/` works
without knowing the IP.

### Sensor profiles

The camera connector accepts several interchangeable modules. They are
electrically compatible, but their best settings differ. This was learned the
hard way: an OV5640 running on OV2640 settings produced fixed vertical stripes
across the whole frame and JPEG frames three times heavier (36–40 KB instead of
9–14 KB).

So nothing sensor-specific is hard-coded. At boot the firmware reads the
sensor's PID and picks a profile (`PROFILES` in `src/main.cpp`):

| Sensor | XCLK | Mirror / flip | Confidence |
|--------|------|---------------|------------|
| OV2640 | 20 MHz | 1 / 1 | measured: VGA 29–34 fps |
| OV3660 | 20 MHz | 1 / 0 | taken from Freenove's example, untested here |
| OV5640 | 8 MHz at boot, **adaptive 16 / 8** | 1 / 0 | measured: 8 MHz clean in light and dark; adaptive thresholds calibrated; orientation confirmed on this board |
| unknown | 10 MHz | 1 / 0 | conservative fallback |

Settings saved with `/save` are stored in NVS under keys prefixed with the PID
(`5640_xclk`, `0026_vf`, …) and override the profile on the next boot. Swapping
modules back and forth therefore needs no reflashing — each sensor loads its
own settings. `/status` reports `sensor`, `pid` and `profile` (`default` or
`saved`), and because `hub/collect.py` stores `/status` with every dataset
session, it is always known which sensor captured which frames.

**The XCLK chicken-and-egg problem.** XCLK must be set *before* the camera is
initialised, but the PID is only known *after*. The firmware therefore boots at
20 MHz (both OV2640 and OV5640 come up fine there — reading the PID goes over
SCCB, which is unaffected by the stripes on the parallel data bus), reads the
PID, and if the profile wants a different frequency it calls
`esp_camera_deinit()` and initialises again. `/control?var=xclk` changes the
frequency live via `set_xclk()`, which is ideal for experiments; a clean re-init
at boot is used for the persisted value because the sensor's PLL registers are
configured for the input clock at initialisation time.

### Adaptive XCLK (OV5640)

On the OV5640 the frame rate is set by XCLK (~1 fps per MHz; JPEG quality
changes frame size but not fps), and vertical stripes appear only when **dark
and** fast: with lights on 20 MHz is clean, in the dark 20 MHz nearly destroys
the frame while 8 MHz stays clean. Any fixed value forces a choice between
daytime speed (16 MHz ≈ 16 fps) and night-time quality (8 MHz ≈ 8.7 fps).

The adaptive mode switches between `xclk_hi` = 16 MHz (light) and
`xclk_lo` = 8 MHz (dark). It is **enabled by default** in the OV5640 profile.

**What it decides on.** The signal must not only separate light from dark —
after a frequency switch it must move *away* from the opposite threshold.
If it moves towards it, every switch undoes itself and the mode flaps.

Frame brightness is useless (auto-exposure keeps it constant). Two
exposure-based candidates were ruled out by calibration on this board:

| signal | spread 16 vs 8 MHz, light | spread, dark | 16 → 8 MHz in light |
|--------|--------------------------:|-------------:|---------------------|
| exposure × gain | 77% | 0% | halves — **towards** the switch-back threshold |
| exposure × gain ÷ XCLK | 12% | 67% | roughly unchanged |
| **gain** | 29% | 0% | **rises 21 → 28 — away from it** |

In the dark both exposure (885 lines) and gain (248) read identically at both
frequencies: auto-exposure is pinned at both ceilings. The firmware therefore
decides on **sensor gain**: 21–28 in room light, 248 in the dark.

- `thr_hi = 60` — gain below this means light (≈2× above the brightest reading);
- `thr_lo = 160` — gain above this means dark (well below the 248 ceiling);
- the gap is deliberately wide: **dusk was not measured**;
- the condition must hold for 5 s in a row, readings are ignored for 4 s after
  a switch while auto-exposure re-converges, and gain is EMA-smoothed;
- `auto_switches` in `/status` counts switches — if it climbs quickly the mode is
  flapping and the thresholds need to move apart.

Setting `xclk` manually disables the adaptive mode, so it doesn't fight a
tuning sweep. When enabled, the board boots at `xclk_lo` — a clean image first,
speed once the light is confirmed. Full numbers and the two refuted hypotheses:
[docs/benchmarks.md](../../docs/benchmarks.md).

**Registers.** `status.aec_value` / `status.agc_gain` in the driver are manual
set-points, not the live state of the auto loop, so the OV5640 registers are
read directly: exposure `0x3500[3:0]`, `0x3501`, `0x3502` (20 bits, 1/16 line);
real gain `0x350A[1:0]`, `0x350B` (10 bits, 1/16×). The addresses come from the
datasheet and are verified on this board only behaviourally — values rise in
the dark and saturate. The 248 ceiling most likely matches the OV5640 default
AEC gain ceiling (`0x00F8`), which has not been checked.

**Recalibrating** (another module, another room):

```bash
# lights on, static scene, no other viewers
.venv\Scripts\python.exe hub\camera_tune.py --host <ip> --xclk 16 8 --quality 12
# lights off, same scene
.venv\Scripts\python.exe hub\camera_tune.py --host <ip> --xclk 16 8 --quality 12
```

Pick `thr_hi` above the highest gain seen in light at either frequency and
`thr_lo` below the lowest gain seen in the dark, with a clear gap, then:

```bash
curl "http://<ip>/control?var=thr_hi&val=<n>"
curl "http://<ip>/control?var=thr_lo&val=<n>"
curl "http://<ip>/control?var=auto&val=1"
curl "http://<ip>/save"
```

`/control?var=auto&val=1` returns 400 unless `0 < thr_hi < thr_lo`.

### Why two HTTP servers

The stream handler blocks forever in a `while(true)` loop. If it lived on
port 80, `/status` and `/control` would stop responding for as long as anything
was streaming. Splitting the stream onto its own server on port 81 keeps
telemetry alive — which is exactly what made it possible to diagnose stream
problems from the Python side.

### `/status` fields

```json
{
  "uptime_s": 383, "reset_reason": "POWERON", "rssi": -60,
  "ip": "192.168.0.184", "heap_free": 235752, "psram_free": 8247743,
  "streaming": false, "clients": 0, "fps": 0.0, "frame_kb": 15,
  "sensor": "OV5640", "pid": "0x5640", "profile": "default",
  "framesize": 8, "quality": 12, "xclk_mhz": 16, "hmirror": 1, "vflip": 0,
  "aec_exposure": 885, "aec_gain": 21, "light_need": 18585, "auto_gain_ema": 22,
  "auto_xclk": 1, "auto_xclk_hi": 16, "auto_xclk_lo": 8,
  "auto_thr_lo": 160, "auto_thr_hi": 60, "auto_switches": 1
}
```

* **`reset_reason`** separates two entirely different classes of problem.
  `BROWNOUT` means the power supply sagged — fix the PSU or the cable.
  `PANIC` / `TASK_WDT` means the firmware crashed or hung — fix the code.
  `POWERON` / `EXT_PIN` is a normal start or the reset button.
* **`clients`** is the number of viewers currently attached to the stream.
  On the OV2640 at 30 fps two clients shared the bandwidth (13.3 + 7.4 fps where
  a single one got 21). On the OV5640 a second client once received **no frames
  at all** while a browser tab was open. Not investigated separately — close
  other viewers before measuring.
* **`fps`** is the aggregate across all clients. It is computed in `loop()`
  rather than inside the stream handler, because several handler instances run
  concurrently and each would otherwise overwrite the others' value.
* **`aec_gain` / `auto_gain_ema`** — raw and smoothed sensor gain (1/16×); the
  smoothed value drives the adaptive XCLK. **`light_need`** (exposure × gain) is
  kept for calibration only.

### `/control` variables

| Variable | Values | Notes |
|----------|--------|-------|
| `framesize` | 5 = QVGA, 8 = VGA, 9 = SVGA, 11 = HD | |
| `quality` | 0–63 | **lower = better quality** and larger frames; 10–12 is the practical band |
| `brightness`, `contrast`, `ae_level` | −2…2 | |
| `hmirror`, `vflip` | 0 / 1 | |
| `xclk` | 4–24 MHz | live via `set_xclk()`; **disables adaptive mode** |
| `auto` | 0 / 1 | adaptive XCLK; refused unless `0 < thr_hi < thr_lo` |
| `thr_lo`, `thr_hi` | gain, 1/16× | dark / light thresholds |
| `xclk_hi`, `xclk_lo` | 4–24 MHz | frequencies for light / dark |

```bash
curl "http://esp32cam.local/control?var=framesize&val=8"
curl "http://esp32cam.local/control?var=quality&val=12"
```

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
[cam] сенсор OV5640 (PID=0x5640), налаштування: профіль
[cam] профіль: виміряно: 8 МГц чисто завжди; авто-XCLK 16/8 за підсиленням 60/160
[cam] переініціалізація на XCLK 8 МГц
[cam] XCLK 8 МГц, framesize 8, quality 12, дзеркало 1, переворот 0
[auto] авто-XCLK: УВІМКНЕНО, 16/8 МГц, підсилення темно>160 світло<60
[wifi] OK  ip=192.168.0.184  rssi=-60 dBm
[mdns] також доступна як http://esp32cam.local
[http] сторінка : http://192.168.0.184/
[http] потік    : http://192.168.0.184:81/stream
[auto] світло (підсилення 25 < 60) -> XCLK 16 МГц
```

The last line appears once the room is lit. The minimum is ~10 s after boot
(4 s for auto-exposure to settle, then 5 s of consistent readings); on the first
boot on this board it took ~63 s — possibly the light was switched on later,
not investigated.

`E (…) gdma: gdma_disconnect(299): no peripheral is connected to the channel`
may appear right after `переініціалізація`. It is printed by the DMA driver
during `esp_camera_deinit()`; the following init succeeds and the stream works,
so it looks cosmetic — not investigated further.

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
Current usage: RAM 15.1%, flash 26.0% of 3 MB.

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
[docs/S3CamWroom1.png](../../docs/S3CamWroom1.png). The same pinout serves all
supported modules.

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

### Tunables in `src/main.cpp`

| Name | Default | Notes |
|------|---------|-------|
| `PROFILES` | see table above | per-sensor XCLK, resolution, quality, orientation, adaptive thresholds |
| `BOOT_XCLK_MHZ` | 20 | frequency used only to read the PID before the profile is known |
| `AUTO_HOLD_S` | 5 | seconds a condition must hold before the adaptive mode switches |
| `AUTO_SETTLE_MS` | 4000 | readings ignored after a switch while auto-exposure re-converges |

The camera is configured with `fb_count = 2` and `CAMERA_GRAB_LATEST`: while one
frame is being sent over the network the next is already being captured, and
stale frames are discarded rather than queued. Without this, latency accumulates
indefinitely. The Python client repeats the same pattern on its side.

---

## Measured performance

### OV5640 (current module)

VGA, JPEG quality 12, single client, `hub/camera_tune.py`:

| XCLK | fps, light | fps, dark | KB/frame, light | KB/frame, dark | stripes, light | stripes, dark |
|-----:|-----------:|----------:|----------------:|---------------:|---------------:|--------------:|
| 20 | 18.1 | 7.4 | 15.8 | 43.9 | 0.98 | 11.17 |
| 16 | 16.2 | 15.5 | 15.5 | 29.3 | 0.92 | 5.35 |
| 12 | 13.1 | 12.9 | 15.4 | 17.9 | 0.92 | 1.76 |
| 10 | 10.8 | 9.1 | 15.5 | 15.6 | 0.93 | 1.07 |
| 8 | 8.7 | 8.7 | 15.5 | 15.6 | 0.90 | 0.98 |

The stripe score is relative (compare rows, not against zero); checked by eye:
11.17 is a nearly destroyed frame, 1.76 faint stripes on a plain wall, 0.98 clean.
Noise does not compress, which is why dark frames at high XCLK are so heavy.

### OV2640 (previous module)

VGA, JPEG quality 12, XCLK 20 MHz, single client:

| Mode | Resolution | fps | frame size |
|------|------------|----:|-----------:|
| VGA | 640×480 | 29–34 | ~9–14 KB |
| SVGA | 800×600 | 29 | — |
| HD | 1280×720 | 16–18 | — |

VGA and SVGA both hit the OV2640's ~30 fps ceiling, so neither the ESP32 nor
Wi-Fi is the limit there. HD halves it because the larger frame no longer fits
through the sensor's DVP bus and the network in time.

---

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| Random reboots under load | power supply — needs a stable 5 V at ≥1 A and a decent cable. Check `reset_reason` for `BROWNOUT` |
| `Reason: 201 - NO_AP_FOUND` in the log | the board cannot see an access point with that name at all — **not** a password problem (that is `202 AUTH_FAIL` or `15`). After 12 s the firmware scans and prints every visible 2.4 GHz network and whether the configured SSID is among them. Visible on a phone but not in the list → the network is 5 GHz only |
| `psram=0 МБ` in the boot log | wrong `board_build.psram_type`; try `qio` |
| Camera init fails | ribbon cable seating; also check power |
| Python client gets few or no frames | another viewer (usually a browser tab) is attached — check `clients` in `/status` |
| Vertical stripes, worse in the dark | XCLK too high for the light level. On OV5640 use 8 MHz or the adaptive mode; on another module run `hub/camera_tune.py` with lights on and off |
| Image upside down or mirrored | toggle `hmirror` / `vflip` on the debug page, then `/save` |
| Adaptive mode keeps switching | `auto_switches` climbing — move `thr_hi` / `thr_lo` further apart |
| Board not found on upload | hold `BOOT`, tap `RST`, release `BOOT`; verify the port with `pio device list` |
| Overheating during long HD runs | expected; reduce resolution or add airflow for 24/7 operation |

---

## Reference

Freenove's own examples are a useful cross-check for the pinout and sensor
setup:

```
C:\ESP_dev\Freenove_Ultimate_Starter_Kit_for_ESP32_S3-main\C\Sketches\
```
