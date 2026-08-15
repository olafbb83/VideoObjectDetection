# VideoDetection

Person detection and behaviour analysis on a live video stream from a
**Freenove ESP32-S3-WROOM CAM**, processed with YOLO11 on a laptop and
(eventually) on a Raspberry Pi 5.

This is a learning project. Every design decision below was measured rather
than assumed, and the measurements — including the ones that contradicted the
initial reasoning — are recorded in [docs/benchmarks.md](docs/benchmarks.md).

---

## Architecture

```
ESP32-S3 CAM  ──MJPEG over Wi-Fi──>  HUB  ──> YOLO ──> tracking ──> zone/line rules
   "eye"                        (laptop / RPi5)                            │
                                                                   HTTPS + token
                                                              ┌────────────┼────────────┐
                                                           browser      phone         API
```

**YOLO does not run on the ESP32.** The board has ~8 MB of PSRAM and vector
instructions that are only enough for tiny ESP-DL models. The ESP32 is a camera
plus a Wi-Fi transport; all inference happens on the hub.

**The hub is the single consumer of the camera.** One reader thread, one YOLO
pass per frame, and all viewers receive the already-annotated frame. This
matters: connecting several clients directly to the ESP32 makes them share its
bandwidth (measured: 13.3 + 7.4 fps for two clients where one got 21), and each
client would additionally re-run the model on the same video. Through the hub,
three simultaneous viewers get 19.7 / 19.9 / 19.8 fps and the model runs once.

---

## Hardware

| Device | Role | Measured performance |
|--------|------|----------------------|
| Freenove ESP32-S3-WROOM CAM (OV2640) | video source | VGA 640×480 ≈ 30 fps, SVGA ≈ 29, HD 1280×720 ≈ 17 |
| Intel Core Ultra 7 155H, 32 GB | development, training, inference | YOLO11n @640 on iGPU: 14.3 ms (70 fps) |
| Raspberry Pi 5, 8 GB | target 24/7 deployment | not yet deployed (stage 7) |

The laptop exposes three compute devices through OpenVINO — CPU, the Arc iGPU
and the NPU (AI Boost). All three are benchmarked in
[docs/benchmarks.md](docs/benchmarks.md).

---

## Repository layout

```
firmware/esp32s3_cam/   PlatformIO project for the camera firmware (MJPEG server)
hub/                    Python: stream client, YOLO, tracking, rules, web server
docs/                   plan, benchmark results, camera pinout, zone config
models/                 model weights and exported models (git-ignored)
secrets/                token, TLS certificate and key (git-ignored)
tests/                  geometry tests that need neither camera nor model
captures/               saved frames for the future dataset (git-ignored)
```

---

## Quick start

### 1. Firmware

See [firmware/esp32s3_cam/README.md](firmware/esp32s3_cam/README.md) for the
full flashing guide. In short:

```bash
cp firmware/esp32s3_cam/src/config.example.h firmware/esp32s3_cam/src/config.h
# edit config.h: put in your 2.4 GHz Wi-Fi credentials
C:\Users\Lenovo\.platformio\penv\Scripts\pio.exe run -d firmware/esp32s3_cam -t upload -t monitor
```

The serial log prints the camera's IP. It is also reachable over mDNS as
`http://esp32cam.local/`.

### 2. Python environment

```bash
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r hub\requirements.txt
.venv\Scripts\python.exe -m pip install -r hub\requirements-ml.txt
```

`requirements.txt` is lightweight (OpenCV, FastAPI, requests) and is enough for
stages 1–2. `requirements-ml.txt` adds PyTorch, Ultralytics and OpenVINO
(~250 MB total; the Windows PyPI wheel of PyTorch is CPU-only, which is fine
because acceleration comes from OpenVINO).

Python 3.14 is used and works — cp314 wheels exist for the whole stack.

### 3. Export models for the accelerators

```bash
.venv\Scripts\python.exe hub\export.py
```

This downloads `yolo11n.pt` / `yolo11n-pose.pt` and exports both to OpenVINO at
input sizes 320 and 640. **The input size is baked into the exported graph**, so
320 and 640 need separate directories. Ultralytics recognises the format from
the directory-name suffix (`_openvino_model`), so the size goes at the front:
`yolo11n_640_openvino_model`.

### 4. Run

```bash
# just look at the raw camera stream
.venv\Scripts\python.exe hub\view.py

# detection in a local window
.venv\Scripts\python.exe hub\detect.py --model 640 --device intel:gpu

# detection + tracking + skeletons
.venv\Scripts\python.exe hub\detect.py --model 640 --device intel:gpu --track --pose

# the hub: HTTPS + token, viewable from a phone
.venv\Scripts\python.exe hub\make_cert.py          # once
.venv\Scripts\python.exe hub\server.py --model 640 --device intel:gpu --track
```

The server prints a ready-to-open URL containing the token.

---

## Tools

| Script | Purpose |
|--------|---------|
| `hub/view.py` | raw MJPEG viewer; records clips and samples frames for the dataset |
| `hub/detect.py` | detection / tracking / pose / zone rules in a local window |
| `hub/server.py` | the hub: HTTPS, token auth, MJPEG re-streaming, JSON API |
| `hub/bench.py` | benchmark backends and devices (`--task detect pose`) |
| `hub/export.py` | export models to OpenVINO / NCNN / ONNX at a given input size |
| `hub/make_cert.py` | self-signed TLS certificate with correct SAN entries |
| `hub/zone_editor.py` | draw zones and lines with the mouse over a real frame |
| `hub/track_quality.py` | measure tracking quality; sweep tracker configurations |
| `tests/test_zones.py` | zone/line geometry tests — no camera, no model |

### Useful flags of `hub/detect.py`

```
--model 320|640|<path>   shorthand expands to the exported OpenVINO directory
--device intel:gpu       cpu | intel:cpu | intel:gpu | intel:npu
--track                  stable IDs, trajectory trails, time-in-frame
--pose                   17-point skeletons (replaces the detector, see below)
--zones [file]           zone/line rules; implies --track
--classes                no values = all 80 COCO classes; default is person only
--list-classes           print every class the model knows and exit
--source file.mp4        replay a recording instead of the live camera
--no-window --seconds N  headless measurement run
```

---

## The hub API

Everything except `/health` requires a token. The token is generated on first
start into `secrets/token.txt`.

| Endpoint | Description |
|----------|-------------|
| `GET /health` | liveness check — **no token**, so monitoring scripts don't carry the secret |
| `GET /` | web page: video, live statistics, event log |
| `GET /stream.mjpg` | annotated MJPEG stream |
| `GET /snapshot.jpg` | single annotated frame |
| `GET /api/status` | fps, inference time, detections, occupancy, counters |
| `GET /api/events` | recent zone and line events |

The token is accepted either as `Authorization: Bearer <token>` or as
`?token=<token>`. The query-string form exists because an `<img>` tag cannot
send headers, and MJPEG has to live inside an `<img>`. The trade-off is that the
token ends up in browser history; acceptable on a home network over HTTPS, but
worth rotating before exposing the hub to the internet.

TLS uses a self-signed certificate, so browsers warn once per device. Let's
Encrypt only issues certificates for real domain names, so it is not an option
for `192.168.x.x`. When a domain does appear, only the two files in `secrets/`
need replacing — no code changes.

**Browsers ignore the certificate's CN field and look only at SAN.** Any address
you want to use must be listed there:

```bash
.venv\Scripts\python.exe hub\make_cert.py --extra rpi5.local --extra 192.168.1.77 --force
```

---

## What has been built

### Stage 1 — the camera streams

Custom firmware serving MJPEG on port 81, plus telemetry on port 80. The stream
handler blocks forever, so it lives on a separate HTTP server; otherwise
`/status` and `/control` would stop responding while streaming.

Measured: VGA 29–34 fps, SVGA 29, HD 16–18. VGA and SVGA both hit the OV2640
sensor's ~30 fps ceiling — neither Wi-Fi nor the ESP32 is the bottleneck.
**VGA is the working mode**: SVGA is equally fast but produces heavier frames,
and YOLO resizes everything to 640 anyway.

### Stage 2 — Python receiver

MJPEG is parsed by hand rather than through `cv2.VideoCapture`, because
VideoCapture queues frames: if your consumer is slower than the camera, latency
grows without bound. A reader thread keeps **only the newest frame**; anything
the consumer didn't pick up is overwritten.

Two bugs found by measurement here:

* `cv2.waitKey(1)` costs **~15 ms on Windows**, not 1 ms — it is implemented on
  a system timer with 15.6 ms granularity. Display dropped from 22 to 10 fps.
  `cv2.pollKey()` costs 0.4 ms.
* The read buffer was 64 KB while a frame is ~14 KB. The underlying
  `http.client` blocks until it has the requested number of bytes, so frames
  arrived in bursts of ~4.6 with 200 ms gaps. Harmless while the consumer was
  instant — but once YOLO was added, the rest of each burst was overwritten and
  23 fps of input produced 10 fps of output. An 8 KB buffer makes arrival even
  (~39 ms apart) at identical throughput.

### Stage 3 — detection

YOLO11n, COCO, filtered to the `person` class by default. Accelerators give a
3–4× speedup over PyTorch on CPU:

| backend | device | imgsz | median | fps |
|---------|--------|------:|-------:|----:|
| PyTorch | CPU | 640 | 57.5 ms | 17.4 |
| OpenVINO | CPU | 640 | 40.0 ms | 25.0 |
| OpenVINO | **iGPU** | 640 | **14.3 ms** | **70.1** |
| OpenVINO | NPU | 640 | 21.8 ms | 45.8 |
| OpenVINO | NPU | 320 | 6.5 ms | 153.2 |

The NPU wins at 320 and is the most consistent (p95 8.0 ms against a 6.5 ms
median); the iGPU wins at 640. NPU scales worse with input size — 3.2× slower
from 320 to 640, against 1.8× for the iGPU.

### Stage 4 — the hub

FastAPI over HTTPS on port 8443, token required everywhere except `/health`,
`secrets.compare_digest` for the comparison (plain `==` returns early on the
first mismatch, which leaks the secret through response timing). EC P-256 keys
rather than RSA — same strength, noticeably cheaper handshakes on a Pi.

### Stage 5 — tracking, zones and pose

**Tracking (ByteTrack).** Detection has no memory; the tracker adds time via a
Kalman filter plus association. Configuration was tuned against a fixed
recording, because comparing configurations on a live camera is meaningless —
the person moves differently every run.

| configuration | coverage | unique IDs | median track life |
|---------------|---------:|-----------:|------------------:|
| stock (conf 0.35) | 75% | 18 | 0.7 s |
| **tuned** (conf 0.45, match_thresh 0.9, new_track_thresh 0.5) | **73%** | **6** | **3.9 s** |

The tuned config lives in `hub/trackers/bytetrack_tuned.yaml` and is the
default. `match_thresh` turned out to be the effective knob; `track_buffer`
changed nothing at all, because a longer memory still needs an IoU match to
revive a lost track — and a person who walks away and returns elsewhere has zero
overlap.

Six IDs is close to the floor: the person leaves the frame about four times in
that recording, and ByteTrack matches on geometry and motion, not appearance, so
every re-entry legitimately starts a new track.

**Zones and lines.** Three decisions that determine whether this works at all:

* The anchor point is the **bottom-centre of the box (the feet)**, not the
  centre. For a floor zone, using the centre fires when the person's torso is
  over it — roughly a metre early.
* Line crossing tests **segment against segment**, not a change of side relative
  to an infinite line. Otherwise someone walking three metres past a doorway,
  but collinear with it, counts as having entered.
* Two debounces: 3 frames to confirm a zone transition (a box on a boundary
  jitters by a couple of pixels), and 1.5 s before forgetting a vanished track.
  The second one matters because 62% of detection gaps last 1–3 frames — without
  it the system emitted a spurious "exited" + "entered" pair on the same track.

Zone coordinates are normalised to 0..1, so the config survives a resolution
change.

**Pose.** `--pose` **replaces** the detector instead of adding a second model:
YOLO11n-pose detects people itself and returns boxes and 17 keypoints in one
pass. The cost is +4% on the iGPU but **+32% on CPU** — which is the number that
matters, because the Raspberry Pi 5 has no accelerator.

---

## Known limitations

* **Viewers of the ESP32 stream share its bandwidth.** An open browser tab on
  `http://esp32cam.local/` roughly halves the fps available to the Python
  client. Close it before measuring; `/status` reports `clients`.
* **Only 80 COCO classes.** Doors, windows and tools will never be detected, no
  matter how low the confidence threshold goes.
* **No re-identification.** A person who leaves and returns gets a new track ID.
  Telling *which* person it is is a different problem and out of scope.
* **Close-ups break both pose and zones.** When someone fills the frame their
  feet are cut off: the anchor point jumps, and only 6–7 of 17 keypoints remain
  visible. The torso angle then correctly reports "unknown" rather than a
  fabricated number.
* **The detector is the ceiling.** Tracks break where the model is unsure
  (10th-percentile confidence 0.56). Better tracking parameters cannot fix that
  — a fine-tuned detector can, which is stage 6.
* `view.py --record` writes files with a declared 25 fps regardless of the real
  capture rate, so absolute durations measured from a recording can be ~12% off.
  Comparisons between configurations are unaffected.

---

## Roadmap

| # | Stage | Status |
|---|-------|--------|
| 0 | Project skeleton | done |
| 1 | Camera streams MJPEG | done |
| 2 | Python receiver, OpenCV | done |
| 3 | Detection, OpenVINO benchmarks | done |
| 4 | Hub: HTTPS, token, web UI | done |
| 5 | Tracking, zones, lines, pose | done except fall detection |
| 6 | Own dataset and fine-tuning | next |
| 7 | Deployment to Raspberry Pi 5 | planned |
| 8 | Optional: on-device motion wake, VLM event descriptions | planned |

Full plan with reasoning: [docs/PLAN.md](docs/PLAN.md).
All measurements: [docs/benchmarks.md](docs/benchmarks.md).
