"""
Етап 4 — хаб: приймає потік з ESP32, проганяє через YOLO, віддає результат
у браузер і на телефон по HTTPS із токеном.

Ендпоінти:
  GET /health         — БЕЗ токена, щоб моніторинг не світив секретом у скриптах
  GET /               — сторінка з відео і статистикою
  GET /stream.mjpg    — анотований MJPEG-потік
  GET /snapshot.jpg   — один анотований кадр
  GET /api/status     — JSON зі станом конвеєра

Про токен у посиланні. Правильний спосіб передати секрет — заголовок
Authorization. Але тег <img> у браузері власних заголовків не шле, а весь
сенс MJPEG саме в тому, що його показує звичайний <img>. Тому підтримуємо
обидва варіанти: заголовок для API-клієнтів і ?token= для браузера.

Плата за зручність: токен у URL потрапляє в історію браузера й у логи
проксі. У домашній мережі за HTTPS це прийнятно, але якщо колись виставиш
хаб назовні — краще перевипустити токен і ходити тільки заголовком.

Запуск:
  python hub/make_cert.py                 # один раз
  python hub/server.py --model 640 --device intel:gpu
"""

from __future__ import annotations

import argparse
import os
import secrets
import ssl
import sys
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import Depends, FastAPI, HTTPException, Query, Request
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse

from detect import TUNED_TRACKER, draw_detections, resolve_model, summarize
from pipeline import DetectionPipeline
from zones import RuleEngine, load_config as load_zones

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SECRETS_DIR = PROJECT_ROOT / "secrets"
TOKEN_PATH = SECRETS_DIR / "token.txt"
CERT_PATH = SECRETS_DIR / "cert.pem"
KEY_PATH = SECRETS_DIR / "key.pem"

BOUNDARY = "hubframe"

_pipeline: DetectionPipeline | None = None
_token: str = ""


# ---------------------------------------------------------------------------
# Токен
# ---------------------------------------------------------------------------

def load_or_create_token() -> str:
    """Читає токен з secrets/token.txt, створюючи його при першому запуску."""
    if TOKEN_PATH.exists():
        token = TOKEN_PATH.read_text(encoding="utf-8").strip()
        if token:
            return token

    SECRETS_DIR.mkdir(parents=True, exist_ok=True)
    token = secrets.token_urlsafe(32)
    TOKEN_PATH.write_text(token + "\n", encoding="utf-8")
    print(f"[auth] створено новий токен: {TOKEN_PATH}")
    return token


def require_token(request: Request, token: str | None = Query(default=None)) -> None:
    """
    Перевіряє токен із заголовка Authorization: Bearer <token> або ?token=<token>.

    compare_digest замість == принципово: звичайне порівняння рядків
    завершується на першому неспівпадінні, і за часом відповіді можна
    посимвольно підібрати секрет. compare_digest виконується за однаковий час
    незалежно від того, де саме розійшлися рядки.
    """
    supplied = token
    auth = request.headers.get("authorization", "")
    if auth.lower().startswith("bearer "):
        supplied = auth[7:].strip()

    if not supplied or not secrets.compare_digest(supplied, _token):
        raise HTTPException(status_code=401, detail="потрібен дійсний токен")


# ---------------------------------------------------------------------------
# Застосунок
# ---------------------------------------------------------------------------

@asynccontextmanager
async def lifespan(app: FastAPI):
    yield
    if _pipeline is not None:
        _pipeline.stop()


app = FastAPI(title="VideoDetection hub", lifespan=lifespan, docs_url=None, redoc_url=None)


@app.get("/health")
def health() -> JSONResponse:
    """Без токена: щоб перевіряти живучість сервісу, не світячи секретом."""
    running = _pipeline is not None and _pipeline.stats.frames_processed > 0
    return JSONResponse({"status": "ok" if running else "starting"})


@app.get("/api/status", dependencies=[Depends(require_token)])
def api_status() -> JSONResponse:
    if _pipeline is None:
        raise HTTPException(status_code=503, detail="конвеєр не запущено")
    return JSONResponse(_pipeline.stats.as_dict())


@app.get("/api/events", dependencies=[Depends(require_token)])
def api_events(limit: int = Query(default=50, ge=1, le=200)) -> JSONResponse:
    """Останні події зон і ліній, найновіші першими."""
    if _pipeline is None or _pipeline.engine is None:
        return JSONResponse({"events": [], "note": "правила не увімкнено (--zones)"})
    evs = list(_pipeline.engine.events)[-limit:]
    return JSONResponse({
        "events": [e.as_dict() | {"text": e.human()} for e in reversed(evs)],
    })


@app.get("/snapshot.jpg", dependencies=[Depends(require_token)])
def snapshot() -> Response:
    if _pipeline is None:
        raise HTTPException(status_code=503, detail="конвеєр не запущено")
    jpeg = _pipeline.snapshot()
    if jpeg is None:
        raise HTTPException(status_code=503, detail="кадру ще немає")
    return Response(content=jpeg, media_type="image/jpeg")


@app.get("/stream.mjpg", dependencies=[Depends(require_token)])
def stream() -> StreamingResponse:
    if _pipeline is None:
        raise HTTPException(status_code=503, detail="конвеєр не запущено")

    pipeline = _pipeline

    def generate():
        # Звичайний (не async) генератор: Starlette крутить його в пулі потоків,
        # тому блокуюче очікування кадру не гальмує весь сервер.
        pipeline.viewer_joined()
        last_seen = 0
        try:
            while True:
                jpeg, last_seen = pipeline.wait_for_frame(last_seen, timeout=5.0)
                if jpeg is None:
                    continue
                yield (
                    f"\r\n--{BOUNDARY}\r\n"
                    f"Content-Type: image/jpeg\r\n"
                    f"Content-Length: {len(jpeg)}\r\n\r\n"
                ).encode() + jpeg
        finally:
            # Спрацює і коли глядач просто закрив вкладку
            pipeline.viewer_left()

    return StreamingResponse(
        generate(),
        media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}",
        headers={"Cache-Control": "no-store", "Connection": "close"},
    )


@app.get("/", response_class=HTMLResponse, dependencies=[Depends(require_token)])
def index(token: str | None = Query(default=None)) -> HTMLResponse:
    # Токен прокидуємо далі в URL потоку: <img> не вміє слати заголовки
    t = token or ""
    return HTMLResponse(PAGE.replace("__TOKEN__", t))


PAGE = """<!doctype html><html lang=uk><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>VideoDetection</title>
<style>
:root{color-scheme:dark}
body{margin:0;padding:12px;background:#101214;color:#e6e6e6;
     font:14px/1.5 system-ui,-apple-system,sans-serif}
.wrap{max-width:900px;margin:0 auto}
h1{font-size:16px;font-weight:600;margin:0 0 10px;color:#9fb3c8}
img{width:100%;border-radius:10px;background:#000;display:block}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));
      gap:8px;margin-top:12px}
.card{background:#1a1e22;border-radius:8px;padding:10px}
.k{font-size:11px;color:#7d8f9f;text-transform:uppercase;letter-spacing:.04em}
.v{font-size:20px;font-weight:600;margin-top:2px}
.det{margin-top:12px;background:#1a1e22;border-radius:8px;padding:10px;min-height:42px}
.tag{display:inline-block;background:#2a4d3a;color:#8fe0a8;border-radius:5px;
     padding:2px 8px;margin:2px 4px 2px 0;font-size:13px}
.off{color:#ff8b8b}
h2{font-size:13px;font-weight:600;color:#7d8f9f;margin:16px 0 6px;
   text-transform:uppercase;letter-spacing:.04em}
.log{list-style:none;margin:0;padding:0;max-height:230px;overflow-y:auto;
     background:#1a1e22;border-radius:8px}
.log li{padding:7px 10px;border-bottom:1px solid #23282d;font-size:13px;
        display:flex;gap:8px;align-items:baseline}
.log li:last-child{border-bottom:none}
.log time{color:#6f8296;font-size:11px;flex-shrink:0;font-variant-numeric:tabular-nums}
.dwell{color:#ffcf6b}.cross{color:#8fd0ff}
</style>
<div class=wrap>
  <h1>VideoDetection — ESP32-S3 CAM</h1>
  <img id=v alt="потік">
  <div class=grid>
    <div class=card><div class=k>камера</div><div class=v id=recv>-</div></div>
    <div class=card><div class=k>обробка</div><div class=v id=proc>-</div></div>
    <div class=card><div class=k>інференс</div><div class=v id=inf>-</div></div>
    <div class=card><div class=k>глядачів</div><div class=v id=view>-</div></div>
    <div class=card><div class=k>треків</div><div class=v id=trk>-</div></div>
  </div>
  <div class=det id=det>—</div>
  <div class=det id=occ style="display:none"></div>
  <h2 id=evh style="display:none">Події</h2>
  <ul id=evs class=log></ul>
</div>
<script>
const token = "__TOKEN__";
const q = token ? "?token=" + encodeURIComponent(token) : "";
document.getElementById("v").src = "/stream.mjpg" + q;

async function tick(){
  try{
    const r = await fetch("/api/status" + q);
    if(!r.ok) throw new Error(r.status);
    const s = await r.json();
    recv.textContent = s.recv_fps.toFixed(1) + " fps";
    proc.textContent = s.process_fps.toFixed(1) + " fps";
    inf.textContent  = s.infer_ms.toFixed(1) + " ms";
    view.textContent = s.viewers;
    trk.textContent  = s.tracks_active + " / " + s.tracks_total;
    recv.className = "v" + (s.camera_connected ? "" : " off");
    const d = s.detections || {};
    const keys = Object.keys(d);
    det.innerHTML = keys.length
      ? keys.map(k => `<span class=tag>${k} &times;${d[k]}</span>`).join("")
      : "&mdash;";

    const oc = s.occupancy || {}, cn = s.counters || {};
    const parts = Object.keys(oc).map(k => `<span class=tag>${k}: ${oc[k]}</span>`)
      .concat(Object.keys(cn).map(k => `<span class=tag>${k} ${cn[k]}</span>`));
    occ.style.display = parts.length ? "" : "none";
    occ.innerHTML = parts.join("");
  }catch(e){ det.textContent = "немає зв'язку з хабом"; }
}

// Події тягнемо окремо: вони змінюються рідко, а /api/status раз на секунду
async function events(){
  try{
    const r = await fetch("/api/events?limit=40" + (q ? "&" + q.slice(1) : ""));
    if(!r.ok) return;
    const s = await r.json();
    if(!s.events.length) return;
    evh.style.display = "";
    evs.innerHTML = s.events.map(e => {
      const cls = e.kind === "zone_dwell" ? "dwell"
                : e.kind === "line_cross" ? "cross" : "";
      const t = new Date(e.wall * 1000).toLocaleTimeString("uk-UA");
      return `<li><time>${t}</time><span class="${cls}">${e.text}</span></li>`;
    }).join("");
  }catch(e){}
}
setInterval(tick, 1000); tick();
setInterval(events, 2000); events();
</script>
</html>"""


# ---------------------------------------------------------------------------

def main() -> int:
    global _pipeline, _token

    ap = argparse.ArgumentParser(description="Хаб VideoDetection")
    ap.add_argument("--url", default=os.environ.get("CAM_URL", "http://esp32cam.local:81/stream"))
    ap.add_argument("--model", default="640", help="шлях до моделі або 320 / 640")
    ap.add_argument("--device", default=None, help="cpu | intel:gpu | intel:npu")
    ap.add_argument("--conf", type=float, default=None,
                    help="типово 0.35, з --track 0.45 (див. detect.py: вища "
                         "планка менше рве треки)")
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--imgsz", type=int, default=None)
    ap.add_argument("--classes", type=int, nargs="*", default=[0])
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8443)
    ap.add_argument("--jpeg-quality", type=int, default=80)
    ap.add_argument("--track", action="store_true", help="увімкнути трекінг зі стабільними ID")
    ap.add_argument("--tracker", default=str(TUNED_TRACKER))
    ap.add_argument("--no-trail", action="store_true", help="не малювати хвости траєкторій")
    ap.add_argument("--zones", nargs="?", const=str(PROJECT_ROOT / "docs" / "zones.json"),
                    help="JSON із зонами й лініями (типово docs/zones.json)")
    ap.add_argument("--http", action="store_true",
                    help="без TLS (тільки для локальної відладки)")
    args = ap.parse_args()

    # Правила спираються на track_id — без трекінгу вони безглузді
    if args.zones:
        args.track = True

    if args.imgsz is None:
        args.imgsz = int(args.model) if args.model.isdigit() else 640
    if args.conf is None:
        args.conf = 0.45 if args.track else 0.35

    if not args.http and not (CERT_PATH.exists() and KEY_PATH.exists()):
        print(f"[hub] немає сертифіката {CERT_PATH}")
        print("[hub] згенеруй його: python hub/make_cert.py")
        return 1

    _token = load_or_create_token()

    from ultralytics import YOLO
    import uvicorn

    model_path = resolve_model(args.model)
    print(f"[hub] модель {model_path}")
    is_ov = model_path.rstrip("/\\").endswith("_openvino_model")
    model = YOLO(model_path, task="detect") if is_ov else YOLO(model_path)

    engine = None
    if args.zones:
        zones_cfg, lines_cfg = load_zones(args.zones)
        engine = RuleEngine(zones_cfg, lines_cfg)
        print(f"[hub] правила: зон {len(zones_cfg)}, ліній {len(lines_cfg)}")

    _pipeline = DetectionPipeline(
        args.url, model,
        conf=args.conf, iou=args.iou, imgsz=args.imgsz,
        classes=args.classes if args.classes else None,
        device=args.device, jpeg_quality=args.jpeg_quality,
        draw_fn=draw_detections, summarize_fn=summarize,
        track=args.track, tracker=args.tracker, show_trail=not args.no_trail,
        engine=engine,
    ).start()

    scheme = "http" if args.http else "https"
    shown_host = "localhost" if args.host in ("0.0.0.0", "::") else args.host
    print(f"\n[hub] {scheme}://{shown_host}:{args.port}/?token={_token}\n")
    if not args.http:
        print("[hub] сертифікат самопідписаний — браузер попередить один раз")
        print("[hub] відкривати ТІЛЬКИ за адресою зі списку SAN (hub/make_cert.py)\n")

    ssl_kwargs = {}
    if not args.http:
        ssl_kwargs = {
            "ssl_certfile": str(CERT_PATH),
            "ssl_keyfile": str(KEY_PATH),
            # TLS 1.2+ : нижче нічого сучасного не потрібно, а старе — діряве
            "ssl_version": ssl.PROTOCOL_TLS_SERVER,
        }

    uvicorn.run(app, host=args.host, port=args.port, log_level="warning", **ssl_kwargs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
