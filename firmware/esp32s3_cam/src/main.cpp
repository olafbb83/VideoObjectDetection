/**********************************************************************
 * VideoDetection — Етап 1: камера як джерело відео
 *
 * Прошивка робить рівно одне: віддає MJPEG-потік по Wi-Fi, щоб хаб
 * (ноут / RPi5) міг його читати через OpenCV і згодовувати YOLO.
 *
 * Ендпоінти:
 *   http://<ip>/          — сторінка з відео і посиланнями
 *   http://<ip>:81/stream — MJPEG-потік (це те, що читає Python)
 *   http://<ip>/jpg       — один кадр JPEG
 *   http://<ip>/status    — JSON з телеметрією (fps, rssi, heap, налаштування)
 *   http://<ip>/control?var=framesize&val=8 — зміна параметрів на льоту
 *
 * Що таке MJPEG: сервер тримає одне HTTP-з'єднання відкритим і пише в нього
 * нескінченну послідовність JPEG-кадрів, розділених текстовим "boundary".
 * Це не відеокодек — між кадрами немає жодного стиснення, кожен кадр
 * незалежний. Тому потік важкий по трафіку, але простий і має мінімальну
 * затримку, і його «з коробки» читають браузер, VLC та OpenCV.
 **********************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"

#include "camera_pins.h"

#if !__has_include("config.h")
#error "Немає src/config.h — скопіюй src/config.example.h у src/config.h і впиши свій Wi-Fi"
#endif
#include "config.h"

// ---------------------------------------------------------------------------
// Налаштування камери, які варто крутити на етапі 1
// ---------------------------------------------------------------------------

// Тактова частота сенсора. 20 МГц = більше FPS, але на деяких екземплярах
// OV2640 з'являються смуги/артефакти. Якщо картинка «пливе» — став 10000000.
static const int XCLK_FREQ_HZ = 20000000;

// Стартова роздільність. FRAMESIZE_VGA = 640x480 — хороший баланс:
// YOLO все одно ресайзить вхід до 640, тож більше зазвичай не дає точності,
// а канал і CPU їсть помітно.
static const framesize_t START_FRAMESIZE = FRAMESIZE_VGA;

// Якість JPEG: МЕНШЕ значення = КРАЩА якість і більший кадр (0..63).
// 10-12 — типовий робочий діапазон для стріму.
static const int START_JPEG_QUALITY = 12;

// ---------------------------------------------------------------------------
// MJPEG multipart-протокол
// ---------------------------------------------------------------------------
#define PART_BOUNDARY "frameboundary"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART_HEADER  = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t s_web_server    = nullptr;  // порт 80: сторінка, статус, керування
static httpd_handle_t s_stream_server = nullptr;  // порт 81: тільки потік

// Телеметрія.
//
// Клієнтів стріму може бути кілька одночасно — вони ділять між собою спільну
// пропускну здатність (виміряно: два клієнти дають 13.3 + 7.4 fps там, де один
// давав 21). Тому рахувати FPS усередині обробника не можна: кожен екземпляр
// перезаписував би спільну змінну своїм значенням, і в /status потрапляло б
// сміття. Замість цього обробники лише інкрементують спільний лічильник кадрів,
// а сукупний FPS раз на секунду рахує loop().
static volatile uint32_t s_frames_total  = 0;
static volatile float    s_fps           = 0.0f;   // сумарно по всіх клієнтах
static volatile size_t   s_last_frame_kb = 0;
static volatile int      s_clients       = 0;

// ---------------------------------------------------------------------------
// Діагностика ребутів
//
// Для системи, що має працювати цілодобово, важливо не просто побачити,
// що плата перезавантажилась, а знати ЧОМУ. ESP32 зберігає причину скидання
// між ребутами, і вона одразу розділяє два різні класи проблем:
//   BROWNOUT       — просадка живлення (слабкий БЖ/кабель) → залізо
//   PANIC / WDT    — креш або зависання в коді → софт
//   POWERON / EXT  — нормальний старт або кнопка RST
// ---------------------------------------------------------------------------
static const char *reset_reason_str() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXT_PIN";
    case ESP_RST_SW:       return "SW_RESTART";
    case ESP_RST_PANIC:    return "PANIC";        // виняток у коді
    case ESP_RST_INT_WDT:  return "INT_WDT";      // зависла перервана секція
    case ESP_RST_TASK_WDT: return "TASK_WDT";     // задача не віддала CPU
    case ESP_RST_WDT:      return "OTHER_WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";     // просадка живлення
    case ESP_RST_DEEPSLEEP:return "DEEPSLEEP";
    case ESP_RST_SDIO:     return "SDIO";
    default:               return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// Ініціалізація камери
// ---------------------------------------------------------------------------
static bool camera_init() {
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = XCLK_FREQ_HZ;
  config.pixel_format = PIXFORMAT_JPEG;   // сенсор сам стискає — ESP32 не витрачає CPU
  config.frame_size   = START_FRAMESIZE;
  config.jpeg_quality = START_JPEG_QUALITY;

  if (psramFound()) {
    // Два буфери + GRAB_LATEST: поки один кадр віддається по мережі,
    // другий уже знімається. LATEST означає «віддавай найсвіжіший,
    // старі викидай» — це прибирає накопичення затримки.
    config.fb_count    = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode   = CAMERA_GRAB_LATEST;
  } else {
    Serial.println("[cam] УВАГА: PSRAM не знайдено — падаємо до QVGA й одного буфера");
    config.frame_size  = FRAMESIZE_QVGA;
    config.fb_count    = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[cam] esp_camera_init() failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("[cam] сенсор PID=0x%02x\n", s->id.PID);

  // Модуль Freenove змонтований «догори ногами» відносно типового корпусу
  s->set_hmirror(s, 1);
  s->set_vflip(s, 1);
  s->set_brightness(s, 1);
  s->set_ae_level(s, 0);

  return true;
}

// ---------------------------------------------------------------------------
// HTTP-хендлери
// ---------------------------------------------------------------------------

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Connection", "close");

  s_clients++;
  Serial.printf("[stream] клієнт підключився (усього %d)\n", s_clients);

  char part_header[64];

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("[stream] esp_camera_fb_get() повернув null");
      res = ESP_FAIL;
      break;
    }

    size_t hdr_len = snprintf(part_header, sizeof(part_header), STREAM_PART_HEADER, fb->len);

    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part_header, hdr_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);

    s_last_frame_kb = fb->len / 1024;
    esp_camera_fb_return(fb);   // ОБОВ'ЯЗКОВО: не повернеш буфер — камера стане намертво

    if (res != ESP_OK) break;   // клієнт відвалився

    s_frames_total++;           // сукупний FPS порахує loop()
  }

  s_clients--;
  Serial.printf("[stream] клієнт відключився (лишилось %d)\n", s_clients);
  return res;
}

static esp_err_t jpg_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

static esp_err_t status_handler(httpd_req_t *req) {
  sensor_t *s = esp_camera_sensor_get();
  char json[512];
  snprintf(json, sizeof(json),
           "{\"uptime_s\":%lu,\"reset_reason\":\"%s\",\"rssi\":%d,\"ip\":\"%s\","
           "\"heap_free\":%u,\"psram_free\":%u,"
           "\"streaming\":%s,\"clients\":%d,\"fps\":%.1f,\"frame_kb\":%u,"
           "\"framesize\":%d,\"quality\":%d,\"xclk_mhz\":%d}",
           (unsigned long)(millis() / 1000), reset_reason_str(),
           WiFi.RSSI(), WiFi.localIP().toString().c_str(),
           (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram(),
           s_clients > 0 ? "true" : "false", s_clients, s_fps, (unsigned)s_last_frame_kb,
           s->status.framesize, s->status.quality, XCLK_FREQ_HZ / 1000000);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

// /control?var=framesize&val=8  — щоб міряти FPS на різних режимах не перепрошиваючись
static esp_err_t control_handler(httpd_req_t *req) {
  char query[64], var[16], val_s[16];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(query, "val", val_s, sizeof(val_s)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need ?var=&val=");
    return ESP_FAIL;
  }

  int       val = atoi(val_s);
  sensor_t *s   = esp_camera_sensor_get();
  int       rc  = -1;

  if      (!strcmp(var, "framesize"))  rc = s->set_framesize(s, (framesize_t)val);
  else if (!strcmp(var, "quality"))    rc = s->set_quality(s, val);
  else if (!strcmp(var, "brightness")) rc = s->set_brightness(s, val);
  else if (!strcmp(var, "contrast"))   rc = s->set_contrast(s, val);
  else if (!strcmp(var, "ae_level"))   rc = s->set_ae_level(s, val);
  else if (!strcmp(var, "hmirror"))    rc = s->set_hmirror(s, val);
  else if (!strcmp(var, "vflip"))      rc = s->set_vflip(s, val);

  if (rc != 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown var or bad value");
    return ESP_FAIL;
  }
  Serial.printf("[control] %s = %d\n", var, val);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "ok", 2);
}

static esp_err_t index_handler(httpd_req_t *req) {
  static const char page[] PROGMEM = R"HTML(<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>ESP32-S3 CAM</title>
<style>body{background:#111;color:#ddd;font:14px system-ui;margin:0;padding:12px;text-align:center}
img{max-width:100%;border-radius:8px}a{color:#6cf}pre{text-align:left;display:inline-block}</style>
<h3>ESP32-S3 CAM</h3>
<img id=v><p><a href=/status>/status</a> &middot; <a href=/jpg>/jpg</a></p>
<p>Роздільність:
<button onclick="c('framesize',5)">QVGA</button>
<button onclick="c('framesize',8)">VGA</button>
<button onclick="c('framesize',9)">SVGA</button>
<button onclick="c('framesize',11)">HD</button></p>
<pre id=s></pre>
<script>
v.src='http://'+location.hostname+':81/stream';
function c(k,val){fetch('/control?var='+k+'&val='+val)}
setInterval(async()=>{s.textContent=JSON.stringify(await(await fetch('/status')).json(),null,1)},1000);
</script>)HTML";

  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, page, strlen(page));
}

static void start_servers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.max_uri_handlers = 8;
  cfg.stack_size       = 8192;
  cfg.lru_purge_enable = true;

  httpd_uri_t uri_index   = {"/",        HTTP_GET, index_handler,   nullptr};
  httpd_uri_t uri_jpg     = {"/jpg",     HTTP_GET, jpg_handler,     nullptr};
  httpd_uri_t uri_status  = {"/status",  HTTP_GET, status_handler,  nullptr};
  httpd_uri_t uri_control = {"/control", HTTP_GET, control_handler, nullptr};

  if (httpd_start(&s_web_server, &cfg) == ESP_OK) {
    httpd_register_uri_handler(s_web_server, &uri_index);
    httpd_register_uri_handler(s_web_server, &uri_jpg);
    httpd_register_uri_handler(s_web_server, &uri_status);
    httpd_register_uri_handler(s_web_server, &uri_control);
  }

  // Потік — на окремому порту й окремому сервері: stream_handler блокується
  // назавжди, і якби він жив на порту 80, /status і /control перестали б
  // відповідати під час стріму.
  cfg.server_port      = 81;
  cfg.ctrl_port       += 1;
  cfg.max_uri_handlers = 1;

  httpd_uri_t uri_stream = {"/stream", HTTP_GET, stream_handler, nullptr};
  if (httpd_start(&s_stream_server, &cfg) == ESP_OK) {
    httpd_register_uri_handler(s_stream_server, &uri_stream);
  }
}

// ---------------------------------------------------------------------------

static void wifi_connect() {
#if USE_STATIC_IP
  IPAddress ip(STATIC_IP), gw(GATEWAY_IP), mask(SUBNET_MASK), dns(DNS_IP);
  if (!WiFi.config(ip, gw, mask, dns)) Serial.println("[wifi] статичний IP не застосувався");
#endif
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // без цього FPS плаває: радіо засинає між кадрами
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[wifi] підключення");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\n[wifi] OK  ip=%s  rssi=%d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== VideoDetection / ESP32-S3 CAM ===");
  Serial.printf("[sys] причина старту: %s\n", reset_reason_str());
  Serial.printf("[sys] flash=%u МБ  psram=%u МБ  cpu=%u МГц\n",
                ESP.getFlashChipSize() / (1024 * 1024),
                ESP.getPsramSize() / (1024 * 1024),
                ESP.getCpuFreqMHz());

  if (!camera_init()) {
    Serial.println("[cam] ініціалізація провалилась — перевір шлейф камери й живлення");
    while (true) delay(1000);
  }

  wifi_connect();

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mdns] також доступна як http://%s.local\n", MDNS_HOSTNAME);
  }

  start_servers();

  Serial.printf("[http] сторінка : http://%s/\n",         WiFi.localIP().toString().c_str());
  Serial.printf("[http] потік    : http://%s:81/stream\n", WiFi.localIP().toString().c_str());
}

void loop() {
  // Сукупний FPS по всіх клієнтах: рахуємо тут, а не в обробнику стріму,
  // бо обробників може бути кілька і кожен затирав би чуже значення.
  static uint32_t last_frames = 0;
  static uint32_t last_ms     = 0;

  uint32_t now_ms = millis();
  if (last_ms != 0 && now_ms > last_ms) {
    uint32_t frames = s_frames_total;
    s_fps = (float)(frames - last_frames) * 1000.0f / (float)(now_ms - last_ms);
    last_frames = frames;
  }
  last_ms = now_ms;

  // Решта роботи — в задачах HTTP-сервера. Тут лише наглядаємо за Wi-Fi.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[wifi] з'єднання втрачено, перепідключаюсь");
    WiFi.reconnect();
    delay(2000);
  }
  delay(1000);
}
