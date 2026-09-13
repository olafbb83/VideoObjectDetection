/**********************************************************************
 * VideoDetection — камера як джерело відео
 *
 * Прошивка робить рівно одне: віддає MJPEG-потік по Wi-Fi, щоб хаб
 * (ноут / RPi5) міг його читати через OpenCV і згодовувати YOLO.
 *
 * Ендпоінти:
 *   http://<ip>/          — сторінка з відео, кнопками налаштувань, статусом
 *   http://<ip>:81/stream — MJPEG-потік (це те, що читає Python)
 *   http://<ip>/jpg       — один кадр JPEG
 *   http://<ip>/status    — JSON з телеметрією, сенсором, профілем, автоекспозицією
 *   http://<ip>/control?var=xclk&val=10 — зміна параметрів на льоту
 *   http://<ip>/save      — зберегти поточні налаштування для ЦЬОГО сенсора
 *   http://<ip>/reset     — забути збережене, повернути профіль за замовчуванням
 *
 * Змінні /control:
 *   framesize, quality, brightness, contrast, ae_level, hmirror, vflip
 *   xclk      — частота такту, МГц. РУЧНА зміна вимикає авторежим XCLK
 *   auto      — 1/0: адаптивний XCLK за освітленням (лише для сенсорів,
 *               що вміють віддавати стан автоекспозиції)
 *   thr_lo    — підсилення (1/16 x) ВИЩЕ цього -> темно, перемикаємось на xclk_lo
 *   thr_hi    — підсилення НИЖЧЕ цього -> світло, повертаємось на xclk_hi
 *   xclk_hi, xclk_lo — частоти для світла й темряви
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
#include <Preferences.h>

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"

#include "camera_pins.h"

#if !__has_include("config.h")
#error "Немає src/config.h — скопіюй src/config.example.h у src/config.h і впиши свій Wi-Fi"
#endif
#include "config.h"

// ---------------------------------------------------------------------------
// Профілі сенсорів
//
// На цю плату стають різні модулі з однаковим роз'ємом: OV2640, OV3660,
// OV5640. Електрично вони сумісні, але оптимальні налаштування в них різні,
// і перевірено це на собі: OV5640 на налаштуваннях першого модуля (OV3660) дав вертикальні
// нерухомі смуги й утричі важчі JPEG.
//
// Тому налаштування не зашиті константами, а обираються за PID сенсора,
// який драйвер читає при ініціалізації. Поверх профілю можуть лягти
// налаштування, збережені у флеш (NVS) — теж окремо для кожного PID. Так
// заміна модуля туди й назад не вимагає перепрошивки: кожен сенсор підтягує
// своє.
//
// Поле note — чесна позначка, наскільки значенням можна вірити.
// ---------------------------------------------------------------------------
struct SensorProfile {
  uint16_t    pid;
  const char *name;
  int         xclk_mhz;      // тактова частота, яку МИ подаємо на сенсор
  framesize_t framesize;
  int         quality;       // МЕНШЕ = краща якість і більший кадр (0..63)
  int         hmirror;
  int         vflip;
  bool        aec_readable;  // чи вміємо читати стан автоекспозиції (для авто-XCLK)
  int         xclk_hi;       // авто-XCLK: частота при достатньому світлі
  int         xclk_lo;       // авто-XCLK: частота в темряві
  bool        auto_default;  // чи вмикати авто-XCLK без збережених налаштувань
  int         thr_lo;        // авто-XCLK: підсилення вище -> темно (0 = не відкалібровано)
  int         thr_hi;        // авто-XCLK: підсилення нижче -> світло
  const char *note;
};

static const SensorProfile PROFILES[] = {
  // Виміряно на двох екземплярах у тій самій кімнаті, що й OV3660 та OV5640:
  // смуг у темряві немає ні на якій частоті. Орієнтація 1/1 — для OV2640 №1;
  // №2 (HDF3M-811) стоїть повернутим на 180° і потребує 0/0. Прошивка
  // їх не розрізняє (PID однаковий), тож після заміни орієнтацію виправляють
  // вручну.
  {OV2640_PID, "OV2640", 20, FRAMESIZE_VGA, 12, 1, 1, false, 20, 20, false, 0, 0,
   "виміряно на 2 модулях: 20 МГц без смуг і вдень, і вночі; VGA 19-24 fps"},
  // Рідний модуль, що йшов із платою Freenove, — на етапах 1-5 єдиний. Спершу
  // помилково вважався OV2640 (модель узяли
  // з документації, PID у лозі не перевіряли), тож усе виміряне тоді — про OV3660.
  // Орієнтацію (дзеркало 0, переворот 1) підтвердив користувач. На етапах 1-5
  // прошивка ставила дзеркало 1, тож тодішні кадри, найімовірніше, були
  // віддзеркалені зліва направо — у кадрі кімнати без тексту цього не видно.
  {OV3660_PID, "OV3660", 20, FRAMESIZE_VGA, 12, 0, 1, false, 20, 20, false, 0, 0,
   "виміряно: 20 МГц без смуг і вдень, і вночі; VGA 25-26 fps, кадр 13-16 КБ"},
  // Виміряно (docs/benchmarks.md):
  //  - FPS ~ 1 кадр на МГц XCLK; у темряві на 20 МГц кадр майже знищений
  //    смугами, на 8 МГц чисто і вдень, і вночі — тому старт на 8;
  //  - підсилення зі світлом 21-28, у темряві впирається в стелю 248 —
  //    звідси пороги авто-XCLK 60 / 160;
  //  - орієнтацію (дзеркало 1, переворот 0) підтвердив користувач.
  {OV5640_PID, "OV5640", 8, FRAMESIZE_VGA, 12, 1, 0, true, 16, 8, true, 160, 60,
   "виміряно: 8 МГц чисто завжди; авто-XCLK 16/8 за підсиленням 60/160"},
};

// Для невідомого сенсора — повільніший такт: менше шансів на биті дані,
// а швидкість можна підняти вручну.
static const SensorProfile UNKNOWN_PROFILE = {
  0, "unknown", 10, FRAMESIZE_VGA, 12, 1, 0, false, 10, 10, false, 0, 0,
  "невідомий сенсор, консервативні значення"};

// На цій частоті стартуємо, поки PID ще невідомий. На 20 МГц успішно
// піднімались і OV3660, і OV5640 — читання PID іде по SCCB (по суті I2C),
// і смуги на паралельній шині даних йому не заважають.
static const int BOOT_XCLK_MHZ = 20;

// Активні налаштування: профіль + можливі збережені поверх нього
struct CamSettings {
  int         xclk_mhz;
  framesize_t framesize;
  int         quality;
  int         hmirror;
  int         vflip;
};

static const SensorProfile *s_profile     = &UNKNOWN_PROFILE;
static uint16_t             s_pid         = 0;
static bool                 s_from_saved  = false;   // чи лягли збережені з NVS
static Preferences          s_prefs;

// ---------------------------------------------------------------------------
// Адаптивний XCLK за освітленням
//
// Навіщо. Виміряно на OV5640: зі світлом 16 МГц дає ~16 fps без смуг, а 8 МГц —
// лише 8.7 fps. У темряві 16 МГц дає сильні смуги, 8 МГц — чисто. Постійне
// значення змушує обирати між денною швидкістю й нічною якістю.
//
// ЗА ЧИМ ВИРІШУВАТИ. Цей вибір зроблено за вимірюваннями, і обидва мої
// теоретичні варіанти до них не дожили (docs/benchmarks.md):
//
//   Не яскравість кадру: автоекспозиція якраз і тримає її сталою.
//
//   Не експозиція × підсилення. Ідея була, що при зміні частоти один
//   множник виграє рівно стільки, скільки програє інший. Зі світлом добуток
//   на 16 і 8 МГц розійшовся на 77%.
//
//   Не той самий добуток, поділений на XCLK (гіпотеза «регістр рахує рядки,
//   а не час»). Зі світлом розкид упав до 12%, але в темряві виріс до 67%.
//
// У темряві експозиція (885) і підсилення (248) виявились ОДНАКОВИМИ на обох
// частотах — автоекспозиція вперлась у стелю обох. 885 рядків сенсор тримає
// на 16 МГц навіть при світлі в кімнаті.
//
// Вирішує ПІДСИЛЕННЯ. Воно відділяє світло від темряви з великим запасом
// (21-28 проти 248), а головне — після перемикання частоти зсувається в
// правильний бік. Зі світлом при переході 16 -> 8 МГц воно РОСТЕ (21 -> 28),
// тобто віддаляється від порогу повернення, а в темряві не змінюється.
// Тож перемикання саме себе закріплює і не смикається туди-назад.
// Для порівняння: добуток експозиції на підсилення на тому ж переході падав
// удвічі — прямо назустріч порогу повернення.
//
// Обмеження: виміряно лише два рівні освітлення. Сутінки — ні. Тому між
// порогами широкий проміжок (60..160), умова має триматись 5 с поспіль,
// а /status лічить перемикання (auto_switches): якщо число швидко росте,
// режим смикається, і пороги треба розводити.
// ---------------------------------------------------------------------------
struct AutoXclk {
  bool    enabled;
  int     xclk_hi;
  int     xclk_lo;
  int32_t thr_to_lo;   // підсилення вище -> темно
  int32_t thr_to_hi;   // підсилення нижче -> світло
};

static AutoXclk s_auto = {false, 20, 20, 0, 0};

static const int      AUTO_HOLD_S    = 5;     // скільки секунд поспіль має триматись умова
static const uint32_t AUTO_SETTLE_MS = 4000;  // після перемикання AEC перелаштовується

static int32_t  s_aec_exposure  = -1;   // сире значення регістрів, рядки
static int32_t  s_aec_gain      = -1;   // сире значення, 1/16 x
static float    s_gain_ema      = -1;   // згладжене підсилення — сигнал рішення
static uint32_t s_settle_until  = 0;
static int      s_votes_lo      = 0;
static int      s_votes_hi      = 0;
static uint32_t s_auto_switches = 0;

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
// Клієнтів стріму може бути кілька одночасно, тому рахувати FPS усередині
// обробника не можна: кожен екземпляр перезаписував би спільну змінну своїм
// значенням. Обробники лише інкрементують спільний лічильник кадрів,
// а сукупний FPS раз на секунду рахує loop().
static volatile uint32_t s_frames_total  = 0;
static volatile float    s_fps           = 0.0f;   // сумарно по всіх клієнтах
static volatile size_t   s_last_frame_kb = 0;
static volatile int      s_clients       = 0;

// ---------------------------------------------------------------------------
// Діагностика ребутів
//
// Для системи, що має працювати цілодобово, важливо не просто побачити,
// що плата перезавантажилась, а знати ЧОМУ:
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
// Збереження налаштувань у NVS, окремо для кожного сенсора
//
// Ключі мають вигляд "5640_xclk": префікс — PID у hex. NVS обмежує ключ
// 15 символами, цього вистачає з запасом.
// ---------------------------------------------------------------------------
static void nvs_key(char *out, size_t n, const char *field) {
  snprintf(out, n, "%04x_%s", s_pid, field);
}

static CamSettings settings_from_profile(const SensorProfile *p) {
  return {p->xclk_mhz, p->framesize, p->quality, p->hmirror, p->vflip};
}

static AutoXclk auto_from_profile(const SensorProfile *p) {
  return {p->auto_default && p->aec_readable, p->xclk_hi, p->xclk_lo, p->thr_lo, p->thr_hi};
}

static CamSettings load_settings() {
  CamSettings cs = settings_from_profile(s_profile);
  s_auto = auto_from_profile(s_profile);
  char key[16];

  nvs_key(key, sizeof(key), "set");
  s_from_saved = s_prefs.getUChar(key, 0) == 1;
  if (s_from_saved) {
    nvs_key(key, sizeof(key), "xclk"); cs.xclk_mhz  = s_prefs.getInt(key, cs.xclk_mhz);
    nvs_key(key, sizeof(key), "fs");   cs.framesize = (framesize_t)s_prefs.getInt(key, cs.framesize);
    nvs_key(key, sizeof(key), "q");    cs.quality   = s_prefs.getInt(key, cs.quality);
    nvs_key(key, sizeof(key), "hm");   cs.hmirror   = s_prefs.getInt(key, cs.hmirror);
    nvs_key(key, sizeof(key), "vf");   cs.vflip     = s_prefs.getInt(key, cs.vflip);

    nvs_key(key, sizeof(key), "auto"); s_auto.enabled   = s_prefs.getUChar(key, s_auto.enabled) == 1;
    nvs_key(key, sizeof(key), "xhi");  s_auto.xclk_hi   = s_prefs.getInt(key, s_auto.xclk_hi);
    nvs_key(key, sizeof(key), "xlo");  s_auto.xclk_lo   = s_prefs.getInt(key, s_auto.xclk_lo);
    nvs_key(key, sizeof(key), "tlo");  s_auto.thr_to_lo = s_prefs.getInt(key, s_auto.thr_to_lo);
    nvs_key(key, sizeof(key), "thi");  s_auto.thr_to_hi = s_prefs.getInt(key, s_auto.thr_to_hi);
  }

  // Якщо авторежим увімкнений, стартуємо на темновій частоті, а не на
  // збереженій. Інакше, якщо /save натиснули вдень, коли авто вже підняв
  // XCLK до 16, плата після нічного ребуту стартувала б на 16 зі смугами
  // й чекала б кілька секунд, поки авто зведе частоту вниз.
  if (s_auto.enabled && s_profile->aec_readable) cs.xclk_mhz = s_auto.xclk_lo;
  return cs;
}

static void save_current_settings() {
  sensor_t *s = esp_camera_sensor_get();
  char key[16];
  nvs_key(key, sizeof(key), "xclk"); s_prefs.putInt(key, s->xclk_freq_hz / 1000000);
  nvs_key(key, sizeof(key), "fs");   s_prefs.putInt(key, s->status.framesize);
  nvs_key(key, sizeof(key), "q");    s_prefs.putInt(key, s->status.quality);
  nvs_key(key, sizeof(key), "hm");   s_prefs.putInt(key, s->status.hmirror);
  nvs_key(key, sizeof(key), "vf");   s_prefs.putInt(key, s->status.vflip);
  nvs_key(key, sizeof(key), "auto"); s_prefs.putUChar(key, s_auto.enabled ? 1 : 0);
  nvs_key(key, sizeof(key), "xhi");  s_prefs.putInt(key, s_auto.xclk_hi);
  nvs_key(key, sizeof(key), "xlo");  s_prefs.putInt(key, s_auto.xclk_lo);
  nvs_key(key, sizeof(key), "tlo");  s_prefs.putInt(key, s_auto.thr_to_lo);
  nvs_key(key, sizeof(key), "thi");  s_prefs.putInt(key, s_auto.thr_to_hi);
  nvs_key(key, sizeof(key), "set");  s_prefs.putUChar(key, 1);
  s_from_saved = true;
}

static void forget_saved_settings() {
  char key[16];
  for (const char *field : {"xclk", "fs", "q", "hm", "vf", "auto", "xhi", "xlo", "tlo", "thi", "set"}) {
    nvs_key(key, sizeof(key), field);
    s_prefs.remove(key);
  }
  s_from_saved = false;
}

static void apply_runtime(const CamSettings &cs) {
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, cs.framesize);
  s->set_quality(s, cs.quality);
  s->set_hmirror(s, cs.hmirror);
  s->set_vflip(s, cs.vflip);
  s->set_brightness(s, 1);
  s->set_ae_level(s, 0);
}

// ---------------------------------------------------------------------------
// Стан автоекспозиції
//
// Поля status.aec_value / status.agc_gain у драйвері для цього не годяться:
// там лежать значення, які задають ВРУЧНУ, а не живий стан автоматики.
// Тому читаємо регістри сенсора напряму.
//
// Адреси OV5640 з даташиту. Перевірено на цій платі лише поведінково:
// значення помітно ростуть у темряві й впираються в стелю.
//   0x3500[3:0], 0x3501, 0x3502 — експозиція, 20 біт, у 1/16 рядка
//   0x350A[1:0], 0x350B         — реальне підсилення, 10 біт, у 1/16 x
// Стеля підсилення 248, яку видно в темряві, найімовірніше збігається з
// типовим значенням регістрів стелі AEC (0x3A18/0x3A19 = 0x00F8) — не перевірено.
// ---------------------------------------------------------------------------
static bool read_aec(int32_t *exposure_lines, int32_t *gain16) {
  if (!s_profile->aec_readable) return false;
  sensor_t *s = esp_camera_sensor_get();

  int e0 = s->get_reg(s, 0x3500, 0x0F);
  int e1 = s->get_reg(s, 0x3501, 0xFF);
  int e2 = s->get_reg(s, 0x3502, 0xFF);
  int g0 = s->get_reg(s, 0x350A, 0x03);
  int g1 = s->get_reg(s, 0x350B, 0xFF);
  if (e0 < 0 || e1 < 0 || e2 < 0 || g0 < 0 || g1 < 0) return false;

  *exposure_lines = ((e0 << 16) | (e1 << 8) | e2) >> 4;
  *gain16         = (g0 << 8) | g1;
  return true;
}

static void reset_auto_signal() {
  s_settle_until = millis() + AUTO_SETTLE_MS;
  s_gain_ema     = -1;   // згладжене значення після зміни частоти недійсне
  s_votes_lo = s_votes_hi = 0;
}

static void set_xclk_now(int mhz, const char *why) {
  sensor_t *s = esp_camera_sensor_get();
  s->set_xclk(s, LEDC_TIMER_0, mhz);
  reset_auto_signal();
  Serial.printf("[auto] %s -> XCLK %d МГц\n", why, mhz);
}

// Раз на секунду з loop(): читаємо стан завжди (для /status і калібрування),
// а рішення приймаємо лише коли авторежим увімкнений.
static void auto_xclk_tick() {
  int32_t exp_lines, gain16;
  if (millis() < s_settle_until) return;
  if (!read_aec(&exp_lines, &gain16)) return;

  s_aec_exposure = exp_lines;
  s_aec_gain     = gain16;
  // Експоненційне згладжування: одиночний спалах чи тінь не мають перемикати режим
  s_gain_ema = (s_gain_ema < 0) ? gain16 : (s_gain_ema * 0.7f + gain16 * 0.3f);

  if (!s_auto.enabled) return;
  if (s_auto.thr_to_lo <= 0 || s_auto.thr_to_hi <= 0) return;   // не відкалібровано

  sensor_t *s   = esp_camera_sensor_get();
  int       cur = s->xclk_freq_hz / 1000000;

  if (cur != s_auto.xclk_lo && s_gain_ema > s_auto.thr_to_lo) {
    if (++s_votes_lo >= AUTO_HOLD_S) {
      s_auto_switches++;
      char why[64];
      snprintf(why, sizeof(why), "темно (підсилення %.0f > %d)", s_gain_ema, (int)s_auto.thr_to_lo);
      set_xclk_now(s_auto.xclk_lo, why);
      return;
    }
  } else {
    s_votes_lo = 0;
  }

  if (cur != s_auto.xclk_hi && s_gain_ema >= 0 && s_gain_ema < s_auto.thr_to_hi) {
    if (++s_votes_hi >= AUTO_HOLD_S) {
      s_auto_switches++;
      char why[64];
      snprintf(why, sizeof(why), "світло (підсилення %.0f < %d)", s_gain_ema, (int)s_auto.thr_to_hi);
      set_xclk_now(s_auto.xclk_hi, why);
    }
  } else {
    s_votes_hi = 0;
  }
}

// ---------------------------------------------------------------------------
// Ініціалізація камери
// ---------------------------------------------------------------------------
static bool camera_start(int xclk_mhz, framesize_t framesize, int quality) {
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

  config.xclk_freq_hz = xclk_mhz * 1000000;
  config.pixel_format = PIXFORMAT_JPEG;   // сенсор сам стискає — ESP32 не витрачає CPU
  config.frame_size   = framesize;
  config.jpeg_quality = quality;

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
    Serial.printf("[cam] esp_camera_init() на %d МГц: помилка 0x%x\n", xclk_mhz, err);
    return false;
  }
  return true;
}

static bool camera_init() {
  // Крок 1. Проблема курки й яйця: XCLK треба задати ДО ініціалізації, а який
  // сенсор стоїть — стає відомо лише ПІСЛЯ неї. Тому стартуємо на частоті,
  // на якій піднімаються всі відомі нам модулі, і читаємо PID.
  if (!camera_start(BOOT_XCLK_MHZ, FRAMESIZE_VGA, 12)) return false;

  sensor_t *s = esp_camera_sensor_get();
  s_pid = s->id.PID;

  s_profile = &UNKNOWN_PROFILE;
  for (const auto &p : PROFILES) {
    if (p.pid == s_pid) { s_profile = &p; break; }
  }

  CamSettings cs = load_settings();
  Serial.printf("[cam] сенсор %s (PID=0x%04x), налаштування: %s\n",
                s_profile->name, s_pid, s_from_saved ? "ЗБЕРЕЖЕНІ у флеш" : "профіль");
  Serial.printf("[cam] профіль: %s\n", s_profile->note);

  // Крок 2. Якщо цьому сенсору потрібна інша частота — переініціалізуємо.
  //
  // Можна було б просто викликати set_xclk() на льоту (і /control так і
  // робить, і авторежим теж). Але регістри сенсора, зокрема його PLL,
  // налаштовуються при ініціалізації під ту частоту, що була на вході.
  // Чиста переініціалізація на старті дає правильну частоту з самого початку
  // і коштує лише ~1 с.
  if (cs.xclk_mhz != BOOT_XCLK_MHZ) {
    Serial.printf("[cam] переініціалізація на XCLK %d МГц\n", cs.xclk_mhz);
    esp_camera_deinit();
    delay(100);
    if (!camera_start(cs.xclk_mhz, cs.framesize, cs.quality)) {
      // Не піднялось на новій частоті — краще працювати на стартовій, ніж ніяк
      Serial.println("[cam] не вдалося, повертаюсь на стартову частоту");
      if (!camera_start(BOOT_XCLK_MHZ, cs.framesize, cs.quality)) return false;
    }
  }

  apply_runtime(cs);

  s = esp_camera_sensor_get();
  Serial.printf("[cam] XCLK %d МГц, framesize %d, quality %d, дзеркало %d, переворот %d\n",
                s->xclk_freq_hz / 1000000, s->status.framesize, s->status.quality,
                s->status.hmirror, s->status.vflip);
  if (s_profile->aec_readable) {
    Serial.printf("[auto] авто-XCLK: %s, %d/%d МГц, підсилення темно>%d світло<%d%s\n",
                  s_auto.enabled ? "УВІМКНЕНО" : "вимкнено",
                  s_auto.xclk_hi, s_auto.xclk_lo, (int)s_auto.thr_to_lo, (int)s_auto.thr_to_hi,
                  (s_auto.thr_to_lo <= 0 || s_auto.thr_to_hi <= 0) ? " (НЕ відкалібровано)" : "");
    // Після ініціалізації автоекспозиції треба кілька секунд, щоб зійтись
    reset_auto_signal();
  }
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
  char json[1024];
  // sensor і profile потрапляють у метадані кожної сесії датасету
  // (hub/collect.py зберігає /status на старті) — тож завжди видно,
  // яким сенсором і на якій частоті знято кадри.
  // light_need (експозиція × підсилення) лишено для калібрувань
  // hub/camera_tune.py; рішення авторежиму приймається за auto_gain_ema.
  snprintf(json, sizeof(json),
           "{\"uptime_s\":%lu,\"reset_reason\":\"%s\",\"rssi\":%d,\"ip\":\"%s\","
           "\"heap_free\":%u,\"psram_free\":%u,"
           "\"streaming\":%s,\"clients\":%d,\"fps\":%.1f,\"frame_kb\":%u,"
           "\"sensor\":\"%s\",\"pid\":\"0x%04x\",\"profile\":\"%s\","
           "\"framesize\":%d,\"quality\":%d,\"xclk_mhz\":%d,"
           "\"hmirror\":%d,\"vflip\":%d,"
           "\"aec_exposure\":%d,\"aec_gain\":%d,\"light_need\":%d,\"auto_gain_ema\":%.0f,"
           "\"auto_xclk\":%d,\"auto_xclk_hi\":%d,\"auto_xclk_lo\":%d,"
           "\"auto_thr_lo\":%d,\"auto_thr_hi\":%d,\"auto_switches\":%u}",
           (unsigned long)(millis() / 1000), reset_reason_str(),
           WiFi.RSSI(), WiFi.localIP().toString().c_str(),
           (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram(),
           s_clients > 0 ? "true" : "false", s_clients, s_fps, (unsigned)s_last_frame_kb,
           s_profile->name, s_pid, s_from_saved ? "saved" : "default",
           s->status.framesize, s->status.quality, s->xclk_freq_hz / 1000000,
           s->status.hmirror, s->status.vflip,
           (int)s_aec_exposure, (int)s_aec_gain,
           (s_aec_exposure >= 0 && s_aec_gain >= 0) ? (int)(s_aec_exposure * s_aec_gain) : -1,
           s_gain_ema,
           s_auto.enabled ? 1 : 0, s_auto.xclk_hi, s_auto.xclk_lo,
           (int)s_auto.thr_to_lo, (int)s_auto.thr_to_hi, (unsigned)s_auto_switches);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json, strlen(json));
}

// /control?var=xclk&val=10 — підбір налаштувань без перепрошивки.
// Зміни живуть до ребуту; щоб лишились — /save.
static esp_err_t control_handler(httpd_req_t *req) {
  char query[96], var[16], val_s[24];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
      httpd_query_key_value(query, "val", val_s, sizeof(val_s)) != ESP_OK) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "need ?var=&val=");
    return ESP_FAIL;
  }

  long      val = atol(val_s);
  sensor_t *s   = esp_camera_sensor_get();
  int       rc  = -1;

  if      (!strcmp(var, "framesize"))  rc = s->set_framesize(s, (framesize_t)val);
  else if (!strcmp(var, "quality"))    rc = s->set_quality(s, val);
  else if (!strcmp(var, "brightness")) rc = s->set_brightness(s, val);
  else if (!strcmp(var, "contrast"))   rc = s->set_contrast(s, val);
  else if (!strcmp(var, "ae_level"))   rc = s->set_ae_level(s, val);
  else if (!strcmp(var, "hmirror"))    rc = s->set_hmirror(s, val);
  else if (!strcmp(var, "vflip"))      rc = s->set_vflip(s, val);
  else if (!strcmp(var, "xclk")) {
    // Межі з запасом: нижче 4 МГц сенсори не гарантують роботу,
    // вище 24 — за межами того, що тягне паралельна шина ESP32
    if (val >= 4 && val <= 24) {
      // Ручна частота вимикає авторежим, інакше він за кілька секунд
      // перемкне її назад — і, наприклад, зіпсує прогін hub/camera_tune.py
      if (s_auto.enabled) {
        s_auto.enabled = false;
        Serial.println("[auto] вимкнено: частоту задано вручну");
      }
      rc = s->set_xclk(s, LEDC_TIMER_0, val);
      reset_auto_signal();
    }
  }
  else if (!strcmp(var, "auto")) {
    if (!s_profile->aec_readable) {
      rc = -1;   // сенсор не віддає стан експозиції — вирішувати нема за чим
    } else if (val && (s_auto.thr_to_lo <= 0 || s_auto.thr_to_hi <= 0 ||
                       s_auto.thr_to_hi >= s_auto.thr_to_lo)) {
      // Без гістерезису (thr_hi < thr_lo) режим смикався б на межі
      httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                          "calibrate first: need 0 < thr_hi < thr_lo");
      return ESP_FAIL;
    } else {
      s_auto.enabled = val != 0;
      s_votes_lo = s_votes_hi = 0;
      rc = 0;
    }
  }
  else if (!strcmp(var, "thr_lo"))  { s_auto.thr_to_lo = val; rc = val > 0 ? 0 : -1; }
  else if (!strcmp(var, "thr_hi"))  { s_auto.thr_to_hi = val; rc = val > 0 ? 0 : -1; }
  else if (!strcmp(var, "xclk_hi")) { if (val >= 4 && val <= 24) { s_auto.xclk_hi = val; rc = 0; } }
  else if (!strcmp(var, "xclk_lo")) { if (val >= 4 && val <= 24) { s_auto.xclk_lo = val; rc = 0; } }

  if (rc != 0) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown var or bad value");
    return ESP_FAIL;
  }
  Serial.printf("[control] %s = %ld\n", var, val);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "ok", 2);
}

static esp_err_t save_handler(httpd_req_t *req) {
  save_current_settings();
  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("[cam] збережено для %s: XCLK %d, fs %d, q %d, дзеркало %d, переворот %d, "
                "авто %d (%d/%d МГц, підсилення темно>%d світло<%d)\n",
                s_profile->name, s->xclk_freq_hz / 1000000, s->status.framesize,
                s->status.quality, s->status.hmirror, s->status.vflip,
                s_auto.enabled, s_auto.xclk_hi, s_auto.xclk_lo,
                (int)s_auto.thr_to_lo, (int)s_auto.thr_to_hi);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "saved", 5);
}

static esp_err_t reset_handler(httpd_req_t *req) {
  forget_saved_settings();
  CamSettings cs = settings_from_profile(s_profile);
  s_auto = auto_from_profile(s_profile);
  sensor_t *s = esp_camera_sensor_get();
  s->set_xclk(s, LEDC_TIMER_0, cs.xclk_mhz);
  reset_auto_signal();
  apply_runtime(cs);
  Serial.printf("[cam] збережене для %s забуто, повернуто профіль\n", s_profile->name);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, "reset", 5);
}

static esp_err_t index_handler(httpd_req_t *req) {
  static const char page[] PROGMEM = R"HTML(<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>ESP32-S3 CAM</title>
<style>body{background:#111;color:#ddd;font:14px system-ui;margin:0;padding:12px;text-align:center}
img{max-width:100%;border-radius:8px}a{color:#6cf}pre{text-align:left;display:inline-block}
p{margin:8px 0}button{margin:2px;padding:4px 10px}.hint{color:#888;font-size:12px}</style>
<h3 id=t>ESP32-S3 CAM</h3>
<img id=v><p><a href=/status>/status</a> &middot; <a href=/jpg>/jpg</a></p>
<p>Роздільність:
<button onclick="c('framesize',5)">QVGA</button>
<button onclick="c('framesize',8)">VGA</button>
<button onclick="c('framesize',9)">SVGA</button>
<button onclick="c('framesize',11)">HD</button></p>
<p>XCLK, МГц:
<button onclick="c('xclk',20)">20</button>
<button onclick="c('xclk',16)">16</button>
<button onclick="c('xclk',12)">12</button>
<button onclick="c('xclk',10)">10</button>
<button onclick="c('xclk',8)">8</button>
<span class=hint>(ручна зміна вимикає авто)</span></p>
<p>Авто-XCLK:
<button onclick="c('auto',1)">увімкнути</button>
<button onclick="c('auto',0)">вимкнути</button>
<span class=hint id=ah></span></p>
<p>Якість JPEG:
<button onclick="c('quality',10)">10</button>
<button onclick="c('quality',12)">12</button>
<button onclick="c('quality',16)">16</button>
<button onclick="c('quality',20)">20</button>
<span class=hint>(менше = якісніше й важче)</span></p>
<p>Орієнтація:
<button onclick="t('hmirror')">дзеркало</button>
<button onclick="t('vflip')">переворот</button></p>
<p><button onclick="g('/save')">зберегти для цього сенсора</button>
<button onclick="g('/reset')">скинути до профілю</button></p>
<pre id=s></pre>
<script>
let st={};
v.src='http://'+location.hostname+':81/stream';
async function c(k,val){const r=await fetch('/control?var='+k+'&val='+val);
 if(!r.ok)alert(await r.text());tick()}
function t(k){c(k,st[k]?0:1)}
function g(u){fetch(u).then(tick)}
async function tick(){st=await(await fetch('/status')).json();
 document.getElementById('t').textContent='ESP32-S3 CAM · '+st.sensor+' · профіль: '+st.profile;
 document.getElementById('ah').textContent=(st.auto_xclk?'увімкнено':'вимкнено')+
  ' · підсилення '+st.auto_gain_ema+' (темно>'+st.auto_thr_lo+', світло<'+st.auto_thr_hi+')'+
  ' · перемикань '+st.auto_switches;
 s.textContent=JSON.stringify(st,null,1)}
setInterval(tick,1000);tick();
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
  httpd_uri_t uri_save    = {"/save",    HTTP_GET, save_handler,    nullptr};
  httpd_uri_t uri_reset   = {"/reset",   HTTP_GET, reset_handler,   nullptr};

  if (httpd_start(&s_web_server, &cfg) == ESP_OK) {
    httpd_register_uri_handler(s_web_server, &uri_index);
    httpd_register_uri_handler(s_web_server, &uri_jpg);
    httpd_register_uri_handler(s_web_server, &uri_status);
    httpd_register_uri_handler(s_web_server, &uri_control);
    httpd_register_uri_handler(s_web_server, &uri_save);
    httpd_register_uri_handler(s_web_server, &uri_reset);
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

// Діагностика, коли плата не може знайти мережу.
//
// Коди причин, які найчастіше бачиш у лозі:
//   201 NO_AP_FOUND   — точки з такою назвою в ефірі НЕ ВИДНО (назва, 5 ГГц,
//                       прихована мережа, далеко від роутера)
//   202 AUTH_FAIL     — мережу видно, пароль не підійшов
//   15  4WAY_HANDSHAKE_TIMEOUT — теж зазвичай неправильний пароль
//
// При 201 гадати марно: скануємо ефір і показуємо, що плата бачить насправді.
// ESP32 сканує тільки 2.4 ГГц, тож якщо мережі немає в списку, а телефон
// її бачить — вона майже напевно лише на 5 ГГц.
static const char *auth_name(int a) {
  static const char *names[] = {"OPEN", "WEP", "WPA", "WPA2", "WPA/WPA2",
                                "WPA2-ENT", "WPA3", "WPA2/WPA3", "WAPI"};
  return (a >= 0 && a < (int)(sizeof(names) / sizeof(names[0]))) ? names[a] : "?";
}

static void wifi_diagnose() {
  // Назву друкуємо в лапках і з довжиною: зайвий пробіл у кінці інакше не видно
  Serial.printf("\n[wifi] шукаю \"%s\" (%u символів). Сканую ефір 2.4 ГГц...\n",
                WIFI_SSID, (unsigned)strlen(WIFI_SSID));

  WiFi.disconnect(false, false);  // активна спроба підключення заважає скану
  delay(100);
  int n = WiFi.scanNetworks();

  if (n <= 0) {
    Serial.println("[wifi] не видно жодної мережі 2.4 ГГц — перевір антену й відстань до роутера");
    return;
  }

  int exact = -1, similar = -1;
  Serial.printf("[wifi] видно мереж: %d\n", n);
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    Serial.printf("   %-32s  кан %2d  %4d dBm  %s\n",
                  ssid.length() ? ("\"" + ssid + "\"").c_str() : "(прихована)",
                  WiFi.channel(i), WiFi.RSSI(i), auth_name((int)WiFi.encryptionType(i)));

    if (ssid == WIFI_SSID) {
      exact = i;
    } else {
      String a = ssid, b = String(WIFI_SSID);
      a.trim(); b.trim();
      if (a.equalsIgnoreCase(b)) similar = i;
    }
  }

  if (exact >= 0) {
    Serial.printf("[wifi] мережу ЗНАЙДЕНО: канал %d, %d dBm, %s\n",
                  WiFi.channel(exact), WiFi.RSSI(exact),
                  auth_name((int)WiFi.encryptionType(exact)));
    if (WiFi.RSSI(exact) < -80)
      Serial.println("[wifi] сигнал дуже слабкий — підсунь плату ближче до роутера");
  } else if (similar >= 0) {
    Serial.printf("[wifi] точного збігу немає, але є \"%s\" — відрізняється регістром "
                  "або пробілами. Виправ WIFI_SSID у config.h\n",
                  WiFi.SSID(similar).c_str());
  } else {
    Serial.println("[wifi] такої мережі серед видимих 2.4 ГГц НЕМАЄ. Варіанти: "
                   "мережа тільки на 5 ГГц, прихована, або опечатка в назві");
  }
  WiFi.scanDelete();
}

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
  unsigned long started   = millis();
  unsigned long last_scan = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");

    // Не підключились за 12 с — дивимось, що плата взагалі бачить в ефірі.
    // Далі повторюємо раз на хвилину, поки не з'явиться зв'язок.
    unsigned long now = millis();
    if (now - started > 12000 && (last_scan == 0 || now - last_scan > 60000)) {
      wifi_diagnose();
      last_scan = millis();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
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

  s_prefs.begin("cam", false);

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

  // Раз на секунду: стан автоекспозиції і, якщо увімкнено, рішення про XCLK
  auto_xclk_tick();

  // Решта роботи — в задачах HTTP-сервера. Тут лише наглядаємо за Wi-Fi.
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[wifi] з'єднання втрачено, перепідключаюсь");
    WiFi.reconnect();
    delay(2000);
  }
  delay(1000);
}
