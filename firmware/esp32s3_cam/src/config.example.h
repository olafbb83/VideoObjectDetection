#pragma once
//
// Скопіюй цей файл у config.h і впиши свої дані.
// config.h у .gitignore — паролі в репозиторій не потрапляють.

#define WIFI_SSID     "твоя_мережа_2.4GHz"
#define WIFI_PASSWORD "твій_пароль"

// Ім'я в мережі: камера буде доступна як http://esp32cam.local
// (працює, якщо роутер/ОС підтримують mDNS; інакше користуйся IP з логу)
#define MDNS_HOSTNAME "esp32cam"

// --- Статичний IP ---
// Дуже бажано: інакше після ребуту роутера IP зміниться і Python-клієнт
// на хабі загубить камеру. Постав 1 і впиши адреси своєї мережі.
#define USE_STATIC_IP 0
#define STATIC_IP     192, 168, 1, 50
#define GATEWAY_IP    192, 168, 1, 1
#define SUBNET_MASK   255, 255, 255, 0
#define DNS_IP        192, 168, 1, 1
