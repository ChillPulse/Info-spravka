# SOURCES MATRIX v8.1 FINAL — 5 источников (после модернизации Ё-радио)

**Статус:** FINAL, переопределяет все предыдущие списки в MASTER SPEC v7.2/v8.0
**Дата:** 2026-07-29
**Репо:** https://github.com/ChillPulse/Youradio_Omnia_ESP32S3 + https://github.com/ChillPulse/Info-spravka

## 1. Итоговый список (заказчик подтвердил)

| # | User Source (что видит пользователь) | Группа | Clock Domain | И2С роль | Модуль | Примечание |
|---|--------------------------------------|--------|--------------|----------|--------|------------|
| 1 | **yoRadio: WEB** | RADIO | DOMAIN_RADIO | AT32 MASTER / ESP32-S3 SLAVE | ESP32-S3 YoRadio | Интернет-радио, mode 0 |
| 2 | **yoRadio: SD** | RADIO | DOMAIN_RADIO | AT32 MASTER / ESP32-S3 SLAVE | ESP32-S3 YoRadio | SD карта, mode 1, работает без WiFi |
| 3 | **yoRadio: USB-Flash** | RADIO | DOMAIN_RADIO | AT32 MASTER / ESP32-S3 SLAVE | ESP32-S3 YoRadio | USB Flash Host MSC GPIO19/20 + 5V питание + полифьюз 0.75А 6В + кондеры, mode usb / mode 3 |
| 4 | **BT401: Bluetooth** | BT401 | DOMAIN_BT401 | BT401 MASTER → 74HC125 → AT32 SLAVE | BT401 | BT аудио |
| 5 | **BT401: USB Audio (PC/Android)** | BT401 | DOMAIN_BT401 | BT401 MASTER → 74HC125 → AT32 SLAVE | BT401 | USB Audio от ПК/Android |

**Отключено, не используется:** BT401: TF card, BT401: U-disk (ранее были в v7.2 как 6 источников, теперь исключены).

## 2. Clock Domains

### DOMAIN_RADIO (AT32 — MASTER)
- AT32 I2S3 = MASTER (BCLK, LRCK генерирует AT32)
- ESP32-S3 YoRadio = SLAVE (I2S_ROLE_SLAVE в Audio.cpp, уже сделано в V7G)
- Содержит 3 submode: WEB, SD, USB-Flash — все одинаково тактуются, переключение внутри без полного domain switch.
- Fs определяется по RATE_PIN: GPIO4 ESP32-S3 → PA9 STM32/AT32: LOW=44.1k, HIGH=48k. Handshake: mute HIGH → переключить RATE → delay 120ms → unmute (файл yoRadio/myoptions.h + src/core/player.cpp syncRateToStmPins).
- Подтверждено тестами: скорость нормальная, дропы исчезли (Test 7G).

### DOMAIN_BT401 (BT401 — MASTER)
- BT401 = MASTER (MCLK непрерывно стабилен независимо от DATA, подтверждение в docs/vendor_bt401_mclk_confirm.md + BT401_Frequently Asked Questions_FAQ_V3.pdf)
- AT32 I2S2/I2S3 = SLAVE
- Содержит 2 submode: Bluetooth, USB Audio (PC/Android). Переключение BT ↔ USB Audio без полного domain switch, но с BT Fs watch + BT_RESYNC.
- MCLK: 11.2896 MHz = 44.1k (256×), 12.288 MHz = 48k. Если не 11.2896/12.288 → ERR:0x0602 BT_MCLK_UNKNOWN, hold mute, UI "Unsupported sample rate" (только 44.1/48 в v8.x, нет ресемплера — Non-Goals).
- Измерение MCLK: Метод A input capture период + Метод B fallback gate window подсчет фронтов 10-50ms.
- BT_RESYNC click-free: AMP_MUTE HIGH → DSP fade-out TAU_MUTE_IN 200ms → Stop DMA I2S2 RX + I2S3 TX → DeInit I2S2+3 → пересчет коэффициентов DSP → Init slave 16-bit in 32-slot Philips → Start DMA → fade-in TAU_MUTE_OUT 350ms → MUTE LOW.

## 3. Переключение источников — правила

| Переключение | Тип | Что происходит | Звук |
|---|---|---|---|
| WEB ↔ SD | внутри RADIO | MODE:WEB/SD, soft mute 50-120ms, смена плейлиста, сохранение списка | click-free via GainSmoother |
| SD ↔ USB-Flash | внутри RADIO | MODE:SD/USB, тот же I2S master, PROGRESS/META продолжается | click-free |
| WEB ↔ USB-Flash | внутри RADIO | аналогично | click-free |
| BT ↔ USB Audio | внутри BT401 | BT401_MODE:BT/USB_AUDIO, Fs watch, при смене Fs → BT_RESYNC | click-free если Fs та же, resync если Fs поменялась |
| RADIO ↔ BT401 | между доменами | Полный STATE_SWITCHING: mute HIGH → fade-out → stop DMA → deinit → init новый домен → start DMA → fade-in → mute LOW | click-free гарантированно |

## 4. Команды протокола (FINAL)

### UI (ESP32-S2 AURA / AT32) → yoRadio (ESP32-S3)
Сохраняем весь старый CLI из файла UART (mode 0/1/2, prev, next, toggle, stop, start/play, vol, vol+/vol-, vol x, audioinfo, smartstart, list, play x, info, dspon, dim, sleep, tzo, date, version, heap, boot, reset, wifi.con/list/status/rssi...) — полная обратная совместимость.

Новые (v8.1):
```
mode usb / mode 3          // USB-Flash
seek <ms>                  // абсолютный seek ms
seek_rel <+/-ms>           // относительный для hold
seek_percent <0-1000>      // промилле 0..1000 (точнее percent)
sdpos <byte_pos>           // байтовая позиция (усилить существующую)
seek_start + / seek_start - // начало ускоренной перемотки вперед/назад
seek_stop                  // конец hold
shuffle on/off
repeat off/one/all
shuffle_repeat on/off
random
usb_scan
usb_list
usb_play <path_or_index>
status                     // расширенный статус
ping <seq>
```

**Поведение кнопок (важно, исправлено):**
- Short NEXT = следующий трек/станция
- Short PREV = предыдущий (если >3-5 сек текущего → рестарт текущего)
- Hold NEXT = перемотка вперед внутри трека с ускорением 2x→4x→8x→16x (5s шаг 0-2с, 10s 2-4с, 30s 4-7с, 60s >7с)
- Hold PREV = перемотка назад аналогично
- Encoder rotate = громкость, press short = Play/Pause, long = Source/Seek mode

### yoRadio → UI (AT32 → ESP32-S2)
Старые:
```
##CLI.META#: [Соединение]
##AUDIO.INFO#: Closing audio file ... / buffers freed, free Heap: 105128 bytes / connect to: "radiorecord..."
##CLI.NAMESET#: 246 Record Hypnotic
##CLI.META#: MINILOGUE - Deep Motions
##CLI.VOL#: 12
##CLI.PLAYING#
#CLI.LISTNUM#: 1: ...
##SYS.DATE#: 2026-02-25T21:45:12+03:00
#WIFI.STATUS# / #WIFI.RSSI#
##ERROR#: audio is not a file
```

Новые v8.1 (для красивого прогресса):
```
##PROGRESS#: <current_ms> <duration_ms> <state> <percent_x10> <file_idx> <file_cnt>
  state: 0=stop 1=play 2=pause 3=seeking 4=buffering 5=error
  percent_x10: 0..1000 (0.0%..100.0%)
  частота: 2-5 Гц обычно, до 10 Гц при seek

##META#: title=...;artist=...;album=...;year=...;bitrate=...;fmt=...;sr=...;ch=...
##TRACK#: idx=...;total=...;path=...;playlist=...
##SD.STATUS#: mounted/ejected/indexing/error size=... fs=... files=...
##USB.STATUS#: mounted/ejected/error size=... fs=... files=...
##AUDIO.ID3# (если остается)
```

### OMNIA v·core ↔ UI (AT32 ↔ S2) — спектр и т.д.
- SRC:RADIO/BT401
- MODE:WEB/SD/USB (USB = USB-Flash) — FINAL
- BT401_MODE:BT/USB_AUDIO (TF/UDISK DISABLED)
- VOL: dBx10, MUTE:ON/OFF/TOGGLE, GAIN, HEADROOM, LIM_GR, CLIP, SR:44100/48000
- SPEC:16/32 (основной визуал), FFT_FULL только диагностика
- FEAT:CENTROID/LOUD/FLUX/ATTACK/BASS, SCOPE_POINTS, METERS, DIAG, EVT:Denied
- PROGRESS/META/TRACK форвардятся с yoRadio

## 5. Железо Ё-радио — финал пинов

### ESP32-S3 ↔ STM32F401 / AT32 (I2S + RATE/MUTE)
- ESP GPIO17 → PB4 (STM) DATA OUT (I2S3ext_SD)
- PB3 (STM) → ESP GPIO16 BCLK
- PA15 (STM) → ESP GPIO18 LRCK/WS
- PB5 (STM) → PCM5102A DIN, PB3 → BCK, PA15 → LCK/WS
- ESP GPIO2 → PA8 MUTE (HIGH=mute)
- ESP GPIO4 → PA9 RATE (LOW=44.1 HIGH=48)
- GND общий

### USB-Flash Host
- ESP GPIO19 = D- , GPIO20 = D+ (USB OTG)
- USB-A female: VBUS 5V 1A+ отдельный LDO/DC-DC, 100nF+10uF на VBUS, TVS опционально, полифьюз 0.75А 6В (как уже сделано), кондеры прямо на разъеме.
- Onboard USB-C — только Device, поэтому Host через GPIO19/20 — правильный путь.

### BT401
- BT401 I2S MASTER → 74HC125 → AT32 I2S SLAVE
- Используем только BT + USB Audio (PC/Android). TF/U-disk не подключаем, не разводим, в прошивке BT401 игнорируем.

## 6. UI представление (Source Picker)

```
┌─────────────────────────────────────┐
│ Source Picker [v8.1 FINAL 5]       │
│                                     │
│  ● yoRadio: WEB      [icon globe]   │
│  ○ yoRadio: SD       [icon sd]      │
│  ○ yoRadio: USB-Flash[icon usb]     │
│  ○ BT401: Bluetooth  [icon bt]      │
│  ○ BT401: USB Audio  [icon pc]      │
│                                     │
│  Description: USB Flash via ESP32-S3│
│  Status: mounted 32GB FAT32 files 124│
│  Preview: Artist — Title (no switch)│
│  OK = apply via SRC: + MODE: + toast│
└─────────────────────────────────────┘
```

- Перебор — без переключения домена.
- OK — применяет `SRC:RADIO` + `MODE:USB` или `SRC:BT401` + `BT401_MODE:BT`, toast 2 сек "yoRadio: USB-Flash" etc.

## 7. Проверки после этапа источников

- [ ] Список ровно 5, TF/U-disk отсутствуют в меню, в коде BT401 игнорируются
- [ ] Переключение внутри RADIO (WEB↔SD↔USB-Flash) — без полного domain switch, без щелчка, PROGRESS продолжается, MUTE handshake работает, Fs меняется via RATE_PIN
- [ ] Переключение внутри BT401 (BT↔USB Audio) — без полного domain switch, Fs watch работает, BT_RESYNC при смене 44.1↔48 без щелчка
- [ ] RADIO ↔ BT401 — полный safe switch click-free, AMP_MUTE HIGH→fade-out→stop DMA→deinit→init→start→fade-in→LOW
- [ ] Unsupported SR → ERR:0x0602 + mute hold + UI "Unsupported sample rate"
- [ ] PROGRESS 2-5Hz, SEEK hold ускорение, NEXT/PREV short vs hold не конфликтуют
- [ ] USB-Flash hot-plug: выдернуть во время play → stop + ##USB.STATUS#: ejected без краша/heap leak, вставить → авто rescan list

## 8. Связь с остальным ТЗ

- Этот файл переопределяет все упоминания 6 источников в MASTER SPEC v7.2 и в ранее сгенерированном v8.0. В будущей прошивке OMNIA везде использовать только 5.
- Остальные разделы ТЗ (AURA визуализаторы Gyre/Veil/OscilloAether/Hydra Flux/Needle/Neon Thin, External Flash GPIO10-13 any vendor JEDEC, Pinmap verification, Non-Goals, Storage migration NVS Journal, DMA-only Performance Contract, Dual PEQ Room 16 + Creative 12 + Tone separate, Smoothness Soft Unmute 400ms, Preset Library 32, Chain View, Room Studio 3-level с Delays 0-30ms step 0.1, PEQ Editor live curve, Volume Right Arc полукруг с dB, Network Primary via yoRadio) остаются без изменений и стыкуются логически с этой матрицей 5 источников.
