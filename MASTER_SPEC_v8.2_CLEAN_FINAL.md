# OMNIA / STHENOS / v·core — MASTER SPEC v8.2 CLEAN FINAL
**Статус:** FINAL для реализации (чистая версия без неактуальных цитат)
**Дата:** 2026-07-29, Kazan Europe/Moscow
**Основание:** MASTER SPEC v7.2 + все комментарии/дополнения из https://github.com/ChillPulse/Info-spravka/blob/52b24e5ff66e6dc8d7f65cd75a81e20131fd56cf/2%20Comments%20and%20corrections%20to%20the%20Technical%20Specification%20v7.2 + обсуждения визуализаторов, RGB дыхания, Dual PEQ, External Flash, Network + репо Ё-радио https://github.com/ChillPulse/Youradio_Omnia_ESP32S3
**Важное уточнение заказчика v8.1:** источников только 5 = yoRadio WEB/SD/USB-Flash + BT401 Bluetooth/USB Audio (PC/Android). TF/U-disk BT401 отключены.
**Цель:** Флагман без лагов, премиальный AURA UI, Dual PEQ (Room 16 + Creative 12), Soft Mute/Unmute, Visual Pack 12 стилей, подготовленное Ё-радио как законченный модуль.

> Этот документ — **чистая сборка только актуальных пунктов**. Все неактуальные пункты из v7.0/v7.1 (например 6 источников с TF/U-disk, полный цитатник старого ТЗ) — **удалены**. Где были изменения — оставлено только FINAL решение. Проверка полноты — в `AUDIT_v8.1_VERIFICATION_AND_YORADIO_ROADMAP.md`

---
## 0) Ссылки
- MASTER SPEC v7.2 оригинал (для истории): https://github.com/ChillPulse/Info-spravka/blob/52b24e5ff66e6dc8d7f65cd75a81e20131fd56cf/MASTER%20SPEC%20v7.2
- Все обсуждения 2213 строк: https://github.com/ChillPulse/Info-spravka/blob/52b24e5ff66e6dc8d7f65cd75a81e20131fd56cf/2%20Comments%20and%20corrections%20to%20the%20Technical%20Specification%20v7.2
- Ё-радио репо (pin rate уже работает): https://github.com/ChillPulse/Youradio_Omnia_ESP32S3
- Info-spravka (UART, SD Screen, BT401 FAQ): https://github.com/ChillPulse/Info-spravka/tree/main
- Pinterest референсы:
  - Gyre вращающийся: https://pin.it/6rexP5pTg
  - Veil аура: https://pin.it/KJa0Y3f8r
  - OscilloAether: https://pin.it/46QMjiLsa
  - Hydra Flux частицы-головастики вверх: https://pin.it/5i7yFQSJG
  - Neon Thin тонкие неон полоски: https://pin.it/7Drdm2Lkr

---
# ЧАСТЬ 0 — Приоритет №0: Ё-радио (YoRadio Maleksm на ESP32-S3) — полный путь модернизации для OMNIA

## 0.1 Зачем первым и стратегия
Цитата заказчика: "сначала необходимо подготовить наше ё-радио" + "я уже подпаялся к выводам под USB в нашем esp32-s3 пины 20 и 19, отдельное мощное питание, полифьюз 0.75А 6В, кондеры на разъёме". **Стратегия:** делаем Ё-радио законченным standalone модулем на тестовом стенде STM32F401RCT6 + PCM5102A, контролируя UART 115200 (позже 460800/921600) и слух. Только потом интеграция в OMNIA (AT32 v·core + ESP32-S2 AURA). BT401 будет давать только Bluetooth и USB Audio PC/Android, TF/U-disk не используем.

## 0.2 Текущий рубеж V7G — рабочая база из репо

**Архитектура стенда (подтверждено):**
- STM32F401RCT6 = I2S MASTER full-duplex DMA
- ESP32-S3 YoRadio = I2S SLAVE (Audio.cpp I2S_ROLE_SLAVE, декодеры aac/flac/mp3/opus/vorbis не трогали)
- PCM5102A = DAC

**Пины FINAL (не ломать):**
- ESP GPIO17 → STM PB4 DATA OUT (I2S3ext_SD)
- STM PB3 → ESP GPIO16 BCLK
- STM PA15 → ESP GPIO18 LRCK/WS
- STM PB5 → PCM5102A DIN, PB3 → BCK, PA15 → LCK/WS
- ESP GPIO2 → STM PA8 MUTE HIGH=mute
- ESP GPIO4 → STM PA9 RATE LOW=44.1k HIGH=48k (дефолт HIGH 48k)
- GND общий
- STM UART PA2/PA3 115200 тех. лог s/s1000/s0 + gain/fade/mute/restart

**Софт в репо:**
- yoRadio/src/core/player.cpp: STM_RATE_PIN=4, pinMode OUTPUT, helper syncRateToStmPins() каждые 100ms: getSampleRate() 44.1/48 изменилась? → setOutputPins(false) mute → digitalWrite RATE_PIN → delay 120ms STM retune → setOutputPins(true) unmute. В loop после Audio::loop() → syncRateToStmPins()
- STM main.c V7G: PA9 RATE вход, PA8 MUTE, auto retune: fade mute → stop DMA → deinit I2S → reinit на новую Fs → start DMA → unmute. Убрало ускоренное воспроизведение, дропы, заикания через время. Крутить PLL / держать 48k / менять Audio.cpp без handshake — не решало.
- Heap после старта >90KB (логи: buffers freed free Heap 105128 / 106344)

**Проверка этапа 0:** треки разной Fs WEB/SD → скорость норм >30мин без дропов, RATE в логах STM меняется, info показывает правильный SR.

## 0.3 USB-Flash Hardware (уже частично сделано, финализировать)

Ты сделал: GPIO19 D- / GPIO20 D+ ESP32-S3, мощное 5V, полифьюз 0.75А 6В, кондеры на USB-A мама. Правильно, т.к. onboard USB-C обычно только Device, а S3 имеет internal OTG PHY на 19/20.

**Финализировать:** USB-A female, 100nF+10uF VBUS рядом, TVS опционально, 5V 1A+ LDO/DC-DC, GND общий. Полифьюз оставить.

**Проверка:** флешки FAT32/exFAT 8-64GB определяются, FS читается, играют, не просаживают 3.3V, ток <750mA.

## 0.4 Баги YoRadio Maleksm на фикс

**Bug Shuffle пропускает 1 трек:** оригинал YoRadio official page, off-by-one в random / границе size-1. Фикс: переписать playlist manager, random без повторов пока не проиграны все (если shuffle repeat), unit-тест 10 треков shuffle → 10 уникальных без пропуска. Команда shuffle on/off.

**Bug SD требует WiFi:** условие WiFi connected блокирует SD init. Фикс: убрать зависимость, SD Mounted даже при WiFi disconnected, NTP optional. Тест: роутер выкл, SD вставлена → играет.

**Bug sdpos слабый / нет progress:** усилить команду sdpos + добавить seek (ниже).

## 0.5 Progress + Seek — флагманская фича для SD/USB-Flash (и WEB где duration известен)

**AudioI2S библиотека имеет:** getAudioCurrentTime(), getAudioFileDuration(), getFilePos(), getFileSize(), setAudioPlayPosition()/setAudioPlayTime(), canSeek.

**UART исходящее YoRadio→AT32→S2 (новое):**
```
##PROGRESS#: <cur_ms> <dur_ms> <state> <percent_x10> <idx> <total>
  state 0=stop 1=play 2=pause 3=seeking 4=buffering 5=error
  percent_x10 0..1000 (0.0%..100.0%)
  частота 2-5Hz обычно, до 10Hz при seek
##META#: title=...;artist=...;album=...;year=...;bitrate=...;fmt=...;sr=...;ch=...
##TRACK#: idx=...;total=...;path=...
##SD.STATUS#: mounted/ejected/indexing/error size=... fs=... files=...
##USB.STATUS#: mounted/ejected/error size=... fs=... files=...
```
Совместимость: ##CLI.META#, ##AUDIO.INFO#, ##CLI.NAMESET#, ##CLI.VOL#, ##CLI.PLAYING#, ##SYS.DATE#, #CLI.LISTNUM#, #WIFI.STATUS#, ##ERROR# оставляем.

**UART входящее AT32/S2→YoRadio (сохраняем весь старый CLI + новое):**
Старый: mode 0/1/2, prev/next/toggle/stop/start/play/vol/vol+/vol-/vol x, audioinfo, smartstart, list, play x, info, dspon, dim, sleep, tzo, date, version, heap, boot, reset, wifi.list/con/status/rssi...
Новое:
```
seek <ms>
seek_rel <+/-ms>
seek_percent <0-1000>
sdpos <byte_pos>
seek_start + / seek_start -  // начало hold
seek_stop
shuffle on/off
repeat off/one/all
shuffle_repeat on/off
random
usb_scan / usb_list / usb_play <path/index>
mode usb / mode 3
status
ping <seq>
```

**Кнопки FINAL (исправлено, чтобы NEXT/PREV не конфликтовали с seek):**
- Short NEXT → следующий трек/станция
- Short PREV → предыдущий (если прошло >3-5с текущего → рестарт текущего)
- Hold NEXT → перемотка вперед внутри трека с ускорением 2x→4x→8x→16x: 5s шаг 0-2с, 10s 2-4с, 30s 4-7с, 60s >7с + live шкала
- Hold PREV → назад аналогично
- Encoder rotate → громкость, short → Play/Pause, long → Source/Context/Seek mode
- Реализация seek: soft-mute 50-120ms → setFilePos/setAudioPlayTime → unmute, серия SEEK_REL при hold 100-200ms.

**Web UI:** обновить (yoRadio/data) slider elapsed/remaining.

**Проверка:** MP3/FLAC/AAC/WAV seek начало/середина/конец без щелчка, hold ускорение, PROGRESS 2-5Hz без heap спама, кириллица ID3 не ломает, NEXT/PREV не конфликтуют.

## 0.6 Shuffle / Repeat дизайн

- OFF+OFF: по порядку, стоп в конце
- OFF+ALL: по порядку цикл
- OFF+ONE: повтор одного
- ON+OFF (No Repeat): случайный без повторов → стоп
- ON+ALL: случайный, после конца новая перестановка
- ON+ONE: shuffle игнорируется, повтор одного
- UI иконки shuffle + repeat.

## 0.7 USB-Flash Playback софт путь (ESP-IDF / Arduino)

1. USB Host MSC: usb_host_install + task + driver 19/20
2. Callback connect/disconnect → ##USB.STATUS#
3. FatFS mount esp_vfs_fat FAT32/FAT16/exFAT
4. Scan рекурсивно фильтр .mp3 .flac .aac .m4a .wav .ogg .opus
5. Index как SD отдельный список USB
6. Audio connecttoFS VFS file handle
7. Команды list/play/next/prev/seek/progress/meta идентично SD
8. Hot-plug safe unmount + toast
9. Кириллица длинные пути >2GB.

**Этапы:** USB-0 hardware check, USB-1 Host VID/PID, USB-2 Mount list root, USB-3 Play one file, USB-4 Full playlist, USB-5 Hot-plug + Web UI.

**Проверка каждого:** определяется, монтируется, листается, играет без дропов, seek работает, выдергивание → stop без краша/heap leak, повторная вставка → rescan.

## 0.8 UART расширение для OMNIA — все уарты, baud, framing

- **Стенд STM32F401:** PA2/PA3 115200 тех. лог s/s1000/s0.
- **OMNIA AT32:** baud 460800/921600 цель, DMA UART + кольцевые буферы + очереди FreeRTOS HIGH commands/SPEC, MEDIUM PROGRESS, LOW STATUS, Rate limits SPEC 20-30Hz PROGRESS 2-5Hz (10Hz seek) META on-change STATUS 1/5с, AT32 = умный хаб парсит валидирует rate-limit форвардит seq/timestamp, Quality Scaling снижает rate при перегрузке.
- **Framing:** бинарный для high-rate [0xAA msg_id len LSB/MSB seq payload CRC8/16], текстовый CLI \r\n max 256 байт UTF-8.
- **Доку:** docs/uart_protocol_v1.md (примеры пакетов, state machine 5 источников, error codes), docs/yoRadio_modernization.md (5 этапов), docs/vendor_bt401_mclk_confirm.md (MCLK stable).

## 0.9 Интеграция в OMNIA после готовности

1. Заменить STM32F401 на AT32 v·core: AT32 I2S MASTER в RADIO домене (WEB/SD/USB-Flash) сохраняет RATE/MUTE handshake (следит за GPIO4 RATE).
2. AT32 форвардит PROGRESS/META/TRACK/STATUS на S2, S2 рендерит шкалу time percent.
3. Поднять baud 921600, forwarding YoRadio→AT32→S2.
4. Primary control App↔WiFi↔YoRadio S3→UART→AT32→UART→S2, Secondary optional S2 MQTT/WebSocket HA/OTA.
5. Переключение между 5 источниками click-free, no overrun_count рост, resync_count только при Fs change.

**Чеклист готовности Ё-радио к интеграции:** Shuffle без пропусков, SD без WiFi, Progress+Seek SD/USB 2-5Hz ускорение без кликов, USB mount/list/play/seek/meta/hot-plug 1ч+ нет leaks, ID3 UTF-8 кириллица, логи не спамят, CLI совместимость, Web UI progress/seek/USB, Heap >90KB free, I2S handshake без дропов >30мин.

---
# ЧАСТЬ 1 — System Overview + Sources FINAL 5

## 1.1 Роли МК
- **AT32 v·core Real-time аудио ядро:** I2S тракт, DSP Dual PEQ, meters/SPEC/SCOPE, limiter, domain switch, supervisor, Time Manager NTP via yoRadio UART always active даже в BT401 домене.
- **ESP32-S2 mini FN4R2 UI AURA:** 4MB Flash + 2MB PSRAM, 240MHz single-core, native USB Device USB-CDC сервис, RMT для WS2812/IR, render/themes/visualizers, IR self-learn, RGB breathing, BH1750/VEML7700.
- **ESP32-S3 YoRadio:** WEB/SD/USB-Flash декодер, I2S SLAVE в RADIO домене, USB Host MSC GPIO19/20 для USB-Flash, RATE/MUTE GPIO2/4, UART CLI + PROGRESS/META.

Почему так: RT аудио и UI не мешают, UI можно богатить без риска DMA/I2S.

## 1.2 FINAL SOURCES = 5 (переопределяет v7.2 6)

| # | Source | Группа | Domain | I2S | Модуль |
|---|--------|--------|--------|-----|--------|
| 1 | yoRadio: WEB mode 0 | RADIO | DOMAIN_RADIO | AT32 MASTER / S3 SLAVE | ESP32-S3 |
| 2 | yoRadio: SD mode 1 | RADIO | DOMAIN_RADIO | AT32 MASTER / S3 SLAVE | ESP32-S3 |
| 3 | yoRadio: USB-Flash mode usb/3 | RADIO | DOMAIN_RADIO | AT32 MASTER / S3 SLAVE | ESP32-S3 GPIO19/20 Host |
| 4 | BT401: Bluetooth | BT401 | DOMAIN_BT401 | BT401 MASTER → 74HC125 → AT32 SLAVE | BT401 |
| 5 | BT401: USB Audio PC/Android | BT401 | DOMAIN_BT401 | BT401 MASTER → 74HC125 → AT32 SLAVE | BT401 |

**Отключено:** BT401 TF card, U-disk — не разводим, не показываем, деактивируем в прошивке.

**Clock Domains 2, переключение:**
- Внутри RADIO WEB↔SD↔USB-Flash: без полного domain switch, MODE:WEB/SD/USB + soft mute/crossfade.
- Внутри BT401 BT↔USB Audio: без полного domain switch, но BT Fs watch + BT_RESYNC при 44.1↔48.
- RADIO ↔ BT401: полный STATE_SWITCHING click-free: MUTE HIGH → fade-out TAU_MUTE_IN 200ms → Stop DMA I2S2 RX + I2S3 TX → DeInit → recalc coeff → Init slave 16-bit in 32-slot Philips → Start DMA → fade-in TAU_MUTE_OUT 350ms → MUTE LOW.
- Vendors: BT401 MCLK непрерывно стабилен независимо от DATA (FAQ pdf + docs/vendor_bt401_mclk_confirm.md), MCLK 11.2896=44.1k 12.288=48k, измерение метод A input capture период + метод B gate window 10-50ms подсчет фронтов, если не 11.2896/12.288 → ERR:0x0602 hold mute UI Unsupported.
- yoRadio RATE/MUTE handshake уже реализован V7G (см. Часть 0).

---
# ЧАСТЬ 2 — Hardware

## 2.1 ESP32-S2 mini FN4R2 Pinmap FINAL

**Single source of truth:** firmware/omnia_ui_s2mini/include/pinmap_ui.h

**Занятые FINAL:**
- ENC_A 1, ENC_B 2, ENC_BTN 3, BUTTONS_ADC 4 (ADC1), IR_IN 5 (RMT RX), RGB_DIN 6 → 74AHCT125, VCORE_UART_TX 7 → AT32, VCORE_UART_RX 8 ← AT32, TFT_BL 16 LEDC PWM, TFT_DC 17, TFT_RST 18, TFT_CS 21, I2C_SDA 33 BH1750/VEML7700, I2C_SCL 34, TFT_MOSI 35 SPI, TFT_SCK 36 SPI, TFT_MISO 37 optional
- **External SPI Flash optional strongly recommended:** EXT_FLASH_CS 10, MOSI 11, SCK 12, MISO 13 — отдельный SPI host SPI2/FSPI или SPI3, НЕ делить с TFT (критично Performance Contract). WP/HOLD → 3.3V или GPIO14. Footprint SOP-8 + 100nF + pull-up CS. Любая марка 3.3V SPI NOR: Winbond W25Q, GigaDevice GD25Q, Macronix MX25, ISSI, XMC, BOYA, FM — auto-detect JEDEC ID 0x9F + SFDP, generic driver, fallback unknown try generic. Boot: JEDEC size detect, если ≥4MB → LittleFS /assets (шрифты Needle backgrounds иконки темы splash), если нет → silent fallback internal 4MB. Diagnostics: Manufacturer, Device ID, Size, Free space в Service Menu.

**Свободные:** 9,14,15,19,20,38,39,40,41,42 и т.д. Strapping GPIO0/45/46 не использовать, JTAG 39-42 оставлять свободными по возможности.

**UART концепция FINAL:**
- S2↔AT32 GPIO7/8 единственный выделенный, baud 460800/921600
- Диагностика S2 = USB-CDC нативный, не занимает GPIO, работает всегда
- Диагностика AT32 = свой отдельный UART
- UART0 GPIO43/44 ROM логи не трогаем

**Правило ТЗ:** обязательный bring-up тест при новой ревизии платы: проверка всех GPIO на конфликты, особенно TFT 35/36/37, UART 7/8, Ext Flash 10-13, strapping. При другой разводке меняется только pinmap_ui.h + комментарий ревизии.

## 2.2 WS2812 / RGB кольцо — дыхание

- DIN от 3.3В напрямую НЕ допускается — обязателен SN74AHCT125 level shifter.
- Рядом с кольцом электролит 470-1000uF.
- Ночной режим lux<20 → max brightness 10-15%.

**RGB Ring Effects (включая дыхание, которое ты спросил почему упустил — теперь детально):**

| Профиль | Поведение | Применение |
|---|---|---|
| **Standby Breathing** | Медленный пульс период T 8с (6-12с configurable), атака Ta 0.8с, спад Td T-Ta, b_max 8% (ночью 3-5%), b_min 0%, обновление 25Hz, формула: если t<Ta b=b_min+(b_max-b_min)*(t/Ta)^2 иначе x=(t-Ta)/Td b=b_min+(b_max-b_min)*exp(-4.5*x), gamma 2.0 b_norm=b/100 b_pwm=(b_norm^2*255) r=(theme_r*b_pwm)>>8 и т.д. | STANDBY, IDLE |
| Clock Pulse | Тик раз в секунду, короткий импульс | Часы |
| Ambient Slow | Медленный HSV перелив 5-10с | AURA Veil sync |
| Source Color | BT401 Bluetooth синий, BT401 USB Audio голубой/синий, yoRadio WEB зеленый, SD оранжевый, USB-Flash желтый/оранжевый (разные оттенки), Radio общий | При смене источника + постоянный оттенок |
| Party | Только явное вкл, реагирует на bass/energy | Party Safe preset |
| Off | Выкл | Настройка |

- Яркость авто: BH1750/VEML7700 опрос 1/2с, backlight PWM alpha 0.05 плавный шаг.
- Датчик света размещать чтобы не видел TFT/RGB иначе автоколебания.
- Привязка к темам AURA: ring_accent_r/g/b из theme struct, colormap спектр low теплее.
- Sync с визуализаторами: Gyre/Veil/Hydra Flux могут пульсировать кольцо (опционально).

**Проверка RGB дыхания:** Standby → кольцо дышит 8с без flicker, ночью 3-5% яркость, при смене источника — вспышка цвета источника, при воспроизведении — Ambient Slow без нагрузки CPU (RMT).

## 2.3 Power / Backfeed

- USB 5V и внешний 5V только с развязкой диоды/ideal diode или строгая механическая блокировка, запрет backfeed.
- Для USB-Flash Host: отдельное 5V 1A+ питание, общий GND.

## 2.4 Storage

- ESP32-S2: 4MB internal Flash LittleFS + NVS (wear-leveling) + 2MB PSRAM.
- **Карта:** Themes/fonts/icons/boot splash → LittleFS internal или external Flash /assets, Presets до 32 фабричных + до 64 total NVS, Room profiles до 5 LittleFS, Last state NVS journal, Favorites NVS, IR bindings ~12 ~1.7KB NVS, Language RU/EN NVS, TFT controller type NVS, AURA theme NVS, AURA quality NVS, DSP params RAM AT32, Event log 100 ring RAM AT32, Crash log 10 NVS ESP32, Waterfall history до 240 строк 75KB PSRAM optional, Glyph LRU cache 32-64 PSRAM/DRAM.
- **NVS dirty+timeout+standby:** флаги DIRTY_VOL/SOURCE/PRESET/DISPLAY/VISUAL/IR/SYSTEM, mark_nvs_dirty() ставит timestamp, nvs_tick() каждые 4000ms коммитит если грязно, при STANDBY/AMP OFF принудительно all. Громкость писать только если не менялась 3+с или standby, НЕ при каждом шаге энкодера! EQ/пресеты после выхода из редактора, IR после обучения, TFT при подтверждении Wizard, Theme после transition.
- **LittleFS структура:** /assets/fonts/body_16.bin title_20.bin h1_32.bin mono_12.bin, icons/src_bt.bin src_web.bin src_sd.bin src_usb.bin wifi_0..4 amp_on/off note/clip/lim, themes/obsidian.bin carbon... amber (AURA_Theme struct + colormap), boot/logo.bin, lang/ru.bin en.bin — бинарный быстрый чтение, глифы кириллица Ёё U+0400..04FF + ASCII.
- **External Flash (раздел 2.1) optional strongly recommended:** если ассетов много (все 8 тем × шрифты × Needle фоны × splash) → внешняя W25Q32/64. Решение при наполнении, архитектура поддерживает оба.
- **Old custom NVS Journal magic/seq/type/len/payload/CRC32 compaction <20% free из v7.0 — ЗАМЕНЕН на ESP-NVS.** Исключение: если настройки хранятся напрямую во внешней SPI NOR не через NVS — применяется journal формат v7.0 per docs/10_storage_nvs.md.

---
# ЧАСТЬ 3 — UI Performance Contract + Smoothness

## 3.1 Память и буферизация
ESP32-S2FN4R2 4MB Flash + 2MB PSRAM. Fullscreen framebuffer 320x240 RGB565 = 150KB → PSRAM. Архитектура: frame_buf PSRAM 150KB, scratch_buf 320x40 ~25KB PSRAM, tile_dma 320x16 ~10KB DRAM внутренняя быстрая, vis_a/b ping-pong 320x16 DRAM. Правило: большое → PSRAM, критическое/DMA → DRAM. На STM32F401 было 64KB SRAM без framebuffer, теперь можем плавные transition + persistence.

## 3.2 Рендер модель
frame_buf PSRAM → dirty-rect copy → tile_dma DRAM ping-pong → SPI DMA → TFT GRAM. Цикл: определить dirty-зоны виджетов с флагом dirty, рендер в frame_buf, нарезать тайлы 320x16, DMA push, пока DMA шлет один тайл — CPU рендерит следующий.

## 3.3 Dirty-widget и запреты
Widget struct x0 y0 x1 y1 dirty. В норм работе никакого full-screen redraw, только dirty-зоны. SPI clock 40MHz целевой, Service Menu SPI Clock Test авто-понижение 80→40→20 при артефактах. Запреты (нарушение = лаг = дешево): НЕЛЬЗЯ delay/vTaskDelay(0) в UI loop критических задач, НЕЛЬЗЯ рисовать TFT-API из ISR, НЕЛЬЗЯ full-screen redraw в норм работе, НЕЛЬЗЯ блокирующие в task энкодера/кнопок, НЕЛЬЗЯ читать пиксели из GRAM, НЕЛЬЗЯ логировать из DMA ISR, НЕЛЬЗЯ PSRAM доступ из ISR.

## 3.4 FreeRTOS задачи S2
ui_render_task Core0 prio3 8KB, ui_input_task prio5 4KB (вытесняет render при нажатии → мгновенная реакция), ui_comm_task prio4 4KB, rgb_task prio2 2KB RMT, sensor_task prio1 2KB BH1750/VEML. Одноядерный, поэтому input должен вытеснять render.

## 3.5 Частоты виджетов
Часы HH:MM 1/мин, статус-бар event-driven, метаданные event-driven + toast 3-4с, мини VU/Spectrum 20-25Hz, FFT bars SPEC 20-25Hz, Waterfall 20-25Hz 1 строка hw scroll, Lissajous 20-30Hz, Volume OSD 60Hz + timeout 2.5с, RGB 25-50Hz RMT, сенсор 1/2с, backlight alpha 0.05.

## 3.6 Flagship Smoothness System (аудио+визуал, отвечает на вопрос про плавность mute)

**Аудио:** единый GainSmoother 1-pole IIR current+= (target-current)*alpha alpha=1-exp(-block/(fs*tau)). TAU_VOL 80ms, TAU_MUTE_IN 200ms затухание, TAU_MUTE_OUT 350-450ms нарастание (по умолчанию 400ms raised-cosine/exponential, приятно и безопасно для слуха и акустики), TAU_SRC 20ms. GainSmoother применяется к Master Volume, Mute/Unmute, Source trim/headroom, вкл/выкл эффектов меняющих амплитуду, Dry/wet crossfade AETHER/Night/Anti-Ad, любые enable переключения, Preset/Room transitions, Limiter makeup. Soft Unmute рампа 100-800ms default 400ms от -∞ к target volume по raised-cosine от точного положения регулятора громкости (то что показывает Volume Arc), защита слуха max ramp rate limited, при первом включении после долгого простоя более длинный soft-start Optional Safe Volume ceiling. Soft Mute 80-150ms. Все continuous параметры gain/Fc/Q/Width/DryWet и biquad коэффициенты с ramping минимум 5-10ms, запрещены мгновенные gain changes. Source domain switch эталон (mute HIGH fade-out → stop DMA → deinit → recalc → init → start → fade-in → LOW). Look-ahead Limiter гладкий gain reduction.

**Soft Bypass всех блоков:** не hard on/off, а короткий crossfade 20-50ms.

**Визуал:** запрет flicker/tearing. Только DMA SPI + double buffer / tear-free partial update + vsync по TE пину если выведен TEON 0x35. Все анимации Motion Tokens Fast 140ms easeOutCubic для нажатия HUD, Normal 220ms переход экранов/toast, Slow 420ms смена темы ambient. Формулы ease_out_cubic 1 - (1-t)^3, ease_in_out_cubic t<0.5 4t^3 else 1-( -2t+2)^3 /2. Постоянный frame pacing приоритет стабильности над пиком FPS, при нагрузке Quality Scaling снижает эффекты раньше мерцания. Яркость colormap transitions crossfade. Volume Ballistics разная скорость вверх/вниз + ускорение при быстром вращении энкодера как у хороших интегральников.

**Проверка плавности:** Unmute → 400ms плавный без щелчка до точного уровня регулятора (осциллограф), быстрое вращение энкодера → ускоряется без ступеней, анимации без flicker FPS jitter <10%, нет мерцания даже при 100% DSP load.

---
# ЧАСТЬ 4 — AURA Visual System + HUD

## 4.1 Design System
Grid 4px, insets 8/12/16/24, corner 12px карточки 999px пилюли, stroke 1px light 2px dark. Typography H1 часы/громкость OSD, Title Preset/Station, Body Meta artist-title, MonoSmall SR/Bits/Bitrate/Headroom/GR/Debug. Цветовые токены theme struct bg/surface/outline/text/muted/accent/accent2/ok/warn/danger/spec_colormap[8]/ring_accent_r/g/b.

Темы 8: Obsidian глубокий черный голубой акцент, Carbon темно-серый белый, Aurora темный aurora-green/violet, Studio почти черный оранжевый, Night Warm очень темный amber, Ivory светлая опционально, Emerald изумрудный, Amber янтарный. Реализация extern const AURA_Theme theme_obsidian... const AURA_Theme *aura_active указатель COL(token). Плавная смена темы 300-500ms lerp tokens за 5-10 шагов.

## 4.2 Interaction: Press + Focus ring
Press эффект 80-140ms: карточка темнее/светлее delta surface ±15%, outline ярче accent, опционально уменьшение внутреннего отступа симуляция scale, dirty-rect только зоны. Focus ring флагманский под энкодер: активный accent рамка 2px, неактивные outline 1px muted_text, переход focus ease 140ms. Без focus ring — случайное тыканье, с focus — дорогой Hi-Fi.

## 4.3 HUD / Overlays — Volume Right Arc полукруг с цифровым dB (обновлено)

Триггер любое изменение громкости. Анимация slide-in справа Fast 140ms easeOutCubic, timeout 1.2-2.5с, скрытие slide-out+fade Fast 140ms. Форма: полукруглая дуга от нижнего правого до верхнего правого угла (270° сектор). Реализация дёшево: предрасчитать таблицу точек y→x_start/x_end или маску сегментов 240px, рисовать как набор горизонтальных отрезков только dirty-rect правой полосы 96px, заполнение по уровню accent vs muted. Содержимое: крупное -18.0 dB H1 шрифт обязательно читаемо издалека с ПДУ, шкала дуга 75%, тип источника MonoSmall. При MUTE danger-акцент MUTE, UNMUTE -18.0 dB 2с. Исчезает по таймауту или тапу. Цвет и glow зависят от уровня и темы.

Now Playing Toast: триггер новый NAMESET/META, появление slide-up+fade-in Normal 220ms, длительность 3-4с, скрытие fade-out Fast 140ms, карточка radius 12 surface+accent outline ♪ Artist/Station Title Track, dirty-rect карточки, если новый NAMESET пока показывается → обновить текст + перезапустить таймер. Toast notifications queue max 3: TOAST_PRESET_SAVED/TIME_SYNCED/WIFI_WEAK/LIM_ACTIVE/SR_UNSUPPORTED/IR_LEARNED/SOURCE_CHANGED/PRESET_CHANGED/BASSMONO... Позиция снизу центр. Pro-Overlay строка поверх визуализатора SR 48k | HR -6.0 | GR 2.4 | CLIP 0 | CPU 38% 5-10Hz Settings Visual Pro Overlay ON/OFF. Limiter/Clip micro-pulse красный импульс в углу 24x24 300-500ms dirty-rect только угол не мигание всего.

---
# ЧАСТЬ 5 — UI State Machine + Screens

STATE: HOME, IDLE заставка/screensaver по таймауту 15/30/60с, VISUAL_FULL fullscreen visualizer playback screensaver, OVERLAY_VOL, OVERLAY_NOWPLAY, LIST wheel picker v2, MENU, PRESETS, EQ live curve, GUIDED, ROOM, DIAG, STANDBY, SERVICE скрытый, RESCUE Display Setup Wizard. Правило: любое действие энкодер/IR/кнопка немедленно из IDLE/VISUAL_FULL на предыдущий экран. Volume HUD overlay не меняет базовый state. Toast overlay.

HOME 320x240: Статус-бар 20px [BT401/USB Audio/WEB/SD/USB-Flash][48k/16][320k][23:45][WiFi] event-driven, Preset/Room 24px Preset: Cinema Anti-AD [ROOM●], Meta Artist/Track, Indicators [CLIP●][LIM●] -18.0 dB, Mini visual 0/24px опц Off/Mini VU/Mini Spectrum Settings Visual Home Mini Visual.

IDLE: таймаут → UI_STATE_IDLE, любое действие → возврат. Доступные заставки (нагрузка): Clock Large очень низкая, Waterfall низкая hw scroll, Dual VU + Peak Hold низкая, Lissajous Vectorscope средняя, Spectrum Bars средняя. Fullscreen Visualizer: Settings Visual Fullscreen Visualizer OFF/After 10s/Always Type: Spectrum Pro/Waterfall Pro/Vectorscope Pro/Oscilloscope/Aura Ambient/Gyre/Veil/OscilloAether/Hydra Flux/Needle/Neon Thin etc. При смене трека в VISUAL_FULL Now Playing overlay 3-4с.

LIST Wheel Picker v2: центральный полная яркость Title accent outline, ±1 75% нормальный, ±2 50% MonoSmall, дальше не рисуются, индикатор N/Max + тонкая полоска 4px акцент, инерция WheelState velocity position last_tick encoder_delta*VELOCITY_FACTOR velocity*=0.85 position+=velocity focused_idx roundf, перерисовываем только полосу списка dirty-rect при изменении focused → press эффект.

STANDBY: 23:45 H1 дата Вт 06.06.26 Body статус WiFi BT401 -18dB MonoSmall RGB Standby Breathing 8с max 20% ночное 22-07.

Text Metadata политика: Парсинг Station icy-name Meta "Artist – Title" разделители " - " " — " " / ", Fit test в пикселях шрифта влезает 1 строка → целиком, не влезает → word-wrap по словам, нет пробелов → перенос по UTF-8 codepoint boundary, не влезает 2 строки → ellipsis … + Pager mode default без анимации каждые 5-7с Artist→Title→Station только смена прямоугольника dirty-rect без wipe/slide при смене трека сброс в начало pager, INFO/OK → Now Playing экран 4-6 строк. Optional Marquee Settings Visual Text Overflow Ellipsis+Pager default | Marquee 20Hz pos_x=f(time) не прибавить пиксель а вычислять из времени нет дерганья при пропуске кадра если DMA занято пропустить тик.

MAIN MENU (полная структура актуальная, без TF/U-disk):
HOME
├─ Source: yoRadio:WEB / SD / USB-Flash / BT401:Bluetooth / BT401:USB Audio (FINAL 5)
├─ Presets & Favorites (Favorites быстрый switch, All presets список с desc+ikonки цепочки, Edit preset live EQ curve)
├─ Guided Tuning мастер по шагам
├─ Room Correction (Room profile ON/OFF, Import via USB-CDC omniactl.py, Quick/Guided/Advanced, Delays L/R/Sub, Scale)
├─ Shuffle: OFF / Repeat / No Repeat / Shuffle ON/OFF Repeat OFF/ONE/ALL
├─ Visual (Idle Visual Clock/Waterfall/VU/Lissajous/Spectrum/Gyre/Veil..., Fullscreen Visualizer OFF/After 10s/Always Type 12 стилей, Show Now Playing ON/OFF, Pro Overlay ON/OFF, Home Mini Visual Off/Mini VU/Mini Spectrum, Text Overflow Ellipsis+Pager/Marquee, Idle Timeout 15/30/60)
├─ Display & Theme (Theme 8 Obsidian... Amber, Brightness Auto/Manual/Schedule, Language RU/EN)
├─ RGB Ring (Profile Standby Breathing/Clock Pulse/Ambient Slow/Source Color/Party/Off, Brightness Auto/Manual)
├─ Remote Control Learn/Test/Clear All
├─ Sleep Timer
├─ Diagnostics / Event Log Error log 100 событий SRAM/Heap AT32+ESP32-S2 DSP metrics HEADROOM/LIM_GR/CLIP_CNT/CPU load USB/SD status
└─ System Firmware AT32 version ESP32-S2 version Flash ID capacity Heap SRAM Boot Screen Logo/Logo+Shimmer/Minimal/Diagnostics Reset Settings
> Display Controller только в Service Menu!

Buttons FINAL (исправлено):
- Enc rotate громкость home / скролл список/меню
- Enc press OK/Select home=mute toggle long → MAIN MENU
- PREV short предыдущая станция/трек (если прошло >3-5с текущего → рестарт текущего) long page up в списке
- NEXT short следующая long page down, hold внутри трека перемотка с ускорением (см. Часть 0) — важно: ранее в v7.2 было hold page up/down, теперь hold = seek
- BACK назад/выход long → HOME
- SOURCE BT401↔Radio краткое, long список 5 источников
- FAV/PRESET short следующий favourite long список всех пресетов с описаниями, Audition on scroll ON/OFF default OFF
- AMP POWER вкл/выкл усилитель long → STANDBY

---
# ЧАСТЬ 6 — Fullscreen Visualizer Pack (почему ты спросил про визуал заторы — здесь полный список, ничего не упущено)

Все стили Quality Scaling High/Med/Low, theme colormap, motion tokens, 20-30Hz target, dirty-rect, RGB кольцо опционально, виджет на HOME.

**Базовые must-have из v7.2 сохранены:**
- Spectrum Pro: вход SPEC:16/32 uint8 0..255 лог, бары лог X линей Y, сглаженная линия вершин, заливка градиентом 2-3 уровня без альфы accent→bg, Peak dots медленный спад persistence 1-3с, Adaptive palette low теплее accent high холоднее, обновление 20-30 fullscreen 10-20 mini, ballistics attack быстрый release медленный bar_val+= (new-bar)*0.8 если new>val else 0.15.
- Waterfall Pro: ключ hw вертикальный скролл ST7789/ILI9341 VSCRDEF 0x33 + VSCSAD 0x37 scroll_ptr=(scroll_ptr+LINE_H)%SCROLL_AREA_H VSCSAD(scroll_ptr) → аппаратный сдвиг без перерисовки! Рисуем новую строку спектра в tile_dma colormap spec_colormap[8] темы + тонкий current trace 2px поверх. ESP32-S2 бонус PSRAM история до 240 строк 320x240x1byte ~75KB scrub энкодером назад/вперед.
- Vectorscope Pro / Lissajous: вход SCOPE_POINTS:N:x0,y0,... downsample L/R, сетка muted_text, фигура line strip/точки, persistence 0.5-1с PSRAM буфер fade -1 brightness каждый кадр, корреляция -1..+1 шкала.
- Oscilloscope Stereo: SCOPE_POINTS или OSC пакет, триггер по нулю, режимы L/R/Mono/Side L-R.
- Aura Ambient медленный 5-10Hz фон gradient sweep 2-3 цвета lerp к target_hue из centroid loud→brightness поверх мягкие размытые облака из бинов.

**Новые флагманы из обсуждений (ссылки Pinterest):**

### 6.1 AURA Gyre — вращающийся радиальный спектроанализатор https://pin.it/6rexP5pTg
- Полярные координаты bin→угол+длина амплитуда, плавное вращение phase-offset по времени + beat/centroid, сглаженные бары/лучи + peak hold + лёгкий trail PSRAM persistence buffer fade, colormap тема adaptive low теплее, precomputed sin/cos table, dirty-rect кольца/сектора, 16-32 бина. Quality High 32 bins full trail Med 16 short Low 16 no trail. Пример: круговой/радиальный спектр бары или лучи из центра часто с вращением glow particles soft trails → must-have Visual Pack.

### 6.2 AURA Veil — ethereal aura fluid glow https://pin.it/KJa0Y3f8r
- Мягкое свечение fluid-like облака organic glow, вход SPEC 16/32 + FEAT:CENTROID (0..255 холоднее) + FEAT:LOUD яркость, multi-layer soft blobs полупрозрачные эллипсы радиальные falloff по бинам vignette shimmer, без real Gaussian blur аппроксимация concentric rings falloff pre-baked glow sprites flash/PSRAM multi-pass dimming, 5-15Hz низкая нагрузка, идеально как ambient эволюция Aura Ambient.

### 6.3 OscilloAether — гибрид осциллограф+спектр https://pin.it/46QMjiLsa
- Вход SPEC + SCOPE points, варианты Split верх scope низ spectrum, Overlay спектр как толщина/цвет волны, Classic spectrogram-oscillo, 20-30Hz лёгкий. Рекомендуем как OscilloAether.

### 6.4 Hydra Flux — восходящие частицы-головастики с хвостами, хаос от FFT (уточнённая концепция "гидра" = хаотично по траектории вверх движущиеся частицы и от траектории зависит хвост) https://pin.it/5i7yFQSJG
- **Было недопонимание:** не ветвящееся дерево/кораллы, а частичная система tadpole-comets: частицы преимущественно вверх летят, скорость разброс плотность живость зависят от FFT/энергии/атаки/centroid/басов, за ними трейлы повторяют путь, постоянный поток whiptail.
- Технический бриф:
```c
#define MAX_PARTICLES_HIGH 180
#define MAX_PARTICLES_MED 100
#define MAX_PARTICLES_LOW 50
#define TRAIL_LEN_HIGH 14
#define TRAIL_LEN_MED 10
#define TRAIL_LEN_LOW 7
#define SPAWN_ZONE_Y 220
typedef struct {
 float x,y,vx,vy,chaos,life; uint8_t bright; uint16_t color;
 float trail_x[14],trail_y[14]; uint8_t trail_len;
} Particle;
```
- Spawn внизу, rate от energy/bass/attack/flux, Update Euler + силы bins lift от bass chaos от centroid/flux скорость loudness разброс high bins, Trail ring-buffer 8-20 точек polyline fade голова яркая хвост dim, Render голова капля/круг + 1-2 glow layer двойная линия dim outer хвост polyline alpha falloff, Memory 20-40KB PSRAM, Mapping low→lift/thickness mid→chaos high→sparkle/velocity energy→spawn_rate centroid→hue flux/attack→burst, Particle pool 80-250, dirty-rect only активных, Quality Scaling режет число/trail/glow. Красиво дорого смотрится.

### 6.5 AURA Needle — аналоговый стрелочный индикатор VU/PPM (быстрая победа)
- Фон шкала корпус стекло риски подпись dB/VU/% реалистичная статичная картинка asset RGB565 или сжатый в Flash/LittleFS (или external Flash) несколько вариантов под темы Obsidian dark metal Ivory vintage Studio 200-280px.
- Стрелка программно каждый кадр или при изменении: тонкая линия/полигон треугольник+хвостовик + anti-aliasing 1-2px + dim outer glow, pivot ось угол=f(level) ballistics attack/release обязательны VU 300ms PPM 10ms attack 1.5s release dual peak опционально, dirty-rect крошечный область стрелки. Классика VU-метров.

### 6.6 Neon Thin Spectrum — очень тонкие яркие неон-бары как в старых аудио устройствах https://pin.it/7Drdm2Lkr
- Бары 1-2px промежуток 1px, peak dots, высокий контраст темный фон glow двойная линия dim outer, выглядит дорого. Варианты: Vintage Thin Bars, LED Dot Matrix столбик из точек как светодиодные индикаторы, Outline Only только линия без заливки, Mirror Spectrum симметрия от центра 320x240. Нагрузка почти та же что обычные bars. Референс тонкие полоски как в старых аудио устройствах.

### 6.7 Остальные стили (чтобы закрыть "визуал заторы" — ты спрашивал почему упустил)
- **Vintage LED Matrix / LED Dot Matrix:** пиксельная/матричная эстетика как старые спектральные анализаторы, столбик из точек.
- **Classic Bars:** толстые/тонкие mirrored peak-hold.
- **Waterfall / Spectrogram** уже был hw scroll.
- **Particles / Energy Field:** более абстрактный рой (расширение Hydra Flux).
- **Waveform Landscape + reflection:** осциллограф с отражением.
- **Minimal HUD:** только важные meters + time + source.
- Дополнительно v7.3/v7.4 опционально: Radial Waterfall, Frequency Rings, Beat Reactive Bloom, Mirror Spectrum, Outline Only.

Каждый стиль имеет 3 уровня Quality, привязку к RGB кольцу опционально, возможность быть виджетом HOME.

**Проверка Visual Pack:** Quality без краша, 20-30 FPS High 15-20 Med 10-15 Low нет flicker/tearing, Theme colormap lerp 300-500ms, PSRAM usage framebuffer 150KB + history + particles + glyph cache <2MB, Hydra Flux particles не накапливаются trail fade spawn_rate соответствует энергии, Needle стрелка плавная ballistics как аналоговый.

## 6.8 Boot Splash + Soft Vignette
- Boot Screen: Logo static, Logo+shimmer блик полоска 40px +30% яркости движется по X 1.5с easeInOutCubic premиальный, Minimal OMNIA STHENOS, Diagnostics boot версии heap flash ID.
- Ассеты LittleFS internal 4MB или external SPI Flash assets.
- Soft Vignette в fullscreen visualizer: не alpha-blend а 2-3 уровня затемнения по краям top/bottom 20px полоса surface 50% bg left/right 16px статичная рамка не анимируется не тратит CPU.

---
# ЧАСТЬ 7 — DSP Architecture FINAL — Dual PEQ (ответ на вопрос про два peq)

## 7.1 Итоговое решение Вариант B (к чему пришли в обсуждениях)

**Было проблема:** если Room Correction занимает 12-18 полос из 20, то на пресетные эффекты остаётся 2-8 полос → толку от эффектов ноль, цепочка ДСП не тянет всё одновременно (правило "все эффекты не будут работать одновременно").

**Решение Dual PEQ (утверждено):**
- **RoomPEQ:** до 16 полос глобальный слой коррекции комнаты, всегда поверх любого пресета, хранит type Fc Q gain_db enabled, Delay L/R/Sub 0-30ms step 0.1 + Trims ±6dB + Scale 0-100% (0=bypass 100=полная).
- **CreativePEQ (Preset/User PEQ):** 12 полос полностью в распоряжении пресетов и ручного творчества, типы PK/LS/HS/HP/BP/Notch Fc log 20-20k Q 0.2-10 gain ±12..18dB safety clamp, macro Tilt/Presence/Air/Punch.
- **Tone:** отдельный очень лёгкий блок 2 biquad low-shelf + high-shelf после/до PEQ, CPU копейки, чтобы не палить полосы PEQ (раньше был HPF/BPF Tone регулятор — теперь это Tone separate).
- **Room всегда перед creative эффектами**, чтобы Room коррекция не съедала эффекты, а эффекты слышны: Preset = характер/сцена, Room = компенсация помещения/расстановки.

**Финальный signal flow (жёстко зафиксирован):**
```
Input I2S domain
→ DC Block / Subsonic HPF защита всегда ON
→ ROOM STAGE global если room_enable:
      RoomPEQ ≤16 + Delays L/R/Sub + Trims ±6dB + Scale 0-100%
→ CREATIVE STAGE preset/user:
      CreativePEQ 12 + Tone Bass/Treble separate 2 biquad + Loudness + AETHER WIDE/CINEMA + BassMono + Psychobass + Night + BassTame + Anti-Ad
→ Headroom / Volume / Soft-Mute GainSmoother soft unmute 400ms default raised-cosine
→ Look-ahead Limiter always last ceiling -1dBFS 5-10ms
→ Output + Meters/SPEC/SCOPE/PROGRESS
```
Room Stage перед safety-limiter чтобы пики от коррекции ловились лимитером.

**Преимущества Dual PEQ:**
- CPU real-time не растёт сильно (все те же biquads но разделены, 16+12=28 vs 20 раньше, но управляемо через Load Profiles Lite/Music/Cinema).
- Пользовательский PEQ остаётся мощным, Room можно A/B ON/OFF, несколько профилей до 5.
- Запас на будущее: Room Scale 0-100% + возможность пресету давать мягкий tilt/offset на Room Advanced.
- Load Profiles автоматически снижают количество активных полос при нехватке CPU сначала Creative потом Room: Lite Room 8 max Creative 6, Music Room 12 Creative 12, Cinema Room 16 Creative 12 + AETHER WIDE без ambience vs CINEMA ambience conditional.

## 7.2 RoomProfile data model FINAL

```c
typedef struct {
 char name[32];
 uint8_t enable; // global
 uint8_t rpeq_bands; // до 16
 struct { uint8_t type; float fc,q,gain_db; uint8_t enabled; } rpeq[16];
 float delay_l_ms, delay_r_ms, delay_sub_ms; // 0-30.0 step 0.1
 float trim_l_db, trim_r_db; // ±6dB
 float scale_percent; // 0-100%
 uint8_t version; uint32_t crc;
} RoomProfile;
```

**Импорт REW/RAW расширен:** стандартные REW строки Filter 1 ON PK Fc 55.0 Hz Gain -3.5 dB Q 4.20 + наши Delay L: 0.3 Delay R: 0.0 Delay Sub: 2.5 Trim L: -0.5 или команды RDELAY L 0.3. USB-CDC протокол ROOM.NEW FS ROOM.EN RPEQ RDELAY ROOM.SAVE slot ROOM.APPLY → OK/ERR. rew_parser.py + omniactl.py import-rew учатся парсить Delay.

## 7.3 Room Studio живой 3-уровневый UX (было скупой плоский диалог — теперь живой)

**Level 1 Quick Room:** Room Correction ON/OFF большой switch, Current Profile [Sofa/Desk/Living/Custom], A/B Compare Original↔Corrected мгновенно visual feedback, Import REW USB-CDC, Quick delay trims, Room Scale 0-100%.

**Level 2 Guided Room Tuning (без микрофона умный диалог):** Low-end focus warble tones 40/50/63/80/100/125 Hz -18dBFS генерируются AT32, UI "Где сильнее гул/boom?" → выбор → система предлагает cut Q depth, Null finder типовые комнатные моды, Target curve preview Flat/House/Night, A/B каждом шаге, "Применить рекомендованные 3-5 фильтров в room slots".

**Level 3 Advanced Room:** полный список до 16 room-фильтров lock-иконка, Delay L/R/Sub 0-30ms step 0.1 + phase invert опционально, Trims ±6dB, Target curve + correction range 20-200Hz default 20-500 full, Export/Import profile, Slot ownership visual locked by Room, Undo/History 5 действий.

## 7.4 AETHER Engine ЭФИР — состав и регулировки (куда делись HPF/BPF/стерео расширитель/объёмный звук — не исчезли, а объединены в ЭФИР)

AETHER составной движок не монолит (маркетинг "ЭФИР"):
1. M/S Encoder↔Decoder
2. HF-only Widening Side×width низ защищён Bass protect Fc 120..200Hz Width 0.8..1.6 Low/Med/High
3. Decorrelator 2-4 all-pass или micro-Haas Decor 0..100%
4. Ambience Schroeder-lite только CINEMA wet очень маленький 0..15% hard limit 30% Advanced
5. Dry/Wet crossfade click-free GainSmoother

Режимы OFF/WIDE для музыки/CINEMA для кино (+ambience) / HEADPHONES TBD v2. UI обычный Mode+Width ступени Advanced все числом + Bass protect Fc. Quality Scaling ambience→0 первым. Позиция в pipeline после тональных PEQ блоков до компрессоров. Реализация: M/S encode M=(L+R)*0.5 S=(L-R)*0.5 HPF на S ветке Bass protect, Width S_protected*width + S*(1-width), decode L_out M+S_wide R_out M-S_wide, Decorrelator 2-4 all-pass на S ветке с весом decor, Ambience 2 comb 2048/1536 + 1 all-pass на Side с весом ambience очень малая доля S_final S_decor+ambience, Dry/wet crossfade L_out L_dry*gs_dry.current + L_wet*gs_wet.current.

DSP Load Profiles: Lite PEQ20+Tone+Limiter+Meters+SPEC, Music + AETHER WIDE без ambience + Loudness + BassMono, Cinema + Anti-Ad + Night + BassTame + AETHER CINEMA ambience conditional. UI уважает allow_mask пресета + Load Profile, запрещённые → EVT:Denied + toast.

**Остальные DSP блоки (чтобы тянуло естественно):**
- DC Blocker R=1-(2π*0.5/Fs) y=x - x1 + R*y1 всегда ON
- Biquad DF2T b0 b1 b2 a1 a2 s1 s2 y=b0*x+s1 s1=b1*x - a1*y + s2 s2=b2*x - a2*y, PEQ LS HS формулы.
- Headroom Manager: auto_preamp = -(max_boost+1dB) компенсация сканируем gains всех полос + Tone bass/treble, применяем через GainSmoother.
- Mute Subsystem mute_user от пользователя + mute_system от системы SRC switch ошибка standby итоговый любой=молчание, команды MUTE UNMUTE TOGGLE.
- Anti-Ad downward-only без boost, Night mode compression, BassTame dynamic bass, Loudness, BassMono низ в моно tighter image, Psychobass очень аккуратные субгармоники вау без грязи, Delay L/R/Sub 0-30ms дешево, Look-ahead limiter 5-10ms ceiling -1dBFS always last safety.

## 7.5 Preset Library ≥32 фабричных

Структура Preset: name[32] desc[96] tags битмаска MUSIC/CINEMA/NIGHT/VOICE/BASS/ROOM/PARTY/DESK/HEADPHONE, chain_mask включен по умолчанию, allow_mask какие можно дополнительно включать/регулировать (чтобы не тянуть всё одновременно: пользователь регулирует параметры уже включенных в пределах allow, может включать дополнительные только из allow_mask через crossfade, порядок цепочки менять нельзя, Room всегда глобальный поверх), version crc.

**Пример 32 фабричных с desc для чего (чтобы учесть все ситуации и предпочтения):** Music 1-10 Neutral Warm Bright Detail Vocal Focus Wide Tight Bass Low Volume Party Safe Rock Punch Jazz Smooth, Cinema/TV 11-18 Reference Anti-Ad Night Dialog+ AETHER Bass Control Explosive News/Speech, Room/Speakers/Desk 19-24 Small Big/Living Desk/Nearfield Sofa Late Night Bookshelf Friendly Sub+Satellites, Bass/Special/Fix 25-32 Bass Boost Bass Mono Psychobass Light Harsh Treble Fix Sibilance Fix Podcast/Voice Gaming/Immersive Test/Flat -18dBFS. Каждый desc 70-90 символов, в UI при переборе показывать desc + иконки цепочки бейджи PEQ/Tone/Loud/Room/Ae, при пролистывании не применять сразу а по OK опция Audition on scroll ON/OFF default OFF.

**DSP Chain View экран:** порядок Input→DC/Subsonic→RoomPEQ+Delays→CreativePEQ→Tone→Loudness→AETHER→BassMono→Psychobass→Night→BassTame→Anti-Ad→Headroom→Volume/Mute→Limiter Output, схематично Music:Wide [Apply][Edit][A/B] с desc Tags Chain DC→Sub→[RoomPEQ 3]→[CreativePEQ]→Tone→Loud→[AETHER WIDE]… →Limiter, список [RoomPEQ 3 bands global Room:Sofa ON] [CreativePEQ ON 4 bands ←OK edit] [Tone Bass+1.5 Treble+0.5] [AETHER WIDE Width 1.35 Decor 40%] [Night OFF [+] Add allowed] [Anti-Ad OFF not allowed] … Room ON Delays L0.0 R0.2 Sub1.5ms.

## 7.6 PEQ Editor flagship на маленьком экране

График 20-20k лог X live curve 30-40 точек дешево, подсветка текущей полосы fc, внизу Band 7 250Hz -2.5dB Q1.2 MonoSmall. Управление Rotate Gain ±0.5dB, Press переключить Gain→Fc→Q, Long ON/OFF, BACK выйти только тогда сохранить NVS, PREV/NEXT или Select band mode rotate выбирает index. Почему флагман: видит кривую а не таблицу, работает на 320x240. Режимы Overlay curve+spectrum Before/After Solo band, макросы Tilt Presence Air Punch влияют на free slots или deltas.

## 7.7 Source Picker интересный + Favorites

Source Picker карусель с крупными иконками 5 sources WEB/SD/USB-Flash/BT/USB Audio + статус playing/rate/signal описание Bluetooth from phone etc справа/снизу, при фокусе предпросмотр не переключаем домен сразу OK применяет SRC:* + MODE/BT401_MODE + toast BT401: USB Audio 2с. FAV/PRESET short следующий favourite long Favorites Browser список избранных с desc. Source Color RGB кольцо.

---
# ЧАСТЬ 8 — Остальное (IR, TFT, Service, Non-Goals, AMP, Protocol, Testing)

## 8.1 IR пульт self-learn
TSOP4838/VS1838B 38kHz LOW GPIO5 RMT RX предпочтительно питание 3.3/5В pull-up 10k. RMT 1MHz resolution 128 symbols idle_threshold 20ms конец кадра. Нормализация найти медиану коротких квант разделить длительности квантовать допуск ±25% fingerprint CRC32. Самообучение Settings Remote Learn выбрать действие – Нажми кнопку 3 раза AURA toast счетчик 1/3..2/3..3/3 захватить 3 кадра вычислить fingerprint если 2/3 совпадают сохранить NVS toast IR learned OK else не распознано. Runtime матч точный CRC → действие, Σ|dt-ref|/ref<0.25 → найдено else игнор. Hold Repeat кадры <200ms похожи → Hold hold_count 0-3 однократно debounce 4-10 авто-повтор VOL 0.5dB 150ms 11+ ускорение 1.0dB 100ms. Маппинг IR→UART VOL VOL:NNN dBx10 MUTE TOGGLE NEXT/PREV PLAY/PAUSE TOGGLE SOURCE SRC:...+BT401_MODE/MODE AMP TOGGLE MENU BACK OK FAV PRESET:LOAD slot=N. Хранение IR_Binding action mode fingerprint dt_norm[64] dt_count valid ~12*140≈1.7KB NVS.

## 8.2 TFT universal driver
Единый API tft_init/set_window/push_pixels_rgb565/set_rotation/invert/scroll_define/scroll_start/backlight_set. SPI DMA via esp_lcd or spi_device_queue_trans MOSI 35 MISO 37 SCK 36 CS 21 DC 17 RST 18 BL 16 clock 40MHz max_transfer 320*16*2 queue 7 pre_cb DC. SPI Clock Test авто-понижение 80→40→20MHz нарисовать тест-паттерн полосы прочитать если MISO подключен ошибки понизить сохранить NVS spi_freq_hz. Waterfall hw scroll ST7789 VSCRDEF 0x33 VSCSAD 0x37 ILI9341 VSCRDEF VSCRSADD scroll_ptr=(+LINE_H)%SCROLL_AREA_H tft_scroll_start(scroll_ptr) waterfall_render_line в tile_dma colormap tile_dma DMA push только новой строки. TE pin optional TEON 0x35 GPIO interrupt TE → синхронизация DMA push VSync устраняет tearing.

## 8.3 Display Wizard / Rescue / Service Menu
Wizard первый запуск/NVS пустой: RST дисплея init как ST7789 нарисовать тест ST7789? цветные полосы RGB синий ждать 3с или ENC_BTN, RST init как ILI9341 тест ILI9341? RGB зеленый ждать 3с/ENC, ENC press подтвердить текущий сохранить NVS reboot BACK следующий немедленно цикл пока не подтверждено. Rescue вслепую без экрана: при старте удержать ENC_BTN 5с войти Rescue тот же цикл RGB синий ST7789 зеленый ILI9341 подтверждение 3 вспышки белым save NVS reboot РАБОТАЕТ ДАЖЕ ЕСЛИ ЭКРАН НИЧЕГО НЕ ПОКАЗЫВАЕТ. Service Menu BACK+ENC_BTN 5с: Display Controller ST7789/ILI9341/SPI Clock Test, Flash JEDEC ID hex SFDP info page/sector/capacity LittleFS used/free, I2S test tone синус AT32, RGB test red/green/blue/white/rainbow/off, IR raw capture raw RMT тайминги fingerprint CRC32, UART tests v·core ping INFO.REQ loopback, Sensor test BH1750/VEML7700 lux gain, USB/SD status mounted/ejected/error size fs free files, Factory Reset Clear NVS/LittleFS/IR, Diagnostics Event Log 100 ring SRAM/Heap AT32+ESP32 DSP metrics HEADROOM/LIM_GR/CLIP_CNT/CPU load Visual Load Meter, Boot Screen selection Logo/Logo+Shimmer/Minimal/Diagnostics. Time Manager NTP via yoRadio UART always active даже в BT401 домене RSSI.

## 8.4 Non-Goals FINAL (чтобы документ не расползался)
- Нет Dolby Atmos/Dolby Digital лицензируемого — вместо AETHER ЭФИР WIDE стерео расширение CINEMA псевдо-surround
- Нет ресемплера только 44.1/48 (BT_RESYNC+RATE), если MCLK не 11.2896/12.288 → ERR:0x0602 mute hold Unsupported
- Нет PCM стриминга по UART на UI — только SPEC:16/32 SCOPE FEAT DIAG PROGRESS META
- FFT_FULL только диагностика/service, SPEC основной визуальный протокол
- Нет микрофона auto-EQ реального времени — Room REW-import Guided manual
- Нет тяжелых chorus/flanger/reverb full / multiband mastering compressor в базовой цепи
- WiFi Web UI/app на S2 secondary/optional primary via yoRadio

## 8.5 AMP / Sequencing + FAULT explicit отказ
FAULT/FAULTZ pin усилителя НЕ используется в логике чтобы усилитель заменяемый разные vendor полярность поведение разное, защиты по аудио-метрикам clip peak RMS, limiter, реле/mute/soft-start, температурные/токовые датчики отдельно. Правило жёсткое.

## 8.6 Protocol FINAL 5 sources (всё необходимое UARTы, режимы включая USB-Flash)

**AT32→S2:** SRC:RADIO/BT401, MODE:WEB/SD/USB (USB=USB-Flash FINAL), BT401_MODE:BT/USB_AUDIO TF/UDISK DISABLED, VOL dBx10 MUTE ON/OFF/TOGGLE GAIN HEADROOM LIM_GR CLIP SR 44100/48000 CH STEREO/MONO PRESET LOAD slot=N SAVE AETHER OFF/WIDE/CINEMA WIDTH DECOR AMBIENCE BASS_PROTECT ANTIAD/NIGHT/BASSTAME/LOUDNESS/BASSMONO/PSYCHOBASS DELAY L/R/Sub ms METERS SPEC:16/32 FEAT:CENTROID/LOUD/FLUX/ATTACK/BASS SCOPE_POINTS DIAG EVT:Denied ERR INFO.REQ/RSP SUBSCRIBE:SPEC/SCOPE/METERS/DIAG/PROGRESS/META ON/OFF rate.

**YoRadio→AT32/S2:** старые ##CLI.META# ##AUDIO.INFO# ##CLI.NAMESET# ##CLI.VOL# ##CLI.PLAYING# ##SYS.DATE# #CLI.LISTNUM# #WIFI.STATUS# #WIFI.RSSI# ##ERROR# + новые ##PROGRESS# cur_ms dur_ms state 0stop1play2pause3seeking4buffering5error percent_x10 0..1000 idx total 2-5Hz до 10Hz seek, ##META# title=artist=album=year=bitrate=fmt=sr=ch, ##TRACK# idx/total/path/playlist, ##SD.STATUS# mounted/ejected/indexing/error size fs files, ##USB.STATUS# mounted/ejected/error, ##AUDIO.ID3#.

**AT32/S2→YoRadio:** старые mode 0/1/2 prev/next/toggle/stop/start/play/vol/vol+/vol-/vol x audioinfo smartstart list play x info dspon dim sleep tzo date version heap boot reset wifi.list/con/station/con/ssid status rssi discon... + новые seek / seek_rel / seek_percent / sdpos / seek_start +/- / seek_stop / shuffle on/off / repeat off/one/all / shuffle_repeat / random / usb_scan/list/play / mode usb / status / ping.

**YoRadio I2S + RATE/MUTE:** ESP GPIO17→PB4 DATA, PB3→16 BCLK, PA15→18 LRCK, GPIO2→PA8 MUTE, GPIO4→PA9 RATE LOW44.1 HIGH48, GND общий, syncRateToStmPins() каждые 100ms mute→RATE→120ms→unmute, I2S_ROLE_SLAVE в Audio.cpp.

**Проверка протокола:** Rate limits SPEC 20-30Hz PROGRESS 2-5Hz (10Hz seek) META on-change, бинарный framing high-rate [0xAA msg_id len seq payload CRC], текст \r\n max256 UTF-8, AT32 валидирует seq/timestamp, потеря SPEC не крашит UI, PROGRESS без duration WEB не ломает шкалу.

## 8.7 Testing & Checklists после каждого этапа (обязательно)

**Этап 0 Bring-up S2 mini + pinmap EXT_FLASH Any Vendor + TFT + UART + USB-CDC:** прошивка собирается бут без panic USB-CDC логи, TFT wizard color bars SPI 40MHz test, Encoder buttons ADC IR RMT RGB ring red/green/blue I2C lux, UART 7/8 460800 ping/pong, Ext Flash с чипом и без оба бут JEDEC читается LittleFS mount чтения ассетов, heap PSRAM framebuffer 150KB.

**Этап 1 YoRadio prep standalone STM32F401 стенд:** V7G рубеж скорость норм дропы исчезли, I2S handshake MUTE/RATE при Fs смене, SD без WiFi играет, Shuffle bug fixed 10 треков без пропуска, Progress+Seek PROGRESS 2-5Hz seek без клика hold ускорение, USB-Flash mount/list/play/seek/meta/hot-plug 1ч без leak, Web UI progress/seek/USB, CLI совместимость, Heap >90KB free после 1ч +100 seek.

**Этап 2 BT401 domain:** MCLK vendor confirm docs/vendor_bt401_mclk_confirm.md, MCLK 11.2896/12.288 метод A input capture + B gate window 10-50ms, Fs change 44.1↔48 BT_RESYNC click-free mute HIGH fade-out stop DMA deinit recalc coeff init slave 16-bit 32-slot Philips start DMA fade-in LOW, BT↔USB Audio без полного domain switch Fs watch работает, Unsupported SR ERR:0x0602 mute hold UI Unsupported, RADIO↔BT401 полный safe switch.

**Этап 3 UI Performance + AURA + Storage:** dirty-rect только изменённые no full redraw, SPI DMA-only no blocking, FreeRTOS prio input вытесняет render мгновенная реакция, NVS dirty+timeout+standby LittleFS assets Ext Flash optional fallback, Theme 8 transition 300-500ms lerp motion tokens Fast/Normal/Slow cubic easing focus ring press effect, Volume Right Arc полукруг с dB читается 3м timeout 1.2-2.5с Now Playing Toast 3-4с Pro Overlay.

**Этап 4 Visualizers (включая дыхание RGB):** Spectrum Pro Waterfall Pro hw scroll VSCRDEF/VSCSAD только новая строка DMA push Vectorscope persistence PSRAM Oscillo trigger, Gyre Veil OscilloAether Hydra Flux particle pool 50/100/180 trail 7/10/14 Needle asset+software needle ballistics Neon Thin 1-2px glow Vintage LED Matrix Classic Bars Waterfall Particles Energy Field Waveform Landscape Minimal HUD, Quality High/Med/Low 20-30 FPS High theme colormap RGB ring sync breathing, Waterfall history PSRAM 75KB scrub encoder, No flicker/tearing frame pacing stable, RGB дыхание Standby Breathing 8с exp gamma 2.0 b_max 8% ночью 3-5% sync с визуализаторами.

**Этап 5 DSP Dual PEQ:** RoomPEQ 16 + CreativePEQ 12 + Tone separate работают Room поверх любого пресета, Room Studio 3 уровня Quick Guided Advanced Delay L/R/Sub 0-30ms 0.1ms trims ±6dB Scale 0-100% REW import Delay, PEQ Editor live curve 30-40 точек Gain/Fc/Q ramping click-free A/B macros Tilt Presence Air Punch, GainSmoother везде Soft Unmute 400ms raised-cosine Soft Mute 80-150ms bypass crossfade, AETHER WIDE/CINEMA width/decor/ambience/bass_protect Quality Scaling ambience→0 first, Anti-Ad downward-only Night BassTame Loudness BassMono Psychobass Delay Limiter last -1dBFS, Load Profiles Lite/Music/Cinema allow_mask EVT:Denied toast, Safety headroom auto_preamp -(max_boost+1dB) clip micro-pulse 24x24 300-500ms.

**Этап 6 IR+RGB+Sensors+Buttons:** IR self-learn 3 нажатия 2/3 match NVS fingerprint CRC32 similarity <0.25 hold acceleration, RGB breathing 8с exp gamma 2.0 ночной лимит 10-15% lux<20 Source Color, BH1750/VEML 1/2с backlight alpha 0.05 placement не видит TFT/RGB, Buttons short NEXT/PREV трек long hold seek ускорение Enc rotate громкость short Play/Pause long Menu FAV short next fav long Favorites Browser Audition on scroll.

**Этап 7 Presets+Chain+Source Picker:** 32 фабричных desc/tags/chain_mask/allow_mask icons chain, Chain View allow add not allowed gray, Preset Detail Apply/Edit/A/B, Source Picker карусель 5 FINAL preview без domain switch OK toast, Favorites Browser long FAV recommended preset.

**Этап 8 Network Control:** Primary via yoRadio UART forwarding App↔WiFi↔YoRadio S3↔UART↔AT32↔UART↔S2, Secondary optional S2 MQTT/WebSocket no conflict MAC/IP latency <200ms, preset/room control from phone via yoRadio web/MQTT, OTA via yoRadio + optional S2.

**Этап 9 Onboarding+Backup+Diagnostics+Service:** First Boot Wizard язык тема Display Wizard pink noise -18dBFS source default visual demo Safe Volume, Backup/Restore presets NVS Room LittleFS omniactl.py Web UI, Service Menu BACK+ENC 5с display controller SPI Clock Test auto downgrade Flash JEDEC+SFDP I2S test tone RGB IR UART Sensor USB/SD status Factory Reset Diagnostics Event Log 100 ring Crash log 10 NVS Heap DSP metrics Visual Load Meter Explain strings Safety meter Help texts, Time Manager NTP via yoRadio UART always active even BT401.

**Этап 10 Full integration OMNIA:** заменить STM32 стенд на AT32 сохранять RATE/MUTE handshake поднять baud 921600, AT32 форвардит PROGRESS/META/TRACK/STATUS на S2 S2 шкала time percent, переключение между 5 FINAL sources click-free no overrun_count рост resync_count только Fs change, длительный тест 24ч WEB/SD/USB-Flash/BT/USB-Audio shuffle seek next/prev volume presets room A/B visualizers IR RGB sensor auto brightness, Stress выдернуть SD/USB во время play смена Fs внутри BT401 unsupported SR WiFi drop power cycle factory reset, финальная дока uart_protocol_v1.md yoRadio_modernization.md vendor_bt401_mclk_confirm.md CHANGELOG.

# ЧАСТЬ 9 — Insertion Checklist (что куда вставлять)

- firmware/omnia_ui_s2mini/include/pinmap_ui.h: EXT_FLASH_CS 10 MOSI 11 SCK 12 MISO 13 optional strongly recommended any vendor JEDEC + коммент, занятые 1-8 10-13 16-18 21 35-37 33-34.
- firmware/omnia_ui_s2mini/: ext_flash.c auto-detect LittleFS /assets mount fallback, visualizers gyre.c veil.c oscillo_aether.c hydra_flux.c PSRAM particle pool 50/100/180 trail 7/10/14 needle.c asset loader + ballistics VU 300ms PPM 10/1500ms neon_thin.c vintage_led_matrix.c классика, rgb_breathing.c формула exp 8с gamma 2.0.
- DSP: dsp_room_stage.c RoomPEQ 16 + delays, dsp_creative_peq.c 12 + Tone 2 biquad, dsp_aether.c M/S Wide Decor Ambience composition, smoothness gain_smoother.c Soft Unmute 400ms.
- docs/: vendor_bt401_mclk_confirm.md, uart_protocol_v1.md (5 sources FINAL), sources_matrix_v8.1.md, yoRadio_modernization.md, 10_storage_nvs.md journal if needed.
- YoRadio repo https://github.com/ChillPulse/Youradio_Omnia_ESP32S3: player.cpp syncRateToStmPins 100ms mute RATE 120ms unmute, myoptions.h STM_RATE_PIN 4 STM_MUTE_PIN 2, USB Host MSC 19/20 + 5V polyfuse 0.75A caps, audioI2S wrappers getAudioCurrentTime/getAudioFileDuration/setAudioPlayPosition/seek, CLI extensions shuffle/repeat/seek/usb status ping, bugfix shuffle skip SD without WiFi, web UI progress/seek/USB, heap monitoring.
- AT32 v·core: DSP Dual PEQ RoomPEQ 16 CreativePEQ 12 delays smooth gain AETHER composition load profiles Lite/Music/Cinema, BT Fs watch + BT_RESYNC 9 шагов, MCLK measurement A+B, RATE/MUTE handshake, UART hub queues prio baud 921600, FAULT pin not used.
- Tests: test_plans/ usb_hotplug shuffle_no_skip progress_seek bt_resync rgb_breathing dual_peq_room_scale visual_quality.

---
**Конец MASTER SPEC v8.2 CLEAN FINAL.** Это чистая версия без лишних цитат старых ТЗ, только актуальные FINAL решения. Включает всё из обсуждений: Dual PEQ Room 16 + Creative 12 + Tone, визуализаторы Gyre/Veil/OscilloAether/Hydra Flux/Needle/Neon Thin/Vintage LED Matrix (визуал заторы закрыты), RGB дыхание 8с exp gamma 2.0 Source Color, External Flash GPIO10-13 any vendor JEDEC, Network Primary via yoRadio, YoRadio путь с UARTами + 5 режимов WEB/SD/USB-Flash/BT/USB Audio, проверки после каждого этапа. Готов к вставке как docs/00_master_spec_v8.2.md + docs/SOURCES_v8.1_MATRIX.md.

АльхамдулиЛлях, двигаемся по чеклистам.

