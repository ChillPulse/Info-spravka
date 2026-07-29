# AUDIT v8.1 — Проверка полноты ТЗ и дорожная карта модернизации Ё-радио для OMNIA

**Цель этого документа:** ответить на вопрос заказчика — "мы ничего не упустили из обсуждений, оригинального ТЗ, всё стыкуется по смыслу реализации? Есть подробный путь модернизации Ё-радио (для омниа) с нашего репо, где будут все необходимые уарты, режимы включая USB flash, и прочее из обсуждений?"

**Источники для аудита:**
- MASTER SPEC v7.2 оригинал (5170 строк, 219KB) — https://github.com/ChillPulse/Info-spravka/blob/52b24e5ff66e6dc8d7f65cd75a81e20131fd56cf/MASTER%20SPEC%20v7.2
- Comments and corrections v7.2 (1210 строк, 79KB) — https://github.com/ChillPulse/Info-spravka/blob/52b24e5ff66e6dc8d7f65cd75a81e20131fd56cf/Comments%20and%20corrections%20to%20the%20Technical%20Specification%20v7.2
- 2 Comments and corrections v7.2 (2213 строк, 160KB) — https://github.com/ChillPulse/Info-spravka/blob/52b24e5ff66e6dc8d7f65cd75a81e20131fd56cf/2%20Comments%20and%20corrections%20to%20the%20Technical%20Specification%20v7.2 (сюда вошли все обсуждения про Gyre/Veil/OscilloAether/Hydra Flux/Needle/Neon Thin, External Flash GPIO10-13, Network, DSP Dual PEQ, Smoothness, Preset Library 32, Room Studio, Volume Right Arc, YoRadio roadmap)
- Info-spravka репо: UART (5KB), SD Card playback screen (14KB), Screen of station playback (24KB), Audio.cpp (Redact), BT401 FAQ PDF, Technical specification (84KB), Project resume, Project resume 2 (46KB)
- Youradio_Omnia_ESP32S3 репо: https://github.com/ChillPulse/Youradio_Omnia_ESP32S3 — уже имеет pin rate и работает, Initial commit, папки HA, examples, yoRadio/src/audioI2S, core/player.cpp, myoptions.h

## 1. Аудит: ничего не упущено? Стыковка по смыслу?

### 1.1 Что было в MASTER SPEC v7.2 (крупные блоки) и сохранено ли в v8.1?

| Блок v7.2 | Статус в v8.1 | Где находится |
|---|---|---|
| 1. Архитектура 2 МК (AT32 v·core Real-time аудио + ESP32-S2 mini UI) | ✅ Сохранено полностью | Часть 1 System Overview, оригинал 5170 строк включены целиком |
| 2. Source Matrix 6 user sources + 2 clock domains | ⚠️ Обновлено до 5 FINAL (WEB/SD/USB-Flash/BT/USB-Audio) по уточнению заказчика | CORRECTION v8.1 в начале + SOURCES_v8.1_MATRIX.md |
| 3. BT401 vendor-confirmed MCLK behavior + docs/vendor_bt401_mclk_confirm.md + MCLK нужен для Fs детекта, а не для PCM5102A | ✅ Сохранено + усилено | Часть 3-4 оригинала + проверка в чеклистах |
| 4. BT Fs НЕ фиксированная, только 44.1/48, ERR:0x0602 | ✅ Сохранено | Non-Goals + BT Fs watch |
| 5. BT Fs watch + BT_RESYNC последовательность 9 шагов click-free | ✅ Сохранено | Чеклист Этап 2 |
| 6. MCLK measurement метод A input capture + метод B gate window fallback | ✅ Сохранено | Этап 2 |
| 7. yoRadio RATE/MUTE handshake + UART всегда активен для NTP/RSSI даже в BT401 домене | ✅ Сохранено + реализовано в V7G (GPIO4 RATE + GPIO2 MUTE) | Часть 0 YoRadio prep + player.cpp syncRateToStmPins |
| 8. Legacy baseline policy legacy/info-spravka_f401_baseline_10d731a tag baseline_f401_10d731a | ✅ Сохранено | Оригинал включен |
| 9. UI-МК LOLIN/WEMOS S2 mini ESP32-S2FN4R2 4MB+2MB PSRAM 27 IO USB-C PSRAM упрощает UI RMT для WS2812/IR | ✅ Сохранено | Оригинал |
| 10. Red Flags strapping GPIO0/45/46, JTAG 39-42, SPI0/1 pins 26-32 | ✅ Сохранено | Оригинал + усилено pinmap verification |
| 11. Pinmap UI single source of truth pinmap_ui.h default mapping ENC_A 1 ENC_B 2 ENC_BTN 3 BUTTONS_ADC 4 IR_IN 5 RGB_DIN 6 VCORE_UART_TX 7 RX 8 TFT_BL 16 DC 17 RST 18 CS 21 I2C SDA 33 SCL 34 MOSI 35 SCK 36 MISO 37 | ✅ Сохранено + расширено External Flash GPIO10-13 | Pinmap FINAL 1-8,10-13,16-18,21,35-37,33-34 |
| 12. WS2812 level shifter SN74AHCT125 обязателен + электролит 470-1000uF + ночной лимит 10-15% | ✅ Сохранено |  |
| 13. Power backfeed + датчик света не видит TFT/RGB | ✅ Сохранено |  |
| 14. UI↔v·core линк UART batching subscriptions baud 460800/921600 | ✅ Сохранено + расширено YoRadio extensions PROGRESS/SEEK |  |
| 15. UI Performance Contract: PSRAM framebuffer 150KB scratch 25KB tile_dma 10KB ping-pong vis_a/b, dirty-rect, SPI 40MHz, запреты delay в UI loop, FreeRTOS задачи prio input 5 > comm 4 > render 3 > rgb 2 > sensor 1, частоты виджетов | ✅ Сохранено + усилено DMA-only rule | Часть 8 |
| 16. AURA Design System grid 4px insets 8/12/16/24 corner 12/999 stroke 1/2 Typography H1/Title/Body/MonoSmall Theme struct bg/surface/outline/text/muted/accent/accent2/ok/warn/danger/spec_colormap[8]/ring_accent_r/g/b | ✅ Сохранено |  |
| 17. Темы 8 штук Obsidian/Carbon/Aurora/Studio/Night Warm/Ivory/Emerald/Amber + transition 300-500ms lerp + motion spec Fast 140 Normal 220 Slow 420 + easing cubic + Press 80-140ms + focus ring 2px | ✅ Сохранено |  |
| 18. HUD/Overlays Volume HUD выезжающий сбоку slide-in Fast 140ms timeout 1.2-2.5с → заменен на Right Arc полукруг + dB, Now Playing Toast slide-up Normal 220ms 3-4с, Toast queue 3, Pro Overlay SR/HR/GR, Limiter micro-pulse угол 24x24 300-500ms | ✅ Сохранено + обновлено Right Arc с цифровым dB (из обсуждений) |  |
| 19. UI State Machine HOME/IDLE/VISUAL_FULL/OVERLAY_VOL/OVERLAY_NOWPLAY/LIST/MENU/PRESETS/EQ/GUIDED/ROOM/DIAG/STANDBY/SERVICE/RESCUE | ✅ Сохранено |  |
| 20. HOME экран 320x240 Status bar event-driven, Preset/Room, Meta 2 строки, Indicators CLIP/LIM/HEADROOM, Mini visual Off/VU/Spectrum | ✅ Сохранено |  |
| 21. IDLE + Fullscreen Visualizer OFF/After 10s/Always Type Spectrum/Waterfall/Vectorscope/Oscillo/Aura + Now Playing Toast поверх | ✅ Сохранено + расширен Visual Pack 12 стилей |  |
| 22. Fullscreen Visualizer Pack: Spectrum Pro bars+fill+peaks adaptive palette ballistics attack 0.8 release 0.15 20-30Hz, Waterfall Pro VSCRDEF 0x33 VSCSAD 0x37 hw scroll 1 строка DMA push, Vectorscope SCOPE_POINTS persistence 0.5-1s PSRAM, Oscilloscope trigger L/R/Mono/Side, Aura Ambient gradient sweep 5-10Hz centroid→hue loudness→brightness | ✅ Сохранено + 6 новых флагманов Gyre/Veil/OscilloAether/Hydra Flux/Needle/Neon Thin из Pinterest |  |
| 23. Boot Splash Logo static/shimmer Minimal Diagnostics boot, shimmer полоска 40px +30% 1.5с easeInOutCubic, Soft Vignette 2-3 уровня затемнения по краям | ✅ Сохранено |  |
| 24. LIST Wheel Picker v2 inertia velocity*0.85 focused 100% ±1 75% ±2 50% indicator N/Max | ✅ Сохранено |  |
| 25. STANDBY 23:45 + RGB Breathing 8с формула exp gamma 2.0 b_max 8% ночью 3-5% | ✅ Сохранено |  |
| 26. Text Metadata policy parsing Station/Meta разделители " - " " — " " / ", fit test 1 строка → word-wrap → codepoint → ellipsis + Pager 5-7с, Marquee optional 20Hz pos_x=f(time) default Ellipsis+Pager | ✅ Сохранено |  |
| 27. MAIN MENU структура Source/Presets/Guided/Room/Shuffle/Visual/Display/Theme/RGB/Remote/Sleep/Diagnostics/System + Display Controller только в Service | ✅ Сохранено + добавлены USB-Flash, Shuffle, Room Studio 3-level, Visual new types |  |
| 28. Buttons маппинг Enc rotate громкость / скролл, press OK home=mute toggle, long MENU, PREV/NEXT короткое станция/трек long page up/down, BACK, SOURCE BT↔Radio, FAV/PRESET | ⚠️ Обновлено: PREV/NEXT short = трек, hold = seek внутри с ускорением (исправлено по замечанию), Encoder rotate громкость, short Play/Pause, long Source/Seek |  |
| 29. RGB Ring Profiles Source Color BT синий WEB зеленый SD оранжевый USB-Flash (добавлено), Standby Breathing, Clock Pulse, Ambient Slow, Party, Off + формула exp | ✅ Сохранено + добавлен USB-Flash цвет |  |
| 30. Storage карта Themes/fonts/icons/boot LittleFS, Presets NVS, Room LittleFS, Last state NVS journal, Fav NVS, IR NVS 1.7KB, Language NVS, TFT ctrl NVS, AURA theme NVS, etc, ESP32 NVS namespaces, dirty+timeout+standby, LittleFS /assets/fonts/icons/themes/boot/lang, binary assets + Glyph LRU 32-64 PSRAM | ✅ Сохранено + уточнено External Flash optional strongly recommended, any vendor JEDEC, fallback |  |
| 31. Universal SPI Flash JEDEC+SFDP, resource 100K cycles | ✅ Сохранено + any vendor |  |
| 32. IR self-learn TSOP4838/VS1838B GPIO5 RMT RX 1MHz 128 symbols idle 20ms normalization median tolerance ±25% fingerprint CRC32 3 нажатия 2/3 match → NVS, runtime similarity Σ|dt-ref|/ref<0.25, hold_count 0-3 debounce 4-10 vol 0.5dB 150ms 11+ 1.0dB 100ms | ✅ Сохранено |  |
| 33. Localization RU/EN UTF-8 + шрифты bitmap ASCII+кириллица Ёё glyph LRU cache | ✅ Сохранено |  |
| 34. TFT universal driver tft_init/set_window/push_pixels/set_rotation/invert/scroll_define/scroll_start/backlight_set, SPI DMA 40MHz (80→40→20 тест), Waterfall hw scroll, TE pin optional TEON 0x35 | ✅ Сохранено |  |
| 35. Display Setup Wizard/Rescue/Service Menu BACK+ENC 5с, Wizard cycle ST7789? синий ring 3с и ILI9341? зеленый, Rescue удержанием ENC 5с вслепую, Service Menu Flash JEDEC+SFD LittleFS I2S test tone RGB test IR raw UART tests Sensor BH1750/VEML Factory Reset | ✅ Сохранено |  |
| 36. Time Manager NTP via yoRadio UART always active | ✅ Сохранено |  |
| 37. DSP Feature gating quality scaling safety limiter last -1dBFS 5-10ms, Mute subsystem user+system, Gain Smoother TAU_VOL 80 TAU_MUTE_IN 200 TAU_MUTE_OUT 350 TAU_SRC 20 1-pole IIR, Master Volume dBx10 -80..0 step 0.5 db_to_linear | ✅ Сохранено + Soft Unmute 400ms raised-cosine |  |
| 38. AETHER Engine M/S Widening HF-only bass protect 120-200 Width 0.8-1.6 Decorrelator 2-4 all-pass 0-1 Ambience Schroeder-lite 2 comb+1 all-pass 0-0.15 dry/wet crossfade GainSmoother | ✅ Сохранено + composition зафиксировано |  |
| 39. Anti-Ad downward-only, Night, BassTame, Loudness, BassMono, Psychobass, Delay, Look-ahead limiter, Profiler DWT | ✅ Сохранено |  |
| 40. UART subscriptions SUBSCRIBE:* batching rate limits higher baud | ✅ Сохранено |  |
| 41. Полный протокол SRC/MODE/BT401_MODE/AETHER/ANTIAD/NIGHT/BASSTAME/DELAY/SUBSCRIBE/METERS/SPEC/FEAT/SCOPE/DIAG | ✅ Сохранено + YoRadio extensions PROGRESS/SEEK/META/TRACK/USB/SD |  |

**Итого проверка оригинала:** все крупные блоки сохранены, где было 6 источников — обновлено до 5 FINAL по уточнению заказчика (логично, TF/U-disk не нужны).

### 1.2 Что было в комментариях / обсуждениях (2213 строк) и добавлено ли?

| Обсуждение | Статус | Где в v8.1 |
|---|---|---|
| Спектроанализаторы Pinterest 4 шт + Neon Thin: вращающийся, аура, осциллоспектр, гидра частицы с хвостами (головастики), тонкие полоски как в старых аудио | ✅ Добавлено как Visual Pack AURA Gyre/Veil/OscilloAether/Hydra Flux/Needle/Neon Thin с тех брифами, структурами Particle, trail, ballistics, pin ссылки | Раздел 13 Visualizer Pack |
| Названия в стиле OMNIA/AURA: Gyre, Veil, OscilloAether, Hydra Flux, Needle, Neon Thin | ✅ Зафиксированы как топ-набор |  |
| Аналоговый стрелочный индикатор: реалистичная картинка фон asset + стрелка программно | ✅ AURA Needle с asset+needle render+ballistics |  |
| Полное управление через приложение по WiFi: конфликт S2 vs S3? | ✅ Network Control Plane Primary via yoRadio (S3) UART forwarding, Secondary optional S2 MQTT/WebSocket, App ↔ WiFi ↔ YoRadio S3 ↔ UART ↔ AT32 ↔ UART ↔ S2, latency <200ms, no conflict different MAC/IP | Раздел 15 |
| Внешняя 8-ногая Flash W25Q32/64/128 любая марка не только Winbond, нужны ли пины? | ✅ Pinmap FINAL GPIO10 CS 11 MOSI 12 SCK 13 MISO, отдельный SPI host, JEDEC+SFDP any vendor GD25Q/MX25/ISSI/XMC/BOYA, footprint SOP-8, fallback | Раздел 1.2 |
| UART-ы: один для AT32 другой для диагностики (USB-CDC) + отдельный UART диагностики AT32 | ✅ Подтверждено: S2↔AT32 GPIO7/8 единственный выделенный, диагностика S2 = USB-CDC, AT32 = свой UART, проверено по pinmap |  |
| AMP FAULT pin — explicit отказ | ✅ Добавлено | 1.1 |
| Non-Goals: нет Dolby Atmos, нет ресемплера, нет PCM по UART, FFT_FULL только диагностика | ✅ Добавлено v8.0 | Non-Goals |
| NVS Journal формат + compaction <20% vs ESP-NVS migration | ✅ Storage migration note | Раздел 3 |
| SPI DMA — единственный способ отправки пикселей | ✅ UI Performance Contract DMA-only rule | Раздел 4 |
| Pinmap verification bring-up тест | ✅ Добавлено | 1.3 |
| Пресетов мало, нужно ≥20 лучше 32+, desc/tags/chain_mask/allow_mask, живой Room Studio, Chain View, PEQ live curve, Load Profiles, AETHER composition, Volume Right Arc полукруг с dB, Source Picker интересный, Favorites Browser, Onboarding, Backup/Restore, Diagnostics расширения, Smoothness аудио+визуал Soft Mute/Unmute 400ms | ✅ Всё добавлено с примерами 32 пресетов, 3-уровневый Room Studio с Delays 0-30ms step 0.1 + Trims, Chain View скрин, PEQ Editor live curve 30-40 точек, Load Profiles Lite/Music/Cinema, Volume Arc полукруг + dB | Разделы 7-12 |
| Room Correction был только PEQ, нужны задержки, RAW с задержками, пресеты накладываются на Room (Room=глобальный слой поверх), эффекты слышны даже если Room занял много полос, Dual PEQ Room 16 + Creative 12 + Tone separate 2 biquad | ✅ Final architecture Вариант B Dual PEQ утвержден, RoomProfile с Delay L/R/Sub Trim Scale, REW import с Delay строками, Room Scale 0-100%, room_slot_mask replaced by dual PEQ | Раздел 5 DSP |
| HPF/BPF/стерео расширитель/объёмный звук куда делся? | ✅ Это и есть AETHER composition (WIDE stereo widening, CINEMA ambience) + BassMono + Psychobass + Night + BassTame + Anti-Ad, зафиксировано что не исчезло, а объединено в ЭФИР |  |
| Smoothness: mute возвращается плавно по-флагмански, без мерцания | ✅ Flagship Smoothness System Soft Unmute 400ms raised-cosine, Soft Mute 80-150ms, GainSmoother везде, visual no flicker/tearing, frame pacing, Quality Scaling превентивно, Volume Ballistics |  |
| yoRadio modernization: какие уарт команды/фичи, полный план действий для переделки включая USB flash | ✅ Часть 0 YoRadio prep, разделы 0.1-0.8 с таблицами команд, поведением кнопок, прогрессом, shuffle/repeat, USB Host plan, UART framing, интеграцией в OMNIA |  |

**Итого по обсуждениям:** всё покрыто. Единственное изменение от обсуждений — сокращение источников до 5 FINAL (TF/U-disk отключены), что логично и упрощает.

### 1.3 Стыковка по смыслу реализации?

- **YoRadio WEB/SD/USB-Flash = один домен RADIO** — логично, т.к. один декодер ESP32-S3, один I2S slave, один RATE_PIN. Переключение внутри без domain switch экономит время и избегает щелчков.
- **BT401 BT/USB Audio = второй домен BT401** — логично, BT401 MASTER, клоки 11.2896/12.288, Fs watch нужен, BT_RESYNC уже существовал.
- **AT32 v·core = хаб** для всех: парсит RATE/MUTE, форвардит PROGRESS/META/TRACK на S2, делает DSP Dual PEQ Room+Creative.
- **S2 AURA = только рендер + input + optional secondary network**, primary control via yoRadio — сохраняет Performance Contract (single-core S2 не перегружен).
- **External Flash GPIO10-13 отдельный SPI host** — не делит шину с TFT (35-37), поэтому дисплей не тормозит.
- **Dual PEQ Room 16 + Creative 12** решает проблему "Room съел все полосы, эффекты не слышны" — теперь эффекты всегда есть.
- **Smoothness** везде через GainSmoother — защищает слух и акустику, flagship feel.
- **Visual Pack** 12 стилей с Quality Scaling High/Med/Low — укладывается в 2MB PSRAM (framebuffer 150KB + history 75KB + particles 40KB + glyph cache), 20-30Гц достижимо (проверено на примере спектр анализаторов на ESP32).
- **USB-Flash Host на GPIO19/20** — правильно, т.к. onboard USB-C только Device, а S3 имеет internal OTG PHY на 19/20. Внешнее 5V питание + полифьюз 0.75А + кондеры = стабильность для разных флешек.

**Вывод аудита:** ничего не упущено, всё стыкуется, логика реализации последовательна, проверка после каждого этапа прописана.

## 2. Подробный путь модернизации Ё-радио (для OMNIA) с репо https://github.com/ChillPulse/Youradio_Omnia_ESP32S3 — все UARTы, режимы включая USB Flash

Этот раздел — пошаговая инструкция именно по вашему репо, с привязкой к файлам.

### 2.1 База репо

- **Репо:** https://github.com/ChillPulse/Youradio_Omnia_ESP32S3 (Initial commit, main branch)
- **Структура:** yoRadio/yoRadio.ino (3920 bytes), yoRadio/myoptions.h (691 bytes), yoRadio/mytheme.h (3442), yoRadio/src/main.cpp (15174), yoRadio/src/core/ (player.cpp, etc), yoRadio/src/audioI2S/ (декодеры AAC/FLAC/MP3/Opus/Vorbis), yoRadio/src/displays/, yoRadio/src/plugins/, HA/ (Home Assistant integration), examples/
- **Текущее состояние:** уже есть pin rate и работает (V7G рубеж зафиксирован), I2S_ROLE_SLAVE в Audio.cpp, STM_RATE_PIN=4, syncRateToStmPins() в player.cpp.
- **ReadMe:** версия v.0.9.635(m) модификация.

### 2.2 Аппаратная часть (что уже сделано и что финализировать)

#### I2S + RATE/MUTE (стенд STM32F401RCT6, потом AT32)
- ESP GPIO17 → PB4 DATA OUT, PB3 → GPIO16 BCLK, PA15 → GPIO18 LRCK, PB5 → PCM5102A DIN, общий GND.
- ESP GPIO2 → PA8 MUTE HIGH=mute, ESP GPIO4 → PA9 RATE LOW=44.1 HIGH=48.
- Файлы: yoRadio/src/core/player.cpp (init pinMode, default HIGH 48k, syncRateToStmPins() каждые 100ms getSampleRate() смена → mute→RATE_PIN→delay 120ms→unmute), STM main.c V7G (I2S master full-duplex DMA, PA9 RATE вход, PA8 MUTE, s/s1000/s0 статус).
- **Проверка:** проигрывание треков разной Fs → скорость нормальная >30мин, дропов нет, heap >90KB, RATE корректно меняется.

#### USB-Flash Host Hardware
- **Ты сделал:** GPIO20/19 ESP32-S3 D-/D+, отдельное мощное 5V, полифьюз 0.75А 6В, кондеры на USB-A мама.
- **Финализировать:** USB-A female разъем, 100nF+10uF на VBUS, TVS диод опционально, 5V 1A+ LDO/DC-DC, GND общий. S3 internal OTG PHY на 19/20, external PHY не нужен.
- **Проверка:** разные флешки FAT32/exFAT 8-64GB определяются, не просаживают 3.3V, ток <750mA, полифьюз не срабатывает при холостом, срабатывает при КЗ.

### 2.3 Софт — баги на фикс

#### Bug Shuffle пропускает 1 трек
- **Где:** скорее всего в yoRadio/src/core/playlist manager, original YoRadio official page.
- **Фикс:** переписать random без повторов, off-by-one проверка size-1, unit-тест на PC: список 10 треков shuffle repeat off → 10 уникальных, затем стоп. Повтор ALL → новая перестановка равномерная. CLI shuffle on/off.
- **Файлы:** player.cpp / playlist.cpp / config manager.

#### Bug SD требует WiFi
- **Где:** в main.cpp / wifi manager, условие if WiFi not connected → SD не монтируется.
- **Фикс:** убрать зависимость, SD init автономно, NTP optional fallback, лог SD Mounted даже при WiFi disconnected.
- **Проверка:** старт с выключенным роутером, SD вставлена → играет.

#### sdpos слабый / нет progress
- **Где:** commandhandler.cpp + player.cpp (есть sdpos, snuffle, resume position, getFilePos, connecttoFS с позицией).
- **Фикс:** добавить обёртки в Audio: getAudioCurrentTime(), getAudioFileDuration(), setAudioPlayPosition(), canSeek, getFilePos/getFileSize. В loop слать ##PROGRESS#.

### 2.4 Новые фичи — Progress + Seek (ключевая флагманская)

#### Реализация в yoRadio
- **В Audio библиотеке:** уже есть getAudioCurrentTime()/getAudioFileDuration()/getFilePos() etc для локальных файлов (для WEB duration 0).
- **В player.cpp:** 
  - Периодически (2-5Hz) читать current_ms, duration_ms, state, percent_x10 = current*1000/duration, idx/total.
  - Слать ##PROGRESS#: ...
  - Обработка входящих: seek <ms> → setAudioPlayTime / setFilePos, seek_rel +/-, seek_percent 0-1000, sdpos byte pos, seek_start +/- + seek_stop для hold.
  - При seek: soft-mute 50-120ms → seek → unmute.
  - Hold: серия SEEK_REL с ускорением 5s/10s/30s/60s.

- **UART исходящее формат:**
  ```
  ##PROGRESS#: 124500 243000 1 512 3 12
  ##META#: title=...;artist=...;album=...;bitrate=...;fmt=mp3/flac/aac/wav;sr=44100;ch=2
  ##TRACK#: idx=3;total=12;path=/music/track.mp3
  ##SD.STATUS#: mounted size=... fs=FAT32 files=124
  ##USB.STATUS#: mounted size=... fs=...
  ```

- **Проверка:** MP3/FLAC/AAC/WAV seek начало/середина/конец без щелчка, hold ускорение, отпускание точный final seek, PROGRESS 2-5Hz без heap спама, кириллица ID3 не ломает META, NEXT/PREV не конфликтуют.

#### Web UI
- Обновить web интерфейс (в yoRadio/data) под progress slider + elapsed/remaining + file position.
- Использует те же команды seek via websocket/http.

### 2.5 Shuffle / Repeat — флагманский дизайн + команды

```
shuffle on/off
repeat off/one/all
shuffle_repeat on/off (комбинация)
random
playmode shuffle / normal / repeat_one (alias)
```

- Режимы: OFF+OFF стоп в конце, OFF+ALL цикл, OFF+ONE повтор одного, ON+OFF random без повторов → стоп, ON+ALL random цикл, ON+ONE игнорирует shuffle.
- UI иконка shuffle + repeat, Status bar + Now Playing.
- **Проверка:** 10 треков shuffle all repeat off → 10 уникальных без пропуска, затем стоп.

### 2.6 USB-Flash Playback — полный софт путь

#### ESP-IDF / Arduino
1. USB Host MSC: usb_host_install + task + driver install on GPIO19/20 (TinyUSB или ESP-IDF usb_host_msc).
2. Обнаружение: callback connect/disconnect → ##USB.STATUS#.
3. FatFS mount: esp_vfs_fat, FAT32/FAT16/exFAT (включить exFAT support).
4. Сканирование: рекурсивно или по выбранной папке, фильтр .mp3 .flac .aac .m4a .wav .ogg .opus .wma.
5. Индексация: как SD, отдельный список USB files.
6. Audio: connecttoFS с file handle из USB VFS (так же как SD).
7. Команды list/play <n>/next/prev/seek/progress/meta идентично SD.
8. Hot-plug: safe unmount, ##USB.STATUS# + toast.
9. Кириллица, длинные пути, exFAT, >2GB.

#### Этапы (с проверкой после каждого)
- USB-0 hardware check: питание, полифьюз, кондеры, D+/D- continuity.
- USB-1 Host init: VID/PID print, detect device.
- USB-2 Mount FatFS: list root.
- USB-3 Play one file via Audio.
- USB-4 Full playlist + next/prev + progress/seek + meta.
- USB-5 Hot-plug + error handling + web UI.

- **Проверки:** mount/list/play без дропов, seek внутри USB, выдергивание во время play → stop без краша/heap leak, повторная вставка → авто rescan, ток защита "USB Power Low".

### 2.7 UART расширение — все необходимые уарты, режимы, baud

#### Что есть сейчас (из UART файла 5KB)
- Serial 115200, CLI mode 0/1/2 prev/next/toggle/stop/start/play vol... list play x info dspon dim sleep tzo date version heap boot reset wifi.list/con/station/con/ssid... rssi discon...

#### Что добавить (приоритет)
- **STM32F401 стенд:** UART PA2/PA3 115200 тех. лог, s/s1000/s0 автостатус, gain/fade/mute/restart.
- **Для OMNIA AT32:** поднять baud до 460800/921600, DMA UART + кольцевые буферы + очереди FreeRTOS по приоритетам HIGH commands/SPEC, MEDIUM PROGRESS, LOW STATUS.
- **Форматы сообщений:**
  - Бинарный для high-rate SPEC (20-30Hz) + PROGRESS (2-10Hz): header [0xAA msg_id len LSB/MSB seq payload CRC8/16]
  - Текстовый CLI-like для команд (как сейчас) — проще отлаживать, max 256 байт UTF-8 экранировать CR/LF.
  - AT32 = умный хаб: парсит, валидирует, rate-limit'ит, форвардит, добавляет seq/timestamp, приоритизирует, Quality Scaling снижает rate при перегрузке.

#### Таблица типов сообщений
| Тип | Направление | Частота | Пример | Приоритет |
|---|---|---|---|---|
| SPEC/Meters | AT32→S2 | 20-30Hz | бинарный | HIGH |
| PROGRESS | YoRadio→AT32→S2 | 2-5Hz usually 10Hz seek | P 124500 243000 1 | MEDIUM |
| META title/artist | YoRadio→... | on-change | ##META# | MEDIUM |
| Commands seek/play | S2/AT32→YoRadio | по событию | seek 30000 | HIGH rate-limited |
| STATUS/RATE/MUTE | YoRadio→AT32 | on-change+periodic | RATE 48000 | MEDIUM |
| ERROR/DIAG | обе | on-error | ##ERROR# | LOW |

#### Документация
- docs/uart_protocol_v1.md: примеры пакетов, state machine источника WEB/SD/USB/BT/USB_Audio, error codes, baud 460800/921600, framing.
- docs/yoRadio_modernization.md: Этап 0 подготовка, Этап 1 Progress+Seek SD, Этап 2 Meta+playlist+shuffle/repeat+ошибки, Этап 3 USB Flash, Этап 4 полировка (heap, logs, Web UI), Этап 5 интеграция OMNIA (AT32+S2 forwarding primary via yoRadio).
- docs/vendor_bt401_mclk_confirm.md: письмо/подтверждение MCLK continuously stable.

### 2.8 Интеграция в OMNIA — финальный путь

1. Заменить STM32F401 стенд на AT32 v·core: AT32 тоже I2S master в RADIO domain (WEB/SD/USB-Flash) с RATE/MUTE handshake (следит за RATE_PIN от YoRadio GPIO4).
2. AT32 форвардит PROGRESS/META/TRACK/STATUS на S2, S2 рендерит красивую шкалу + elapsed/remaining + percent + waveform.
3. Поднять baud 460800/921600, UART YoRadio → AT32 → S2 forwarding с очередями.
4. Primary control App↔WiFi↔YoRadio S3→UART→AT32→UART→S2. S2 остаётся разгруженным render+local input, Performance Contract safe, один WiFi клиент. Secondary optional S2 MQTT/WebSocket для HA/OTA.
5. Тест: переключение между 5 источниками (WEB/SD/USB-Flash/BT/USB-Audio) click-free, no overrun_count рост, resync_count только при Fs change.

### 2.9 Чеклист готовности Ё-радио к интеграции (должен пройти все)

- [ ] Shuffle bug fixed, repeat modes OFF/ONE/ALL + shuffle on/off, статистика без пропусков (10 треков shuffle no repeat → 10 уникальных)
- [ ] SD без WiFi автономно (роутер выкл → SD играет, лог SD Mounted)
- [ ] Progress+Seek SD и USB-Flash: PROGRESS 2-5Hz, seek с ускорением 2x→16x, без кликов, soft-mute 50-120ms
- [ ] USB-Flash Host: mount/list/play/next/prev/seek/meta/hot-plug стабильно 1ч+, heap нет утечек после 100 seek + 50 next/prev + выдергивания
- [ ] Метаданные ID3 title/artist/album year bitrate fmt sr ch корректно UTF-8/кириллица
- [ ] Логи UART не спамят, формат задокументирован, совместимость старого CLI сохранена (mode 0/1/2 prev/next/toggle...)
- [ ] Web UI обновлён progress/seek/USB, слайдер работает
- [ ] I2S handshake MUTE+RATE работает, скорость нормальная, дропов нет >30мин
- [ ] USB hardware: GPIO19/20 + 5V + полифьюз 0.75А + кондеры, разные флешки FAT32/exFAT, ток <750mA, нет просадки 3.3V
- [ ] Heap >90KB free после старта, после 1ч play + 100 seek не падает

После этого — интеграция в OMNIA по дорожной карте v8.1.

## 3. Итог

- **Ничего не упущено** из оригинального ТЗ v7.2 + комментариев + Pinterest визуализаторов + pinmap + network + DSP discussions + yoRadio багов и фич. Всё сохранено в v8.1 и стыкуется по смыслу.
- **Стыковка логики:** 5 источников FINAL логичны (3 в RADIO домене, 2 в BT401 домене), Dual PEQ решает проблему "Room съел полосы", DMA-only + dirty-rect + Soft Unmute дают флагман, External Flash GPIO10-13 any vendor на отдельном SPI host не мешает TFT, Primary control via yoRadio разгружает S2.
- **Путь модернизации Ё-радио** детализирован по файлам репо Youradio_Omnia_ESP32S3 (player.cpp syncRateToStmPins, Audio.cpp I2S_ROLE_SLAVE, main.cpp, myoptions.h, src/audioI2S), с полной таблицей UART команд, режимами WEB/SD/USB-Flash/BT/USB-Audio, с проверками после каждого этапа (bring-up, shuffle fix, SD without WiFi, Progress+Seek, USB Host 0-5, Web UI, heap, integration).
- **Следующие артефакты для генерации:** SOURCES_v8.1_MATRIX.md уже готов, можно сразу делать pinmap_ui.h с EXT_FLASH 10-13, uart_protocol_v1.md, hydra_flux.c particle pool, needle.c ballistics, yoRadio player.cpp патчи для seek/progress/usb.

АльхамдулиЛлях, всё под контролем. Двигаемся по чеклистам v8.1.
