#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

/* =========================
 * AUDIO BUFFER CONFIG
 * ========================= */
#define DMA_FRAMES              128
#define DMA_WORDS               (DMA_FRAMES * 2)   // int16 words: L,R,L,R...
#define HALF_WORDS              (DMA_WORDS / 2)

/* =========================
 * INPUT PINS FOR DIAGNOSTICS
 * ========================= */
#define SRC_MUTE_PORT           GPIOA
#define SRC_MUTE_PIN            GPIO_PIN_8

#define WS_SENSE_PORT           GPIOA
#define WS_SENSE_PIN            GPIO_PIN_15

/* =========================
 * RESYNC CONFIG
 * ========================= */
#define WS_WAIT_TIMEOUT_MS      100

/* =========================
 * GLOBAL HANDLES
 * ========================= */
I2S_HandleTypeDef hi2s3;
UART_HandleTypeDef huart2;

/* DMA handles are defined in stm32f4xx_hal_msp.c */
extern DMA_HandleTypeDef hdma_i2s3_ext_rx;
extern DMA_HandleTypeDef hdma_spi3_tx;

/* =========================
 * AUDIO BUFFERS
 * ========================= */
static int16_t rx_buf[DMA_WORDS] __attribute__((aligned(4)));
static int16_t tx_buf[DMA_WORDS] __attribute__((aligned(4)));

/* =========================
 * RUNTIME STATUS
 * ========================= */
volatile uint32_t cb_half_count = 0;
volatile uint32_t cb_full_count = 0;
volatile uint32_t i2s_err_count = 0;
volatile uint8_t  g_test_mode = 9;

volatile uint32_t g_rx_peak = 0;
volatile uint32_t g_tx_peak = 0;

volatile uint8_t  g_src_pin_state = 0;
volatile uint32_t g_src_edge_count = 0;
volatile uint8_t  g_last_ws_state = 0;

volatile uint8_t  g_force_mute = 0;
volatile uint8_t  g_rearm_busy = 0;

volatile uint32_t g_rearm_count = 0;
volatile uint32_t g_rearm_ok = 0;
volatile uint32_t g_rearm_fail = 0;

/* auto rearm config */
volatile uint8_t  g_auto_rearm_enable = 1;
volatile uint32_t g_warmup_halves_cfg = 8;
volatile uint32_t g_warmup_halves_left = 0;
volatile uint32_t g_post_fall_hold_ms = 100;
volatile uint32_t g_hold_deadline_ms = 0;

/* =========================
 * TRANSITION STATE MACHINE
 * ========================= */
typedef enum
{
    TR_IDLE = 0,
    TR_WAIT_SRC_END,
    TR_WAIT_HOLD,
    TR_WARMUP
} transition_state_t;

volatile transition_state_t g_tr_state = TR_IDLE;

/* =========================
 * PROTOTYPES
 * ========================= */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2S3_Init(void);

static void uart_log(const char *fmt, ...);
static void print_help(void);
static void print_status(void);
static void handle_uart_commands(void);
static void reset_runtime_counters(void);
static void poll_source_pins_and_sm(void);

static inline int16_t sat16_from_i32(int32_t x);
static inline uint32_t abs16_to_u32(int16_t x);
static inline float pcm16_to_f32(int16_t s);
static inline int16_t f32_to_pcm16(float x);
static inline int16_t apply_mode_sample(int16_t s, uint32_t global_word_index, uint8_t mode);

static void zero_tx_block(uint32_t start_word, uint32_t count_words);
static void zero_audio_buffers(void);
static void hard_stop_i2s_path(void);
static uint8_t wait_for_ws_phase_high(uint32_t timeout_ms);
static uint8_t do_rearm_sequence(const char *tag);
static void cycle_warmup_cfg(void);
static void cycle_hold_cfg(void);
static const char *mode_name(uint8_t mode);
static const char *tr_state_name(transition_state_t st);
static void process_block(uint32_t start_word, uint32_t count_words);

void Error_Handler(void);

/* =========================
 * MAIN
 * ========================= */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2S3_Init();

    memset((void *)rx_buf, 0, sizeof(rx_buf));
    memset((void *)tx_buf, 0, sizeof(tx_buf));

    g_src_pin_state = (HAL_GPIO_ReadPin(SRC_MUTE_PORT, SRC_MUTE_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    g_last_ws_state = (HAL_GPIO_ReadPin(WS_SENSE_PORT, WS_SENSE_PIN) == GPIO_PIN_SET) ? 1u : 0u;

    uart_log("\r\n=== OMNIA-V AUTO_REARM TEST (WS HIGH) ===\r\n");
    uart_log("DMA_FRAMES = %u\r\n", DMA_FRAMES);
    uart_log("DMA_WORDS  = %u\r\n", DMA_WORDS);
    uart_log("Initial mode   = %u (%s)\r\n", g_test_mode, mode_name(g_test_mode));
    uart_log("AUTO_REARM     = %s\r\n", g_auto_rearm_enable ? "ON" : "OFF");
    uart_log("Warmup halves  = %lu\r\n", g_warmup_halves_cfg);
    uart_log("Post-fall hold = %lu ms\r\n", g_post_fall_hold_ms);
    print_help();

    if (HAL_I2SEx_TransmitReceive_DMA(&hi2s3,
                                      (uint16_t *)tx_buf,
                                      (uint16_t *)rx_buf,
                                      DMA_WORDS) != HAL_OK)
    {
        uart_log("HAL_I2SEx_TransmitReceive_DMA() FAILED\r\n");
        Error_Handler();
    }

    uart_log("I2S DMA START OK\r\n");

    uint32_t last_log = HAL_GetTick();

    while (1)
    {
        poll_source_pins_and_sm();
        handle_uart_commands();

        if ((HAL_GetTick() - last_log) >= 1000U)
        {
            last_log = HAL_GetTick();
            print_status();
        }
    }
}

/* =========================
 * SMALL HELPERS
 * ========================= */
static inline int16_t sat16_from_i32(int32_t x)
{
    if (x > 32767)  return 32767;
    if (x < -32768) return -32768;
    return (int16_t)x;
}

static inline uint32_t abs16_to_u32(int16_t x)
{
    if (x == -32768) return 32768u;
    return (x < 0) ? (uint32_t)(-x) : (uint32_t)x;
}

static inline float pcm16_to_f32(int16_t s)
{
    return (float)s * (1.0f / 32768.0f);
}

static inline int16_t f32_to_pcm16(float x)
{
    if (x >  0.999969f) x =  0.999969f;
    if (x < -1.000000f) x = -1.000000f;

    int32_t q = (int32_t)(x * 32767.0f);
    return sat16_from_i32(q);
}

/* =========================
 * TEST MODES
 * ========================= */
static inline int16_t apply_mode_sample(int16_t s, uint32_t global_word_index, uint8_t mode)
{
    int32_t y_i32 = s;

    switch (mode)
    {
        case 0:
            return s;

        case 1:
            return s;

        case 2:
            y_i32 = ((int32_t)s) >> 1;
            return sat16_from_i32(y_i32);

        case 3:
            y_i32 = ((int32_t)s) / 2;
            return sat16_from_i32(y_i32);

        case 4:
            if (s == -32768) y_i32 = 32767;
            else             y_i32 = -((int32_t)s);
            return sat16_from_i32(y_i32);

        case 5:
            if ((global_word_index & 1u) == 0u) y_i32 = ((int32_t)s) / 2;
            else                                y_i32 = s;
            return sat16_from_i32(y_i32);

        case 6:
            if ((global_word_index & 1u) != 0u) y_i32 = ((int32_t)s) / 2;
            else                                y_i32 = s;
            return sat16_from_i32(y_i32);

        case 7:
            y_i32 = -(((int32_t)s) / 2);
            return sat16_from_i32(y_i32);

        case 8:
            y_i32 = (((int32_t)s) * 3) / 4;
            return sat16_from_i32(y_i32);

        case 9:
        {
            float x = pcm16_to_f32(s);
            float y = x;
            return f32_to_pcm16(y);
        }

        case 10:
        {
            float x = pcm16_to_f32(s);
            float y = x * 0.5f;
            return f32_to_pcm16(y);
        }

        default:
            return s;
    }
}

/* =========================
 * AUDIO PROCESS
 * ========================= */
static void zero_tx_block(uint32_t start_word, uint32_t count_words)
{
    memset((void *)&tx_buf[start_word], 0, count_words * sizeof(int16_t));
}

static void zero_audio_buffers(void)
{
    memset((void *)rx_buf, 0, sizeof(rx_buf));
    memset((void *)tx_buf, 0, sizeof(tx_buf));
}

static void process_block(uint32_t start_word, uint32_t count_words)
{
    uint8_t silent =
        (g_force_mute != 0u) ||
        (g_rearm_busy != 0u) ||
        (g_tr_state != TR_IDLE);

    if (silent)
    {
        zero_tx_block(start_word, count_words);
        return;
    }

    for (uint32_t i = 0; i < count_words; i++)
    {
        uint32_t gi = start_word + i;
        int16_t s = rx_buf[gi];
        int16_t y = apply_mode_sample(s, gi, g_test_mode);

        tx_buf[gi] = y;

        uint32_t arx = abs16_to_u32(s);
        uint32_t atx = abs16_to_u32(y);

        if (arx > g_rx_peak) g_rx_peak = arx;
        if (atx > g_tx_peak) g_tx_peak = atx;
    }
}

/* =========================
 * PIN POLL + AUTO_REARM SM
 * ========================= */
static void poll_source_pins_and_sm(void)
{
    uint32_t now = HAL_GetTick();

    uint8_t src = (HAL_GPIO_ReadPin(SRC_MUTE_PORT, SRC_MUTE_PIN) == GPIO_PIN_SET) ? 1u : 0u;
    uint8_t ws  = (HAL_GPIO_ReadPin(WS_SENSE_PORT, WS_SENSE_PIN) == GPIO_PIN_SET) ? 1u : 0u;

    g_last_ws_state = ws;

    if (src != g_src_pin_state)
    {
        g_src_pin_state = src;
        g_src_edge_count++;

        if (src)
        {
            uart_log("SRC_MUTE = 1\r\n");

            if (g_auto_rearm_enable)
            {
                g_tr_state = TR_WAIT_SRC_END;
            }
        }
        else
        {
            uart_log("SRC_MUTE = 0\r\n");

            if (g_auto_rearm_enable)
            {
                if (g_tr_state == TR_WAIT_SRC_END || g_tr_state == TR_WAIT_HOLD)
                {
                    g_hold_deadline_ms = now + g_post_fall_hold_ms;
                    g_tr_state = TR_WAIT_HOLD;
                    uart_log("AUTO_REARM: hold %lu ms\r\n", g_post_fall_hold_ms);
                }
            }
        }
    }

    if (!g_auto_rearm_enable)
    {
        if (g_tr_state != TR_IDLE)
        {
            g_tr_state = TR_IDLE;
            g_warmup_halves_left = 0;
        }
        return;
    }

    switch (g_tr_state)
    {
        case TR_IDLE:
            break;

        case TR_WAIT_SRC_END:
            break;

        case TR_WAIT_HOLD:
            if (g_src_pin_state)
            {
                g_tr_state = TR_WAIT_SRC_END;
                break;
            }

            if ((int32_t)(now - g_hold_deadline_ms) >= 0)
            {
                if (do_rearm_sequence("AUTO_REARM"))
                {
                    if (g_warmup_halves_cfg == 0)
                    {
                        g_tr_state = TR_IDLE;
                        uart_log("AUTO_REARM: no warmup, unmuted\r\n");
                    }
                    else
                    {
                        g_warmup_halves_left = g_warmup_halves_cfg;
                        g_tr_state = TR_WARMUP;
                        uart_log("AUTO_REARM: warmup start, halves=%lu\r\n", g_warmup_halves_cfg);
                    }
                }
                else
                {
                    g_tr_state = TR_IDLE;
                }
            }
            break;

        case TR_WARMUP:
            if (g_src_pin_state)
            {
                g_tr_state = TR_WAIT_SRC_END;
                uart_log("AUTO_REARM: new SRC_MUTE during warmup\r\n");
                break;
            }

            if (g_warmup_halves_left == 0)
            {
                g_tr_state = TR_IDLE;
                uart_log("AUTO_REARM: warmup done, unmuted\r\n");
            }
            break;

        default:
            g_tr_state = TR_IDLE;
            break;
    }
}

/* =========================
 * HARD STOP + WS HIGH REARM
 * ========================= */
static void hard_stop_i2s_path(void)
{
    HAL_I2S_DMAStop(&hi2s3);

    if (hi2s3.hdmarx != NULL)
    {
        HAL_DMA_Abort(hi2s3.hdmarx);
        __HAL_DMA_DISABLE(hi2s3.hdmarx);
    }

    if (hi2s3.hdmatx != NULL)
    {
        HAL_DMA_Abort(hi2s3.hdmatx);
        __HAL_DMA_DISABLE(hi2s3.hdmatx);
    }

    __HAL_I2S_DISABLE(&hi2s3);
    HAL_I2S_DeInit(&hi2s3);
}

static uint8_t wait_for_ws_phase_high(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) <= timeout_ms)
    {
        if (HAL_GPIO_ReadPin(WS_SENSE_PORT, WS_SENSE_PIN) == GPIO_PIN_SET)
        {
            /* quick re-check that we're not exactly on the edge */
            uint8_t stable = 1;
            for (volatile uint32_t i = 0; i < 32; i++)
            {
                if (HAL_GPIO_ReadPin(WS_SENSE_PORT, WS_SENSE_PIN) != GPIO_PIN_SET)
                {
                    stable = 0;
                    break;
                }
            }

            if (stable)
                return 1;
        }
    }

    return 0;
}

static uint8_t do_rearm_sequence(const char *tag)
{
    HAL_StatusTypeDef st;

    g_rearm_count++;
    g_rearm_busy = 1;

    uart_log("%s: begin\r\n", tag);

    zero_tx_block(0, DMA_WORDS);

    HAL_NVIC_DisableIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);

    hard_stop_i2s_path();
    zero_audio_buffers();

    MX_I2S3_Init();

    if (!wait_for_ws_phase_high(WS_WAIT_TIMEOUT_MS))
    {
        HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
        HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

        uart_log("%s: WS HIGH wait timeout\r\n", tag);
        g_rearm_fail++;
        g_rearm_busy = 0;
        return 0;
    }

    st = HAL_I2SEx_TransmitReceive_DMA(&hi2s3,
                                       (uint16_t *)tx_buf,
                                       (uint16_t *)rx_buf,
                                       DMA_WORDS);

    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    if (st != HAL_OK)
    {
        uart_log("%s: DMA start FAIL\r\n", tag);
        g_rearm_fail++;
        g_rearm_busy = 0;
        return 0;
    }

    g_rearm_ok++;
    g_rearm_busy = 0;

    uart_log("%s: OK, ws=%u\r\n",
             tag,
             (uint32_t)((HAL_GPIO_ReadPin(WS_SENSE_PORT, WS_SENSE_PIN) == GPIO_PIN_SET) ? 1u : 0u));

    return 1;
}

/* =========================
 * CONFIG CYCLERS
 * ========================= */
static void cycle_warmup_cfg(void)
{
    if (g_warmup_halves_cfg == 0)       g_warmup_halves_cfg = 4;
    else if (g_warmup_halves_cfg == 4)  g_warmup_halves_cfg = 8;
    else if (g_warmup_halves_cfg == 8)  g_warmup_halves_cfg = 16;
    else if (g_warmup_halves_cfg == 16) g_warmup_halves_cfg = 32;
    else                                g_warmup_halves_cfg = 0;

    uart_log("\r\nWarmup half-blocks = %lu\r\n", g_warmup_halves_cfg);
}

static void cycle_hold_cfg(void)
{
    if (g_post_fall_hold_ms == 0)         g_post_fall_hold_ms = 50;
    else if (g_post_fall_hold_ms == 50)   g_post_fall_hold_ms = 100;
    else if (g_post_fall_hold_ms == 100)  g_post_fall_hold_ms = 200;
    else if (g_post_fall_hold_ms == 200)  g_post_fall_hold_ms = 500;
    else if (g_post_fall_hold_ms == 500)  g_post_fall_hold_ms = 1000;
    else                                  g_post_fall_hold_ms = 0;

    uart_log("\r\nPost-fall hold = %lu ms\r\n", g_post_fall_hold_ms);
}

/* =========================
 * HAL CALLBACKS
 * ========================= */
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3)
    {
        cb_half_count++;
        process_block(0, HALF_WORDS);

        if (g_tr_state == TR_WARMUP && g_warmup_halves_left > 0)
        {
            g_warmup_halves_left--;
        }
    }
}

void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3)
    {
        cb_full_count++;
        process_block(HALF_WORDS, HALF_WORDS);

        if (g_tr_state == TR_WARMUP && g_warmup_halves_left > 0)
        {
            g_warmup_halves_left--;
        }
    }
}

void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3)
    {
        i2s_err_count++;
    }
}

/* =========================
 * UART
 * ========================= */
static void uart_log(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (len <= 0) return;
    if (len > (int)sizeof(buf)) len = sizeof(buf);

    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100);
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, 100);
    return len;
}

static const char *mode_name(uint8_t mode)
{
    switch (mode)
    {
        case 0:  return "raw copy";
        case 1:  return "arithmetic identity";
        case 2:  return "signed shift half (>>1)";
        case 3:  return "signed divide half (/2)";
        case 4:  return "invert polarity";
        case 5:  return "attenuate even words only";
        case 6:  return "attenuate odd words only";
        case 7:  return "invert + half (-0.5)";
        case 8:  return "fixed gain 0.75";
        case 9:  return "float bypass unity";
        case 10: return "float gain 0.5";
        default: return "unknown";
    }
}

static const char *tr_state_name(transition_state_t st)
{
    switch (st)
    {
        case TR_IDLE:         return "IDLE";
        case TR_WAIT_SRC_END: return "WAIT_SRC_END";
        case TR_WAIT_HOLD:    return "WAIT_HOLD";
        case TR_WARMUP:       return "WARMUP";
        default:              return "?";
    }
}

static void print_help(void)
{
    uart_log("Commands:\r\n");
    uart_log("  0..9 : set test mode 0..9\r\n");
    uart_log("  a    : set test mode 10 (float gain 0.5)\r\n");
    uart_log("  u    : toggle AUTO_REARM ON/OFF\r\n");
    uart_log("  x    : MANUAL RESYNC now\r\n");
    uart_log("  w    : cycle warmup half-blocks 0/4/8/16/32\r\n");
    uart_log("  p    : cycle post-fall hold 0/50/100/200/500/1000 ms\r\n");
    uart_log("  m    : toggle FORCE_MUTE ON/OFF\r\n");
    uart_log("  s    : status\r\n");
    uart_log("  r    : reset counters/peaks\r\n");
    uart_log("  h    : help\r\n");
    uart_log("\r\n");
}

static void print_status(void)
{
    uart_log("status: mode=%u (%s), auto=%u, tr=%s, force_mute=%u, rearm_busy=%u, rearm_count=%lu, rearm_ok=%lu, rearm_fail=%lu, warmup_cfg=%lu, warmup_left=%lu, hold_ms=%lu, src=%u, src_edges=%lu, ws=%u, half=%lu, full=%lu, i2s_err=%lu, rx_peak=%lu, tx_peak=%lu\r\n",
             g_test_mode,
             mode_name(g_test_mode),
             g_auto_rearm_enable,
             tr_state_name(g_tr_state),
             g_force_mute,
             g_rearm_busy,
             g_rearm_count,
             g_rearm_ok,
             g_rearm_fail,
             g_warmup_halves_cfg,
             g_warmup_halves_left,
             g_post_fall_hold_ms,
             g_src_pin_state,
             g_src_edge_count,
             g_last_ws_state,
             cb_half_count,
             cb_full_count,
             i2s_err_count,
             g_rx_peak,
             g_tx_peak);
}

static void reset_runtime_counters(void)
{
    cb_half_count = 0;
    cb_full_count = 0;
    i2s_err_count = 0;
    g_rx_peak = 0;
    g_tx_peak = 0;
    g_src_edge_count = 0;
    g_rearm_count = 0;
    g_rearm_ok = 0;
    g_rearm_fail = 0;
}

static void handle_uart_commands(void)
{
    uint8_t ch;

    if (HAL_UART_Receive(&huart2, &ch, 1, 0) != HAL_OK)
        return;

    if (ch >= '0' && ch <= '9')
    {
        g_test_mode = (uint8_t)(ch - '0');
        uart_log("\r\nSET MODE = %u (%s)\r\n", g_test_mode, mode_name(g_test_mode));
        return;
    }

    switch (ch)
    {
        case 'a':
        case 'A':
            g_test_mode = 10;
            uart_log("\r\nSET MODE = %u (%s)\r\n", g_test_mode, mode_name(g_test_mode));
            break;

        case 'u':
        case 'U':
            g_auto_rearm_enable = g_auto_rearm_enable ? 0u : 1u;
            if (!g_auto_rearm_enable)
            {
                g_tr_state = TR_IDLE;
                g_warmup_halves_left = 0;
            }
            uart_log("\r\nAUTO_REARM = %s\r\n", g_auto_rearm_enable ? "ON" : "OFF");
            break;

        case 'x':
        case 'X':
            if (do_rearm_sequence("MANUAL_RESYNC"))
            {
                if (g_warmup_halves_cfg == 0)
                {
                    g_tr_state = TR_IDLE;
                    g_warmup_halves_left = 0;
                }
                else
                {
                    g_warmup_halves_left = g_warmup_halves_cfg;
                    g_tr_state = TR_WARMUP;
                    uart_log("MANUAL_RESYNC: warmup start, halves=%lu\r\n", g_warmup_halves_cfg);
                }
            }
            break;

        case 'w':
        case 'W':
            cycle_warmup_cfg();
            break;

        case 'p':
        case 'P':
            cycle_hold_cfg();
            break;

        case 'm':
        case 'M':
            g_force_mute = g_force_mute ? 0u : 1u;
            uart_log("\r\nFORCE_MUTE = %s\r\n", g_force_mute ? "ON" : "OFF");
            break;

        case 's':
        case 'S':
            uart_log("\r\n");
            print_status();
            break;

        case 'r':
        case 'R':
            reset_runtime_counters();
            uart_log("\r\nCounters/peaks reset\r\n");
            print_status();
            break;

        case 'h':
        case 'H':
            uart_log("\r\n");
            print_help();
            break;

        case '\r':
        case '\n':
            break;

        default:
            uart_log("\r\nUnknown cmd: '%c'\r\n", ch);
            print_help();
            break;
    }
}

/* =========================
 * CLOCK
 * ========================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                                | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1
                                | RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================
 * PERIPH INIT
 * ========================= */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = SRC_MUTE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SRC_MUTE_PORT, &GPIO_InitStruct);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_I2S3_Init(void)
{
    hi2s3.Instance = SPI3;
    hi2s3.Init.Mode = I2S_MODE_SLAVE_TX;
    hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
    hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
    hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
    hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
    hi2s3.Init.CPOL = I2S_CPOL_LOW;
    hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
    hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;

    if (HAL_I2S_Init(&hi2s3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* =========================
 * ERROR
 * ========================= */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
