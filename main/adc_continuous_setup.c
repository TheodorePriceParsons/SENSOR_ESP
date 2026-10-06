/*
 * Sensor ESP firmware (ESP32-S3, ESP-IDF v6.0)
 *
 *   - IR beacon detection: continuous ADC sampling + Goertzel lock-in at
 *     1 kHz and 10 kHz.
 *   - Metal detection: two Colpitts oscillators frequency-counted by PCNT.
 *   - Results streamed to the main ESP over UART.
 *
 * Pin map
 *   GPIO1   IR front-end output (ADC1_CH0)
 *   GPIO4   UART1 TX -> main ESP
 *   GPIO5   UART1 RX <- main ESP
 *   GPIO14  Metal detector 0 (74HC14 output)
 *   GPIO13  Metal detector 1 (74HC14 output)
 *
 * Task layout
 *   core 1  adc_sampling  ADC DMA drain + Goertzel (time critical)
 *   core 0  uart_send     50 Hz UART frames
 *   core 0  metal_pcnt    1.5 s frequency gate
 *
 * Only ADC1 pins (GPIO1-10) are usable for the IR input: continuous mode on
 * ADC2 (GPIO11-20) is unreliable on the S3 per Espressif errata, independent
 * of the usual ADC2/Wi-Fi conflict.
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_adc/adc_continuous.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"

static const char *TAG = "sensor";

/* ======================================================================== */
/* Shared result                                                             */
/* ======================================================================== */

// Sent over UART as a raw memcpy, so it must be packed: without it the
// compiler may insert padding that shifts every field on the receiver.
// The main ESP's copy of this struct must be byte-for-byte identical.
typedef struct __attribute__((packed)) {
    uint16_t freq1_mag;      // 1 kHz amplitude, raw ADC counts
    uint16_t freq2_mag;      // 10 kHz amplitude, raw ADC counts
    uint8_t  detected_mask;  // bit0 = 1 kHz above threshold, bit1 = 10 kHz
    float    metal_hz[2];    // oscillator frequency, detector 0 / 1
} ir_result_t;               // 13 bytes

// Length-1 queue used as a mailbox: sampling_task overwrites it with each
// new result and uart_send_task reads the freshest one on its own schedule,
// decoupling the two tasks' rates.
static QueueHandle_t s_result_queue;

/* ======================================================================== */
/* IR detection: continuous ADC + Goertzel                                   */
/* ======================================================================== */

#define ADC_UNIT_USED     ADC_UNIT_1
#define ADC_CHANNEL_USED  ADC_CHANNEL_0      // GPIO1
#define ADC_ATTEN_USED    ADC_ATTEN_DB_12    // ~0-3.1 V full scale
#define SAMPLE_RATE_HZ    50000              // S3 range: 611 Hz - 83.333 kHz
#define READ_LEN_BYTES    256                // bytes per DMA conversion frame

// Block size is chosen so both tones land on exact integer DFT bins
// (k = N * f / fs); off-bin tones leak into neighbouring bins and lose
// magnitude. N = 500 at 50 kHz gives k = 10 (1 kHz) and k = 100 (10 kHz),
// one result every 10 ms. N = 1000 was tested for extra processing gain on
// the 10 kHz channel and gave no improvement.
#define GOERTZEL_BLOCK_SIZE  500
#define IR_FREQ1_HZ          1000.0f
#define IR_FREQ2_HZ          10000.0f

// Thresholds are per tone and should not be compared across tones: the
// analog band-pass does not have flat gain, so the two channels sit at
// different levels for the same emitter power. Tune each by logging its
// magnitude with the beacon on vs. blocked and setting the threshold about
// midway between that channel's noise floor and signal level.
#define IR_DETECT_THRESHOLD_1K   700
#define IR_DETECT_THRESHOLD_10K  70

#define BLOCKS_PER_SEC     (SAMPLE_RATE_HZ / GOERTZEL_BLOCK_SIZE)
#define MAG_LOG_DIVIDER    (BLOCKS_PER_SEC / 5)   // magnitude log at 5 Hz

static TaskHandle_t s_task_handle;

// Driver health counters, logged once per second.
static uint32_t s_read_fail_count = 0;
static uint32_t s_invalid_sample_count = 0;

// ISR context: a DMA conversion frame is ready. Only wake the task.
static bool IRAM_ATTR on_conv_done(adc_continuous_handle_t handle,
                                    const adc_continuous_evt_data_t *edata,
                                    void *user_data)
{
    BaseType_t must_yield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &must_yield);
    return must_yield == pdTRUE;
}

static adc_continuous_handle_t adc_continuous_setup(void)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 1024,              // driver ring buffer
        .conv_frame_size    = READ_LEN_BYTES,    // bytes per interrupt
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &handle));

    adc_digi_pattern_config_t pattern[1] = {
        {
            .atten     = ADC_ATTEN_USED,
            .channel   = ADC_CHANNEL_USED,
            .unit      = ADC_UNIT_USED,
            .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,   // 12-bit on S3
        }
    };

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,  // required on S3
        .pattern_num    = 1,
        .adc_pattern    = pattern,
    };
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    adc_continuous_evt_cbs_t callbacks = {
        .on_conv_done = on_conv_done,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &callbacks, NULL));

    return handle;
}

typedef struct {
    float coeff;    // 2 cos(w), the only value used in the recursion
    float cosine;   // cos(w), for the final real part
    float sine;     // sin(w), for the final imaginary part
} goertzel_coeffs_t;

// Computed once per tone at startup.
static goertzel_coeffs_t goertzel_precompute(float target_hz, float sample_rate_hz, int block_size)
{
    int k = (int)(0.5f + (block_size * target_hz) / sample_rate_hz);
    float omega = (2.0f * (float)M_PI * (float)k) / (float)block_size;

    goertzel_coeffs_t c;
    c.cosine = cosf(omega);
    c.sine   = sinf(omega);
    c.coeff  = 2.0f * c.cosine;
    return c;
}

// Returns the tone's amplitude in raw ADC counts. The raw Goertzel output
// for an amplitude-A sinusoid is ~A*N/2; dividing by N/2 recovers A, which
// is directly comparable to what a scope shows at the ADC pin.
static float goertzel_magnitude(const goertzel_coeffs_t *gc, const float *samples, int n)
{
    float s_prev = 0.0f, s_prev2 = 0.0f;
    for (int i = 0; i < n; i++) {
        float s = samples[i] + gc->coeff * s_prev - s_prev2;
        s_prev2 = s_prev;
        s_prev = s;
    }
    float real = s_prev - s_prev2 * gc->cosine;
    float imag = s_prev2 * gc->sine;
    float raw_mag = sqrtf(real * real + imag * imag);

    return raw_mag / ((float)n / 2.0f);
}

static inline uint16_t saturate_u16(float v)
{
    if (v <= 0.0f)     return 0;
    if (v >= 65535.0f) return UINT16_MAX;
    return (uint16_t)v;
}

static void sampling_task(void *pv)
{
    adc_continuous_handle_t handle = (adc_continuous_handle_t)pv;

    // Static to keep these buffers off the 4 kB task stack.
    static uint8_t               raw_buf[READ_LEN_BYTES];
    static adc_continuous_data_t parsed[READ_LEN_BYTES / SOC_ADC_DIGI_RESULT_BYTES];
    static float                 sample_buf[GOERTZEL_BLOCK_SIZE];
    int      buf_idx = 0;
    uint32_t print_counter = 0;
    uint32_t diag_print_counter = 0;

    goertzel_coeffs_t gc_1k  = goertzel_precompute(IR_FREQ1_HZ, SAMPLE_RATE_HZ, GOERTZEL_BLOCK_SIZE);
    goertzel_coeffs_t gc_10k = goertzel_precompute(IR_FREQ2_HZ, SAMPLE_RATE_HZ, GOERTZEL_BLOCK_SIZE);

    // Set the ISR's notify target before starting conversions, rather than
    // relying on the creator's out-parameter being written first.
    s_task_handle = xTaskGetCurrentTaskHandle();
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t bytes_read = 0;
        esp_err_t ret = adc_continuous_read(handle, raw_buf, READ_LEN_BYTES, &bytes_read, 0);
        if (ret != ESP_OK) {
            s_read_fail_count++;   // ESP_ERR_TIMEOUT: no new frame yet
            continue;
        }

        uint32_t n_samples = 0;
        ESP_ERROR_CHECK(adc_continuous_parse_data(handle, raw_buf, bytes_read, parsed, &n_samples));

        // Fill non-overlapping blocks; process each one as it completes.
        for (uint32_t i = 0; i < n_samples; i++) {
            if (!parsed[i].valid) {
                s_invalid_sample_count++;
                continue;
            }

            sample_buf[buf_idx++] = (float)parsed[i].raw_data;
            if (buf_idx < GOERTZEL_BLOCK_SIZE) {
                continue;
            }
            buf_idx = 0;

            // Remove DC: the front end sits on a bias, and an un-centred
            // block leaks that energy into every bin.
            float mean = 0.0f;
            for (int j = 0; j < GOERTZEL_BLOCK_SIZE; j++) {
                mean += sample_buf[j];
            }
            mean /= GOERTZEL_BLOCK_SIZE;
            for (int j = 0; j < GOERTZEL_BLOCK_SIZE; j++) {
                sample_buf[j] -= mean;
            }

            float mag1 = goertzel_magnitude(&gc_1k, sample_buf, GOERTZEL_BLOCK_SIZE);
            float mag2 = goertzel_magnitude(&gc_10k, sample_buf, GOERTZEL_BLOCK_SIZE);

            // metal_hz is filled in by uart_send_task just before sending.
            ir_result_t latest = {
                .freq1_mag = saturate_u16(mag1),
                .freq2_mag = saturate_u16(mag2),
                .detected_mask = (uint8_t)((mag1 > IR_DETECT_THRESHOLD_1K  ? 0x01 : 0)
                                          | (mag2 > IR_DETECT_THRESHOLD_10K ? 0x02 : 0)),
            };
            xQueueOverwrite(s_result_queue, &latest);

            if (++print_counter >= MAG_LOG_DIVIDER) {
                print_counter = 0;
                ESP_LOGI(TAG, "1kHz mag=%.1f | 10kHz mag=%.1f | mask=0x%02X",
                         (double)mag1, (double)mag2, latest.detected_mask);
            }

            if (++diag_print_counter >= BLOCKS_PER_SEC) {
                diag_print_counter = 0;
                ESP_LOGI(TAG, "adc diag: read_fail=%lu invalid_samples=%lu",
                         (unsigned long)s_read_fail_count,
                         (unsigned long)s_invalid_sample_count);
            }
        }
    }
}

/* ======================================================================== */
/* Metal detection: PCNT frequency counter                                   */
/* ======================================================================== */

// Each Colpitts oscillator (~150 kHz) is squared up by a 74HC14 and its
// rising edges are counted by a dedicated PCNT unit. accum_count lets the
// driver extend the 16-bit hardware counter in software, so long gates
// never overflow. Frequency = count delta / actual elapsed time (esp_timer),
// so vTaskDelay jitter does not affect the result. Resolution is ~1/gate:
// 1.5 s gives ~0.7 Hz. Baseline comparison and the metal/no-metal decision
// are done on the main ESP.
#define METAL_COUNT       2
#define METAL_GATE_MS     1500
#define METAL_HIGH_LIMIT  30000
#define METAL_GLITCH_NS   125     // well below the ~3.3 us oscillator half-period

static const int          s_metal_gpio[METAL_COUNT] = { 14, 13 };
static pcnt_unit_handle_t s_metal_unit[METAL_COUNT];
static volatile float     s_metal_hz[METAL_COUNT];   // aligned 32-bit store, atomic on S3

static void metal_pcnt_init(int i)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit  = -1,                 // count up only
        .high_limit = METAL_HIGH_LIMIT,
        .flags.accum_count = 1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_metal_unit[i]));

    pcnt_glitch_filter_config_t filt = { .max_glitch_ns = METAL_GLITCH_NS };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_metal_unit[i], &filt));

    pcnt_chan_config_t chan_cfg = {
        .edge_gpio_num  = s_metal_gpio[i],
        .level_gpio_num = -1,             // no gating input
    };
    pcnt_channel_handle_t chan;
    ESP_ERROR_CHECK(pcnt_new_channel(s_metal_unit[i], &chan_cfg, &chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
                    PCNT_CHANNEL_EDGE_ACTION_INCREASE,    // rising edge
                    PCNT_CHANNEL_EDGE_ACTION_HOLD));      // falling edge ignored

    // accum_count requires a watch point on the limit it accumulates across.
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_metal_unit[i], METAL_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(s_metal_unit[i]));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_metal_unit[i]));
    ESP_ERROR_CHECK(pcnt_unit_start(s_metal_unit[i]));
}

static void metal_task(void *pv)
{
    int     prev_count[METAL_COUNT];
    int64_t prev_us[METAL_COUNT];

    for (int i = 0; i < METAL_COUNT; i++) {
        metal_pcnt_init(i);
        pcnt_unit_get_count(s_metal_unit[i], &prev_count[i]);
        prev_us[i] = esp_timer_get_time();
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(METAL_GATE_MS));

        for (int i = 0; i < METAL_COUNT; i++) {
            int count;
            pcnt_unit_get_count(s_metal_unit[i], &count);
            int64_t now_us = esp_timer_get_time();

            s_metal_hz[i] = (float)(count - prev_count[i]) * 1e6f
                          / (float)(now_us - prev_us[i]);

            // Re-zero long before the accumulated int could overflow (~2 h).
            if (count > 1000000000) {
                pcnt_unit_clear_count(s_metal_unit[i]);
                count = 0;
            }
            prev_count[i] = count;
            prev_us[i]    = now_us;
        }

        ESP_LOGI(TAG, "metal: f0=%.2f Hz f1=%.2f Hz",
                 (double)s_metal_hz[0], (double)s_metal_hz[1]);
    }
}

/* ======================================================================== */
/* UART link to main ESP                                                     */
/* ======================================================================== */

#define LINK_UART_PORT   UART_NUM_1    // UART0 stays on USB/console
#define LINK_UART_TX     4
#define LINK_UART_RX     5
#define LINK_UART_BAUD   115200
#define LINK_UART_BUF    256
#define LINK_PERIOD_MS   20            // 50 Hz

#define LINK_FRAME_SYNC  0xAA

static void sensor_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = LINK_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(LINK_UART_PORT, LINK_UART_BUF * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART_PORT, LINK_UART_TX, LINK_UART_RX,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// Frame layout (length-prefixed, so ir_result_t can grow without changing
// this code):
//   [0]        0xAA sync byte, lets the receiver resync mid-stream
//   [1]        N = sizeof(ir_result_t). A mismatch with the receiver's
//              struct size means the boards are running different builds;
//              the receiver should drop the frame.
//   [2..2+N)   raw ir_result_t, native little-endian
//   [2+N]      XOR of the N payload bytes (detects, does not correct)
static void uartSendIRResult(const ir_result_t *result)
{
    uint8_t frame[2 + sizeof(ir_result_t) + 1];
    frame[0] = LINK_FRAME_SYNC;
    frame[1] = (uint8_t)sizeof(ir_result_t);
    memcpy(&frame[2], result, sizeof(ir_result_t));

    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(ir_result_t); i++) {
        checksum ^= frame[2 + i];
    }
    frame[2 + sizeof(ir_result_t)] = checksum;

    uart_write_bytes(LINK_UART_PORT, (const char *)frame, sizeof(frame));
}

// Sends the freshest IR + metal readings at a fixed 50 Hz, independent of
// how often each producer updates.
static void uart_send_task(void *pv)
{
    ir_result_t latest = {0};
    sensor_uart_init();

    TickType_t last_wake = xTaskGetTickCount();
    while (1) {
        xQueuePeek(s_result_queue, &latest, 0);   // keeps previous value if none yet
        latest.metal_hz[0] = s_metal_hz[0];
        latest.metal_hz[1] = s_metal_hz[1];

        uartSendIRResult(&latest);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LINK_PERIOD_MS));
    }
}

/* ======================================================================== */

void app_main(void)
{
    s_result_queue = xQueueCreate(1, sizeof(ir_result_t));

    adc_continuous_handle_t handle = adc_continuous_setup();

    xTaskCreatePinnedToCore(sampling_task, "adc_sampling", 4096, handle,
                            configMAX_PRIORITIES - 2, NULL, 1);

    xTaskCreatePinnedToCore(uart_send_task, "uart_send", 4096, NULL,
                            configMAX_PRIORITIES - 3, NULL, 0);

    xTaskCreatePinnedToCore(metal_task, "metal_pcnt", 3072, NULL,
                            5, NULL, 0);
}