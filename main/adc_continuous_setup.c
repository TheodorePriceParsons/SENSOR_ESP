/*
 * Continuous ADC sampling + Goertzel lock-in detection — ESP32-S3, ESP-IDF
 * (current API, not the deprecated driver/adc.h path).
 *
 * Wiring: phototransistor conditioning chain output -> GPIO1 (ADC1_CH0).
 * Any GPIO1-GPIO10 pin works here; ADC1_CH0 is just the example.
 * Do NOT use an ADC2 pin (GPIO11-20) for this — continuous mode on ADC2
 * is unreliable on the S3 per Espressif's own errata, separate from the
 * WiFi-coexistence issue that affects ADC2 on every ESP32 variant.
 */

 #include <string.h>
 #include <stdio.h>
 #include <math.h>
 #include "esp_log.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 #include "esp_adc/adc_continuous.h"
 #include "driver/uart.h"
 #include "driver/gpio.h"

 
 static const char *TAG = "ir_adc";
 
 // Result mailbox: sampling_task overwrites this with its latest correlation
 // result; uart_send_task drains it on its own schedule. A length-1 queue
 // used this way ("mailbox" pattern) fully decouples the two tasks' rates -
 // the UART side always sends whatever's freshest, however often that is.
 //
 // packed: without this the compiler is free to pad the struct out to a
 // 4-byte boundary (6 bytes instead of 5), and that padding byte would get
 // memcpy'd into the UART frame right along with the real fields - the
 // receiver's identical struct would then be reading everything after the
 // padding one byte off. Keep this packed for as long as this struct is
 // sent via raw memcpy over the wire.
 typedef struct __attribute__((packed)) {
     uint16_t freq1_mag;
     uint16_t freq2_mag;
     uint8_t  detected_mask;
 } ir_result_t;
 
 static QueueHandle_t s_result_queue;
 
 // ---- Sampling config -------------------------------------------------
 #define ADC_UNIT_USED     ADC_UNIT_1              // ADC2 is off-limits on S3, see note above
 #define ADC_CHANNEL_USED  ADC_CHANNEL_0            // GPIO1 on S3
 #define ADC_ATTEN_USED    ADC_ATTEN_DB_12          // ~0-3.3V full-scale input range
 #define SAMPLE_RATE_HZ    50000                    // matches your earlier 50kHz design;
                                                      // valid range on S3 is 611Hz-83,333Hz
 #define READ_LEN_BYTES    256                       // bytes pulled per adc_continuous_read() call
 
 // ---- Goertzel / lock-in config ----------------------------------------
 // 500 samples @ 50kHz = exactly 10ms/block (100 evaluations/sec). This
 // block size was chosen so BOTH target frequencies land on exact integer
 // DFT bins (k = block_size * f / fs must come out whole):
 //   1kHz  -> k = 500*1000/50000  = 10   (exact)
 //   10kHz -> k = 500*10000/50000 = 100  (exact)
 // Exact bins means no spectral leakage between the two lock-in tones from
 // windowing effects - if you ever change SAMPLE_RATE_HZ or the target
 // frequencies, re-check that this still divides evenly, or you'll get
 // smeared/leaky magnitude readings.
 #define GOERTZEL_BLOCK_SIZE  500
 #define IR_FREQ1_HZ          1000.0f
 #define IR_FREQ2_HZ          10000.0f

 #define FREQ_SELECT_PIN GPIO_NUM_2
 
 // Independent thresholds per tone - there's no reason these need to
 // match. Your analog front end (RC filter stage in particular) doesn't
 // have flat gain across frequency, so it's normal for the two channels
 // to sit at very different absolute magnitudes even at the same
 // emitter intensity. Tune each by watching its printed mag value with
 // the source on vs. off/blocked, and set it roughly halfway between
 // that channel's own noise floor and its own signal level - don't
 // compare across channels, only within one.
 #define IR_DETECT_THRESHOLD_1K   700
 #define IR_DETECT_THRESHOLD_10K  70
 
 static TaskHandle_t s_task_handle;
 
 // ISR-context callback: fires once the driver has DMA'd a full conversion
 // frame. All it does is wake the processing task — keep it minimal, it's
 // running with interrupts constrained.
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
 
     // Pool size for the driver's internal ring buffer, and how many bytes
     // one "conversion frame" (one interrupt's worth of DMA'd samples) holds.
     adc_continuous_handle_cfg_t handle_cfg = {
         .max_store_buf_size = 1024,
         .conv_frame_size    = READ_LEN_BYTES,
     };
     ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &handle));
 
     // One channel in the pattern list. If you end up sampling a second
     // physical sensor channel (rather than doing both 1kHz/10kHz targets
     // in software off one channel), add a second adc_digi_pattern_config_t
     // entry here and bump pattern_num to 2.
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
         .format         = ADC_DIGI_OUTPUT_FORMAT_TYPE2,  // S3 requires TYPE2 (TYPE1 is classic-ESP32-only)
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
 
 // ---- Goertzel implementation -------------------------------------------
 
 typedef struct {
     float coeff;    // 2*cos(omega), the only value used in the inner loop
     float cosine;   // cos(omega), cached for the final real-part calc
     float sine;     // sin(omega), cached for the final imag-part calc
 } goertzel_coeffs_t;
 
 // Precompute the coefficients for detecting `target_hz` within a block of
 // `block_size` samples taken at `sample_rate_hz`. Call this once at
 // startup per target frequency, not per block.
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
 
 // Runs the Goertzel recursion over `n` samples and returns the magnitude
 // of the target-frequency component, normalized back into raw-ADC-count
 // (amplitude) units. Without the /(n/2) normalization this returns the
 // raw unnormalized DFT-bin magnitude, which for an amplitude-A sinusoid
 // comes out around A*n/2 — i.e. huge, and not something you can eyeball
 // or set a sane threshold against. Dividing it out gives you back "A",
 // directly comparable to what you'd see on a scope across the input.
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
 
 // Sampling/processing task — pin this to whichever core you're keeping
 // clear of WiFi/UART-send load, per your design doc's core split.
 static void sampling_task(void *pv)
 {
     adc_continuous_handle_t handle = (adc_continuous_handle_t)pv;
     uint8_t raw_buf[READ_LEN_BYTES];
 
     // Goertzel coefficients: computed once here since sampling_task's
     // setup code (everything before the while(1)) only ever runs once.
     goertzel_coeffs_t gc_1k  = goertzel_precompute(IR_FREQ1_HZ, SAMPLE_RATE_HZ, GOERTZEL_BLOCK_SIZE);
     goertzel_coeffs_t gc_10k = goertzel_precompute(IR_FREQ2_HZ, SAMPLE_RATE_HZ, GOERTZEL_BLOCK_SIZE);
 
     // Capture our own handle before starting the ADC, so the ISR callback
     // always has a valid target to notify - don't rely on the creator's
     // out-param write racing against this task actually running.
     s_task_handle = xTaskGetCurrentTaskHandle();
     ESP_ERROR_CHECK(adc_continuous_start(handle));
 
     while (1) {
         // Blocks here until on_conv_done() notifies us a frame is ready.
         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
 
         uint32_t bytes_read = 0;
         esp_err_t ret = adc_continuous_read(handle, raw_buf, READ_LEN_BYTES, &bytes_read, 0);
         if (ret != ESP_OK) {
             continue; // ESP_ERR_TIMEOUT just means nothing new yet
         }
 
         // Parse into typed samples (cleaner than manually casting to
         // adc_digi_output_data_t the way older example code does it).
         adc_continuous_data_t parsed[READ_LEN_BYTES / SOC_ADC_DIGI_RESULT_BYTES];
         uint32_t n_samples = 0;
         ESP_ERROR_CHECK(adc_continuous_parse_data(handle, raw_buf, bytes_read, parsed, &n_samples));
 
         // Accumulate samples into a rolling block, run both Goertzel
         // filters once the block fills, then reset for the next one
         // (non-overlapping windows — simplest correct version; if 10ms
         // of detection latency ever feels too slow you can revisit with
         // overlapping/sliding blocks later).
         //
         // static, not stack-local: 500 floats is 2KB, and this task's
         // stack is only 4096 bytes (see xTaskCreatePinnedToCore below) -
         // keeping this in .bss instead avoids eating half the stack.
         static float sample_buf[GOERTZEL_BLOCK_SIZE];
         static int buf_idx = 0;
         static uint32_t print_counter = 0;
 
         for (uint32_t i = 0; i < n_samples; i++) {
             if (!parsed[i].valid) {
                 continue;
             }
 
             sample_buf[buf_idx++] = (float)parsed[i].raw_data;
 
             if (buf_idx < GOERTZEL_BLOCK_SIZE) {
                 continue;
             }
 
             // Block is full. Remove DC offset first - the photodetector
             // chain output sits on a nonzero bias, and an un-centered
             // block leaks that DC energy into every bin including ours.
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
 
             buf_idx = 0; // start the next block from scratch
 
             bool detect_1khz =
    gpio_get_level(FREQ_SELECT_PIN) == 0;

uint8_t detected_mask = 0;

if (detect_1khz) {
    if (mag1 > IR_DETECT_THRESHOLD_1K) {
        detected_mask = 0x01;
    }
} else {
    if (mag2 > IR_DETECT_THRESHOLD_10K) {
        detected_mask = 0x02;
    }
}

ir_result_t latest = {
    .freq1_mag = (uint16_t)mag1,
    .freq2_mag = (uint16_t)mag2,
    .detected_mask = detected_mask,
};
             xQueueOverwrite(s_result_queue, &latest);
 
             // Block rate is 100Hz (10ms/block). Print every 10th block
             // for a readable ~10 lines/sec on the monitor instead of
             // flooding it.
             if (++print_counter >= 10) {
                 print_counter = 0;
                 ESP_LOGI(TAG, "1kHz mag=%.1f | 10kHz mag=%.1f | mask=0x%02X",
                          (double)mag1, (double)mag2, latest.detected_mask);
             }
         }
     }
 }
 
 
 #define LINK_UART_PORT   UART_NUM_1   
 #define LINK_UART_TX     4
 #define LINK_UART_RX     5
 #define LINK_UART_BAUD   115200        
 #define LINK_UART_BUF    256
 
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
 //NOTICE: I TO BE UPDATED with metal detector readings
 // Frame layout, length-prefixed so the struct can grow without touching
 // this framing code — matched by the Arduino-side parser on main ESP:
 //   [0]        sync byte, always 0xAA - lets the receiver resync if it
 //              ever starts listening mid-frame instead of at a frame
 //              boundary
 //   [1]        length byte = sizeof(ir_result_t). If this ever doesn't
 //              match what the receiver's own struct expects, that's a
 //              build-skew signal (one board reflashed, the other not) -
 //              the receiver should log and drop rather than parse.
 //   [2..2+N)   raw memcpy of ir_result_t, native (little-endian on this
 //              chip) byte order — NOT the big-endian manual packing the
 //              old fixed-frame version used. Receiver's struct must be
 //              byte-for-byte identical (same fields, same order, same
 //              types) for memcpy to line up correctly on both ends.
 //   [2+N]      checksum - XOR of all N payload bytes. Detects a
 //              corrupted frame so the receiver can drop it rather than
 //              act on garbage; it doesn't correct anything, just flags
 //              "don't trust this one."
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
 
 // UART send task - pinned to the OTHER core from sampling_task, per your
 static void uart_send_task(void *pv)
 {
     ir_result_t latest = {0};
 
     // Driver setup lives here since, like sampling_task's Goertzel
     // coefficients, this only needs to happen once before the loop starts.
     sensor_uart_init();
 
     while (1) {
         // Waits up to 20ms for a fresh reading, but sends whatever's latest
         // either way - decouples this task's send rate from however fast
         // sampling_task actually produces new correlation results.
         xQueueReceive(s_result_queue, &latest, pdMS_TO_TICKS(20));
 
         uartSendIRResult(&latest);
 
         vTaskDelay(pdMS_TO_TICKS(20));  // ~50Hz send; your doc says ~100Hz is plenty
     }
 }
 
 void app_main(void)
 {
     gpio_config_t switch_config = {};
 
     switch_config.pin_bit_mask =
         1ULL << FREQ_SELECT_PIN;
 
     switch_config.mode =
         GPIO_MODE_INPUT;
 
     switch_config.pull_up_en =
         GPIO_PULLUP_ENABLE;
 
     switch_config.pull_down_en =
         GPIO_PULLDOWN_DISABLE;
 
     switch_config.intr_type =
         GPIO_INTR_DISABLE;
 
     ESP_ERROR_CHECK(
         gpio_config(&switch_config)
     );
 
     s_result_queue =
         xQueueCreate(
             1,
             sizeof(ir_result_t)
         );
 
     adc_continuous_handle_t handle =
         adc_continuous_setup();
 
     xTaskCreatePinnedToCore(
         sampling_task,
         "adc_sampling",
         4096,
         handle,
         configMAX_PRIORITIES - 2,
         NULL,
         1
     );
 
     xTaskCreatePinnedToCore(
         uart_send_task,
         "uart_send",
         4096,
         NULL,
         configMAX_PRIORITIES - 3,
         NULL,
         0
     );
 }