#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>

// Change these to whatever suits
#define R1_PIN 25
#define G1_PIN 26
#define B1_PIN 27
#define R2_PIN 14
#define G2_PIN 21
#define B2_PIN 13
#define A_PIN 23
#define B_PIN 19
#define C_PIN 5
#define D_PIN 17
#define E_PIN 18 // only for larger matrices, so we won't need this
#define LAT_PIN 4
#define OE_PIN 15
#define CLK_PIN 16

#define PANEL_RES_X 64      // Number of pixels wide of each INDIVIDUAL panel module. 
#define PANEL_RES_Y 32     // Number of pixels tall of each INDIVIDUAL panel module.
#define PANEL_CHAIN 1      // Total number of panels chained one to another

const int bit_shift_factor = (int)(log2(256/PANEL_RES_Y));

MatrixPanel_I2S_DMA *display = nullptr;

// sample rate
const uint16_t sample_rate = 44100; // Unit: Hz

// fft constants
typedef float fftData_t;
const uint8_t fft_sample_count_log2 = 11;
const uint16_t fft_sample_count = 1 << fft_sample_count_log2;
const fftData_t fft_sample_count_inv = 1.0f / fft_sample_count;
const fftData_t fft_sampling_freq = (fftData_t) sample_rate;
const uint16_t fft_freq_bin_count = fft_sample_count / 2;
const float fft_freq_step = fft_sampling_freq / fft_sample_count;

// fft variables
fftData_t fft_data_real[fft_sample_count] = {0.0};
fftData_t fft_data_imag[fft_sample_count] = {0.0};
fftData_t magnitude_spectrum_avg[fft_freq_bin_count] = {0};
ArduinoFFT<fftData_t> fft_ = ArduinoFFT<fftData_t>(fft_data_real, fft_data_imag, fft_sample_count, fft_sampling_freq); // Create FFT object

// i2s hardware constants
const i2s_port_t i2s_port = I2S_NUM_1;
const int kI2S_PinClk = 0;
const int kI2S_PinData = 34;

// Connections to INMP441 I2S microphone
#define I2S_WS 32
#define I2S_SD 35
#define I2S_SCK 33

// i2s constants
const i2s_bits_per_sample_t i2s_bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
const uint8_t i2s_bytes_per_sample = i2s_bits_per_sample / 8;
const uint16_t i2s_read_size_bytes = fft_sample_count * i2s_bytes_per_sample;
const uint16_t i2s_buffer_size_samples = 1024;
const uint16_t i2s_buffer_size_bytes = i2s_buffer_size_samples * i2s_bytes_per_sample;
const uint16_t i2s_buffer_count = (3 * fft_sample_count) / (2 * i2s_buffer_size_samples);
const uint8_t i2s_buffer_count_per_fft = fft_sample_count / i2s_buffer_size_samples;
const int i2s_queue_len = 16;

// i2s variables
int16_t mic_read_buffer[fft_sample_count] = {0};
QueueHandle_t i2s_queue = nullptr;

const uint8_t freq_band_count = 16;
const float freq_band_start_hz = 20;
const float freq_band_end_hz[freq_band_count] = {1397, 2774, 4151, 5528, 6905, 8282, 9659, 11036, 12413, 13790, 15167, 16544, 17921, 19298, 20675, 22050}; // Fill this in appropriately: base of 20, add 1377 each bin (1377 = (22050-20)/16)
const float freq_band_amp[freq_band_count]   = {50, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 505, 500}; // Fill this in appropriately – this seemed to work nicely

const uint16_t buf_len = 5;
uint16_t buf_counter = 0;
float freq_band_avg[freq_band_count];
float freq_band_buf[freq_band_count][buf_len];
float freq_band_smoothed[freq_band_count];

bool setup_i2s_mic()
{
    esp_err_t i2s_error;

    // i2s configuration for sampling 16 bit mono audio data
    //
    // Notes related to i2s.c:
    // - 'dma_buf_len', i.e. the number of samples in each DMA buffer, is limited to 1024
    // - 'dma_buf_len' * 'bytes_per_sample' is limted to 4092
    // - 'I2S_CHANNEL_FMT_ONLY_RIGHT' means "mono", i.e. only one channel to be received via i2s (must use RIGHT!!!)

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = sample_rate,
        .bits_per_sample = i2s_bits_per_sample,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = i2s_buffer_count,
        .dma_buf_len = i2s_buffer_size_samples,
        .use_apll = false
    };

    i2s_error = i2s_driver_install(i2s_port, &i2s_config, i2s_queue_len, &i2s_queue);

    if (i2s_error)
    {
        log_e("Failed to start i2s driver. ESP error: %s (%x)", esp_err_to_name(i2s_error), i2s_error);
        return false;
    }

    if (i2s_queue == nullptr)
    {
        log_e("Failed to setup i2s event queue.");
        return false;
    }

    i2s_pin_config_t i2s_pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    i2s_error = i2s_set_pin(i2s_port, &i2s_pin_config);

    if (i2s_error)
    {
        log_e("Failed to set i2s pins. ESP error: %s (%x)", esp_err_to_name(i2s_error), i2s_error);
        return false;
    }

    return true;
}

void setup_led_matrix() {
    HUB75_I2S_CFG::i2s_pins _pins={R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN, A_PIN, B_PIN, C_PIN, D_PIN, E_PIN, LAT_PIN, OE_PIN, CLK_PIN};
    HUB75_I2S_CFG mxconfig(
        PANEL_RES_X, // Module width
        PANEL_RES_Y, // Module height
        PANEL_CHAIN, // chain length
        _pins, // pin mapping
		HUB75_I2S_CFG::ICN2038S
    );
    mxconfig.clkphase = false;

    display = new MatrixPanel_I2S_DMA(mxconfig);
    display->begin();
    display->setBrightness8(60); //0-255
    display->clearScreen();
}

float calc_buf_mean(float* buf, uint16_t len) {
  float total = 0.0;
  for (uint16_t ii=0;ii<len;ii++) {
    total += buf[ii];
  }
  return total/((float)len);
}

void setup() {
    Serial.begin(115200);
    if (!setup_i2s_mic()) {
        log_e("I2S setup failed!");
    }

    setup_led_matrix();

    log_d("Setup successfully completed.");
}

void loop() {
    esp_err_t i2s_error = ESP_OK;
    size_t i2s_bytes_read = 0;
    i2s_error = i2s_read(i2s_port, mic_read_buffer, i2s_read_size_bytes, &i2s_bytes_read, portMAX_DELAY);

    // Check i2s error state after reading
    if (i2s_error)
    {
        log_e("i2s_read failure. ESP error: %s (%x)", esp_err_to_name(i2s_error), i2s_error);
    }

    // Check whether right number of bytes has been read
    if (i2s_bytes_read != i2s_read_size_bytes)
    {
        log_w("i2s_read unexpected number of bytes: %d", i2s_bytes_read);
    }

    // Compute sum of the current sample block
    int32_t block_sum = 0;
    for (uint16_t ii = 0; ii < fft_sample_count; ii++)
    {
        block_sum += mic_read_buffer[ii];
    }
    // Compute average value for the current sample block
    int16_t block_avg = block_sum / fft_sample_count;

    // Constant for normalizing int16 input values to floating point range -1.0 to 1.0
    const fftData_t int16_max_inv = 1.0f / __INT16_MAX__;

    // initialize fft data
    for (uint32_t ii=0; ii<fft_sample_count; ii++) {
        int16_t v = mic_read_buffer[ii] - block_avg;
        fftData_t r = int16_max_inv * v;
        fft_data_real[ii] = r;
        fft_data_imag[ii] = 0.0f;
    }
    // compute fft
    fft_.compute(FFTDirection::Forward);

    // compute magnitudes
    fft_.complexToMagnitude();

    // compute averages in bins
    uint32_t counter = freq_band_start_hz / fft_freq_step;
    uint32_t counter_prev;
    for (uint8_t bin_num=0; bin_num<freq_band_count; bin_num++) {
        freq_band_avg[bin_num] = 0;
        counter_prev = counter;
        while (counter * fft_freq_step < freq_band_end_hz[bin_num]) {
            freq_band_avg[bin_num] += fft_data_real[counter];
            counter++;
        }
        freq_band_avg[bin_num] /= max((float)(counter-1 - counter_prev), 1.0f);
    }

    for (uint8_t ii=0; ii<freq_band_count; ii++) {
        freq_band_buf[ii][buf_counter] = freq_band_avg[ii];
    }
    buf_counter = buf_counter < buf_len-1 ? buf_counter+1 : 0;

    for (uint8_t ii=0; ii<freq_band_count; ii++) {
        freq_band_smoothed[ii] = calc_buf_mean(freq_band_buf[ii], buf_len);
    }

    // display results
    display->clearScreen();
    uint8_t bar_width = 4;
    for (uint8_t ii = 0; ii < freq_band_count; ii++) {
        uint8_t height = min((int)(freq_band_amp[ii] * freq_band_smoothed[ii]), PANEL_RES_Y);
        for (uint8_t jj = 0; jj < height; jj++) {
            for (uint8_t kk = 0; kk < bar_width; kk++) {
                // Write the line of code to display the pixels.
                int red = 255 * jj/PANEL_RES_Y; // greener the lower you go down the bar
                int green = 255 - red; // redder the higher you go up the bar
                display->drawPixel(bar_width*ii+kk, jj, display->color565(red, green, 0));
            }
        }
    }
}