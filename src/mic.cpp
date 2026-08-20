#include "esp32-hal.h"
#include <Arduino.h> 
#include <cmath>
#include "mic.h"

int16_t sample_buffer[1024];

static unsigned long record_since = 0;  // when the current utterance started
static float noise_floor = 0.0f;        // calibrated ambient level
static float smoothed_rms = 0.0f;       // damped/EMA version of the RMS
static int momentum = 0;                // leaky bucket: speech energy, drains in quiet
static bool voice_active = false;       // true while momentum > 0
static bool heard_speech = false;       // set once the first spoken frame arrives

#define VOICE_LEVEL_PADDING 300.0f // RMS above noise floor counts as speech
#define MOMENTUM_CAP 20            // max bucket fill (frames of pause grace)
#define MOMENTUM_UP 5              // fill per loud frame
#define MOMENTUM_DOWN 1            // leak per quiet frame
#define MAX_RECORD_MS 10000        // safety cap so a stuck mic can't hang forever
#define RMS_EMA_ALPHA 0.30f        // lower = more damping of single-frame spikes

void setup_mic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 512, // Lowered from 1024 to ensure memory allocation succeeds
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .mck_io_num   = I2S_PIN_NO_CHANGE, // <--- Moved to the top to fix C++ order
    .bck_io_num   = I2S_SCK,
    .ws_io_num    = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num  = I2S_SD
  };


  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("CRITICAL: Failed to install I2S driver! Error code: %d\n", err);
    return;
  }

  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("CRITICAL: Failed to set I2S pins! Error code: %d\n", err);
  }
}

float dynamic_threshold = 300.0; // Fallback default

void calibrate_mic() {
  Serial.println("\n[+] Calibrating ambient room noise. Shhh...");
  float total_rms = 0;
  int reads = 0;

  // Take 20 quick samples of the room
  for(int j = 0; j < 20; j++) {
    size_t bytes = record();
    if (bytes > 0) {
      int sample_count = bytes / 2;
      if (sample_count == 0) continue;
      int64_t sum_squares = 0;
      for (int i = 0; i < sample_count; i++) {
        int32_t v = sample_buffer[i];
        sum_squares += (int64_t)v * v;
      }
      total_rms += sqrt((float)sum_squares / sample_count);
      reads++;
    }
    delay(20);
  }

  if (reads > 0) {
    noise_floor = total_rms / reads;
    // "Speech padding" decides how far above the room noise speech starts
    dynamic_threshold = noise_floor + VOICE_LEVEL_PADDING;
    smoothed_rms = 0.0f;
    momentum = 0;
    voice_active = false;
    heard_speech = false;
    Serial.printf("[+] Calibration complete! Noise floor: %.2f  Voice level: %.2f\n\n",
                  noise_floor, noise_floor + VOICE_LEVEL_PADDING);
  }
}

size_t record() {
    size_t bytes_read = 0;
    i2s_read(I2S_PORT, sample_buffer, sizeof(sample_buffer), &bytes_read, 100 / portTICK_PERIOD_MS);
    return bytes_read;
}
void reset_silence() {
    record_since = millis(); // mark the start of this utterance (for the cap)
    momentum = 0;
    voice_active = false;
    heard_speech = false;
    smoothed_rms = 0.0f;
}

bool silence_check(size_t bytes_read) {
  if (bytes_read == 0) return false;
  int sample_count = bytes_read / 2;
  if (sample_count == 0) return false;

  // Tiny background-noise filter: strip the block's DC/rumble offset first
  int32_t sum = 0;
  for (int i = 0; i < sample_count; i++) sum += sample_buffer[i];
  float dc = (float)sum / sample_count;

  double sum_squares = 0;
  for (int i = 0; i < sample_count; i++) {
    float v = (float)sample_buffer[i] - dc;
    sum_squares += (double)v * v;
  }
  float raw_rms = sqrt((float)(sum_squares / sample_count));

  // Damping: EMA the RMS so a single-frame blip can't flap the state
  if (smoothed_rms == 0.0f) smoothed_rms = raw_rms;
  smoothed_rms = smoothed_rms * (1.0f - RMS_EMA_ALPHA) + raw_rms * RMS_EMA_ALPHA;
  float rms = smoothed_rms;

  // --- DEBUG BREADCRUMBS ---
  static unsigned long last_print = 0;
  if (millis() - last_print > 250) {
    Serial.printf("Mic RMS: %.2f (voice level %.2f) mom %d/%d%s\n",
                  rms, noise_floor + VOICE_LEVEL_PADDING,
                  momentum, MOMENTUM_CAP,
                  voice_active ? " [VOICE]" : "");
    last_print = millis();
  }
  // -------------------------

  float level = noise_floor + VOICE_LEVEL_PADDING;

  // Leaky-bucket momentum: speech spikes fill the bucket fast (+5 each
  // loud frame, capped at 20), while micro-pauses like breaths and soft
  // consonants only leak 1 point per frame. END fires only when the
  // bucket fully drains to zero.
  if (rms > level) {
    if (!heard_speech) {
      heard_speech = true;
      Serial.println("[+] Voice detected!");
    }
    momentum += MOMENTUM_UP;
    if (momentum > MOMENTUM_CAP) momentum = MOMENTUM_CAP;
    voice_active = true;
  } else {
    if (momentum > 0) momentum -= MOMENTUM_DOWN;
    if (momentum == 0) voice_active = false;
  }

  // END fires only once the bucket drains to zero AFTER speech was actually
  // heard. At record start the bucket is empty, so without this the first
  // quiet frame would instantly end the utterance.
  if (heard_speech && momentum == 0) return true;
  if (millis() - record_since >= MAX_RECORD_MS) return true; // runaway guard
  return false;
}