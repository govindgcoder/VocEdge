#include "esp32-hal.h"
#include <Arduino.h> 
#include <cmath>
#include "mic.h"

int16_t sample_buffer[1024];

static unsigned long silence_start = 0; // tail timer since last valid speech frame
static unsigned long record_since = 0;  // when the current utterance started
static float noise_floor = 0.0f;        // calibrated ambient level
static float smoothed_rms = 0.0f;       // damped/EMA version of the RMS
static bool voice_active = false;       // hysteresis: currently in a speech segment
static int loud_blocks = 0;             // consecutive loud frames for onset gating

#define VOICE_KICK_PADDING 300.0f // must exceed noise floor to (re)start a word
#define VOICE_HOLD_PADDING 80.0f  // speech keeps the window alive above this level
#define VOICE_START_COUNT 2       // frames above kick before voice officially "begins"
#define SILENCE_TAIL_MS 1000      // trailing silence after speech before END
#define MAX_RECORD_MS 10000       // safety cap so a stuck mic can't hang forever
#define RMS_EMA_ALPHA 0.30f       // lower = more damping of single-frame spikes

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
    dynamic_threshold = noise_floor + VOICE_KICK_PADDING;
    smoothed_rms = 0.0f;
    voice_active = false;
    loud_blocks = 0;
    Serial.printf("[+] Calibration complete! Noise floor: %.2f  Kick: %.2f  Hold: %.2f\n\n",
                  noise_floor, noise_floor + VOICE_KICK_PADDING,
                  noise_floor + VOICE_HOLD_PADDING);
  }
}

size_t record() {
    size_t bytes_read = 0;
    i2s_read(I2S_PORT, sample_buffer, sizeof(sample_buffer), &bytes_read, 100 / portTICK_PERIOD_MS);
    return bytes_read;
}
void reset_silence() {
    silence_start = millis(); // reset timer the moment a new session starts
    record_since = millis();  // mark the start of this utterance (for the cap)
    voice_active = false;
    loud_blocks = 0;
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
    Serial.printf("Mic RMS: %.2f (floor %.2f kick %.2f hold %.2f)%s\n",
                  rms, noise_floor,
                  noise_floor + VOICE_KICK_PADDING,
                  noise_floor + VOICE_HOLD_PADDING,
                  voice_active ? " [VOICE]" : "");
    last_print = millis();
  }
  // -------------------------

  float kick = noise_floor + VOICE_KICK_PADDING;
  float hold = noise_floor + VOICE_HOLD_PADDING;

  // Hysteresis via two gates:
  //  - START needs a strong, sustained signal (filters faint background blips)
  //  - once talking, a softer level (hold band) keeps the window alive
  //  - speech only "ends" once a block falls below the hold floor
  if (rms > kick) {
    loud_blocks++;
    if (loud_blocks >= VOICE_START_COUNT && !voice_active) {
      voice_active = true;
      silence_start = millis();
      Serial.println("[+] Voice detected!");
    }
  } else {
    loud_blocks = 0;
    if (rms <= hold) voice_active = false;
  }

  // Extend the recording tail while we're clearly (or softly) still talking
  if (voice_active && rms > hold) silence_start = millis();

  if (millis() - silence_start >= SILENCE_TAIL_MS) return true;
  if (millis() - record_since >= MAX_RECORD_MS) return true; // runaway guard
  return false;
}