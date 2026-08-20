#include "esp32-hal.h"
#include <Arduino.h> 
#include <cmath>
#include "mic.h"

int16_t sample_buffer[1024];

static unsigned long silence_start = 0;

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
      int64_t sum_squares = 0;
      int sample_count = bytes / 2;
      for (int i = 0; i < sample_count; i++) {
        sum_squares += sample_buffer[i] * sample_buffer[i];
      }
      total_rms += sqrt(sum_squares / sample_count);
      reads++;
    }
    delay(20);
  }
  
  if (reads > 0) {
    // Average the room noise, then add 300 as the "speech padding"
    dynamic_threshold = (total_rms / reads) + 300.0; 
    Serial.printf("[+] Calibration complete! Dynamic Threshold set to: %.2f\n\n", dynamic_threshold);
  }
}

size_t record() {
    size_t bytes_read = 0;
    i2s_read(I2S_PORT, sample_buffer, sizeof(sample_buffer), &bytes_read, 100 / portTICK_PERIOD_MS);
    return bytes_read;
}
void reset_silence() {
    silence_start = millis(); // reset timer the moment a new session starts
}

bool silence_check(size_t bytes_read) {
  if (bytes_read == 0) return false;

  int64_t sum_squares = 0;
  int sample_count = bytes_read / 2; 

  for (int i = 0; i < sample_count; i++) {
    sum_squares += sample_buffer[i] * sample_buffer[i];
  }

  float rms = sqrt(sum_squares / sample_count);

  // --- DEBUG BREADCRUMBS ---
  static unsigned long last_print = 0;
  if (millis() - last_print > 250) {
    Serial.printf("Current Mic Volume (RMS): %.2f\n", rms);
    last_print = millis();
  }
  // -------------------------

  // Watch your Serial Monitor! If silence prints as "4500", change this to 5000!
  if (rms > dynamic_threshold) {
    silence_start = millis(); 
  } else if (millis() - silence_start > 1000) { 
    return true;
  }
  return false; 
}