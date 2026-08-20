#ifndef MIC_H
#define MIC_H

#include <cstddef>
#include <cstdint>
#include <driver/i2s.h>

#define I2S_WS 6
#define I2S_SD 4
#define I2S_SCK 5
#define I2S_PORT I2S_NUM_0

extern int16_t sample_buffer[1024]; // global so it doesn't blow up the stack

void setup_mic();

size_t record();

bool silence_check(size_t bytes_read);
void calibrate_mic();

void reset_silence();

#endif
