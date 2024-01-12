#pragma once

#include <functional>
#include <memory>
#include <stdint.h>

#include "i2c.hpp"

#define AUDIO_SAMPLE_RATE (48000)
#define AUDIO_BUFFER_SIZE (AUDIO_SAMPLE_RATE / 5)
#define AUDIO_SAMPLE_COUNT (AUDIO_SAMPLE_RATE / 60)

void audio_init(std::shared_ptr<espp::I2c> internal_i2c);
int16_t* get_audio_buffer0();
int16_t* get_audio_buffer1();
void audio_play_frame(const uint8_t *data, uint32_t num_bytes);
void audio_record_frame(uint8_t *data, uint32_t num_bytes);

bool is_muted();
void set_muted(bool mute);

int get_audio_volume();
void set_audio_volume(int percent);

