#pragma once

#include <functional>
#include <memory>
#include <stdint.h>

#include "i2c.hpp"

#include "audio_config.h"

void audio_init(std::shared_ptr<espp::I2c> internal_i2c);
int16_t* get_audio_buffer0();
int16_t* get_audio_buffer1();
void audio_play_frame(const uint8_t *data, uint32_t num_bytes);
uint32_t audio_record_frame(uint8_t *data, uint32_t num_bytes);

size_t get_audio_input_size();

bool is_muted();
void set_muted(bool mute);

int get_audio_volume();
void set_audio_volume(int percent);

