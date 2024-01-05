#pragma once

#include <functional>
#include <memory>

#include "i2c.hpp"

void usb_audio_init(std::shared_ptr<espp::I2c> i2c);
