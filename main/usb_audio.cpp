#include "usb_audio.hpp"

#include "logger.hpp"

extern "C" {
#include <tusb.h>
#include <class/audio/audio.h>
#include <class/audio/audio_device.h>
#include "tinyusb.h"
}

static espp::Logger logger({.tag="usb audio", .level=espp::Logger::Verbosity::DEBUG});

static std::shared_ptr<espp::I2c> internal_i2c;

void usb_audio_init(std::shared_ptr<espp::I2c> i2c) {
  internal_i2c = i2c;
}

