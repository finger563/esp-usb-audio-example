#include "usb.hpp"

extern "C" {
#include <tusb.h>
#include <class/audio/audio_device.h>
#include "tinyusb.h"
}

// Optional TinyUSB Audio APIs

extern "C" bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  return true;
}

extern "C" bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  return true;
}

extern "C" bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  return true;
}

extern "C" bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  return true;
}
