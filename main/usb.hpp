#pragma once

#include <cstdint>
#include <string>

void usb_init(void);
void usb_set_info(uint16_t vid,
                  uint16_t pid,
                  uint16_t version_bcd,
                  const std::string &manufacturer,
                  const std::string &product,
                  const std::string &serial);
