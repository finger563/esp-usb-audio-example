#include "usb.hpp"

extern "C" {
#include <tusb.h>
#include <class/audio/audio_device.h>
#include "tinyusb.h"
}

#include "logger.hpp"

static espp::Logger logger({.tag="usb", .level=espp::Logger::Verbosity::INFO});

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]  VENDOR | AUDIO | MIDI | HID | MSC | CDC [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                           _PID_MAP(MIDI, 3) | _PID_MAP(AUDIO, 4)  | _PID_MAP(VENDOR, 5) )

#define USB_VID   0xCafe
#define USB_BCD   0x0200

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
static tusb_desc_device_t desc_device =
{
  .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,

    // Use Interface Association Descriptor (IAD)
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass = TUSB_CLASS_MISC, // 0xEF
    .bDeviceSubClass = MISC_SUBCLASS_COMMON, //0x02
    .bDeviceProtocol = MISC_PROTOCOL_IAD, // 0x01

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

// String Descriptor Index
enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
};

const char* string_descriptor[] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},        // 0: is supported language is English (0x0409)
    "Finger563",                 // 1: Manufacturer
    "Finger563 USB Headset",     // 2: Product
    "123456",                    // 3: Serials, should use chip ID
    "USB Speakers",              // 4: Interface 1 string
    "USB Microphone",            // 5: Interface 2 string
};

//------------- USB Endpoint numbers -------------//
enum {
  // Endpoints 0x00 and 0x80 are for CONTROL transfers => do not use
  // Available USB Endpoints: 5 IN/OUT EPs and 1 IN EP
  EP_EMPTY = 0,
  EP_AUDIO_IN = 0x01,
  EP_AUDIO_OUT = 0x01,
};


#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_AUDIO * TUD_AUDIO_HEADSET_STEREO_DESC_LEN)

static uint8_t configuration_descriptor[] = {
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
  // Interface number, string index, EP Out & EP In address, EP size
  TUD_AUDIO_HEADSET_STEREO_DESCRIPTOR(2, EP_AUDIO_OUT, EP_AUDIO_IN | 0x80),
};

//--------------------------------------------------------------------+
// Internal callback functions for various events. These are not part
// of the public API.
//--------------------------------------------------------------------+

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
extern "C" void tud_mount_cb(void) {
  logger.info("USB mounted");
}

// Invoked when device is unmounted
extern "C" void tud_umount_cb(void) {
  logger.info("USB unmounted");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
extern "C" void tud_suspend_cb(bool remote_wakeup_en) {
  logger.info("USB suspended, remote_wakeup_en: {}", remote_wakeup_en);
}

// Invoked when usb bus is resumed
extern "C" void tud_resume_cb(void) {
  logger.info("USB resumed");
}

//--------------------------------------------------------------------+
// Public functions
//--------------------------------------------------------------------+

void usb_init(void) {
  const tinyusb_config_t tusb_cfg = {
    .device_descriptor = &desc_device,
    .string_descriptor = string_descriptor,
    .string_descriptor_count = sizeof(string_descriptor) / sizeof(string_descriptor[0]),
    .external_phy = false,
    .configuration_descriptor = configuration_descriptor,
    .self_powered = false,
    .vbus_monitor_io = 0, // ignored if not self powered
  };
  tinyusb_driver_install(&tusb_cfg);
}

void usb_set_info(uint16_t vid,
                  uint16_t pid,
                  uint16_t version_bcd,
                  const std::string &manufacturer,
                  const std::string &product,
                  const std::string &serial) {
  desc_device.bcdUSB = version_bcd;
  desc_device.idVendor = vid;
  desc_device.idProduct = pid;
  string_descriptor[STRID_MANUFACTURER] = manufacturer.c_str();
  string_descriptor[STRID_PRODUCT] = product.c_str();
  string_descriptor[STRID_SERIAL] = serial.c_str();
}
