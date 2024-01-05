#include "usb.hpp"

extern "C" {
#include <tusb.h>
#include <class/audio/audio_device.h>
#include "tinyusb.h"
}

#include "logger.hpp"

static espp::Logger logger({.tag="usb", .level=espp::Logger::Verbosity::INFO});

//--------------------------------------------------------------------+
// Internal callback functions for various USB events. These are not
// part of the public API.
//--------------------------------------------------------------------+

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]     AUDIO | MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n)  ( (CFG_TUD_##itf) << (n) )
#define USB_PID           (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
    _PID_MAP(MIDI, 3) | _PID_MAP(AUDIO, 4) | _PID_MAP(VENDOR, 5) )

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Interface Association Descriptor (IAD) for CDC
    // As required by USB Specs IAD's subclass must be common class (2) and protocol must be IAD (1)
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0xCafe,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum
{
  ITF_NUM_AUDIO_CONTROL = 0,
  ITF_NUM_AUDIO_STREAMING,
  ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN    	(TUD_CONFIG_DESC_LEN + CFG_TUD_AUDIO * TUD_AUDIO_MIC_ONE_CH_DESC_LEN)

#define EPNUM_AUDIO   0x01

uint8_t const desc_configuration[] =
{
  // Interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

  // Interface number, string index, EP Out & EP In address, EP size
  TUD_AUDIO_MIC_ONE_CH_DESCRIPTOR(/*_itfnum*/ ITF_NUM_AUDIO_CONTROL, /*_stridx*/ 0, /*_nBytesPerSample*/ CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX, /*_nBitsUsedPerSample*/ CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_TX*8, /*_epin*/ 0x80 | EPNUM_AUDIO, /*_epsize*/ CFG_TUD_AUDIO_EP_SZ_IN)
};

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
char const* string_desc_arr [] =
{
    (const char[]) { 0x09, 0x04 }, 	// 0: is supported language is English (0x0409)
    "PaniRCorp",                   	// 1: Manufacturer
    "MicNode",                  		// 2: Product
    "123456",                      	// 3: Serials, should use chip ID
    "UAC2",                      	 	// 4: Audio Interface
};

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
    .string_descriptor = string_desc_arr,
    .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
    .external_phy = false,
    .configuration_descriptor = desc_configuration,
    .self_powered = false,
    .vbus_monitor_io = 0, // ignored if not self powered
  };
  tinyusb_driver_install(&tusb_cfg);
}
