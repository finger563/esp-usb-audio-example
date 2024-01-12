#include <chrono>
#include <thread>

extern "C" {
#include <tusb.h>
}

#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"

#include "box_hal.hpp"
#include "i2s_audio.hpp"
#include "usb.hpp"
#include "usb_audio.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static auto start = std::chrono::high_resolution_clock::now();
  static auto elapsed = [&]() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float>(now - start).count();
  };

  espp::Logger logger({.tag = "USB Audio Example", .level = espp::Logger::Verbosity::INFO});

  logger.info("Bootup");

  // initialize the internal i2c bus (for the codecs)
  auto internal_i2c = std::make_shared<espp::I2c>(espp::I2c::Config{
      .port = box_hal::internal_i2c_port,
      .sda_io_num = box_hal::internal_i2c_sda,
      .scl_io_num = box_hal::internal_i2c_scl,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE});

  logger.info("Initializing audio");
  audio_init(internal_i2c);

  // set the volume to 60%
  set_audio_volume(60);

  logger.info("Initializing USB");
  usb_init();

  logger.info("Intializing USB Audio");
  usb_audio_init();

  // make a simple task that prints "Hello World!" every second
  espp::Task task({
      .name = "Hello World",
      .callback = [&](auto &m, auto &cv) -> bool {
        // static int audio_buffer_index = 0;
        // auto audio_buffer = audio_buffer_index == 0 ? get_audio_buffer0() : get_audio_buffer1();
        // audio_buffer_index = (audio_buffer_index + 1) % 2;
        // audio_record_frame((uint8_t*)audio_buffer, CFG_TUD_AUDIO_EP_SZ_IN);
        // tud_audio_write((uint8_t*)audio_buffer, CFG_TUD_AUDIO_EP_SZ_IN);
        // logger.debug("[{:.3f}] Hello from the task!", elapsed());
        // std::unique_lock<std::mutex> lock(m);
        // cv.wait_for(lock, 1s);
        // we don't want to stop the task, so return false
        return false;
      },
      .stack_size_bytes = 4096,
    });
  // task.start();

  // also print in the main thread
  while (true) {
    logger.debug("[{:.3f}] Hello World!", elapsed());
    std::this_thread::sleep_for(1s);
  }
}
