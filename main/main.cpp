#include <chrono>
#include <thread>

#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"

#include "box_hal.hpp"
#include "usb.hpp"

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
  espp::I2c internal_i2c(espp::I2c::Config{
      .port = box_hal::internal_i2c_port,
      .sda_io_num = box_hal::internal_i2c_sda,
      .scl_io_num = box_hal::internal_i2c_scl,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE});

  // initialize the USB device
  logger.info("Initializing USB");
  usb_init();

  // make a simple task that prints "Hello World!" every second
  espp::Task task({
      .name = "Hello World",
      .callback = [&](auto &m, auto &cv) -> bool {
        logger.debug("[{:.3f}] Hello from the task!", elapsed());
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 1s);
        // we don't want to stop the task, so return false
        return false;
      },
      .stack_size_bytes = 4096,
    });
  task.start();

  // also print in the main thread
  while (true) {
    logger.debug("[{:.3f}] Hello World!", elapsed());
    std::this_thread::sleep_for(1s);
  }
}
