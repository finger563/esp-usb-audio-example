#include "i2s_audio.hpp"

extern "C" {
#include <tusb.h>
}

#include <atomic>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s_std.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_check.h"

#include "event_manager.hpp"
#include "logger.hpp"
#include "task.hpp"

#include "es7210.hpp"
#include "es8311.hpp"

#include "box_hal.hpp"

using namespace box_hal;

/**
 * Look at
 * https://github.com/espressif/esp-idf/blob/master/examples/peripherals/i2s/i2s_codec/i2s_es8311/main/i2s_es8311_example.c
 * and
 * https://github.com/espressif/esp-box/blob/master/components/bsp/src/peripherals/bsp_i2s.c
 */

/* Example configurations */
#define EXAMPLE_MCLK_MULTIPLE   (I2S_MCLK_MULTIPLE_256) // If not using 24-bit data width, 256 should be enough
#define EXAMPLE_MCLK_FREQ_HZ    (AUDIO_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE)

static i2s_chan_handle_t tx_handle = NULL;
static i2s_chan_handle_t rx_handle = NULL;

static int16_t *audio_buffer0;
static int16_t *audio_buffer1;

const std::string mute_button_topic = "mute/pressed";
static std::atomic<bool> muted_{false};
static std::atomic<int> volume_{60};

static espp::Logger logger({.tag = "I2S Audio", .level = espp::Logger::Verbosity::INFO});

int16_t *get_audio_buffer0() {
  return audio_buffer0;
}

int16_t *get_audio_buffer1() {
  return audio_buffer1;
}

static size_t audio_input_size = 0;

size_t get_audio_input_size() {
  return audio_input_size;
}

void update_volume_output() {
  if (muted_) {
    es8311_codec_set_voice_volume(0);
  } else {
    es8311_codec_set_voice_volume(volume_);
  }
}

void set_muted(bool mute) {
  muted_ = mute;
  update_volume_output();
}

bool is_muted() {
  return muted_;
}

void set_audio_volume(int percent) {
  volume_ = percent;
  update_volume_output();
}

int get_audio_volume() {
  return volume_;
}

static esp_err_t i2s_driver_init(void)
{
  logger.info("initializing i2s driver...");
  auto ret_val = ESP_OK;
  logger.info("Using newer I2S standard");
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
  i2s_std_clk_config_t clock_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE);
  i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO);
  i2s_std_config_t std_cfg = {
    .clk_cfg = clock_cfg,
    .slot_cfg = slot_cfg,
    .gpio_cfg = {
      .mclk = i2s_mck_io,
      .bclk = i2s_bck_io,
      .ws = i2s_ws_io,
      .dout = i2s_do_io,
      .din = i2s_di_io,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));

  static i2s_event_callbacks_t rx_callbacks;
  rx_callbacks.on_recv = [](i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) -> bool {
    // buffer that was just filled
    uint8_t *data = (uint8_t*)event->data;
    // number of bytes in the buffer
    audio_input_size = event->size;
    if (audio_input_size > 0) {
      // memcpy(audio_buffer0, data, audio_input_size);
      tud_audio_write(data, audio_input_size / 2);
    }
    return false;
  };

  // ret_val = i2s_channel_register_event_callback(rx_handle, &rx_callbacks, NULL);
  // if (ret_val != ESP_OK) {
  //   logger.error("ERROR registering i2s event callback: {}", ret_val);
  //   return ret_val;
  // }

  ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
  return ret_val;
}

// es7210 is for audio input codec
static esp_err_t es7210_init_default(void)
{
  logger.info("initializing es7210 codec...");
  esp_err_t ret_val = ESP_OK;
  audio_hal_codec_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE;
  cfg.adc_input = AUDIO_HAL_ADC_INPUT_ALL;
  cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
#if AUDIO_SAMPLE_RATE == 48000
  cfg.i2s_iface.samples = AUDIO_HAL_48K_SAMPLES;
#elif AUDIO_SAMPLE_RATE == 44100
  cfg.i2s_iface.samples = AUDIO_HAL_44K_SAMPLES;
#elif AUDIO_SAMPLE_RATE == 16000
  cfg.i2s_iface.samples = AUDIO_HAL_16K_SAMPLES;
#else
#error "Unsupported sample rate"
#endif
  ret_val |= es7210_adc_init(&cfg);
  ret_val |= es7210_adc_config_i2s(cfg.codec_mode, &cfg.i2s_iface);
  ret_val |= es7210_adc_set_gain((es7210_input_mics_t)(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2), GAIN_37_5DB);
  ret_val |= es7210_adc_set_gain((es7210_input_mics_t)(ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4), GAIN_0DB);
  ret_val |= es7210_adc_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);

  if (ESP_OK != ret_val) {
    logger.error("Failed to initialize es7210 (input) codec");
  }

  return ret_val;
}

// es8311 is for audio output codec
static esp_err_t es8311_init_default(void)
{
  logger.info("initializing es8311 codec...");
  esp_err_t ret_val = ESP_OK;
  audio_hal_codec_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_LINE1;
  cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
#if AUDIO_SAMPLE_RATE == 48000
  cfg.i2s_iface.samples = AUDIO_HAL_48K_SAMPLES;
#elif AUDIO_SAMPLE_RATE == 44100
  cfg.i2s_iface.samples = AUDIO_HAL_44K_SAMPLES;
#elif AUDIO_SAMPLE_RATE == 16000
  cfg.i2s_iface.samples = AUDIO_HAL_16K_SAMPLES;
#else
#error "Unsupported sample rate"
#endif

  ret_val |= es8311_codec_init(&cfg);
  ret_val |= es8311_set_bits_per_sample(cfg.i2s_iface.bits);
  ret_val |= es8311_config_fmt((es_i2s_fmt_t)cfg.i2s_iface.fmt);
  ret_val |= es8311_codec_set_voice_volume(volume_);
  ret_val |= es8311_codec_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);

  if (ESP_OK != ret_val) {
    logger.error("Failed to initialize es8311 (output) codec");
  }

  return ret_val;
}

static std::unique_ptr<espp::Task> mute_task;
static QueueHandle_t gpio_evt_queue;

static void gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void init_mute_button(void) {
  // create the gpio event queue
  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
  // setup gpio interrupts for mute button
  gpio_config_t io_conf;
  memset(&io_conf, 0, sizeof(io_conf));
  // interrupt on any edge (since MUTE is connected to flipflop, see note below)
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = (1<<(int)mute_pin);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);

  // update the mute state (since it's a flip-flop and may have been set if we
  // restarted without power loss)
  set_muted(!gpio_get_level(mute_pin));

  // create a task on core 1 for initializing the gpio interrupt so that the
  // gpio ISR runs on core 1
  auto gpio_task = espp::Task::make_unique(espp::Task::Config{
      .name = "gpio",
        .callback = [](auto &m, auto&cv) -> bool {
          gpio_install_isr_service(0);
          gpio_isr_handler_add(mute_pin, gpio_isr_handler, (void*) mute_pin);
          return true; // stop the task
        },
      .stack_size_bytes = 2*1024,
      .core_id = 1
    });
  gpio_task->start();

  // register that we publish the mute button state
  espp::EventManager::get().add_publisher(mute_button_topic, "i2s_audio");

  // start the gpio task
  mute_task = espp::Task::make_unique(espp::Task::Config{
      .name = "mute",
      .callback = [](auto &m, auto&cv) -> bool {
        static gpio_num_t io_num;
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
          // see if it's the mute button
          if (io_num == mute_pin) {
            // invert the state since these are active low switches
            bool pressed = !gpio_get_level(io_num);
            // NOTE: the MUTE is actually connected to a flip-flop which holds
            // state, so pressing it actually toggles the state that we see on
            // the ESP pin. Therefore, when we get an edge trigger, we should
            // read the state to know whether to be muted or not.
            set_muted(pressed);
            // simply publish that the mute button was presssed
            espp::EventManager::get().publish(mute_button_topic, {});
          }
        }
        // don't want to stop the task
        return false;
      },
      .stack_size_bytes = 4*1024,
    });
  mute_task->start();
}

static bool initialized = false;
void audio_init(std::shared_ptr<espp::I2c> internal_i2c) {
  if (initialized) return;

  // probe the I2C bus to see if the codec is there
  logger.info("Probing I2C bus");
  bool found_es7210 = false;
  auto es7210_addresses = {ES7210_AD1_AD0_00, ES7210_AD1_AD0_01, ES7210_AD1_AD0_10, ES7210_AD1_AD0_11};
  for (const auto address : es7210_addresses) {
    if (internal_i2c->probe_device(address)) {
      logger.info("Found ES7210 at address 0x{:02x}", (uint8_t)address);
      found_es7210 = true;
      break;
    }
  }
  auto es8311_addresses = {0x18};
  bool found_es8311 = false;
  for (const auto address : es8311_addresses) {
    if (internal_i2c->probe_device(address)) {
      logger.info("Found ES8311 at address 0x{:02x}", (uint8_t)address);
      found_es8311 = true;
      break;
    }
  }

  if (!found_es7210 || !found_es8311) {
    logger.error("Could not find one of the codecs: ES7210={}, ES8311={}", found_es7210, found_es8311);
    return;
  }

  /* Config power control IO */
  static esp_err_t bsp_io_config_state = ESP_FAIL;
  if (ESP_OK != bsp_io_config_state) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << (int)sound_power_pin;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    bsp_io_config_state = gpio_config(&io_conf);
  }

  /* Checko IO config result */
  if (ESP_OK != bsp_io_config_state) {
    logger.error("Failed initialize power control IO");
  }

  gpio_set_level(sound_power_pin, 1);

  set_es7210_write(std::bind(&espp::I2c::write, internal_i2c.get(),
                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  set_es7210_read(std::bind(&espp::I2c::read_at_register, internal_i2c.get(),
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  set_es8311_write(std::bind(&espp::I2c::write, internal_i2c.get(),
                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  set_es8311_read(std::bind(&espp::I2c::read_at_register, internal_i2c.get(),
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  i2s_driver_init();
  esp_err_t err = ESP_OK;
  err = es7210_init_default();
  if (err != ESP_OK) {
    logger.error("ERROR initializing ES7210 audio: {}", err);
    return;
  }
  err = es8311_init_default();
  if (err != ESP_OK) {
    logger.error("ERROR initializing ES8311 audio: {}", err);
    return;
  }

  audio_buffer0 = (int16_t*)heap_caps_malloc(sizeof(int16_t) * AUDIO_BUFFER_SIZE + 10, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  audio_buffer1 = (int16_t*)heap_caps_malloc(sizeof(int16_t) * AUDIO_BUFFER_SIZE + 10, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);

  // now initialize the mute gpio
  init_mute_button();

  initialized = true;
}

void audio_deinit() {
  if (!initialized) return;
  i2s_channel_disable(tx_handle);
  i2s_channel_disable(rx_handle);
  i2s_del_channel(tx_handle);
  i2s_del_channel(rx_handle);
  initialized = false;
}

void audio_play_frame(const uint8_t *data, uint32_t num_bytes) {
  if (!initialized) {
    return;
  }
  if (tx_handle == NULL) {
    logger.error("ERROR: tx_handle is NULL");
    return;
  }
  if (data == NULL) {
    logger.error("ERROR: data is NULL");
    return;
  }
  size_t bytes_written = 0;
  auto err = ESP_OK;
  err = i2s_channel_write(tx_handle, data, num_bytes, &bytes_written, 100);
  if(num_bytes != bytes_written) {
    logger.error("ERROR to write {} != written {}", num_bytes, bytes_written);
  }
  if (err != ESP_OK) {
    logger.error("ERROR writing i2s channel: {}, '{}'", err, esp_err_to_name(err));
  }
}

uint32_t audio_record_frame(uint8_t *data, uint32_t num_bytes) {
  if (!initialized) {
    return 0;
  }
  if (rx_handle == NULL) {
    logger.error("ERROR: rx_handle is NULL");
    return 0;
  }
  if (data == NULL) {
    logger.error("ERROR: data is NULL");
    return 0;
  }
  size_t bytes_read = 0;
  auto err = ESP_OK;
  err = i2s_channel_read(rx_handle, data, num_bytes, &bytes_read, 100);
  if(num_bytes != bytes_read) {
    // logger.error("ERROR to read {} != read {}", num_bytes, bytes_read);
  }
  if (err != ESP_OK) {
    // logger.error("ERROR reading i2s channel: {}, '{}'", err, esp_err_to_name(err));
  }
  return bytes_read;
}
