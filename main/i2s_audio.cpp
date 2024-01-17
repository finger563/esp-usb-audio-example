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

// #include "esp_codec_dev.h"
// #include "esp_codec_dev_defaults.h"
// #include "es8388.h"

#include "es7210.hpp"
#include "es8311.hpp"
#include "es8388.hpp"

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
  // TODO: update to use es8388
  // es8311_codec_set_voice_volume(volume_);
  es8388_set_voice_volume(volume_);
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

  // NOTE: if we want to try to use the asyncrhonous mode, we can use this code:
  // ----------------------------------------

  // static i2s_event_callbacks_t rx_callbacks;
  // rx_callbacks.on_recv = [](i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx) -> bool {
  //   // buffer that was just filled
  //   uint8_t *data = (uint8_t*)event->data;
  //   // number of bytes in the buffer
  //   audio_input_size = event->size;
  //   if (audio_input_size > 0) {
  //     // memcpy(audio_buffer0, data, audio_input_size);
  //     tud_audio_write(data, audio_input_size / 2);
  //   }
  //   return false;
  // };

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
[[maybe_unused]]
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
  ret_val |= es7210_adc_set_gain((es7210_input_mics_t)(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2), GAIN_24DB);
  ret_val |= es7210_adc_set_gain((es7210_input_mics_t)(ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4), GAIN_24DB);
  ret_val |= es7210_adc_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);

  if (ESP_OK != ret_val) {
    logger.error("Failed to initialize es7210 (input) codec");
  }

  return ret_val;
}

// es8311 is for audio output codec
[[maybe_unused]]
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

[[maybe_unused]]
static esp_err_t es8388_init_default(void) {
  // static audio_codec_i2c_cfg_t i2c_cfg =  {
  //   .port = box_hal::internal_i2c_port,
  //   .addr = ES7210_CODEC_DEFAULT_ADDR,
  // };
  // const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);

  // // static es7210_codec_cfg_t es7210_codec_cfg = {};
  // // es7210_codec_cfg.ctrl_if = i2c_ctrl_if;
  // // es7210_codec_new(&es7210_codec_cfg);

  // static es8388_codec_cfg_t es8388_codec_cfg = {};
  // es8388_codec_cfg.ctrl_if = i2c_ctrl_if;
  // es8388_codec_new(&es8388_codec_cfg);

  logger.info("initializing es8388 codec...");
  esp_err_t ret_val = ESP_OK;
  audio_hal_codec_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_BOTH;
  cfg.adc_input = AUDIO_HAL_ADC_INPUT_ALL;
  cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;
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

  ret_val |= es8388_init(&cfg);
  ret_val |= es8388_set_bits_per_sample(ES_MODULE_ADC_DAC, BIT_LENGTH_16BITS);
  ret_val |= es8388_config_fmt(ES_MODULE_ADC_DAC, (es_i2s_fmt_t)cfg.i2s_iface.fmt);
  ret_val |= es8388_set_voice_volume(volume_);
  ret_val |= es8388_set_mic_gain(MIC_GAIN_24DB);
  ret_val |= es8388_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);

  if (ESP_OK != ret_val) {
    logger.error("Failed to initialize es8388 (input/output) codec");
  }

  return ret_val;
}

static bool initialized = false;
void audio_init(std::shared_ptr<espp::I2c> internal_i2c) {
  if (initialized) return;

  // probe the i2c for the es8388 device address
  std::vector<uint8_t> es8388_addrs = {ES8388_ADDR_CE0, ES8388_ADDR_CE1};
  bool can_communicate = false;
  for (auto addr : es8388_addrs) {
    can_communicate = internal_i2c->probe_device(addr);
    if (can_communicate) {
      break;
    }
  }
  fmt::print("Can communicate with es8388: {}\n", can_communicate);

  // /* Config power control IO */
  // gpio_config_t io_conf;
  // io_conf.intr_type = GPIO_INTR_DISABLE;
  // io_conf.mode = GPIO_MODE_OUTPUT;
  // io_conf.pin_bit_mask = 1ULL << (int)sound_power_pin;
  // io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  // io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  // gpio_config(&io_conf);

  // gpio_set_level(sound_power_pin, 1);

  set_es7210_write(std::bind(&espp::I2c::write, internal_i2c.get(),
                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  set_es7210_read(std::bind(&espp::I2c::read_at_register, internal_i2c.get(),
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  set_es8311_write(std::bind(&espp::I2c::write, internal_i2c.get(),
                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  set_es8311_read(std::bind(&espp::I2c::read_at_register, internal_i2c.get(),
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  set_es8388_write(std::bind(&espp::I2c::write, internal_i2c.get(),
                             std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  set_es8388_read(std::bind(&espp::I2c::read_at_register, internal_i2c.get(),
                            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  i2s_driver_init();
  esp_err_t err = ESP_OK;
  // err = es7210_init_default();
  // if (err != ESP_OK) {
  //   logger.error("ERROR initializing ES7210 audio: {}", err);
  //   return;
  // }
  // err = es8311_init_default();
  // if (err != ESP_OK) {
  //   logger.error("ERROR initializing ES8311 audio: {}", err);
  //   return;
  // }
  err = es8388_init_default();
  if (err != ESP_OK) {
    logger.error("ERROR initializing ES8388 audio: {}", err);
    return;
  }

  audio_buffer0 = (int16_t*)heap_caps_malloc(sizeof(int16_t) * AUDIO_BUFFER_SIZE + 10, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);
  audio_buffer1 = (int16_t*)heap_caps_malloc(sizeof(int16_t) * AUDIO_BUFFER_SIZE + 10, MALLOC_CAP_8BIT | MALLOC_CAP_DMA);

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
