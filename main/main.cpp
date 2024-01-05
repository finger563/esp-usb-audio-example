#include <chrono>
#include <thread>

#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"

#include "box_hal.hpp"
#include "i2s_audio.hpp"
#include "usb.hpp"

extern "C" {
#include <tusb.h>
}

#ifndef AUDIO_SAMPLE_RATE
#define AUDIO_SAMPLE_RATE   48000
#endif

// Audio controls
// Current states
bool mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; 				          // +1 for master channel 0
uint16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX + 1]; 					// +1 for master channel 0
uint32_t sample_freq;

// Range states
audio_control_range_2_n_t(1) volumeRng[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_TX+1]; 			// Volume range state
audio_control_range_4_n_t(1) sampleFreqRng; 						// Sample frequency range state

// Audio test data
uint16_t test_buffer_audio[CFG_TUD_AUDIO_EP_SZ_IN/2];
uint16_t startVal = 0;


// List of supported sample rates
static const uint32_t sample_rates[] = {44100, 48000};
static uint32_t current_sample_rate  = 44100;
static uint8_t clk_valid = 1;

#define N_SAMPLE_RATES  TU_ARRAY_SIZE(sample_rates)

// Audio controls
// Current states
// static int8_t mute[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];       // +1 for master channel 0
// static int16_t volume[CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX + 1];    // +1 for master channel 0

// Buffer for microphone data
static int32_t mic_buf[CFG_TUD_AUDIO_FUNC_1_EP_IN_SW_BUF_SZ / 4];
// Buffer for speaker data
static int32_t spk_buf[CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ / 4];
// Speaker data size received in the last frame
static int spk_data_size;
// Resolution per format
// static const uint8_t resolutions_per_format[CFG_TUD_AUDIO_FUNC_1_N_FORMATS] = {CFG_TUD_AUDIO_FUNC_1_FORMAT_1_RESOLUTION_RX};
// Current resolution, update on format change
static uint8_t current_resolution;


using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static auto start = std::chrono::high_resolution_clock::now();
  static auto elapsed = [&]() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float>(now - start).count();
  };

  espp::Logger logger({.tag = "USB Audio Example", .level = espp::Logger::Verbosity::INFO});

  logger.info("Bootup");

  // // initialize the internal i2c bus (for the codecs)
  // auto internal_i2c = std::make_shared<espp::I2c>(espp::I2c::Config{
  //     .port = box_hal::internal_i2c_port,
  //     .sda_io_num = box_hal::internal_i2c_sda,
  //     .scl_io_num = box_hal::internal_i2c_scl,
  //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
  //     .scl_pullup_en = GPIO_PULLUP_ENABLE});

  // // initialize the audio codecs
  // logger.info("Initializing audio");
  // audio_init(internal_i2c);


  // Init values
  sample_freq = AUDIO_SAMPLE_RATE;
  clk_valid = 1;

  sampleFreqRng.wNumSubRanges = 1;
  sampleFreqRng.subrange[0].bMin = AUDIO_SAMPLE_RATE;
  sampleFreqRng.subrange[0].bMax = AUDIO_SAMPLE_RATE;
  sampleFreqRng.subrange[0].bRes = 0;

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



static espp::Logger logger({.tag = "test", .level = espp::Logger::Verbosity::INFO});

//--------------------------------------------------------------------+
// Application Callback API Implementations
//--------------------------------------------------------------------+

// Invoked when audio class specific set request received for an EP
extern "C" bool tud_audio_set_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
  (void) rhport;
  (void) pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void) channelNum; (void) ctrlSel; (void) ep;

  logger.warn("tud_audio_set_req_ep_cb: unimplemented");

  return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an interface
extern "C" bool tud_audio_set_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request, uint8_t *pBuff)
{
  (void) rhport;
  (void) pBuff;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void) channelNum; (void) ctrlSel; (void) itf;

  logger.warn("tud_audio_set_req_itf_cb: unimplemented");

  return false; 	// Yet not implemented
}

// Invoked when audio class specific set request received for an entity

// Invoked when audio class specific get request received for an EP
extern "C" bool tud_audio_get_req_ep_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t ep = TU_U16_LOW(p_request->wIndex);

  (void) channelNum; (void) ctrlSel; (void) ep;

  //	return tud_control_xfer(rhport, p_request, &tmp, 1);

  logger.warn("tud_audio_get_req_ep_cb: unimplemented");

  return false; 	// Yet not implemented
}

// Invoked when audio class specific get request received for an interface
extern "C" bool tud_audio_get_req_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request)
{
  (void) rhport;

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);

  (void) channelNum; (void) ctrlSel; (void) itf;

  logger.warn("tud_audio_get_req_itf_cb: unimplemented");

  return false; 	// Yet not implemented
}

// Application Callback API Implementations

// Invoked when audio class specific get request received for an entity
extern "C" bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request) {
  audio_control_request_t const *request = (audio_control_request_t const *)p_request;
  logger.debug("tud_audio_get_req_entity_cb: entity: {}, channel: {}, control: {}, length: {}",
    request->bEntityID, request->bChannelNumber, request->bControlSelector, request->wLength);
  // TODO: implement

  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  // uint8_t itf = TU_U16_LOW(p_request->wIndex); 			// Since we have only one audio function implemented, we do not need the itf value
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  // Input terminal (Microphone input)
  if (entityID == 1)
  {
    switch ( ctrlSel )
    {
      case AUDIO_TE_CTRL_CONNECTOR:
      {
        // The terminal connector control only has a get request with only the CUR attribute.
        audio_desc_channel_cluster_t ret;
        memset(&ret, 0, sizeof(ret));
        // Those are dummy values for now
        ret.bNrChannels = 1;
        // ret.bmChannelConfig = 0;
        ret.iChannelNames = 0;

        TU_LOG2("    Get terminal connector\r\n");

        return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));
      }
      break;

        // Unknown/Unsupported control selector
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  // Feature unit
  if (entityID == 2)
  {
    switch ( ctrlSel )
    {
      case AUDIO_FU_CTRL_MUTE:
        // Audio control mute cur parameter block consists of only one byte - we thus can send it right away
        // There does not exist a range parameter block for mute
        TU_LOG2("    Get Mute of channel: %u\r\n", channelNum);
        return tud_control_xfer(rhport, p_request, &mute[channelNum], 1);

      case AUDIO_FU_CTRL_VOLUME:
        switch ( p_request->bRequest )
        {
          case AUDIO_CS_REQ_CUR:
            TU_LOG2("    Get Volume of channel: %u\r\n", channelNum);
            return tud_control_xfer(rhport, p_request, &volume[channelNum], sizeof(volume[channelNum]));

          case AUDIO_CS_REQ_RANGE:
            TU_LOG2("    Get Volume range of channel: %u\r\n", channelNum);

            // Copy values - only for testing - better is version below
            audio_control_range_2_n_t(1)
            ret;

            ret.wNumSubRanges = 1;
            ret.subrange[0].bMin = -90;    // -90 dB
            ret.subrange[0].bMax = 90;		// +90 dB
            ret.subrange[0].bRes = 1; 		// 1 dB steps

            return tud_audio_buffer_and_schedule_control_xfer(rhport, p_request, (void*) &ret, sizeof(ret));

            // Unknown/Unsupported control
          default:
            TU_BREAKPOINT();
            return false;
        }
      break;

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  // Clock Source unit
  if ( entityID == 4 )
  {
    switch ( ctrlSel )
    {
      case AUDIO_CS_CTRL_SAM_FREQ:
        // channelNum is always zero in this case
        switch ( p_request->bRequest )
        {
          case AUDIO_CS_REQ_CUR:
            TU_LOG2("    Get Sample Freq.\r\n");
            return tud_control_xfer(rhport, p_request, &sample_freq, sizeof(sample_freq));

          case AUDIO_CS_REQ_RANGE:
            TU_LOG2("    Get Sample Freq. range\r\n");
            return tud_control_xfer(rhport, p_request, &sampleFreqRng, sizeof(sampleFreqRng));

           // Unknown/Unsupported control
          default:
            TU_BREAKPOINT();
            return false;
        }
      break;

      case AUDIO_CS_CTRL_CLK_VALID:
        // Only cur attribute exists for this request
        TU_LOG2("    Get Sample Freq. valid\r\n");
        return tud_control_xfer(rhport, p_request, &clk_valid, sizeof(clk_valid));

      // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
        return false;
    }
  }

  logger.warn("tud_audio_get_req_entity_cb: unimplemented");
  return false;
}


// Invoked when audio class specific set request received for an entity
extern "C" bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request, uint8_t *buf) {
  audio_control_request_t const *request = (audio_control_request_t const *)p_request;


  // Page 91 in UAC2 specification
  uint8_t channelNum = TU_U16_LOW(p_request->wValue);
  uint8_t ctrlSel = TU_U16_HIGH(p_request->wValue);
  uint8_t itf = TU_U16_LOW(p_request->wIndex);
  uint8_t entityID = TU_U16_HIGH(p_request->wIndex);

  (void) itf;

  // We do not support any set range requests here, only current value requests
  TU_VERIFY(p_request->bRequest == AUDIO_CS_REQ_CUR);

  // If request is for our feature unit
  if ( entityID == 2 )
  {
    switch ( ctrlSel )
    {
      case AUDIO_FU_CTRL_MUTE:
        // Request uses format layout 1
        TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_1_t));

        mute[channelNum] = ((audio_control_cur_1_t*) buf)->bCur;

        TU_LOG2("    Set Mute: %d of channel: %u\r\n", mute[channelNum], channelNum);
      return true;

      case AUDIO_FU_CTRL_VOLUME:
        // Request uses format layout 2
        TU_VERIFY(p_request->wLength == sizeof(audio_control_cur_2_t));

        volume[channelNum] = ((audio_control_cur_2_t*) buf)->bCur;

        TU_LOG2("    Set Volume: %d dB of channel: %u\r\n", volume[channelNum], channelNum);
      return true;

        // Unknown/Unsupported control
      default:
        TU_BREAKPOINT();
      return false;
    }
  }

  logger.warn("tud_audio_set_req_entity_cb: unimplemented");
  return false;
}

extern "C" bool tud_audio_set_itf_close_EP_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
  (void)rhport;

  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  startVal = 0;
  logger.debug("tud_audio_set_itf_close_EP_cb");

  // if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt == 0) {
  //   // Audio streaming stop
  //   // TODO: stop audio
  // }

  return true;
}

extern "C" bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const * p_request) {
  (void)rhport;
  uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
  uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

  logger.debug("Set interface {} alt {}", itf, alt);

  // if (ITF_NUM_AUDIO_STREAMING_SPK == itf && alt != 0) {
  //   // Audio streaming start
  //   // TODO: start audio
  // }

  // Clear buffer when streaming format is changed
  spk_data_size = 0;
  // if(alt != 0) {
  //    current_resolution = resolutions_per_format[alt-1];
  // }

  return true;
}

// Optional TinyUSB Audio APIs

extern "C" bool tud_audio_tx_done_pre_load_cb(uint8_t rhport, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  logger.debug("tud_audio_tx_done_pre_load_cb");
  tud_audio_write ((uint8_t *)test_buffer_audio, CFG_TUD_AUDIO_EP_SZ_IN);
  return true;
}

extern "C" bool tud_audio_tx_done_post_load_cb(uint8_t rhport, uint16_t n_bytes_copied, uint8_t func_id, uint8_t ep_in, uint8_t cur_alt_setting) {
  logger.debug("tud_audio_tx_done_post_load_cb");

  for (size_t cnt = 0; cnt < CFG_TUD_AUDIO_EP_SZ_IN/2; cnt++)
  {
    test_buffer_audio[cnt] = startVal++;
  }
  return true;
}

extern "C" bool tud_audio_rx_done_pre_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  logger.debug("tud_audio_rx_done_pre_read_cb");
  return true;
}

extern "C" bool tud_audio_rx_done_post_read_cb(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id, uint8_t ep_out, uint8_t cur_alt_setting) {
  logger.debug("tud_audio_rx_done_post_read_cb");
  return true;
}
