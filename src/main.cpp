#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include "driver/i2s.h"
#include "idpa_protocol.h"

static inline int32_t clampi32(int32_t v, int32_t lo, int32_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline int16_t clip16(int32_t v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

// ================= NETWORK =================
static WiFiUDP udpAudio;
static WiFiUDP udpCmd;
static WiFiUDP udpSensor;

// ================= CONFIG =================
static const char* AP_SSID = IDPA_SSID;
static const char* AP_PASS = IDPA_PASS;

// I2S OUT (UDA1334A)
static constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
static constexpr int PIN_BCK  = 26;
static constexpr int PIN_WS   = 25;
static constexpr int PIN_DOUT = 22;

// Audio format
static constexpr uint32_t FS = 48000;
static constexpr int PACKET_MS = 5;
static constexpr int FRAMES = (FS * PACKET_MS) / 1000; // 240
static constexpr int AUDIO_BYTES = FRAMES * 2 * 2;     // stereo int16
static constexpr int PKT_BYTES   = 8 + AUDIO_BYTES;

// Ring buffer
static constexpr int RING_PKTS = 64;
static uint8_t ring[RING_PKTS][AUDIO_BYTES];

static volatile uint32_t wr = 0;
static volatile uint32_t rd = 0;
static volatile uint32_t prefill = 0;
static volatile bool started = false;

// Volume / channel mask
static volatile int32_t g_gain_q8_8 = 1024; // 1.25x Default etwas niedriger gegen Stoerrauschen
static volatile uint8_t g_ch_mask   = 0x03;

// ================= FILTERED FFT EXPORT =================
static constexpr int FFT_SIZE = 256;
static constexpr int FFT_BINS = FFT_SIZE / 2;
static constexpr uint32_t FFT_SEND_INTERVAL_MS = 80;

static uint16_t fftFilteredLeftBins[FFT_BINS]  = {0};
static uint16_t fftFilteredRightBins[FFT_BINS] = {0};
static int16_t fftRingLeft[FFT_SIZE] = {0};
static int16_t fftRingRight[FFT_SIZE] = {0};
static volatile uint16_t fftRingHead = 0;
static volatile bool fftFrameReady = false;
static volatile uint32_t fftSeq = 1;
static portMUX_TYPE fftMux = portMUX_INITIALIZER_UNLOCKED;

// ================= CMD =================
#ifndef CMD_SET_VOLUME
#define CMD_SET_VOLUME 0x30
#endif

#ifndef CMD_SET_PIEZO_CH_MASK
#define CMD_SET_PIEZO_CH_MASK 0x31
#endif

#ifndef CMD_SET_FILTER_MODE
#define CMD_SET_FILTER_MODE 0x32
#endif

#ifndef CMD_SET_FILTER_LOW_HZ
#define CMD_SET_FILTER_LOW_HZ 0x33
#endif

#ifndef CMD_SET_FILTER_HIGH_HZ
#define CMD_SET_FILTER_HIGH_HZ 0x34
#endif

#ifndef CMD_SET_NOTCH_ENABLE
#define CMD_SET_NOTCH_ENABLE 0x35
#endif

// ================= FILTER =================
enum FilterMode : uint8_t {
  FILTER_BYPASS   = 0,
  FILTER_HIGHPASS = 1,
  FILTER_LOWPASS  = 2,
  FILTER_BANDPASS = 3,
};

struct Biquad {
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
  float a1 = 0.0f, a2 = 0.0f;
  float z1 = 0.0f, z2 = 0.0f;
};

static Biquad filt1L, filt1R;
static Biquad filt2L, filt2R;
static Biquad notchL, notchR;

static volatile uint8_t  g_filter_mode   = FILTER_BANDPASS;
static volatile int32_t  g_filter_low_hz  = 120;
static volatile int32_t  g_filter_high_hz = 800;
static volatile uint8_t  g_notch_enable   = 1;

static inline void biquad_reset(Biquad& f)
{
  f.z1 = 0.0f;
  f.z2 = 0.0f;
}

static inline int16_t biquad_process(Biquad& f, int16_t x)
{
  const float xf = (float)x;

  float y = f.b0 * xf + f.z1;
  f.z1 = f.b1 * xf - f.a1 * y + f.z2;
  f.z2 = f.b2 * xf - f.a2 * y;

  if (y >  32767.0f) y =  32767.0f;
  if (y < -32768.0f) y = -32768.0f;
  return (int16_t)y;
}

static void fft_copy_frame(int16_t* leftOut, int16_t* rightOut)
{
  portENTER_CRITICAL(&fftMux);
  const uint16_t head = fftRingHead;
  for (int i = 0; i < FFT_SIZE; i++) {
    const uint16_t idx = (uint16_t)((head + i) % FFT_SIZE);
    leftOut[i] = fftRingLeft[idx];
    rightOut[i] = fftRingRight[idx];
  }
  fftFrameReady = false;
  portEXIT_CRITICAL(&fftMux);
}

static void fft_compute(const int16_t* samples, float* mags, uint16_t& peakBin)
{
  static float re[FFT_SIZE];
  static float im[FFT_SIZE];
  static float win[FFT_SIZE];
  static bool winInit = false;

  if (!winInit) {
    for (int i = 0; i < FFT_SIZE; i++) {
      win[i] = 0.5f * (1.0f - cosf((2.0f * PI * i) / (FFT_SIZE - 1)));
    }
    winInit = true;
  }

  for (int i = 0; i < FFT_SIZE; i++) {
    re[i] = (float)samples[i] * win[i];
    im[i] = 0.0f;
  }

  unsigned j = 0;
  for (unsigned i = 1; i < FFT_SIZE; i++) {
    unsigned bit = FFT_SIZE >> 1;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j ^= bit;
    if (i < j) {
      const float tr = re[i]; re[i] = re[j]; re[j] = tr;
      const float ti = im[i]; im[i] = im[j]; im[j] = ti;
    }
  }

  for (unsigned len = 2; len <= FFT_SIZE; len <<= 1) {
    const float ang = -2.0f * PI / (float)len;
    const float wlenCos = cosf(ang);
    const float wlenSin = sinf(ang);

    for (unsigned i = 0; i < FFT_SIZE; i += len) {
      float wCos = 1.0f;
      float wSin = 0.0f;
      for (unsigned k = 0; k < len / 2; k++) {
        const unsigned u = i + k;
        const unsigned v = i + k + len / 2;

        const float vr = re[v] * wCos - im[v] * wSin;
        const float vi = re[v] * wSin + im[v] * wCos;

        re[v] = re[u] - vr;
        im[v] = im[u] - vi;
        re[u] = re[u] + vr;
        im[u] = im[u] + vi;

        const float nextCos = wCos * wlenCos - wSin * wlenSin;
        const float nextSin = wCos * wlenSin + wSin * wlenCos;
        wCos = nextCos;
        wSin = nextSin;
      }
    }
  }

  float maxMag = 0.0f;
  peakBin = 0;
  for (int i = 0; i < FFT_BINS; i++) {
    const float m = sqrtf(re[i] * re[i] + im[i] * im[i]);
    mags[i] = m;
    if (i > 0 && m > maxMag) {
      maxMag = m;
      peakBin = (uint16_t)i;
    }
  }
}

static void fft_send_filtered_packet()
{
  static int16_t leftFrame[FFT_SIZE];
  static int16_t rightFrame[FFT_SIZE];
  static float magsL[FFT_BINS];
  static float magsR[FFT_BINS];

  fft_copy_frame(leftFrame, rightFrame);

  uint16_t peakBinL = 0;
  uint16_t peakBinR = 0;
  fft_compute(leftFrame, magsL, peakBinL);
  fft_compute(rightFrame, magsR, peakBinR);

  float maxMag = 1.0f;
  for (int i = 0; i < FFT_BINS; i++) {
    if (magsL[i] > maxMag) maxMag = magsL[i];
    if (magsR[i] > maxMag) maxMag = magsR[i];
  }

  for (int i = 0; i < FFT_BINS; i++) {
    fftFilteredLeftBins[i]  = (uint16_t)((magsL[i] / maxMag) * 60000.0f);
    fftFilteredRightBins[i] = (uint16_t)((magsR[i] / maxMag) * 60000.0f);
  }

  PiezoFftPacketV1 p{};
  p.header.magic = PKT_MAGIC;
  p.header.version = 1;
  p.header.type = PKT_TYPE_PIEZO_FFT_FILTERED;
  p.header.seq = fftSeq++;
  p.header.t_ms = millis();
  p.header.payload_len = sizeof(PiezoFftPacketV1) - sizeof(PacketHeaderV1);
  p.binCount = FFT_BINS;
  p.reserved = 0;
  p.peakHzLeft = ((float)peakBinL * (float)FS) / (float)FFT_SIZE;
  p.peakHzRight = ((float)peakBinR * (float)FS) / (float)FFT_SIZE;
  memcpy(p.binsLeft, fftFilteredLeftBins, sizeof(fftFilteredLeftBins));
  memcpy(p.binsRight, fftFilteredRightBins, sizeof(fftFilteredRightBins));

  udpSensor.beginPacket(IP_DISPLAY, UDP_SENSOR_PORT);
  udpSensor.write((const uint8_t*)&p, sizeof(p));
  udpSensor.endPacket();
}

static void biquad_set_identity(Biquad& f)
{
  f.b0 = 1.0f; f.b1 = 0.0f; f.b2 = 0.0f;
  f.a1 = 0.0f; f.a2 = 0.0f;
  biquad_reset(f);
}

static void biquad_init_lowpass(Biquad& f, float fs, float f0, float Q)
{
  const float w0 = 2.0f * PI * f0 / fs;
  const float c  = cosf(w0);
  const float s  = sinf(w0);
  const float alpha = s / (2.0f * Q);

  const float b0 = (1.0f - c) * 0.5f;
  const float b1 = 1.0f - c;
  const float b2 = (1.0f - c) * 0.5f;
  const float a0 = 1.0f + alpha;
  const float a1 = -2.0f * c;
  const float a2 = 1.0f - alpha;

  f.b0 = b0 / a0;
  f.b1 = b1 / a0;
  f.b2 = b2 / a0;
  f.a1 = a1 / a0;
  f.a2 = a2 / a0;
  biquad_reset(f);
}

static void biquad_init_highpass(Biquad& f, float fs, float f0, float Q)
{
  const float w0 = 2.0f * PI * f0 / fs;
  const float c  = cosf(w0);
  const float s  = sinf(w0);
  const float alpha = s / (2.0f * Q);

  const float b0 = (1.0f + c) * 0.5f;
  const float b1 = -(1.0f + c);
  const float b2 = (1.0f + c) * 0.5f;
  const float a0 = 1.0f + alpha;
  const float a1 = -2.0f * c;
  const float a2 = 1.0f - alpha;

  f.b0 = b0 / a0;
  f.b1 = b1 / a0;
  f.b2 = b2 / a0;
  f.a1 = a1 / a0;
  f.a2 = a2 / a0;
  biquad_reset(f);
}

static void biquad_init_notch(Biquad& f, float fs, float f0, float Q)
{
  const float w0 = 2.0f * PI * f0 / fs;
  const float c  = cosf(w0);
  const float s  = sinf(w0);
  const float alpha = s / (2.0f * Q);

  const float b0 = 1.0f;
  const float b1 = -2.0f * c;
  const float b2 = 1.0f;
  const float a0 = 1.0f + alpha;
  const float a1 = -2.0f * c;
  const float a2 = 1.0f - alpha;

  f.b0 = b0 / a0;
  f.b1 = b1 / a0;
  f.b2 = b2 / a0;
  f.a1 = a1 / a0;
  f.a2 = a2 / a0;
  biquad_reset(f);
}

static void update_filter_coeffs()
{
  const float fs = (float)FS;
  const float qMain = 0.707f;
  const float qNotch = 10.0f;

  int32_t lowHz  = clampi32(g_filter_low_hz, 20, (int32_t)(FS / 2 - 200));
  int32_t highHz = clampi32(g_filter_high_hz, 40, (int32_t)(FS / 2 - 100));
  if (highHz <= lowHz + 20) highHz = lowHz + 20;
  if (highHz > (int32_t)(FS / 2 - 100)) highHz = (int32_t)(FS / 2 - 100);

  g_filter_low_hz = lowHz;
  g_filter_high_hz = highHz;

  biquad_set_identity(filt1L);
  biquad_set_identity(filt1R);
  biquad_set_identity(filt2L);
  biquad_set_identity(filt2R);

  switch (g_filter_mode) {
    case FILTER_HIGHPASS:
      biquad_init_highpass(filt1L, fs, (float)lowHz, qMain);
      biquad_init_highpass(filt1R, fs, (float)lowHz, qMain);
      break;

    case FILTER_LOWPASS:
      biquad_init_lowpass(filt1L, fs, (float)highHz, qMain);
      biquad_init_lowpass(filt1R, fs, (float)highHz, qMain);
      break;

    case FILTER_BANDPASS:
      biquad_init_highpass(filt1L, fs, (float)lowHz, qMain);
      biquad_init_highpass(filt1R, fs, (float)lowHz, qMain);
      biquad_init_lowpass(filt2L, fs, (float)highHz, qMain);
      biquad_init_lowpass(filt2R, fs, (float)highHz, qMain);
      break;

    case FILTER_BYPASS:
    default:
      break;
  }

  biquad_init_notch(notchL, fs, 90.0f, qNotch);
  biquad_init_notch(notchR, fs, 90.0f, qNotch);
}

static inline void process_audio_block(uint8_t* block)
{
  int16_t* s = (int16_t*)block;
  const bool enL = (g_ch_mask & 0x01) != 0;
  const bool enR = (g_ch_mask & 0x02) != 0;
  const int32_t g = g_gain_q8_8;
  const uint8_t mode = g_filter_mode;
  const bool notchEn = g_notch_enable != 0;

  for (int i = 0; i < FRAMES; i++) {
    const int idxL = 2 * i;
    const int idxR = 2 * i + 1;

    int16_t vL = enL ? s[idxL] : 0;
    int16_t vR = enR ? s[idxR] : 0;

    switch (mode) {
      case FILTER_HIGHPASS:
        vL = biquad_process(filt1L, vL);
        vR = biquad_process(filt1R, vR);
        break;

      case FILTER_LOWPASS:
        vL = biquad_process(filt1L, vL);
        vR = biquad_process(filt1R, vR);
        break;

      case FILTER_BANDPASS:
        vL = biquad_process(filt1L, vL);
        vL = biquad_process(filt2L, vL);
        vR = biquad_process(filt1R, vR);
        vR = biquad_process(filt2R, vR);
        break;

      case FILTER_BYPASS:
      default:
        break;
    }

    if (notchEn) {
      vL = biquad_process(notchL, vL);
      vR = biquad_process(notchR, vR);
    }

    int32_t oL = ((int32_t)vL * g) >> 8;
    int32_t oR = ((int32_t)vR * g) >> 8;

    s[idxL] = clip16(oL);
    s[idxR] = clip16(oR);
  }
}


static void handle_cmd(uint8_t cmd, int16_t value)
{
  switch (cmd) {
    case CMD_SET_VOLUME: {
      const int v = (int)clampi32((int)value, 0, 100);
      // 100% = 1.0x
      g_gain_q8_8 = (256 * v) / 100;
    } break;

    case CMD_SET_PIEZO_CH_MASK: {
      g_ch_mask = ((uint8_t)value) & 0x03;
      if ((g_ch_mask & 0x03) == 0) g_ch_mask = 0x03;
    } break;

    case CMD_SET_FILTER_MODE: {
      const uint8_t m = (uint8_t)clampi32((int32_t)value, 0, 3);
      g_filter_mode = m;
      update_filter_coeffs();
    } break;

    case CMD_SET_FILTER_LOW_HZ: {
      g_filter_low_hz = clampi32((int32_t)value, 20, (int32_t)(FS / 2 - 200));
      update_filter_coeffs();
    } break;

    case CMD_SET_FILTER_HIGH_HZ: {
      g_filter_high_hz = clampi32((int32_t)value, 40, (int32_t)(FS / 2 - 100));
      update_filter_coeffs();
    } break;

    case CMD_SET_NOTCH_ENABLE: {
      g_notch_enable = (value != 0) ? 1 : 0;
    } break;

    default:
      break;
  }
}

// ================= TASKS =================
static void cmd_task(void*)
{
  CmdPacketV1 p{};

  while (true) {
    const int ps = udpCmd.parsePacket();
    if (ps <= 0) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    if (ps < (int)sizeof(CmdPacketV1)) {
      udpCmd.flush();
      continue;
    }

    const int len = udpCmd.read((uint8_t*)&p, sizeof(p));
    if (len != (int)sizeof(p)) continue;
    if (p.magic != CMD_MAGIC) continue;

    handle_cmd(p.cmd, p.value);
  }
}

static void audio_rx_task(void*)
{
  uint8_t pkt[PKT_BYTES];

  while (true) {
    const int ps = udpAudio.parsePacket();
    if (ps <= 0) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    if (ps != PKT_BYTES) {
      udpAudio.flush();
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    const int len = udpAudio.read(pkt, PKT_BYTES);
    if (len != PKT_BYTES) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    const uint16_t magic = (uint16_t)pkt[4] | ((uint16_t)pkt[5] << 8);
    if (magic != AUDIO_MAGIC) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    const uint32_t nextWr = (wr + 1) % RING_PKTS;
    if (nextWr == rd) {
      // Ring voll -> newest packet droppen
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    memcpy(ring[wr], pkt + 8, AUDIO_BYTES);
    wr = nextWr;

    if (!started) {
      if (prefill < 1000000UL) prefill++;
      if (prefill >= 20) started = true;
    }

    taskYIELD();
  }
}

static void audio_play_task(void*)
{
  static uint8_t silence[AUDIO_BYTES] = {0};

  while (true) {
    if (!started) {
      size_t out = 0;
      i2s_write(I2S_PORT, silence, AUDIO_BYTES, &out, portMAX_DELAY);
      continue;
    }

    if (rd == wr) {
      // Underrun -> Stille
      size_t out = 0;
      i2s_write(I2S_PORT, silence, AUDIO_BYTES, &out, portMAX_DELAY);
      continue;
    }

    uint8_t local[AUDIO_BYTES];
    memcpy(local, ring[rd], AUDIO_BYTES);
    rd = (rd + 1) % RING_PKTS;

    process_audio_block(local);

    int16_t* s = (int16_t*)local;
    portENTER_CRITICAL(&fftMux);
    for (int i = 0; i < FRAMES; i++) {
      fftRingLeft[fftRingHead] = s[2 * i + 0];
      fftRingRight[fftRingHead] = s[2 * i + 1];
      fftRingHead = (uint16_t)((fftRingHead + 1) % FFT_SIZE);
    }
    fftFrameReady = true;
    portEXIT_CRITICAL(&fftMux);

    size_t out = 0;
    i2s_write(I2S_PORT, local, AUDIO_BYTES, &out, portMAX_DELAY);
  }
}
static void fft_task(void*)
{
  uint32_t lastSend = 0;
  while (true) {
    const uint32_t now = millis();
    if (fftFrameReady && (now - lastSend >= FFT_SEND_INTERVAL_MS)) {
      lastSend = now;
      fft_send_filtered_packet();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.softAPConfig(IP_AP, IP_AP, IP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASS);
  esp_wifi_set_max_tx_power(78);

  udpAudio.begin(UDP_AUDIO_PORT);
  udpCmd.begin(UDP_CMD_PORT);
  udpSensor.begin(UDP_SENSOR_PORT);

  i2s_config_t cfg{};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = FS;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.dma_buf_count = 10;
  cfg.dma_buf_len = FRAMES;
  cfg.use_apll = true;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);

  i2s_pin_config_t pins{};
  pins.bck_io_num   = (gpio_num_t)PIN_BCK;
  pins.ws_io_num    = (gpio_num_t)PIN_WS;
  pins.data_out_num = (gpio_num_t)PIN_DOUT;
  pins.data_in_num  = I2S_PIN_NO_CHANGE;

  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);

  update_filter_coeffs();

  xTaskCreatePinnedToCore(audio_rx_task,   "audio_rx",   4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(audio_play_task, "audio_play", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(cmd_task,        "cmd_task",   3072, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(fft_task,        "fft_task",   6144, nullptr, 1, nullptr, 0);
}

void loop()
{
}