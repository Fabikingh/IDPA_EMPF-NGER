#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include "idpa_protocol.h"

// ================= NETWORK =================
static WiFiUDP udpAudio;
static WiFiUDP udpCmd;

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
static volatile int32_t g_gain_q8_8 = 256; // 1.0x
static volatile uint8_t g_ch_mask   = 0x03;

// ================= CMD =================
#ifndef CMD_SET_VOLUME
#define CMD_SET_VOLUME 0x30
#endif

#ifndef CMD_SET_PIEZO_CH_MASK
#define CMD_SET_PIEZO_CH_MASK 0x31
#endif

// ================= HELPERS =================
static inline int16_t clip16(int32_t v)
{
  if (v >  32767) return  32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static inline int32_t clampi32(int32_t v, int32_t lo, int32_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ================= FILTER =================
struct HP1 {
  float x1 = 0.0f;
  float y1 = 0.0f;
};

struct LP1 {
  float y = 0.0f;
};

struct BiquadNotch {
  float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
  float a1 = 0.0f, a2 = 0.0f;
  float z1 = 0.0f, z2 = 0.0f;
};

static HP1 hpL, hpR;
static LP1 lpL, lpR;
static BiquadNotch notchL, notchR;

// ---------- HP1 ----------
// y[n] = x[n] - x[n-1] + R * y[n-1]
// R nahe 1.0 = sehr sanfter Hochpass
static constexpr float HP_R = 0.9980f;

static inline int16_t hp_process(HP1& f, int16_t x)
{
  float xf = (float)x;
  float y = xf - f.x1 + HP_R * f.y1;

  f.x1 = xf;
  f.y1 = y;

  if (y >  32767.0f) y =  32767.0f;
  if (y < -32768.0f) y = -32768.0f;
  return (int16_t)y;
}

// ---------- LP1 ----------
// Einfacher 1-Pol-Tiefpass
// kleineres alpha = stärkere Glättung
static constexpr float LP_ALPHA = 0.07f;

static inline int16_t lp_process(LP1& f, int16_t x)
{
  f.y += LP_ALPHA * ((float)x - f.y);

  if (f.y >  32767.0f) f.y =  32767.0f;
  if (f.y < -32768.0f) f.y = -32768.0f;
  return (int16_t)f.y;
}

// ---------- NOTCH ----------
static void notch_init(BiquadNotch& f, float fs, float f0, float Q)
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
  f.z1 = 0.0f;
  f.z2 = 0.0f;
}

static inline int16_t notch_process(BiquadNotch& f, int16_t x)
{
  const float xf = (float)x;

  float y = f.b0 * xf + f.z1;
  f.z1 = f.b1 * xf - f.a1 * y + f.z2;
  f.z2 = f.b2 * xf - f.a2 * y;

  if (y >  32767.0f) y =  32767.0f;
  if (y < -32768.0f) y = -32768.0f;
  return (int16_t)y;
}
static inline void process_audio_block(uint8_t* block)
{
  int16_t* s = (int16_t*)block;
  const bool enL = (g_ch_mask & 0x01) != 0;
  const bool enR = (g_ch_mask & 0x02) != 0;
  const int32_t g = g_gain_q8_8;

  for (int i = 0; i < FRAMES; i++) {
    const int idxL = 2 * i;
    const int idxR = 2 * i + 1;

    int16_t vL = enL ? s[idxL] : 0;
    int16_t vR = enR ? s[idxR] : 0;

    // 1) sanfter Hochpass gegen Drift / tieffrequentes Rumpeln
    vL = hp_process(hpL, vL);
    vR = hp_process(hpR, vR);

    // 2) schmaler Notch gegen den Peak bei ~90 Hz
    vL = notch_process(notchL, vL);
    vR = notch_process(notchR, vR);

    // 3) sanfter Lowpass gegen oberes Rauschen
    vL = lp_process(lpL, vL);
    vR = lp_process(lpR, vR);

    // 4) Gain immer zuletzt
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
      if (prefill >= 12) started = true;
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

    size_t out = 0;
    i2s_write(I2S_PORT, local, AUDIO_BYTES, &out, portMAX_DELAY);
  }
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAPConfig(IP_AP, IP_AP, IP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASS);

  udpAudio.begin(UDP_AUDIO_PORT);
  udpCmd.begin(UDP_CMD_PORT);

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

  notch_init(notchL, (float)FS, 90.0f, 8.0f);
  notch_init(notchR, (float)FS, 90.0f, 8.0f);

  xTaskCreatePinnedToCore(audio_rx_task,   "audio_rx",   4096, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(audio_play_task, "audio_play", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(cmd_task,        "cmd_task",   3072, nullptr, 2, nullptr, 1);
}

void loop()
{
}