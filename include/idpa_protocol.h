#pragma once
#include <stdint.h>
#include <IPAddress.h>

static const char* IDPA_SSID = "IDPA_NET";
static const char* IDPA_PASS = "12345678";

#define CMD_SET_VOLUME 0x30
#define CMD_SET_PIEZO_CH_MASK 0x31

static const IPAddress IP_AP      (192,168,4,1);
static const IPAddress IP_SENDER  (192,168,4,2);
static const IPAddress IP_DISPLAY (192,168,4,3);
static const IPAddress IP_SUBNET  (255,255,255,0);

static constexpr uint16_t UDP_AUDIO_PORT  = 3333;
static constexpr uint16_t UDP_SENSOR_PORT = 5005;
static constexpr uint16_t UDP_CMD_PORT    = 5006;

// Legacy FFT data stream (Empfänger -> Display)
static constexpr uint16_t UDP_FFT_PORT    = 4444;
static constexpr uint16_t FFT_MAGIC       = 0xF17A;
// maximum number of bins that can be included in stream
static constexpr int      FFT_MAX_BINS    = 1024;

// New piezo FFT streams
static constexpr uint8_t  PKT_TYPE_PIEZO_FFT_RAW      = 0x12; // Sender -> Display
static constexpr uint8_t  PKT_TYPE_PIEZO_FFT_FILTERED = 0x13; // Empfänger -> Display
static constexpr uint16_t PIEZO_FFT_BINS_TX           = 128;

#pragma pack(push, 1)

// ---------------- Sensor (dein Format) ----------------
enum : uint16_t { PKT_MAGIC = 0xA55A };
enum : uint8_t  { PKT_TYPE_IMU = 1, PKT_TYPE_ENV = 2, PKT_TYPE_USER = 3 };

struct PacketHeaderV1 {
  uint16_t magic;       // 0xA55A
  uint8_t  version;     // 1
  uint8_t  type;        // PKT_TYPE_*
  uint32_t seq;         // laufend
  uint32_t t_ms;        // millis() am Sender
  uint16_t payload_len; // bytes nach header
};

using PktHeader = PacketHeaderV1;

struct PayloadIMUv1 {
  uint8_t  espId;
  float    ax;
  float    ay;
  float    az;
};

struct SensorPacketV1 {
  PacketHeaderV1 header;
  PayloadIMUv1 imu;     // type=PKT_TYPE_IMU
};

// ---------------- Piezo FFT (Sender/Empfänger -> Display) ----------------
struct PiezoFftPacketV1 {
  PacketHeaderV1 header;
  uint16_t binCount;
  uint16_t reserved;
  float    peakHzLeft;
  float    peakHzRight;
  uint16_t binsLeft[PIEZO_FFT_BINS_TX];
  uint16_t binsRight[PIEZO_FFT_BINS_TX];
};

// ---------------- Commands (Display -> Empfänger) ----------------
// Erweiterbar: cmd + value + optional payload später.
enum : uint32_t { CMD_MAGIC = 0x31444D43 }; // 'CMD1'
enum : uint8_t  { CMD_OPEN_SCREEN = 1, CMD_SET_GAIN = 2, CMD_SET_MODE = 3 };

struct CmdPacketV1 {
  uint32_t magic;   // 'CMD1'
  uint32_t seq;
  uint8_t  cmd;
  int16_t  value;   // param
  uint8_t  reserved;
};

// ---------------- Audio (Sender -> Empfänger) ----------------
// Format: [seq u32][magic u16=0xA55A][flags u16][PCM...]
// PCM: int16 stereo interleaved
static constexpr uint16_t AUDIO_MAGIC = 0xA55A;

// ---------------- Legacy FFT (Empfänger -> Display) ----------------
#pragma pack(push, 1)
struct FftPacket {
  uint16_t magic;       // FFT_MAGIC
  uint32_t seq;
  float    peakHz;
  uint16_t bins[FFT_MAX_BINS];
};
#pragma pack(pop)

#pragma pack(pop)