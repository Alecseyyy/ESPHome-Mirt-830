#pragma once
// =============================================================================
// Mirtek CC1101 ESPHome Component — ESPHome 2026.4.x
// Протокол: Star v1.20 (Mirtek Star 104 / Star 304)
// =============================================================================
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <SPI.h>
#include <vector>
#include <cmath>
#include <cstdio>

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek";

// ─── CRC8 poly 0xA9 ──────────────────────────────────────────────────────────
static uint8_t crc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = d[i];
    for (int j = 0; j < 8; j++) {
      c = ((b ^ c) & 0x80) ? (uint8_t)((c << 1) ^ 0xA9) : (uint8_t)(c << 1);
      b <<= 1;
    }
  }
  return c;
}

// ─── CC1101 registers / strobes ──────────────────────────────────────────────
static const uint8_t CC_SRES  = 0x30, CC_SCAL  = 0x33, CC_SRX   = 0x34;
static const uint8_t CC_STX   = 0x35, CC_SIDLE = 0x36, CC_SFRX  = 0x3A;
static const uint8_t CC_SFTX  = 0x3B, CC_BURST = 0x40, CC_READ  = 0x80;
static const uint8_t CC_TXFIFO = 0x3F, CC_RXFIFO = 0x3F;
static const uint8_t CC_RXBYTES = 0x3B, CC_PARTNUM = 0x30, CC_VERSION = 0x31;
static const uint8_t CC_PA_TABLE = 0x3E;

// RF settings for 433 MHz / FSK / Mirtek protocol
static const uint8_t RF_CFG[] = {
  0x0D,0x2E,0x06,0x4F,0xD3,0x91,0x3C,0x00,
  0x41,0x00,0x16,0x0F,0x00,0x10,0x8B,0x54,
  0xD9,0x83,0x13,0xD2,0xAA,0x31,0x07,0x0C,
  0x08,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,
  0xF8,0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,
  0x00,0x59,0x59,0x3F,0x81,0x35,0x09
};

// ─── Sensor index constants (match __init__.py SENSORS list order) ────────────
enum SensorIdx {
  SI_SUM=0, SI_T1, SI_T2, SI_T3,
  SI_SUM_R, SI_T1_R, SI_T2_R, SI_T3_R,
  SI_KW, SI_KVAR, SI_FREQ, SI_COS,
  SI_V1, SI_V2, SI_V3,
  SI_I1, SI_I2, SI_I3,
  SI_PA, SI_PB, SI_PC,
  SI_QA, SI_QB, SI_QC,
  SI_SA, SI_SB, SI_SC,
  SI_CA, SI_CB, SI_CC,
  SI_TEMP, SI_BAT,
  SI_COUNT
};
enum TextIdx  { TI_TARIFF=0, TI_RELAY, TI_SEAL, TI_TYPE, TI_FW, TI_DATE, TI_TIME, TI_WORK, TI_SYNC, TI_SERIAL, TI_ABON, TI_STATUS, TI_COUNT };
enum BinIdx   { BI_3PHASE=0, BI_RELAY, BI_SEAL_OK, BI_CC1101, BI_COUNT };

// ─── Component class ─────────────────────────────────────────────────────────
class MirtekCC1101 : public PollingComponent {
 public:
  // Called by ESPHome code-gen (from __init__.py to_code)
  void set_cs_pin(GPIOPin *p)   { cs_   = p; }
  void set_gdo0_pin(GPIOPin *p) { gdo0_ = p; }
  void set_meter_address(int a) { addr_ = (uint16_t)a; }
  void set_sensor(int i, sensor::Sensor *s)             { if (i < SI_COUNT)  ss_[i] = s; }
  void set_text_sensor(int i, text_sensor::TextSensor *s){ if (i < TI_COUNT)  ts_[i] = s; }
  void set_binary_sensor(int i, binary_sensor::BinarySensor *s){ if (i < BI_COUNT) bs_[i] = s; }

  // ── Setup ──────────────────────────────────────────────────────────────────
  void setup() override {
    ESP_LOGI(TAG, "Init CC1101 addr=%u", addr_);
    cs_->setup();
    cs_->digital_write(true);
    if (gdo0_) gdo0_->setup();

    SPI.begin();
    SPI.setFrequency(4000000);
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);

    bool ok = cc_init_();
    pub_bin_(BI_CC1101, ok);
    pub_txt_(TI_STATUS, ok ? "CC1101 OK" : "CC1101 ERR");
    if (!ok) ESP_LOGE(TAG, "CC1101 not found! Check SPI wiring.");
    else      ESP_LOGI(TAG, "CC1101 ready, polling every %us", (unsigned)(get_update_interval()/1000));
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Mirtek CC1101 Gateway:");
    ESP_LOGCONFIG(TAG, "  Meter address : %u", addr_);
    ESP_LOGCONFIG(TAG, "  Poll interval : %u ms", get_update_interval());
    LOG_PIN("  CS   pin: ", cs_);
    LOG_PIN("  GDO0 pin: ", gdo0_);
  }

  // ── Poll (called on update_interval) ─────────────────────────────────────
  void update() override { poll_all(); }

  void poll_all() {
    ESP_LOGI(TAG, "=== Poll addr=%u ===", addr_);
    poll_count_++;

    // 1. Тип + дата/время (0x1C)
    bool ok1 = do_cmd_(0x1C, -1, -1, 3) && parse_datetime_();
    // 2. Энергия (0x05, sub=0x04)
    bool ok2 = do_cmd_(0x05, 0x04, -1, 4) && parse_energy_();
    // 3. Мгновенные сумма (0x2B, sub=0x00)
    bool ok3 = do_cmd_(0x2B, 0x00, -1, 4) && parse_instant_();
    // 4. Мгновенные по фазам (0x2B, sub=0x10) — только 3ф
    bool ok4 = true;
    if (three_phase_) ok4 = do_cmd_(0x2B, 0x10, -1, 4) && parse_phase_();
    // 5. Расширенная инфо (0x01) — раз в час
    if (poll_count_ % 12 == 1) do_cmd_(0x01, -1, -1, 5) && parse_ident_();
    // 6. Батарейка (0x1E) — раз в час
    if (poll_count_ % 12 == 1) do_cmd_(0x1E, -1, -1, 3) && parse_battery_();

    bool all_ok = ok1 && ok2 && ok3 && ok4;
    pub_txt_(TI_STATUS, all_ok ? "OK" : "PARTIAL");
    ESP_LOGI(TAG, "=== Done (%s) ===", all_ok ? "OK" : "partial");
  }

  void relay_on()  { send_relay_(0x08); }
  void relay_off() { send_relay_(0x09); }

 protected:
  GPIOPin *cs_{nullptr}, *gdo0_{nullptr};
  uint16_t addr_{1};
  bool three_phase_{true};
  uint32_t poll_count_{0};

  sensor::Sensor        *ss_[SI_COUNT]{};
  text_sensor::TextSensor *ts_[TI_COUNT]{};
  binary_sensor::BinarySensor *bs_[BI_COUNT]{};

  uint8_t sbuf_[20]{}, rbuf_[64]{};
  size_t  rlen_{0};

  // ── CC1101 low-level ──────────────────────────────────────────────────────
  void cc_cs_(bool v) { cs_->digital_write(v); delayMicroseconds(2); }

  void cc_strobe_(uint8_t cmd) {
    cc_cs_(false); SPI.transfer(cmd); cc_cs_(true);
  }
  void cc_wreg_(uint8_t addr, uint8_t val) {
    cc_cs_(false); SPI.transfer(addr & 0x3F); SPI.transfer(val); cc_cs_(true);
  }
  void cc_wburst_(uint8_t addr, const uint8_t *d, size_t n) {
    cc_cs_(false); SPI.transfer((addr & 0x3F) | CC_BURST);
    for (size_t i = 0; i < n; i++) SPI.transfer(d[i]);
    cc_cs_(true);
  }
  uint8_t cc_rreg_(uint8_t addr) {
    cc_cs_(false); SPI.transfer(CC_READ | (addr & 0x3F)); uint8_t v = SPI.transfer(0); cc_cs_(true);
    return v;
  }
  uint8_t cc_rstat_(uint8_t addr) {
    cc_cs_(false); SPI.transfer(CC_READ | CC_BURST | (addr & 0x3F)); uint8_t v = SPI.transfer(0); cc_cs_(true);
    return v;
  }
  bool cc_init_() {
    cc_cs_(true); delayMicroseconds(5);
    cc_cs_(false); delayMicroseconds(10);
    cc_cs_(true); delayMicroseconds(41);
    cc_strobe_(CC_SRES); delay(10);
    cc_wburst_(0x00, RF_CFG, sizeof(RF_CFG));
    cc_strobe_(CC_SCAL); delay(2);
    cc_strobe_(CC_SFRX); cc_strobe_(CC_SFTX); cc_strobe_(CC_SRX);
    uint8_t ver = cc_rstat_(CC_VERSION);
    ESP_LOGD(TAG, "CC1101 version=0x%02X", ver);
    return (ver == 0x14 || ver == 0x04);
  }

  // ── Byte stuffing ─────────────────────────────────────────────────────────
  void stuff_(const uint8_t *in, size_t n, std::vector<uint8_t> &out) {
    out.push_back(in[0]); out.push_back(in[1]); out.push_back(in[2]);
    for (size_t i = 3; i < n - 1; i++) {
      if      (in[i] == 0x55) { out.push_back(0x73); out.push_back(0x11); }
      else if (in[i] == 0x73) { out.push_back(0x73); out.push_back(0x22); }
      else                      out.push_back(in[i]);
    }
    out.push_back(in[n - 1]);
  }
  bool destuff_(const uint8_t *in, size_t n, std::vector<uint8_t> &out) {
    for (size_t i = 0; i < n; i++) {
      if (in[i] == 0x73 && i + 1 < n) {
        if      (in[i+1] == 0x11) { out.push_back(0x55); i++; }
        else if (in[i+1] == 0x22) { out.push_back(0x73); i++; }
        else                         out.push_back(in[i]);
      } else { out.push_back(in[i]); }
    }
    return true;
  }

  // ── Build request packet ──────────────────────────────────────────────────
  // Returns raw length (including stop byte)
  size_t build_pkt_(uint8_t cmd, int sub1, int sub2) {
    uint8_t *b = sbuf_;
    uint8_t data_len = (sub1 < 0) ? 0 : (sub2 < 0) ? 1 : 2;
    b[0] = 0x0F + data_len;          // total len
    b[1] = 0x73; b[2] = 0x55;        // start
    b[3] = 0x20 + data_len;          // Params
    b[4] = 0x00;
    b[5] = addr_ & 0xFF; b[6] = (addr_ >> 8) & 0xFF;
    b[7] = 0xFE; b[8] = 0xFF;
    b[9] = cmd;
    b[10] = b[11] = b[12] = b[13] = 0x00; // PIN
    size_t pos = 14;
    if (sub1 >= 0) b[pos++] = (uint8_t)sub1;
    if (sub2 >= 0) b[pos++] = (uint8_t)sub2;
    b[pos] = crc8(b + 3, pos - 3);   // CRC
    b[pos + 1] = 0x55;               // stop
    return pos + 2;
  }

  // ── Send & receive ────────────────────────────────────────────────────────
  bool do_cmd_(uint8_t cmd, int sub1, int sub2, int expected_pkts) {
    size_t raw_len = build_pkt_(cmd, sub1, sub2);
    std::vector<uint8_t> stuffed;
    stuff_(sbuf_, raw_len, stuffed);
    stuffed[0] = (uint8_t)(stuffed.size() - 1); // update length byte

    // Send
    cc_strobe_(CC_SIDLE);
    cc_strobe_(CC_SFTX);
    cc_strobe_(CC_SFRX);
    cc_cs_(false);
    SPI.transfer(CC_TXFIFO | CC_BURST);
    for (uint8_t b : stuffed) SPI.transfer(b);
    cc_cs_(true);
    cc_strobe_(CC_STX);

    // Wait TX done
    uint32_t t0 = millis();
    while (!gdo0_->digital_read() && millis() - t0 < 200) yield();
    while ( gdo0_->digital_read() && millis() - t0 < 500) yield();

    cc_strobe_(CC_SFRX);
    cc_strobe_(CC_SRX);

    // Receive
    std::vector<uint8_t> raw_all;
    int got = 0;
    t0 = millis();
    while (millis() - t0 < 1200 && got < expected_pkts) {
      uint8_t rxb = cc_rstat_(CC_RXBYTES);
      if (rxb > 0 && rxb < 64) {
        got++;
        cc_cs_(false);
        SPI.transfer(CC_RXFIFO | CC_READ | CC_BURST);
        uint8_t len_b = SPI.transfer(0);
        if (len_b > 0 && len_b < 60) {
          for (uint8_t i = 1; i < len_b; i++) raw_all.push_back(SPI.transfer(0));
        }
        cc_cs_(true);
        cc_strobe_(CC_SIDLE); cc_strobe_(CC_SFRX); cc_strobe_(CC_SFTX); cc_strobe_(CC_SRX);
      }
      yield();
    }

    if (raw_all.empty()) {
      ESP_LOGW(TAG, "cmd=0x%02X timeout", cmd);
      return false;
    }

    std::vector<uint8_t> ds;
    destuff_(raw_all.data(), raw_all.size(), ds);
    if (ds.size() > sizeof(rbuf_)) { ESP_LOGW(TAG, "RX overflow"); return false; }
    memcpy(rbuf_, ds.data(), ds.size());
    rlen_ = ds.size();

    // Validate header
    if (rlen_ < 5 || rbuf_[0] != 0x73 || rbuf_[1] != 0x55) {
      ESP_LOGW(TAG, "bad header"); return false;
    }
    // Address check
    if (rbuf_[4] != (addr_ & 0xFF) || rbuf_[5] != ((addr_ >> 8) & 0xFF)) {
      ESP_LOGW(TAG, "addr mismatch"); return false;
    }
    return true;
  }

  // ── Relay command ─────────────────────────────────────────────────────────
  void send_relay_(uint8_t op) {
    do_cmd_(0x3A, op, -1, 2);
    ESP_LOGI(TAG, "Relay cmd 0x%02X sent", op);
  }

  // ── Parse helpers ─────────────────────────────────────────────────────────
  float u16le_(size_t i)  { return (float)((uint16_t)rbuf_[i] | ((uint16_t)rbuf_[i+1] << 8)); }
  float u32le_(size_t i)  { return (float)(((uint32_t)rbuf_[i]) | ((uint32_t)rbuf_[i+1]<<8) | ((uint32_t)rbuf_[i+2]<<16) | ((uint32_t)rbuf_[i+3]<<24)); }
  float u24le_(size_t i)  { return (float)(((uint32_t)rbuf_[i]) | ((uint32_t)rbuf_[i+1]<<8) | ((uint32_t)rbuf_[i+2]<<16)); }

  // Signed: MSB is sign bit (not 2's complement — Mirtek-specific)
  float s16le_mirtek_(size_t i, float div) {
    bool neg = (rbuf_[i+1] >= 128);
    float v = (float)((uint16_t)rbuf_[i] | ((uint16_t)(rbuf_[i+1] & 0x7F) << 8)) / div;
    return neg ? -v : v;
  }
  float s24le_mirtek_(size_t i, float div) {
    bool neg = (rbuf_[i+2] >= 128);
    float v = (float)(((uint32_t)rbuf_[i]) | ((uint32_t)rbuf_[i+1]<<8) | ((uint32_t)(rbuf_[i+2]&0x7F)<<16)) / div;
    return neg ? -v : v;
  }

  void pub_s_(int i, float v) { if (ss_[i]) ss_[i]->publish_state(v); }
  void pub_txt_(int i, const std::string &v) { if (ts_[i]) ts_[i]->publish_state(v); }
  void pub_bin_(int i, bool v) { if (bs_[i]) bs_[i]->publish_state(v); }

  // ── parse_datetime_ (cmd 0x1C) ────────────────────────────────────────────
  bool parse_datetime_() {
    if (rlen_ < 20) return false;
    const uint8_t *r = rbuf_;
    if (r[6] != 0x1C) return false;

    // Byte[7] = тип счётчика
    uint8_t tp = r[7];
    three_phase_ = !((tp == 0x98) || (tp == 0x99));
    char type_buf[32];
    switch (tp) {
      case 0xA8: case 0xA9: snprintf(type_buf,32,"3ф трансф.акт-реакт"); break;
      case 0x88: case 0x89: snprintf(type_buf,32,"3ф трансф.акт двунапр"); break;
      case 0x80: case 0x81: snprintf(type_buf,32,"3ф акт двунапр"); break;
      case 0x68: case 0x69: snprintf(type_buf,32,"3ф трансф.акт"); break;
      case 0x98: case 0x99: snprintf(type_buf,32,"1ф 2эл.акт-реакт"); break;
      default:               snprintf(type_buf,32,"Тип 0x%02X", tp);
    }
    pub_txt_(TI_TYPE, type_buf);
    pub_bin_(BI_3PHASE, three_phase_);

    // Дата/время (позиции могут варьироваться по прошивке)
    // Формат: сс мм чч ДД ММ ГГ ДН (с байта 11)
    if (rlen_ >= 18) {
      char date_buf[12], time_buf[10];
      snprintf(time_buf, 10, "%02d:%02d:%02d", r[13], r[12], r[11]);
      snprintf(date_buf, 12, "%02d.%02d.%02d", r[14], r[15], r[16]);
      pub_txt_(TI_TIME, time_buf);
      pub_txt_(TI_DATE, date_buf);
    }
    return true;
  }

  // ── parse_energy_ (cmd 0x05 sub=0x04) ────────────────────────────────────
  bool parse_energy_() {
    if (rlen_ < 35) return false;
    const uint8_t *r = rbuf_;
    if (r[6] != 0x05) return false;

    // Положение запятой из конф.байта r[12]
    uint8_t cfg = r[12];
    uint8_t dp  = cfg & 0x03;
    float   div = (dp==0)?1.f:(dp==1)?10.f:(dp==2)?100.f:1000.f;

    // Тариф
    uint8_t cur_t = (cfg >> 2) & 0x03;
    const char *tnames[] = {"День (Т1)","Ночь (Т2)","Полупик (Т3)","Специальный"};
    pub_txt_(TI_TARIFF, tnames[cur_t]);

    // Энергия (смещения по протоколу Star v1.20)
    // Суммарная активная: байты 17-20 (u32 / 100)
    if (rlen_ >= 21) pub_s_(SI_SUM,   u32le_(17) / 100.0f);
    // T1: байты 25-28
    if (rlen_ >= 29) pub_s_(SI_T1,    u32le_(25) / 100.0f);
    // T2: байты 29-32
    if (rlen_ >= 33) pub_s_(SI_T2,    u32le_(29) / 100.0f);
    // T3: байты 33-36
    if (rlen_ >= 37) pub_s_(SI_T3,    u32le_(33) / 100.0f);

    // Реактивная энергия — если длина достаточная (расширенный ответ)
    if (rlen_ >= 53) {
      pub_s_(SI_SUM_R, u32le_(37) / 100.0f);
      pub_s_(SI_T1_R,  u32le_(41) / 100.0f);
      pub_s_(SI_T2_R,  u32le_(45) / 100.0f);
      pub_s_(SI_T3_R,  u32le_(49) / 100.0f);
    }

    ESP_LOGI(TAG, "kWh SUM=%.2f T1=%.2f T2=%.2f T3=%.2f",
      ss_[SI_SUM]  ? ss_[SI_SUM]->state  : 0.f,
      ss_[SI_T1]   ? ss_[SI_T1]->state   : 0.f,
      ss_[SI_T2]   ? ss_[SI_T2]->state   : 0.f,
      ss_[SI_T3]   ? ss_[SI_T3]->state   : 0.f);
    return true;
  }

  // ── parse_instant_ (cmd 0x2B sub=0x00) ───────────────────────────────────
  bool parse_instant_() {
    if (rlen_ < 36) return false;
    const uint8_t *r = rbuf_;
    if (r[6] != 0x2B) return false;

    size_t base = 16; // данные начинаются с байта 16

    if (three_phase_) {
      // Активная мощность суммарная (u24 / 1000 = kW)
      pub_s_(SI_KW,   u24le_(base) / 1000.0f);       base += 3;
      // Реактивная мощность (s24 / 1000)
      pub_s_(SI_KVAR, s24le_mirtek_(base, 1000.0f));  base += 3;
      // Частота (u16 / 100)
      pub_s_(SI_FREQ, u16le_(base) / 100.0f);         base += 2;
      // cos φ (s16 / 1000)
      pub_s_(SI_COS,  s16le_mirtek_(base, 1000.0f));  base += 2;
      // Напряжения (u16 / 100)
      pub_s_(SI_V1, u16le_(base)/100.f); base+=2;
      pub_s_(SI_V2, u16le_(base)/100.f); base+=2;
      pub_s_(SI_V3, u16le_(base)/100.f); base+=2;
      // Токи (u24 / 1000)
      pub_s_(SI_I1, u24le_(base)/1000.f); base+=3;
      pub_s_(SI_I2, u24le_(base)/1000.f); base+=3;
      pub_s_(SI_I3, u24le_(base)/1000.f); base+=3;
    } else {
      // 1-фазный
      pub_s_(SI_KW,   u16le_(base)/1000.f); base+=2;
      pub_s_(SI_KVAR, s16le_mirtek_(base,1000.f)); base+=2;
      pub_s_(SI_FREQ, u16le_(base)/100.f);  base+=2;
      pub_s_(SI_COS,  s16le_mirtek_(base,1000.f)); base+=2;
      pub_s_(SI_V1, u16le_(base)/100.f); base+=2;
      pub_s_(SI_I1, u24le_(base)/1000.f); base+=3;
    }

    ESP_LOGI(TAG, "U=%.1f/%.1f/%.1f I=%.3f/%.3f/%.3f P=%.3fkW Q=%.3f f=%.2f cos=%.3f",
      ss_[SI_V1]?ss_[SI_V1]->state:0.f, ss_[SI_V2]?ss_[SI_V2]->state:0.f, ss_[SI_V3]?ss_[SI_V3]->state:0.f,
      ss_[SI_I1]?ss_[SI_I1]->state:0.f, ss_[SI_I2]?ss_[SI_I2]->state:0.f, ss_[SI_I3]?ss_[SI_I3]->state:0.f,
      ss_[SI_KW]?ss_[SI_KW]->state:0.f, ss_[SI_KVAR]?ss_[SI_KVAR]->state:0.f,
      ss_[SI_FREQ]?ss_[SI_FREQ]->state:0.f, ss_[SI_COS]?ss_[SI_COS]->state:0.f);
    return true;
  }

  // ── parse_phase_ (cmd 0x2B sub=0x10) ─────────────────────────────────────
  bool parse_phase_() {
    if (rlen_ < 40) return false;
    const uint8_t *r = rbuf_;
    if (r[6] != 0x2B) return false;

    size_t b = 16;
    // cos φ по фазам (s16 / 1000)
    pub_s_(SI_CA, s16le_mirtek_(b,1000.f)); b+=2;
    pub_s_(SI_CB, s16le_mirtek_(b,1000.f)); b+=2;
    pub_s_(SI_CC, s16le_mirtek_(b,1000.f)); b+=2;
    // Активная мощность по фазам (u16 / 1000 = kW → convert to W)
    pub_s_(SI_PA, u16le_(b));  b+=2;  // в ваттах
    pub_s_(SI_PB, u16le_(b));  b+=2;
    pub_s_(SI_PC, u16le_(b));  b+=2;
    // Реактивная (s16 / 1000 kVar → var *1000 для согласования)
    pub_s_(SI_QA, s16le_mirtek_(b,1.f)); b+=2;
    pub_s_(SI_QB, s16le_mirtek_(b,1.f)); b+=2;
    pub_s_(SI_QC, s16le_mirtek_(b,1.f)); b+=2;
    // Полная (u16 VA)
    pub_s_(SI_SA, u16le_(b)); b+=2;
    pub_s_(SI_SB, u16le_(b)); b+=2;
    pub_s_(SI_SC, u16le_(b)); b+=2;
    // Температура (байт со знаком: >= 128 → отрицательная)
    if (b < rlen_) {
      float t = (rbuf_[b] >= 128) ? (float)(rbuf_[b] - 128) * -1.f : (float)rbuf_[b];
      pub_s_(SI_TEMP, t);
    }
    return true;
  }

  // ── parse_ident_ (cmd 0x01) ───────────────────────────────────────────────
  bool parse_ident_() {
    if (rlen_ < 20) return false;
    const uint8_t *r = rbuf_;
    if (r[6] != 0x01) return false;

    // Версия ПО: r[12].r[11]
    char fw[12];
    snprintf(fw, 12, "%02d.%02d", r[13], r[12]);
    pub_txt_(TI_FW, fw);

    // Реле (r[8], r[9])
    bool relay_on = false;
    // byte 9 bit 3=relay2, bit 2=relay1; version-dependent logic
    uint8_t sw_ver = r[13];
    bool par_r2 = (r[9] >> 3) & 1;
    bool par_r1 = (r[9] >> 2) & 1;
    relay_on = (sw_ver <= 4 && !par_r1) ? par_r2 : !par_r2;
    pub_txt_(TI_RELAY, relay_on ? "Вкл" : "Выкл");
    pub_bin_(BI_RELAY, relay_on);

    // Пломбы (r[8])
    bool p1 = (r[8] >> 1) & 1; // клеммная крышка
    bool p2 = (r[8] >> 2) & 1; // корпус
    bool p3 = (r[8] >> 3) & 1; // модуль связи
    bool m1 = (r[8] >> 4) & 1; // пост.магнит
    bool m2 = (r[8] >> 5) & 1; // перем.магнит
    bool seals_ok = !(p1||p2||p3||m1||m2);
    std::string seal_str;
    if (seals_ok) {
      seal_str = "OK";
    } else {
      if (p3) seal_str += "Модуль;";
      if (p2) seal_str += "Корпус;";
      if (p1) seal_str += "Клеммы;";
      if (m1) seal_str += "Пост.магнит;";
      if (m2) seal_str += "Перем.магнит;";
    }
    pub_txt_(TI_SEAL, seal_str);
    pub_bin_(BI_SEAL_OK, seals_ok);

    ESP_LOGI(TAG, "FW=%s Relay=%s Seal=%s", fw, relay_on?"Вкл":"Выкл", seal_str.c_str());
    return true;
  }

  // ── parse_battery_ (cmd 0x1E) ─────────────────────────────────────────────
  bool parse_battery_() {
    if (rlen_ < 14) return false;
    const uint8_t *r = rbuf_;
    if (r[6] != 0x1E) return false;
    // Ресурс батарейки: r[10] = оставшийся % (0-100)
    pub_s_(SI_BAT, (float)r[10]);
    return true;
  }
};

}  // namespace mirtek_cc1101
}  // namespace esphome
