#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <CRC8.h>
#include <cmath>
#include <string>
#include <cstring> // Для memset, memcpy, memmove

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek";

// RF-регистры CC1101
static const uint8_t MIRTEK_RF[] = {
  0x0D,0x2E,0x06,0x4F,0xD3,0x91,0x3C,0x00,0x41,0x00,0x16,
  0x0F,0x00,0x10,0x8B,0x54,0xD9,0x83,0x13,0xD2,0xAA,0x31,
  0x07,0x0C,0x08,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,
  0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x59,0x3F,
  0x81,0x35,0x09
};

// Коды команд
static const uint8_t CMD_PING     = 0x01;
static const uint8_t CMD_ENERGY   = 0x05;
static const uint8_t CMD_ABON     = 0x07;
static const uint8_t CMD_FACTORY  = 0x0A;
static const uint8_t CMD_DATETIME = 0x1C;
static const uint8_t CMD_BAT      = 0x1E;
static const uint8_t CMD_INSTANT  = 0x2B;
static const uint8_t CMD_GETINFO  = 0x30;
static const uint8_t CMD_RELAY    = 0x3A;

static const uint8_t SUB_E_ACT   = 0x04;
static const uint8_t SUB_E_REACT = 0x05;
static const uint8_t SUB_I_UI    = 0x00;
static const uint8_t SUB_I_PWR   = 0x10;

static const uint8_t MT_3P  = 0xA8;
static const uint8_t MT_3P2 = 0xA9;
static const uint8_t MT_1P  = 0x98;
static const uint8_t MT_1P1 = 0x90;

enum SI {
  SUM,T1,T2,T3, SUM_R,T1_R,T2_R,T3_R,
  KW,KVAR,FREQ,COS,
  V1,V2,V3, I1,I2,I3,
  PA,PB,PC, QA,QB,QC, SA,SB,SC,
  CA,CB,CC_S, TEMP,BAT, _SCOUNT
};
enum TI { T_TAR,T_RELAY,T_SEAL,T_TYPE,T_FW,T_DATE,T_TIME,T_WORK,T_SYNC,T_SER,T_ABON,T_ST };
enum BI { B_3P,B_REL,B_SEAL,B_CC };

// Хелперы
static inline float mk_s16(uint8_t lo, uint8_t hi, float d) {
  int16_t val = (int16_t)(lo | (hi << 8));
  if (hi >= 128) val -= 32768; // Корректная обработка знака для протокола Миртек
  return (float)val / d;
}

static inline uint32_t mk_u32le(const uint8_t *b) {
  return ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0];
}

static inline uint16_t mk_u16le(const uint8_t *b) {
  return (uint16_t)((b[1] << 8) | b[0]);
}

class MirtekCC1101 : public Component {
 public:
  void set_sensor(int i, sensor::Sensor *s) { if (i >= 0 && i < 32) ss_[i] = s; }
  void set_text_sensor(int i, text_sensor::TextSensor *s) { if (i >= 0 && i < 12) ts_[i] = s; }
  void set_binary_sensor(int i, binary_sensor::BinarySensor *s) { if (i >= 0 && i < 4) bs_[i] = s; }

  void set_meter_address(int a) { addr_ = a; }
  void set_cs_pin(int p)  { cs_ = p; }
  void set_gdo0_pin(int p) { gdo0_ = p; }

  sensor::Sensor *ss_[32]{nullptr};
  text_sensor::TextSensor *ts_[12]{nullptr};
  binary_sensor::BinarySensor *bs_[4]{nullptr};

  int addr_{0}, cs_{5}, gdo0_{22};
  bool cc1101_ok_{false};
  int poll_cnt_{0};
  bool three_phase_{true};
  uint8_t meter_type_{MT_3P};

  float sum_{NAN},t1_{NAN},t2_{NAN},t3_{NAN};
  float sum_r_{NAN},t1_r_{NAN},t2_r_{NAN},t3_r_{NAN};
  float kw_{NAN},kvar_{NAN},freq_{NAN},cos_{NAN};
  float v1_{NAN},v2_{NAN},v3_{NAN}, i1_{NAN},i2_{NAN},i3_{NAN};
  float pa_{NAN},pb_{NAN},pc_{NAN}, qa_{NAN},qb_{NAN},qc_{NAN};
  float sa_{NAN},sb_{NAN},sc_{NAN}, ca_{NAN},cb_{NAN},cc_{NAN};
  float temp_{NAN}, bat_pct_{NAN};
  
  std::string tariff_{"—"}, relay_{"—"}, seal_{"—"}, type_name_{"—"};
  std::string fw_{"—"}, meter_date_{"—"}, meter_time_{"—"};
  std::string work_time_{"—"}, sync_time_{"—"}, serial_{"—"}, abon_{"—"}, status_{"Init"};

  static const int BUF = 64;
  uint8_t tx_[BUF]{}, raw_[BUF]{}, res_[BUF]{};
  int raw_len_{0}, res_len_{0};
  CRC8 crc_;

  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void setup() override {
    ESP_LOGI(TAG, "Init CC1101 cs=%d gdo0=%d addr=%d", cs_, gdo0_, addr_);
    ELECHOUSE_cc1101.setGDO0(gdo0_);
    cc1101_ok_ = ELECHOUSE_cc1101.getCC1101();
    
    if (bs_[B_CC]) bs_[B_CC]->publish_state(cc1101_ok_);
    if (!cc1101_ok_) {
      ESP_LOGE(TAG, "CC1101 SPI error!");
      status_ = "CC1101 ERROR"; pub_text_();
      return;
    }

    ELECHOUSE_cc1101.SpiStrobe(0x30); // SRES
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x00, (byte *)MIRTEK_RF, 0x2F);
    ELECHOUSE_cc1101.SpiStrobe(0x33); // SIDLE
    delay(1);
    ELECHOUSE_cc1101.SpiStrobe(0x3A); // SFRX
    ELECHOUSE_cc1101.SpiStrobe(0x3B); // SFTX
    ELECHOUSE_cc1101.SpiStrobe(0x34); // SRX
    status_ = "Ready"; pub_text_();
  }

  void loop() override {}

  void poll_all() {
    if (!cc1101_ok_) return;
    poll_cnt_++;
    ESP_LOGI(TAG, "=== Poll #%d addr=%d ===", poll_cnt_, addr_);

    if (xact_(CMD_PING))             parse_ping_();
    if (xact_(CMD_DATETIME))         parse_datetime_();
    if (xact_(CMD_ENERGY, 1, SUB_E_ACT))   parse_energy_(false);
    if (xact_(CMD_ENERGY, 1, SUB_E_REACT)) parse_energy_(true);
    if (xact_(CMD_INSTANT, 1, SUB_I_UI))   parse_ui_();
    if (three_phase_ && xact_(CMD_INSTANT, 1, SUB_I_PWR)) parse_pwr_();
    if (xact_(CMD_GETINFO))          parse_getinfo_();
    if (xact_(CMD_BAT))              parse_bat_();
    if (xact_(CMD_FACTORY, 1, 1))    parse_str_(CMD_FACTORY, 1);
    if (xact_(CMD_ABON, 1, 1))       parse_str_(CMD_ABON, 1);
    if (xact_(CMD_ABON, 1, 6))       parse_str_(CMD_ABON, 6);

    pub_all_();
    status_ = "OK #" + std::to_string(poll_cnt_);
    pub_text_();
  }

  void relay_on()  { ctrl_relay_(0x00); }
  void relay_off() { ctrl_relay_(0x01); }

 private:
  void ctrl_relay_(uint8_t cmd) {
    build_(CMD_RELAY, 2, 0x00, cmd); 
    send_();
    recv_(600);
    relay_ = (cmd == 0) ? "On" : "Off";
    pub_text_();
  }

  void build_(uint8_t cmd, int sub_cnt = -1, uint8_t s1 = 0, uint8_t s2 = 0) {
    int dlen = (sub_cnt < 0) ? 0 : sub_cnt;
    uint8_t tmp[44]{}; int n = 0;
    tmp[n++] = 0x73; tmp[n++] = 0x55;
    tmp[n++] = (uint8_t)(0x20 | (dlen & 0x1F));
    tmp[n++] = 0x00;
    tmp[n++] = (uint8_t)(addr_ & 0xFF);
    tmp[n++] = (uint8_t)((addr_ >> 8) & 0xFF);
    tmp[n++] = 0xFE; tmp[n++] = 0xFF;
    tmp[n++] = cmd;
    tmp[n++] = 0; tmp[n++] = 0; tmp[n++] = 0; tmp[n++] = 0;
    if (sub_cnt >= 1) tmp[n++] = s1;
    if (sub_cnt >= 2) tmp[n++] = s2;

    crc_.restart(); crc_.setPolynome(0xA9);
    for (int i = 2; i < n; i++) crc_.add(tmp[i]);
    tmp[n++] = crc_.calc();
    tmp[n++] = 0x55;

    memset(tx_, 0, BUF); int out = 0;
    tx_[out++] = tmp[0]; tx_[out++] = tmp[1];
    for (int i = 2; i < n - 1; i++) {
      if (tmp[i] == 0x55) { tx_[out++] = 0x73; tx_[out++] = 0x11; }
      else if (tmp[i] == 0x73) { tx_[out++] = 0x73; tx_[out++] = 0x22; }
      else { tx_[out++] = tmp[i]; }
    }
    tx_[out++] = 0x55;
    
    int actual_len = out;
    memmove(tx_ + 1, tx_, actual_len); 
    tx_[0] = (uint8_t)actual_len;
  }

  void send_() {
    ELECHOUSE_cc1101.SpiStrobe(0x33); delay(1);
    ELECHOUSE_cc1101.SpiStrobe(0x3B); ELECHOUSE_cc1101.SpiStrobe(0x36);
    ELECHOUSE_cc1101.SpiWriteReg(0x3E, 0xC4);
    ELECHOUSE_cc1101.SendData(tx_, tx_[0] + 1);
    ELECHOUSE_cc1101.SpiStrobe(0x3A); ELECHOUSE_cc1101.SpiStrobe(0x34);
  }

  bool recv_(uint32_t ms) {
    memset(raw_, 0, BUF); memset(res_, 0, BUF);
    raw_len_ = 0; res_len_ = 0;
    uint8_t piece[BUF]{}; uint32_t t0 = millis();
    while (millis() - t0 < ms) {
      if (ELECHOUSE_cc1101.CheckReceiveFlag()) {
        int len = ELECHOUSE_cc1101.ReceiveData(piece);
        for (int i = 1; i < len && raw_len_ < BUF - 1; i++) raw_[raw_len_++] = piece[i];
        ELECHOUSE_cc1101.SpiStrobe(0x36); ELECHOUSE_cc1101.SpiStrobe(0x3A);
        ELECHOUSE_cc1101.SpiStrobe(0x3B); ELECHOUSE_cc1101.SpiStrobe(0x34);
      }
      yield();
    }
    if (!raw_len_) return false;

    int j = 0;
    for (int i = 0; i < raw_len_; i++) {
      res_[i - j] = raw_[i]; res_len_++;
      if (raw_[i] == 0x73 && i + 1 < raw_len_) {
        if (raw_[i+1] == 0x11) { res_[i-j] = 0x55; i++; j++; }
        else if (raw_[i+1] == 0x22) { i++; j++; }
      }
    }
    return res_len_ > 5;
  }

  bool valid_(uint8_t cmd) {
    if (res_len_ < 10) return false;
    if (res_[0] != 0x73 || res_[1] != 0x55) return false;
    if (res_[6] != (uint8_t)(addr_ & 0xFF)) return false;
    if (res_[8] != cmd) return false;

    crc_.restart(); crc_.setPolynome(0xA9);
    int ce = res_len_ - 2;
    if (res_len_ > 3 && res_[res_len_ - 3] == 0x76) ce = res_len_ - 3;
    for (int i = 2; i < ce; i++) crc_.add(res_[i]);
    return (res_[ce] == crc_.calc());
  }

  bool xact_(uint8_t cmd, int sub = -1, uint8_t s = 0) {
    build_(cmd, sub, s, 0); send_();
    if (!recv_(800)) return false;
    return valid_(cmd);
  }

  void parse_ping_() {
    meter_type_ = res_[9];
    three_phase_ = (meter_type_ == MT_3P || meter_type_ == MT_3P2);
    if (three_phase_) type_name_ = "3-Phase";
    else type_name_ = "1-Phase";
    
    char fw[10]; snprintf(fw, 10, "%02u.%02u", res_[14], res_[13]);
    fw_ = fw;
    
    bool r2 = (res_[11] >> 3) & 1;
    relay_ = ((res_[14] < 5) ? r2 : !r2) ? "On" : "Off";
    seal_ = ((res_[10] >> 1) & 0x07) ? "ALARM" : "OK";
  }

  void parse_datetime_() {
    static const char *dow[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    char dt[32], tm[16];
    snprintf(dt, 32, "20%02u-%02u-%02u (%s)", res_[19], res_[18], res_[17], dow[res_[16] % 7]);
    snprintf(tm, 16, "%02u:%02u:%02u", res_[15], res_[14], res_[13]);
    meter_date_ = dt; meter_time_ = tm;
  }

  void parse_energy_(bool react) {
    float s = (float)mk_u32le(res_ + 19) / 100.0f;
    float a = (float)mk_u32le(res_ + 27) / 100.0f;
    float b = (float)mk_u32le(res_ + 31) / 100.0f;
    float c = (float)mk_u32le(res_ + 35) / 100.0f;
    uint8_t nt = ((res_[14] >> 6) & 0x03) + 1;
    if (!react) {
      sum_=s; t1_=a; t2_=(nt>=2)?b:NAN; t3_=(nt>=3)?c:NAN;
    } else {
      sum_r_=s; t1_r_=a; t2_r_=(nt>=2)?b:NAN; t3_r_=(nt>=3)?c:NAN;
    }
  }

  void parse_ui_() {
    const uint8_t *r = res_;
    if (three_phase_) {
      kw_   = (float)(r[18]|(r[19]<<8)|(r[20]<<16))/1000.0f;
      freq_ = (float)mk_u16le(r+24)/100.0f;
      v1_   = (float)mk_u16le(r+28)/100.0f; v2_=(float)mk_u16le(r+30)/100.0f; v3_=(float)mk_u16le(r+32)/100.0f;
      i1_   = (float)(r[34]|(r[35]<<8)|(r[36]<<16))/1000.0f;
    } else {
      kw_   = (float)mk_u16le(r+18)/1000.0f;
      v1_   = (float)mk_u16le(r+26)/100.0f;
      i1_   = (float)(r[32]|(r[33]<<8)|(r[34]<<16))/1000.0f;
    }
  }

  void parse_pwr_() {
    temp_ = (res_[42] >= 128) ? -(float)(res_[42]-128) : (float)res_[42];
  }

  void parse_getinfo_() {
    work_time_ = fmt_sec_(mk_u32le(res_+18));
    sync_time_ = fmt_sec_(mk_u32le(res_+32));
  }

  void parse_bat_() {
    if (res_len_ < 15) return;
    bat_pct_ = (res_[13] > 0) ? (100.0f * res_[14] / res_[13]) : NAN;
  }

  void parse_str_(uint8_t cmd, uint8_t field) {
    char buf[31]{}; 
    if (res_len_ > 14 + 30) memcpy(buf, res_ + 14, 30);
    if (cmd == CMD_FACTORY) serial_ = buf;
    if (cmd == CMD_ABON) abon_ = buf;
  }

  std::string fmt_sec_(uint32_t s) {
    uint32_t d=s/86400, h=(s%86400)/3600, m=(s%3600)/60;
    char b[32];
    if (d > 0) snprintf(b, 32, "%u d %02u:%02u", (unsigned)d, (unsigned)h, (unsigned)m);
    else snprintf(b, 32, "%02u:%02u", (unsigned)h, (unsigned)m);
    return b;
  }

  void pf_(int i, float v) { if (i < 32 && ss_[i] && !std::isnan(v)) ss_[i]->publish_state(v); }

  void pub_all_() {
    pf_(SUM,sum_); pf_(T1,t1_); pf_(T2,t2_); pf_(T3,t3_);
    pf_(KW,kw_); pf_(V1,v1_); pf_(V2,v2_); pf_(V3,v3_);
    pf_(I1,i1_); pf_(TEMP,temp_); pf_(BAT,bat_pct_);
    pub_text_();
  }

  void pub_text_() {
    if (ts_[T_TAR])   ts_[T_TAR]->publish_state(tariff_);
    if (ts_[T_TYPE])  ts_[T_TYPE]->publish_state(type_name_);
    if (ts_[T_SER])   ts_[T_SER]->publish_state(serial_);
    if (ts_[T_ST])    ts_[T_ST]->publish_state(status_);
    if (bs_[B_3P])    bs_[B_3P]->publish_state(three_phase_);
  }
};

} // namespace mirtek_cc1101
} // namespace esphome
