#pragma once
/**
 * mirtek_cc1101.h — ESPHome native component для счётчиков Миртек Star 104/304
 * Протокол Star v1.20, CC1101 433 МГц
 * https://github.com/Alecseyyy/ESPHome-Mirt-830
 */

// ESPHome заголовки — явный порядок важен
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

// Сторонние библиотеки
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <CRC8.h>
#include <cmath>
#include <string>

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek";

// ── RF-регистры CC1101 (47 байт, 433 МГц) ────────────────────────────────
static const uint8_t MIRTEK_RF[] = {
  0x0D,0x2E,0x06,0x4F,0xD3,0x91,0x3C,0x00,0x41,0x00,0x16,
  0x0F,0x00,0x10,0x8B,0x54,0xD9,0x83,0x13,0xD2,0xAA,0x31,
  0x07,0x0C,0x08,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,
  0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x59,0x3F,
  0x81,0x35,0x09
};

// ── Коды команд протокола ─────────────────────────────────────────────────
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

// ── Индексы массива сенсоров ss[] ─────────────────────────────────────────
enum SI {
  SUM,T1,T2,T3, SUM_R,T1_R,T2_R,T3_R,
  KW,KVAR,FREQ,COS,
  V1,V2,V3, I1,I2,I3,
  PA,PB,PC, QA,QB,QC, SA,SB,SC,
  CA,CB,CC_S, TEMP,BAT, _SCOUNT
};
// Индексы ts[]
enum TI { T_TAR,T_RELAY,T_SEAL,T_TYPE,T_FW,T_DATE,T_TIME,T_WORK,T_SYNC,T_SER,T_ABON,T_ST };
// Индексы bs[]
enum BI { B_3P,B_REL,B_SEAL,B_CC };

// ── Вспомогательные функции декодирования ────────────────────────────────
static inline float mk_s16(uint8_t lo, uint8_t hi, float d) {
  return (hi >= 128) ? float(lo | ((hi - 128) << 8)) / -d
                     : float(lo | (hi << 8)) / d;
}
static inline float mk_u32le(const uint8_t *b) {
  return float(((uint32_t)b[3]<<24)|((uint32_t)b[2]<<16)|((uint32_t)b[1]<<8)|b[0]);
}
static inline float mk_u16le(const uint8_t *b) {
  return float((uint16_t)b[1] << 8 | b[0]);
}

// ════════════════════════════════════════════════════════════════════════════
class MirtekCC1101 : public Component {
 public:
  // ── Сеттеры (вызываются из __init__.py to_code) ───────────────────────
  void set_sensor(int i, sensor::Sensor *s)                    { if (i < 32) ss_[i] = s; }
  void set_text_sensor(int i, text_sensor::TextSensor *s)      { if (i < 12) ts_[i] = s; }
  void set_binary_sensor(int i, binary_sensor::BinarySensor *s){ if (i < 4)  bs_[i] = s; }

  void set_meter_address(int a) { addr_ = a; }
  void set_cs_pin(int p)        { cs_   = p; }
  void set_gdo0_pin(int p)      { gdo0_ = p; }

  // ── Массивы сенсоров ──────────────────────────────────────────────────
  sensor::Sensor        *ss_[32]{};
  text_sensor::TextSensor *ts_[12]{};
  binary_sensor::BinarySensor *bs_[4]{};

  // ── Состояние ─────────────────────────────────────────────────────────
  int   addr_{0}, cs_{5}, gdo0_{22};
  bool  cc1101_ok_{false};
  int   poll_cnt_{0};
  bool  three_phase_{true};
  uint8_t meter_type_{MT_3P};

  // Данные
  float sum_{NAN},t1_{NAN},t2_{NAN},t3_{NAN};
  float sum_r_{NAN},t1_r_{NAN},t2_r_{NAN},t3_r_{NAN};
  float kw_{NAN},kvar_{NAN},freq_{NAN},cos_{NAN};
  float v1_{NAN},v2_{NAN},v3_{NAN};
  float i1_{NAN},i2_{NAN},i3_{NAN};
  float pa_{NAN},pb_{NAN},pc_{NAN};
  float qa_{NAN},qb_{NAN},qc_{NAN};
  float sa_{NAN},sb_{NAN},sc_{NAN};
  float ca_{NAN},cb_{NAN},cc_{NAN};
  float temp_{NAN}, bat_pct_{NAN};
  std::string tariff_{"—"}, relay_{"—"}, seal_{"—"}, type_name_{"—"};
  std::string fw_{"—"}, meter_date_{"—"}, meter_time_{"—"};
  std::string work_time_{"—"}, sync_time_{"—"};
  std::string serial_{"—"}, abon_{"—"}, status_{"Init"};

  // ── Буферы CC1101 ─────────────────────────────────────────────────────
  static const int BUF = 64;
  uint8_t tx_[BUF]{}, raw_[BUF]{}, res_[BUF]{};
  int raw_len_{0}, res_len_{0};
  uint8_t my_crc_{0};
  CRC8 crc_;

  // ════════════════════════════════════════════════════════════════════════
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
    ELECHOUSE_cc1101.SpiStrobe(0x30);
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x00, (byte *)MIRTEK_RF, 0x2F);
    ELECHOUSE_cc1101.SpiStrobe(0x33); delay(1);
    ELECHOUSE_cc1101.SpiStrobe(0x3A);
    ELECHOUSE_cc1101.SpiStrobe(0x3B);
    ELECHOUSE_cc1101.SpiStrobe(0x34);
    ESP_LOGI(TAG, "CC1101 OK, RX mode");
    status_ = "Ready"; pub_text_();
  }

  void loop() override {}

  // ── Публичный API ─────────────────────────────────────────────────────
  void poll_all() {
    if (!cc1101_ok_) return;
    poll_cnt_++;
    ESP_LOGI(TAG, "=== Poll #%d addr=%d ===", poll_cnt_, addr_);

    if (xact_(CMD_PING, -1))             parse_ping_();
    if (xact_(CMD_DATETIME, -1))         parse_datetime_();
    if (xact_(CMD_ENERGY, 1, SUB_E_ACT))   parse_energy_(false);
    if (xact_(CMD_ENERGY, 1, SUB_E_REACT)) parse_energy_(true);
    if (xact_(CMD_INSTANT, 1, SUB_I_UI))   parse_ui_();
    if (three_phase_ && xact_(CMD_INSTANT, 1, SUB_I_PWR)) parse_pwr_();
    if (xact_(CMD_GETINFO, -1))          parse_getinfo_();
    if (xact_(CMD_BAT, -1))              parse_bat_();
    if (xact_(CMD_FACTORY, 1, 1))        parse_str_(CMD_FACTORY, 1);
    if (xact_(CMD_ABON, 1, 1))           parse_str_(CMD_ABON, 1);
    if (xact_(CMD_ABON, 1, 6))           parse_str_(CMD_ABON, 6);

    pub_all_();
    char st[24]; snprintf(st, 24, "OK #%d", poll_cnt_);
    status_ = st; pub_text_();
    ESP_LOGI(TAG, "=== Done ===");
  }

  void relay_on()  { ctrl_relay_(0x00); }
  void relay_off() { ctrl_relay_(0x01); }

 private:
  void ctrl_relay_(uint8_t cmd) {
    build_(CMD_RELAY, 2, 0x00, cmd); send_();
    recv_(600);
    relay_ = (cmd == 0) ? "On" : "Off";
    if (ts_[T_RELAY]) ts_[T_RELAY]->publish_state(relay_);
    if (bs_[B_REL])   bs_[B_REL]->publish_state(cmd == 0);
  }

  // ── Протокол: формирование пакета ────────────────────────────────────
  void build_(uint8_t cmd, int sub_cnt = -1, uint8_t s1 = 0, uint8_t s2 = 0) {
    int dlen = (sub_cnt < 0) ? 0 : sub_cnt;
    uint8_t tmp[44]{}; int n = 0;
    tmp[n++] = 0x73; tmp[n++] = 0x55;
    tmp[n++] = uint8_t(0x20 | (dlen & 0x1F));
    tmp[n++] = 0x00;
    tmp[n++] = uint8_t(addr_ & 0xFF);
    tmp[n++] = uint8_t((addr_ >> 8) & 0xFF);
    tmp[n++] = 0xFE; tmp[n++] = 0xFF;
    tmp[n++] = cmd;
    tmp[n++] = 0; tmp[n++] = 0; tmp[n++] = 0; tmp[n++] = 0;
    if (sub_cnt >= 1) tmp[n++] = s1;
    if (sub_cnt >= 2) tmp[n++] = s2;
    crc_.restart(); crc_.setPolynome(0xA9);
    for (int i = 2; i < n; i++) crc_.add(tmp[i]);
    tmp[n++] = crc_.calc(); tmp[n++] = 0x55;
    // Байтстаффинг
    memset(tx_, 0, BUF); int out = 0;
    tx_[out++] = tmp[0]; tx_[out++] = tmp[1];
    for (int i = 2; i < n - 1; i++) {
      if      (tmp[i] == 0x55) { tx_[out++] = 0x73; tx_[out++] = 0x11; }
      else if (tmp[i] == 0x73) { tx_[out++] = 0x73; tx_[out++] = 0x22; }
      else                      { tx_[out++] = tmp[i]; }
    }
    tx_[out++] = 0x55;
    memmove(tx_ + 1, tx_, out); tx_[0] = (uint8_t)out;
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
    // Обратный байтстаффинг
    int j = 0;
    for (int i = 0; i < raw_len_; i++) {
      res_[i - j] = raw_[i]; res_len_++;
      if (raw_[i] == 0x73 && i + 1 < raw_len_) {
        if      (raw_[i+1] == 0x11) { res_[i-j] = 0x55; i++; j++; }
        else if (raw_[i+1] == 0x22) { i++; j++; }
      }
    }
    // CRC
    crc_.reset(); crc_.setPolynome(0xA9);
    int ce = res_len_ - 2;
    if (res_len_ > 3 && res_[res_len_ - 3] == 0x76) ce = res_len_ - 3;
    for (int i = 2; i < ce; i++) crc_.add(res_[i]);
    my_crc_ = crc_.calc();
    return true;
  }

  bool valid_(uint8_t cmd) {
    if (res_len_ < 10) return false;
    if (res_[0] != 0x73 || res_[1] != 0x55) return false;
    if (res_[6] != uint8_t(addr_ & 0xFF)) return false;
    if (res_[8] != cmd) { ESP_LOGW(TAG, "cmd %02X!=%02X", res_[8], cmd); return false; }
    if (res_[res_len_-2] != my_crc_) { ESP_LOGW(TAG, "crc %02X!=%02X", my_crc_, res_[res_len_-2]); return false; }
    if (res_[res_len_-1] != 0x55) return false;
    return true;
  }

  bool xact_(uint8_t cmd, int sub = -1, uint8_t s = 0) {
    build_(cmd, sub, s, 0); send_();
    if (!recv_(800)) { ESP_LOGW(TAG, "timeout cmd=%02X", cmd); return false; }
    if (!valid_(cmd)) { ESP_LOGW(TAG, "invalid cmd=%02X", cmd); return false; }
    return true;
  }

  // ── Парсеры ───────────────────────────────────────────────────────────
  void parse_ping_() {
    meter_type_ = res_[9];
    three_phase_ = (meter_type_ == MT_3P || meter_type_ == MT_3P2);
    if      (meter_type_ == MT_3P || meter_type_ == MT_3P2) type_name_ = "3ph transformer";
    else if (meter_type_ == MT_1P)  type_name_ = "1ph 2-element";
    else if (meter_type_ == MT_1P1) type_name_ = "1ph 1-element";
    else { char b[8]; snprintf(b, 8, "0x%02X", meter_type_); type_name_ = b; }
    char fw[8]; snprintf(fw, 8, "%02u.%02u", res_[14], res_[13]);
    fw_ = fw;
    bool p3=(res_[10]>>3)&1,p2=(res_[10]>>2)&1,p1=(res_[10]>>1)&1;
    bool m1=(res_[10]>>4)&1,m2=(res_[10]>>5)&1;
    seal_ = (!p1&&!p2&&!p3&&!m1&&!m2) ? "OK" : "ALARM";
    bool r2=(res_[11]>>3)&1;
    relay_ = ((res_[14]<5) ? r2 : !r2) ? "On" : "Off";
    ESP_LOGI(TAG, "Type=%s FW=%s Relay=%s Seal=%s", type_name_.c_str(), fw, relay_.c_str(), seal_.c_str());
  }

  void parse_datetime_() {
    static const char *dow[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    char dt[24], tm[10];
    snprintf(dt, 24, "%02u-%02u-20%02u %s", res_[17], res_[18], res_[19], dow[res_[16] % 7]);
    snprintf(tm, 10, "%02u:%02u:%02u", res_[15], res_[14], res_[13]);
    meter_date_ = dt; meter_time_ = tm;
    static const char *ts[] = {"Day","Night","Halfpeak","Special"};
    tariff_ = ts[(res_[10] >> 2) & 0x03];
  }

  void parse_energy_(bool react) {
    float s = mk_u32le(res_+19)/100.0f;
    float a = mk_u32le(res_+27)/100.0f;
    float b = mk_u32le(res_+31)/100.0f;
    float c = mk_u32le(res_+35)/100.0f;
    uint8_t nt = ((res_[14] >> 6) & 0x03) + 1;
    if (!react) {
      sum_=s; t1_=a; t2_=(nt>=2)?b:NAN; t3_=(nt>=3)?c:NAN;
      static const char *ts[] = {"Day","Night","Halfpeak","Special"};
      tariff_ = ts[(res_[14] >> 2) & 0x03];
    } else {
      sum_r_=s; t1_r_=a; t2_r_=(nt>=2)?b:NAN; t3_r_=(nt>=3)?c:NAN;
    }
    ESP_LOGI(TAG, "%s SUM=%.2f T1=%.2f T2=%.2f T3=%.2f",
      react?"kVArh":"kWh", s, a, b, c);
  }

  void parse_ui_() {
    const uint8_t *r = res_;
    if (three_phase_) {
      kw_   = float(r[18]|(r[19]<<8)|(r[20]<<16))/1000.0f;
      kvar_ = (r[23]>=128)
        ? float(r[21]|(r[22]<<8)|((r[23]-128)<<16))/-1000.0f
        : float(r[21]|(r[22]<<8)|(r[23]<<16))/1000.0f;
      freq_ = mk_u16le(r+24)/100.0f;
      cos_  = mk_s16(r[26],r[27],1000.0f);
      v1_=mk_u16le(r+28)/100.0f; v2_=mk_u16le(r+30)/100.0f; v3_=mk_u16le(r+32)/100.0f;
      i1_=float(r[34]|(r[35]<<8)|(r[36]<<16))/1000.0f;
      i2_=float(r[37]|(r[38]<<8)|(r[39]<<16))/1000.0f;
      i3_=float(r[40]|(r[41]<<8)|(r[42]<<16))/1000.0f;
    } else {
      kw_  =mk_u16le(r+18)/1000.0f; kvar_=mk_s16(r[20],r[21],1000.0f);
      freq_=mk_u16le(r+22)/100.0f;  cos_ =mk_s16(r[24],r[25],1000.0f);
      v1_=mk_u16le(r+26)/100.0f; v2_=NAN; v3_=NAN;
      i1_=float(r[32]|(r[33]<<8)|(r[34]<<16))/1000.0f; i2_=NAN; i3_=NAN;
    }
    ESP_LOGI(TAG, "U=%.1f/%.1f/%.1f I=%.3f/%.3f/%.3f P=%.3fkW f=%.2f", v1_,v2_,v3_,i1_,i2_,i3_,kw_,freq_);
  }

  void parse_pwr_() {
    const uint8_t *r = res_;
    ca_=mk_s16(r[18],r[19],1000.0f); cb_=mk_s16(r[20],r[21],1000.0f); cc_=mk_s16(r[22],r[23],1000.0f);
    pa_=mk_u16le(r+24); pb_=mk_u16le(r+26); pc_=mk_u16le(r+28);
    qa_=mk_s16(r[30],r[31],1.0f); qb_=mk_s16(r[32],r[33],1.0f); qc_=mk_s16(r[34],r[35],1.0f);
    sa_=mk_u16le(r+36); sb_=mk_u16le(r+38); sc_=mk_u16le(r+40);
    temp_=(r[42]>=128)?-(float)(r[42]-128):(float)r[42];
    ESP_LOGI(TAG, "Pa=%.0f Pb=%.0f Pc=%.0f T=%.0f", pa_,pb_,pc_,temp_);
  }

  void parse_getinfo_() {
    uint32_t w  = mk4le_(res_+18);
    uint32_t sy = mk4le_(res_+32);
    work_time_ = fmt_sec_(w);
    sync_time_ = fmt_sec_(sy);
  }

  void parse_bat_() {
    if (res_len_ < 15) return;
    uint8_t tot = res_[13], rem = res_[14];
    bat_pct_ = (tot > 0) ? (100.0f * rem / tot) : NAN;
  }

  void parse_str_(uint8_t cmd, uint8_t field) {
    if (res_len_ < 14) return;
    char buf[31]{}; memcpy(buf, res_+14, 30);
    for (int i=29; i>=0 && (buf[i]==' ' || buf[i]==0); i--) buf[i]=0;
    if (cmd == CMD_FACTORY && field == 1) serial_ = buf;
    if (cmd == CMD_ABON) {
      if (field == 1) abon_ = buf;
      else if (field == 6 && strlen(buf) > 0) { abon_ += " / "; abon_ += buf; }
    }
  }

  uint32_t mk4le_(const uint8_t *b) {
    return ((uint32_t)b[3]<<24)|((uint32_t)b[2]<<16)|((uint32_t)b[1]<<8)|b[0];
  }

  std::string fmt_sec_(uint32_t s) {
    uint32_t d=s/86400, h=(s%86400)/3600, m=(s%3600)/60;
    char b[24];
    if (d) snprintf(b, 24, "%ud %02u:%02u", (unsigned)d, (unsigned)h, (unsigned)m);
    else   snprintf(b, 24, "%02u:%02u", (unsigned)h, (unsigned)m);
    return b;
  }

  // ── Публикация сенсоров ───────────────────────────────────────────────
  void pf_(int i, float v) { if (ss_[i] && !std::isnan(v)) ss_[i]->publish_state(v); }

  void pub_all_() {
    pf_(SUM,sum_); pf_(T1,t1_); pf_(T2,t2_); pf_(T3,t3_);
    pf_(SUM_R,sum_r_); pf_(T1_R,t1_r_); pf_(T2_R,t2_r_); pf_(T3_R,t3_r_);
    pf_(KW,kw_); pf_(KVAR,kvar_); pf_(FREQ,freq_); pf_(COS,cos_);
    pf_(V1,v1_); pf_(V2,v2_); pf_(V3,v3_);
    pf_(I1,i1_); pf_(I2,i2_); pf_(I3,i3_);
    pf_(PA,pa_); pf_(PB,pb_); pf_(PC,pc_);
    pf_(QA,qa_); pf_(QB,qb_); pf_(QC,qc_);
    pf_(SA,sa_); pf_(SB,sb_); pf_(SC,sc_);
    pf_(CA,ca_); pf_(CB,cb_); pf_(CC_S,cc_);
    pf_(TEMP,temp_); pf_(BAT,bat_pct_);
    pub_text_();
  }

  void pub_text_() {
    if (ts_[T_TAR])   ts_[T_TAR]->publish_state(tariff_);
    if (ts_[T_RELAY]) ts_[T_RELAY]->publish_state(relay_);
    if (ts_[T_SEAL])  ts_[T_SEAL]->publish_state(seal_);
    if (ts_[T_TYPE])  ts_[T_TYPE]->publish_state(type_name_);
    if (ts_[T_FW])    ts_[T_FW]->publish_state(fw_);
    if (ts_[T_DATE])  ts_[T_DATE]->publish_state(meter_date_);
    if (ts_[T_TIME])  ts_[T_TIME]->publish_state(meter_time_);
    if (ts_[T_WORK])  ts_[T_WORK]->publish_state(work_time_);
    if (ts_[T_SYNC])  ts_[T_SYNC]->publish_state(sync_time_);
    if (ts_[T_SER])   ts_[T_SER]->publish_state(serial_);
    if (ts_[T_ABON])  ts_[T_ABON]->publish_state(abon_);
    if (ts_[T_ST])    ts_[T_ST]->publish_state(status_);
    if (bs_[B_3P])    bs_[B_3P]->publish_state(three_phase_);
    if (bs_[B_REL])   bs_[B_REL]->publish_state(relay_ == "On");
    if (bs_[B_SEAL])  bs_[B_SEAL]->publish_state(seal_ == "OK");
  }
};

}  // namespace mirtek_cc1101
}  // namespace esphome
