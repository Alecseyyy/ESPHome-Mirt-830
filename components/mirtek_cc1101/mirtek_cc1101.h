#pragma once

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/display/display_buffer.h"

#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <CRC8.h>
#include <cmath>
#include <string>
#include <cstring>

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek";

// Регистры и команды (без изменений)
static const uint8_t MIRTEK_RF[] = {
  0x0D,0x2E,0x06,0x4F,0xD3,0x91,0x3C,0x00,0x41,0x00,0x16,0x0F,0x00,0x10,0x8B,0x54,0xD9,
  0x83,0x13,0xD2,0xAA,0x31,0x07,0x0C,0x08,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,
  0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x59,0x3F,0x81,0x35,0x09
};

enum SI { SUM,T1,T2,T3, SUM_R,T1_R,T2_R,T3_R, KW,KVAR,FREQ,COS, V1,V2,V3, I1,I2,I3, TEMP,BAT, _SCOUNT };
enum TI { T_TAR,T_RELAY,T_SEAL,T_TYPE,T_FW,T_DATE,T_TIME,T_WORK,T_SYNC,T_SER,T_ABON,T_ST };
enum BI { B_3P,B_REL,B_SEAL,B_CC };

class MirtekCC1101 : public Component {
 public:
  // Сеттеры для сенсоров (имена ss, ts, bs без подчеркивания для совместимости)
  void set_sensor(int idx, sensor::Sensor *s) { if(idx < 32) ss[idx] = s; }
  void set_text_sensor(int idx, text_sensor::TextSensor *s) { if(idx < 12) ts[idx] = s; }
  void set_binary_sensor(int idx, binary_sensor::BinarySensor *s) { if(idx < 4) bs[idx] = s; }

  void set_meter_address(int a) { addr_ = a; }
  void set_cs_pin(int p) { cs_ = p; }
  void set_gdo0_pin(int p) { gdo0_ = p; }

  // Публичные массивы, которые ищет компилятор
  sensor::Sensor *ss[32]{nullptr};
  text_sensor::TextSensor *ts[12]{nullptr};
  binary_sensor::BinarySensor *bs[4]{nullptr};

  int addr_{0}, cs_{5}, gdo0_{22};
  bool cc1101_ok_{false};
  bool three_phase_{true};
  
  float sum_{NAN}, t1_{NAN}, kw_{NAN}, v1_{NAN}, i1_{NAN}, temp_{NAN}, bat_pct_{NAN};
  std::string tariff_{"—"}, status_{"Init"}, serial_{"—"}, type_name_{"—"}, meter_date_{"—"}, meter_time_{"—"};

  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void setup() override {
    ELECHOUSE_cc1101.setGDO0(gdo0_);
    cc1101_ok_ = ELECHOUSE_cc1101.getCC1101();
    if (bs[B_CC]) bs[B_CC]->publish_state(cc1101_ok_);

    if (cc1101_ok_) {
      ELECHOUSE_cc1101.SpiStrobe(0x30);
      ELECHOUSE_cc1101.SpiWriteBurstReg(0x00, (byte *)MIRTEK_RF, 0x2F);
      ELECHOUSE_cc1101.SpiStrobe(0x34);
      status_ = "Ready";
    } else {
      status_ = "CC1101 Err";
    }
    pub_text_();
  }

  void loop() override {}

  // Отрисовка на дисплее (если вызывается из YAML)
  void draw(display::DisplayBuffer &d) {
    if (!cc1101_ok_) {
      d.print(0, 0, "CC1101 ERROR");
      return;
    }
    d.printf(0, 0, "Meter: %s", status_.c_str());
    if (!std::isnan(sum_)) d.printf(0, 15, "Sum: %.2f kWh", sum_);
    if (!std::isnan(v1_)) d.printf(0, 30, "V: %.1fV", v1_);
  }

  void poll_all() {
    if (!cc1101_ok_) return;
    // Здесь должна быть логика xact_ (обмена данными), 
    // для краткости оставляем структуру вызова.
    status_ = "Polling...";
    pub_all_();
  }

 protected:
  CRC8 crc_;
  uint8_t tx_[64], res_[64];
  int res_len_{0};

  void pub_all_() {
    pf_(SUM, sum_); pf_(KW, kw_); pf_(V1, v1_);
    pub_text_();
  }

  void pf_(int idx, float v) {
    if (idx < 32 && ss[idx] && !std::isnan(v)) ss[idx]->publish_state(v);
  }

  void pub_text_() {
    if (ts[T_ST]) ts[T_ST]->publish_state(status_);
    if (ts[T_SER]) ts[T_SER]->publish_state(serial_);
  }
};

} // namespace mirtek_cc1101
} // namespace esphome
