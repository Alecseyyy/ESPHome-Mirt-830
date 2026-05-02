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

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek";

// Перечисления для индексов (согласно вашей конфигурации)
enum SI { SUM, T1, T2, T3, SUM_R, T1_R, T2_R, T3_R, KW, KVAR, FREQ, COS, V1, V2, V3, I1, I2, I3, TEMP, BAT, _SCOUNT };
enum TI { T_TAR, T_RELAY, T_SEAL, T_TYPE, T_FW, T_DATE, T_TIME, T_WORK, T_SYNC, T_SER, T_ABON, T_ST };
enum BI { B_3P, B_REL, B_SEAL, B_CC };

class MirtekCC1101 : public Component {
 public:
  // Массивы указателей на сенсоры (имена должны точно совпадать с теми, что в main.cpp)
  sensor::Sensor *ss[32]{nullptr};
  text_sensor::TextSensor *ts[12]{nullptr};
  binary_sensor::BinarySensor *bs[4]{nullptr};

  // Методы-сеттеры, которые вызывает ESPHome при сборке
  void set_sensor(int idx, sensor::Sensor *s) { if (idx < 32) ss[idx] = s; }
  void set_text_sensor(int idx, text_sensor::TextSensor *s) { if (idx < 12) ts[idx] = s; }
  void set_binary_sensor(int idx, binary_sensor::BinarySensor *s) { if (idx < 4) bs[idx] = s; }

  void set_meter_address(int a) { addr_ = a; }
  void set_cs_pin(int p) { cs_ = p; }
  void set_gdo0_pin(int p) { gdo0_ = p; }

  float get_setup_priority() const override { return setup_priority::LATE; }

  void setup() override {
    ESP_LOGI(TAG, "Starting Mirtek CC1101...");
    ELECHOUSE_cc1101.setGDO0(gdo0_);
    cc1101_ok_ = ELECHOUSE_cc1101.getCC1101();
    
    if (bs[B_CC]) bs[B_CC]->publish_state(cc1101_ok_);

    if (cc1101_ok_) {
      ELECHOUSE_cc1101.Init();
      // Тут можно добавить специфичные настройки регистров CC1101
      status_ = "Ready";
    } else {
      status_ = "CC1101 Error";
      ESP_LOGE(TAG, "CC1101 not found!");
    }
    publish_status_();
  }

  void loop() override {}

  // Метод для отрисовки (если используется в YAML)
  void draw(display::DisplayBuffer &d) {
    d.printf(0, 0, "Mirtek: %s", status_.c_str());
  }

 protected:
  int addr_{0}, cs_{5}, gdo0_{22};
  bool cc1101_ok_{false};
  std::string status_ = "Init";
  std::string serial_ = "Unknown";

  void publish_status_() {
    if (ts[T_ST]) ts[T_ST]->publish_state(status_);
  }
};

}  // namespace mirtek_cc1101
}  // namespace esphome
