#include "mirtek_cc1101.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek_cc1101";

// --------------------------------------------------------------------------
// SPI low-level helpers (bit-banged via GPIOPin since cc1101 needs full control)
// We use the Arduino SPI via esphome raw HAL calls on the SPI bus pins.
// For simplicity and maximum compatibility, we use direct byte transfer via
// the platform SPI peripheral. ESPHome's SPIDevice is NOT used here because
// CC1101 requires chip-select control around every burst operation.
// We drive CS manually and use Arduino SPI.transfer().
// --------------------------------------------------------------------------

#ifdef ARDUINO
#include <SPI.h>

void MirtekCC1101::spi_begin_() {
  cs_pin_->digital_write(false);
  delayMicroseconds(1);
}

void MirtekCC1101::spi_end_() {
  cs_pin_->digital_write(true);
  delayMicroseconds(1);
}

void MirtekCC1101::cc1101_write_reg_(uint8_t addr, uint8_t val) {
  spi_begin_();
  SPI.transfer(addr & 0x3F);
  SPI.transfer(val);
  spi_end_();
}

void MirtekCC1101::cc1101_write_burst_(uint8_t addr, const uint8_t *data, size_t len) {
  spi_begin_();
  SPI.transfer((addr & 0x3F) | CC1101_BURST);
  for (size_t i = 0; i < len; i++) SPI.transfer(data[i]);
  spi_end_();
}

uint8_t MirtekCC1101::cc1101_read_reg_(uint8_t addr) {
  spi_begin_();
  SPI.transfer(CC1101_READ | (addr & 0x3F));
  uint8_t val = SPI.transfer(0x00);
  spi_end_();
  return val;
}

uint8_t MirtekCC1101::cc1101_strobe_(uint8_t cmd) {
  spi_begin_();
  uint8_t status = SPI.transfer(cmd);
  spi_end_();
  return status;
}

bool MirtekCC1101::cc1101_check_() {
  // Read part number – should be 0x00 for CC1101
  uint8_t partnum = cc1101_read_reg_(CC1101_READ | CC1101_BURST | CC1101_PARTNUM);
  uint8_t version = cc1101_read_reg_(CC1101_READ | CC1101_BURST | CC1101_VERSION);
  ESP_LOGD(TAG, "CC1101 partnum=0x%02X version=0x%02X", partnum, version);
  return (version == 0x14 || version == 0x04);
}

void MirtekCC1101::cc1101_reset_() {
  cs_pin_->digital_write(true);
  delayMicroseconds(5);
  cs_pin_->digital_write(false);
  delayMicroseconds(10);
  cs_pin_->digital_write(true);
  delayMicroseconds(41);
  cc1101_strobe_(CC1101_SRES);
  delay(10);
}

bool MirtekCC1101::cc1101_init_() {
  cc1101_reset_();
  cc1101_write_burst_(0x00, MIRTEK_RF_SETTINGS, sizeof(MIRTEK_RF_SETTINGS));
  cc1101_strobe_(CC1101_SCAL);
  delay(2);
  cc1101_strobe_(CC1101_SFRX);
  cc1101_strobe_(CC1101_SFTX);
  cc1101_strobe_(CC1101_SRX);
  return cc1101_check_();
}
#else
// Stub for IDF/non-Arduino – users should implement or use Arduino framework
void MirtekCC1101::spi_begin_() {}
void MirtekCC1101::spi_end_() {}
void MirtekCC1101::cc1101_write_reg_(uint8_t, uint8_t) {}
void MirtekCC1101::cc1101_write_burst_(uint8_t, const uint8_t *, size_t) {}
uint8_t MirtekCC1101::cc1101_read_reg_(uint8_t) { return 0; }
uint8_t MirtekCC1101::cc1101_strobe_(uint8_t) { return 0; }
void MirtekCC1101::cc1101_reset_() {}
bool MirtekCC1101::cc1101_check_() { return false; }
bool MirtekCC1101::cc1101_init_() { return false; }
#endif

// --------------------------------------------------------------------------
// Packet building
// --------------------------------------------------------------------------

static uint8_t mirtek_crc(const uint8_t *data, size_t len) {
  return crc8_a9(data, len);
}

void MirtekCC1101::build_short_packet_(uint8_t cmd, uint8_t packet_type) {
  packet_type_ = packet_type;
  uint8_t *b = send_buf_;
  b[0]  = 0x0F;  // total length 15
  b[1]  = 0x73;  // start1
  b[2]  = 0x55;  // start2
  b[3]  = 0x20;  // Parameters (D=1, Len=0)
  b[4]  = 0x00;  // reserve
  b[5]  = meter_address_ & 0xFF;
  b[6]  = (meter_address_ >> 8) & 0xFF;
  b[7]  = 0xFE;
  b[8]  = 0xFF;
  b[9]  = cmd;
  b[10] = 0x00; b[11] = 0x00; b[12] = 0x00; b[13] = 0x00;  // PIN
  b[14] = mirtek_crc(b + 3, b[0] - 4);  // CRC over bytes [3..13]
  b[15] = 0x55;  // stop
}

void MirtekCC1101::build_long_packet_(uint8_t cmd, uint8_t sub, uint8_t packet_type) {
  packet_type_ = packet_type;
  uint8_t *b = send_buf_;
  b[0]  = 0x10;  // total length 16
  b[1]  = 0x73;
  b[2]  = 0x55;
  b[3]  = 0x21;
  b[4]  = 0x00;
  b[5]  = meter_address_ & 0xFF;
  b[6]  = (meter_address_ >> 8) & 0xFF;
  b[7]  = 0xFE;
  b[8]  = 0xFF;
  b[9]  = cmd;
  b[10] = 0x00; b[11] = 0x00; b[12] = 0x00; b[13] = 0x00;
  b[14] = sub;
  b[15] = mirtek_crc(b + 3, b[0] - 4);
  b[16] = 0x55;
}

void MirtekCC1101::build_long2b_packet_(uint8_t cmd, uint8_t sub1, uint8_t sub2, uint8_t packet_type) {
  packet_type_ = packet_type;
  uint8_t *b = send_buf_;
  b[0]  = 0x11;
  b[1]  = 0x73;
  b[2]  = 0x55;
  b[3]  = 0x22;
  b[4]  = 0x00;
  b[5]  = meter_address_ & 0xFF;
  b[6]  = (meter_address_ >> 8) & 0xFF;
  b[7]  = 0xFE;
  b[8]  = 0xFF;
  b[9]  = cmd;
  b[10] = 0x00; b[11] = 0x00; b[12] = 0x00; b[13] = 0x00;
  b[14] = sub1;
  b[15] = sub2;
  b[16] = mirtek_crc(b + 3, b[0] - 4);
  b[17] = 0x55;
}

void MirtekCC1101::apply_byte_stuffing_(const uint8_t *in, size_t in_len, std::vector<uint8_t> &out) {
  // First 3 bytes (len, start1, start2) pass through unchanged
  out.push_back(in[0]);
  out.push_back(in[1]);
  out.push_back(in[2]);
  for (size_t i = 3; i < in_len - 1; i++) {
    if (in[i] == 0x55) {
      out.push_back(0x73);
      out.push_back(0x11);
    } else if (in[i] == 0x73) {
      out.push_back(0x73);
      out.push_back(0x22);
    } else {
      out.push_back(in[i]);
    }
  }
  // Last byte (stop 0x55) unchanged
  out.push_back(in[in_len - 1]);
}

bool MirtekCC1101::remove_byte_stuffing_(const uint8_t *in, size_t in_len, std::vector<uint8_t> &out) {
  for (size_t i = 0; i < in_len; i++) {
    if (in[i] == 0x73 && (i + 1) < in_len) {
      if (in[i + 1] == 0x11) {
        out.push_back(0x55);
        i++;
      } else if (in[i + 1] == 0x22) {
        out.push_back(0x73);
        i++;
      } else {
        out.push_back(in[i]);
      }
    } else {
      out.push_back(in[i]);
    }
  }
  return true;
}

// --------------------------------------------------------------------------
// Send / receive
// --------------------------------------------------------------------------

bool MirtekCC1101::send_packet_() {
#ifdef ARDUINO
  // Build byte-stuffed TX buffer
  uint8_t raw_len = send_buf_[0] + 1;  // total bytes including stop
  std::vector<uint8_t> stuffed;
  apply_byte_stuffing_(send_buf_, raw_len, stuffed);

  // Update length in first byte
  stuffed[0] = (uint8_t)(stuffed.size() - 1);  // CC1101 sends stuffed[0]+1 bytes

  cc1101_strobe_(CC1101_SCAL);
  delay(1);
  cc1101_strobe_(CC1101_SFTX);
  cc1101_strobe_(CC1101_SIDLE);
  cc1101_write_reg_(CC1101_PA_TABLE, 0xC4);  // 10 dBm

  // Write to TX FIFO
  spi_begin_();
  SPI.transfer(CC1101_TXFIFO | CC1101_BURST);
  for (uint8_t b : stuffed) SPI.transfer(b);
  spi_end_();

  cc1101_strobe_(CC1101_STX);
  // Wait for TX to complete (GDO0 goes high then low)
  uint32_t t0 = millis();
  while (gdo0_pin_->digital_read() == false && millis() - t0 < 100) yield();
  while (gdo0_pin_->digital_read() == true  && millis() - t0 < 500) yield();

  cc1101_strobe_(CC1101_SFRX);
  cc1101_strobe_(CC1101_SRX);
  ESP_LOGD(TAG, "Packet sent (%u bytes stuffed)", (unsigned)stuffed.size());
  return true;
#else
  return false;
#endif
}

bool MirtekCC1101::receive_packet_() {
#ifdef ARDUINO
  uint32_t timeout_ms = 800;
  uint32_t t0 = millis();
  int pack_count = 0;
  result_len_ = 0;

  std::vector<uint8_t> raw_all;

  while (millis() - t0 < timeout_ms && pack_count < (int)packet_type_) {
    // Check if data available: RXBYTES
    uint8_t rxb = cc1101_read_reg_(CC1101_READ | CC1101_BURST | 0x3B);  // RXBYTES
    if (rxb > 0) {
      pack_count++;
      uint8_t len_byte = 0;
      // Read first byte to get length
      spi_begin_();
      SPI.transfer(CC1101_RXFIFO | CC1101_READ | CC1101_BURST);
      len_byte = SPI.transfer(0x00);
      spi_end_();

      if (len_byte == 0 || len_byte > 50) {
        // Flush and continue
        cc1101_strobe_(CC1101_SIDLE);
        cc1101_strobe_(CC1101_SFRX);
        cc1101_strobe_(CC1101_SFTX);
        cc1101_strobe_(CC1101_SRX);
        continue;
      }

      spi_begin_();
      SPI.transfer(CC1101_RXFIFO | CC1101_READ | CC1101_BURST);
      for (uint8_t i = 1; i < len_byte; i++) {
        raw_all.push_back(SPI.transfer(0x00));
      }
      spi_end_();

      cc1101_strobe_(CC1101_SIDLE);
      cc1101_strobe_(CC1101_SFRX);
      cc1101_strobe_(CC1101_SFTX);
      cc1101_strobe_(CC1101_SRX);
    }
    yield();
  }

  if (raw_all.empty()) {
    ESP_LOGW(TAG, "No data received");
    return false;
  }

  std::vector<uint8_t> destuffed;
  remove_byte_stuffing_(raw_all.data(), raw_all.size(), destuffed);

  if (destuffed.size() > sizeof(result_buf_)) {
    ESP_LOGW(TAG, "Result buffer overflow");
    return false;
  }

  memcpy(result_buf_, destuffed.data(), destuffed.size());
  result_len_ = destuffed.size();

  // Compute CRC over bytes [2 .. len-3]
  if (result_len_ < 5) return false;

  size_t crc_end = result_len_ - 2;
  // Handle Mirtek firmware quirk (0x76 variant)
  if (result_len_ >= 3 && result_buf_[result_len_ - 3] == 0x76) {
    crc_end = result_len_ - 3;
  }
  my_crc_ = mirtek_crc(result_buf_ + 2, crc_end - 2);

  ESP_LOGD(TAG, "Received %u bytes, CRC=0x%02X", (unsigned)result_len_, my_crc_);
  return true;
#else
  return false;
#endif
}

// --------------------------------------------------------------------------
// Parsers
// --------------------------------------------------------------------------

bool MirtekCC1101::parse_datetime_() {
  const uint8_t *r = result_buf_;
  if (result_len_ < 22) return false;
  if (r[0] != 0x73 || r[1] != 0x55) return false;
  if (r[6] != (meter_address_ & 0xFF) || r[7] != ((meter_address_ >> 8) & 0xFF)) return false;
  if (r[8] != 0x1C) return false;

  // Detect meter type from byte [9]
  switch (r[9]) {
    case 0xA8: case 0xA9:
      three_phase = true;
      meter_type_str = "3ф трансф. акт-реакт. (A8)";
      break;
    case 0x98: case 0x99:
      three_phase = false;
      meter_type_str = "1ф 2эл. акт-реакт. (98)";
      break;
    case 0x88: case 0x89:
      three_phase = true;
      meter_type_str = "3ф трансф. акт. двунапр. (88)";
      break;
    case 0x80: case 0x81:
      three_phase = true;
      meter_type_str = "3ф акт. двунапр. (80)";
      break;
    case 0x68: case 0x69:
      three_phase = true;
      meter_type_str = "3ф трансф. акт. одноnapр. (68)";
      break;
    default:
      meter_type_str = "Тип 0x" + format_hex(r[9]);
      three_phase = (r[9] & 0x08) == 0;
      break;
  }

  // Status byte[10]
  bool sy = (r[10] >> 6) & 1;

  char buf[64];
  snprintf(buf, sizeof(buf), "%02d-%02d-%02d %02d:%02d:%02d",
           r[17], r[18], r[19], r[15], r[14], r[13]);
  datetime_str = std::string(buf);

  if (sy) datetime_str += " *SYNC";

  publish_binary_(time_synced_, sy);
  publish_text_(meter_type_, meter_type_str);
  publish_text_(meter_datetime_, datetime_str);
  return true;
}

bool MirtekCC1101::parse_energy_() {
  const uint8_t *r = result_buf_;
  if (result_len_ < 45) return false;
  if (r[0] != 0x73 || r[1] != 0x55) return false;
  if (r[6] != (meter_address_ & 0xFF) || r[7] != ((meter_address_ >> 8) & 0xFF)) return false;
  if (r[8] != 0x05) return false;

  // Configuration byte determines decimal point position
  uint8_t cfg = r[14];
  uint8_t dp_bits = cfg & 0x03;
  float divisor = 1.0f;
  switch (dp_bits) {
    case 0: divisor = 1.0f;    break;
    case 1: divisor = 10.0f;   break;
    case 2: divisor = 100.0f;  break;
    case 3: divisor = 1000.0f; break;
  }
  // Active tariffs count from bits 7:6
  uint8_t tariff_count = (cfg >> 6) + 1;  // 0->1, 1->2, 2->3, 3->4
  uint8_t current_tariff = (cfg >> 2) & 0x03;

  // Parse energy values (positions as per Arduino code)
  energy_sum_val = (float)(
    ((uint32_t)r[22] << 24) | ((uint32_t)r[21] << 16) | ((uint32_t)r[20] << 8) | r[19]
  ) / 100.0f;
  energy_t1_val = (float)(
    ((uint32_t)r[30] << 24) | ((uint32_t)r[29] << 16) | ((uint32_t)r[28] << 8) | r[27]
  ) / 100.0f;
  energy_t2_val = (float)(
    ((uint32_t)r[34] << 24) | ((uint32_t)r[33] << 16) | ((uint32_t)r[32] << 8) | r[31]
  ) / 100.0f;
  if (tariff_count >= 3 && result_len_ >= 43) {
    energy_t3_val = (float)(
      ((uint32_t)r[38] << 24) | ((uint32_t)r[37] << 16) | ((uint32_t)r[36] << 8) | r[35]
    ) / 100.0f;
  }

  switch (current_tariff) {
    case 0: tariff_str = "День (Т1)";    break;
    case 1: tariff_str = "Ночь (Т2)";   break;
    case 2: tariff_str = "Полупик (Т3)"; break;
    case 3: tariff_str = "Специальный"; break;
    default: tariff_str = "N/A";
  }

  publish_sensor_(energy_sum_, energy_sum_val);
  publish_sensor_(energy_t1_, energy_t1_val);
  publish_sensor_(energy_t2_, energy_t2_val);
  if (!std::isnan(energy_t3_val)) publish_sensor_(energy_t3_, energy_t3_val);
  publish_text_(tariff_text_, tariff_str);

  ESP_LOGI(TAG, "Energy SUM=%.2f T1=%.2f T2=%.2f T3=%.2f Tariff=%s",
           energy_sum_val, energy_t1_val, energy_t2_val, energy_t3_val, tariff_str.c_str());
  return true;
}

bool MirtekCC1101::parse_voltage_current_() {
  const uint8_t *r = result_buf_;
  if (result_len_ < 43) return false;
  if (r[0] != 0x73 || r[1] != 0x55) return false;
  if (r[6] != (meter_address_ & 0xFF) || r[7] != ((meter_address_ >> 8) & 0xFF)) return false;
  if (r[8] != 0x2B) return false;

  if (r[9] == 0x98 || r[9] == 0x99) {
    // Single-phase meter
    p_total = (float)((uint16_t)r[18] | ((uint16_t)r[19] << 8));
    bool q_neg = (r[21] >= 128);
    q_total = q_neg ?
      (float)((uint16_t)r[20] | ((uint16_t)(r[21] - 128) << 8)) / -1000.0f :
      (float)((uint16_t)r[20] | ((uint16_t)r[21] << 8)) / 1000.0f;
    freq = (float)((uint16_t)r[22] | ((uint16_t)r[23] << 8)) / 100.0f;
    bool pf_neg = (r[25] >= 128);
    pf = pf_neg ?
      (float)((uint16_t)r[24] | ((uint16_t)(r[25] - 128) << 8)) / -10.0f :
      (float)((uint16_t)r[24] | ((uint16_t)r[25] << 8)) / 10.0f;
    va = (float)((uint16_t)r[26] | ((uint16_t)r[27] << 8)) / 100.0f;
    vb = (float)((uint16_t)r[28] | ((uint16_t)r[29] << 8)) / 100.0f;
    vc = (float)((uint16_t)r[30] | ((uint16_t)r[31] << 8)) / 100.0f;
    ia = (float)((uint32_t)r[32] | ((uint32_t)r[33] << 8) | ((uint32_t)r[34] << 16)) / 1000.0f;
    ib = (float)((uint32_t)r[35] | ((uint32_t)r[36] << 8) | ((uint32_t)r[37] << 16)) / 1000.0f;
    ic = (float)((uint32_t)r[38] | ((uint32_t)r[39] << 8) | ((uint32_t)r[40] << 16)) / 1000.0f;
  } else {
    // Three-phase meter
    p_total = (float)((uint32_t)r[18] | ((uint32_t)r[19] << 8) | ((uint32_t)r[20] << 16));
    bool q_neg = (r[23] >= 128);
    q_total = q_neg ?
      (float)((uint32_t)r[21] | ((uint32_t)r[22] << 8) | ((uint32_t)(r[23] - 128) << 16)) / -1000.0f :
      (float)((uint32_t)r[21] | ((uint32_t)r[22] << 8) | ((uint32_t)r[23] << 16)) / 1000.0f;
    freq = (float)((uint16_t)r[24] | ((uint16_t)r[25] << 8)) / 100.0f;
    bool pf_neg = (r[27] >= 128);
    pf = pf_neg ?
      (float)((uint16_t)r[26] | ((uint16_t)(r[27] - 128) << 8)) / -10.0f :
      (float)((uint16_t)r[26] | ((uint16_t)r[27] << 8)) / 10.0f;
    va = (float)((uint16_t)r[28] | ((uint16_t)r[29] << 8)) / 100.0f;
    vb = (float)((uint16_t)r[30] | ((uint16_t)r[31] << 8)) / 100.0f;
    vc = (float)((uint16_t)r[32] | ((uint16_t)r[33] << 8)) / 100.0f;
    ia = (float)((uint32_t)r[34] | ((uint32_t)r[35] << 8) | ((uint32_t)r[36] << 16)) / 1000.0f;
    ib = (float)((uint32_t)r[37] | ((uint32_t)r[38] << 8) | ((uint32_t)r[39] << 16)) / 1000.0f;
    ic = (float)((uint32_t)r[40] | ((uint32_t)r[41] << 8) | ((uint32_t)r[42] << 16)) / 1000.0f;
  }

  publish_sensor_(voltage_a_, va);
  if (three_phase) {
    publish_sensor_(voltage_b_, vb);
    publish_sensor_(voltage_c_, vc);
  }
  publish_sensor_(current_a_, ia);
  if (three_phase) {
    publish_sensor_(current_b_, ib);
    publish_sensor_(current_c_, ic);
  }
  publish_sensor_(power_active_, p_total);
  publish_sensor_(power_reactive_, q_total);
  publish_sensor_(frequency_, freq);
  publish_sensor_(power_factor_, pf);

  ESP_LOGI(TAG, "Va=%.2f Vb=%.2f Vc=%.2f Ia=%.3f Ib=%.3f Ic=%.3f P=%.0f Q=%.3f f=%.2f cos=%.2f",
           va, vb, vc, ia, ib, ic, p_total, q_total, freq, pf);
  return true;
}

bool MirtekCC1101::parse_power_() {
  const uint8_t *r = result_buf_;
  if (result_len_ < 44) return false;
  if (r[0] != 0x73 || r[1] != 0x55) return false;
  if (r[6] != (meter_address_ & 0xFF) || r[7] != ((meter_address_ >> 8) & 0xFF)) return false;
  if (r[8] != 0x2B) return false;

  auto signed_pf = [&](size_t idx) -> float {
    bool neg = (r[idx + 1] >= 128);
    return neg ?
      (float)((uint16_t)r[idx] | ((uint16_t)(r[idx + 1] - 128) << 8)) / -10.0f :
      (float)((uint16_t)r[idx] | ((uint16_t)r[idx + 1] << 8)) / 10.0f;
  };
  auto signed_kvar = [&](size_t idx) -> float {
    bool neg = (r[idx + 1] >= 128);
    return neg ?
      (float)((uint16_t)r[idx] | ((uint16_t)(r[idx + 1] - 128) << 8)) / -1000.0f :
      (float)((uint16_t)r[idx] | ((uint16_t)r[idx + 1] << 8)) / 1000.0f;
  };

  float cosinA = signed_pf(18);
  float cosinB = signed_pf(20);
  float cosinC = signed_pf(22);

  pa = (float)((uint16_t)r[24] | ((uint16_t)r[25] << 8));
  pb = (float)((uint16_t)r[26] | ((uint16_t)r[27] << 8));
  pc = (float)((uint16_t)r[28] | ((uint16_t)r[29] << 8));

  float qa = signed_kvar(30);
  float qb = signed_kvar(32);
  float qc = signed_kvar(34);

  float sa = (float)((uint16_t)r[36] | ((uint16_t)r[37] << 8));
  float sb = (float)((uint16_t)r[38] | ((uint16_t)r[39] << 8));
  float sc = (float)((uint16_t)r[40] | ((uint16_t)r[41] << 8));

  bool t_neg = (r[42] >= 128);
  temp = t_neg ? (float)(r[42] - 128) * -1.0f : (float)r[42];

  publish_sensor_(pf_a_, cosinA);
  publish_sensor_(pf_b_, cosinB);
  publish_sensor_(pf_c_, cosinC);
  publish_sensor_(power_a_, pa);
  publish_sensor_(power_b_, pb);
  publish_sensor_(power_c_, pc);
  publish_sensor_(reactive_a_, qa);
  publish_sensor_(reactive_b_, qb);
  publish_sensor_(reactive_c_, qc);
  publish_sensor_(apparent_a_, sa);
  publish_sensor_(apparent_b_, sb);
  publish_sensor_(apparent_c_, sc);
  publish_sensor_(temperature_, temp);

  ESP_LOGI(TAG, "Pa=%d Pb=%d Pc=%d Qa=%.3f Qb=%.3f Qc=%.3f Sa=%.0f Sb=%.0f Sc=%.0f T=%.1f",
           (int)pa, (int)pb, (int)pc, qa, qb, qc, sa, sb, sc, temp);
  return true;
}

bool MirtekCC1101::parse_extended_info_() {
  const uint8_t *r = result_buf_;
  if (result_len_ < 42) return false;
  if (r[0] != 0x73 || r[1] != 0x55) return false;
  if (r[6] != (meter_address_ & 0xFF) || r[7] != ((meter_address_ >> 8) & 0xFF)) return false;
  if (r[8] != 0x30) return false;

  hw_str = std::to_string((int)r[13]);

  char sw_buf[16];
  snprintf(sw_buf, sizeof(sw_buf), "%02d.%02d", (int)r[15], (int)r[14]);
  sw_str = sw_buf;

  // Status byte 1 (r[10])
  bool par_w  = (r[10] >> 7) & 1;
  bool par_sy = (r[10] >> 6) & 1;
  bool par_m2 = (r[10] >> 5) & 1;
  bool par_m1 = (r[10] >> 4) & 1;
  bool par_p3 = (r[10] >> 3) & 1;
  bool par_p2 = (r[10] >> 2) & 1;
  bool par_p1 = (r[10] >> 1) & 1;

  // Status byte 2 (r[11])
  bool par_n  = (r[11] >> 7) & 1;
  bool par_r2 = (r[11] >> 3) & 1;
  bool par_r1 = (r[11] >> 2) & 1;

  // Seal state
  seal_str = "";
  if (!par_p1 && !par_p2 && !par_p3 && !par_m1 && !par_m2) {
    seal_str = "OK";
  } else {
    if (par_p3) seal_str += "Вскрыт модуль связи; ";
    if (par_p2) seal_str += "Вскрыт корпус; ";
    if (par_p1) seal_str += "Вскрыта клеммная крышка; ";
    if (par_m1) seal_str += "Постоянный магнит; ";
    if (par_m2) seal_str += "Переменный магнит; ";
  }

  // Relay state
  // Version check: SW version <= 4 and par_r1=0 -> old logic
  bool old_relay_logic = ((int)r[15] <= 4 && !par_r1);
  if (old_relay_logic) {
    relay_str = par_r2 ? "Вкл" : "Выкл";
  } else {
    relay_str = par_r2 ? "Выкл" : "Вкл";
  }

  publish_text_(hw_version_, hw_str);
  publish_text_(sw_version_, sw_str);
  publish_text_(seal_state_, seal_str);
  publish_text_(relay_state_, relay_str);

  publish_binary_(seal_cover_, par_p2);
  publish_binary_(seal_terminal_, par_p1);
  publish_binary_(seal_module_, par_p3);
  publish_binary_(magnet_dc_, par_m1);
  publish_binary_(magnet_ac_, par_m2);
  publish_binary_(relay_on_, !par_r2);  // active = relay holding load
  publish_binary_(current_imbalance_, par_n);
  publish_binary_(time_synced_, par_sy);

  ESP_LOGI(TAG, "HW=%s SW=%s Seal=%s Relay=%s", hw_str.c_str(), sw_str.c_str(), seal_str.c_str(), relay_str.c_str());
  return true;
}

// --------------------------------------------------------------------------
// Publish helpers
// --------------------------------------------------------------------------

void MirtekCC1101::publish_sensor_(sensor::Sensor *s, float val) {
  if (s && !std::isnan(val)) s->publish_state(val);
}

void MirtekCC1101::publish_text_(text_sensor::TextSensor *s, const std::string &val) {
  if (s) s->publish_state(val);
}

void MirtekCC1101::publish_binary_(binary_sensor::BinarySensor *s, bool val) {
  if (s) s->publish_state(val);
}

// --------------------------------------------------------------------------
// Setup / Update
// --------------------------------------------------------------------------

void MirtekCC1101::setup() {
  ESP_LOGI(TAG, "Initialising Mirtek CC1101 gateway (addr=%u)", meter_address_);
  cs_pin_->setup();
  cs_pin_->digital_write(true);
  if (gdo0_pin_) gdo0_pin_->setup();

#ifdef ARDUINO
  SPI.begin();
  SPI.setFrequency(4000000);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
#endif

  if (!cc1101_init_()) {
    ESP_LOGE(TAG, "CC1101 init failed! Check SPI wiring.");
    publish_text_(last_status_, "CC1101 ERROR");
  } else {
    ESP_LOGI(TAG, "CC1101 init OK");
    publish_text_(last_status_, "OK");
  }
}

void MirtekCC1101::update() {
  ESP_LOGD(TAG, "Starting Mirtek poll cycle");
  bool ok = true;

  // 1. Date/time request (cmd=0x1C, 3 sub-packets)
  build_short_packet_(0x1C, 3);
  if (send_packet_() && receive_packet_()) {
    parse_datetime_();
  } else {
    ok = false;
  }

  // 2. Energy readout (cmd=0x05, sub=0x04 = absolute active, 4 sub-packets)
  build_long_packet_(0x05, 0x04, 4);
  if (send_packet_() && receive_packet_()) {
    parse_energy_();
  } else {
    ok = false;
  }

  // 3. Instantaneous voltage + current (cmd=0x2B, sub=0x00, 4 sub-packets)
  build_long_packet_(0x2B, 0x00, 4);
  if (send_packet_() && receive_packet_()) {
    parse_voltage_current_();
  } else {
    ok = false;
  }

  // 4. Instantaneous power (only for 3-phase, cmd=0x2B, sub=0x10, 4 sub-packets)
  if (three_phase) {
    build_long_packet_(0x2B, 0x10, 4);
    if (send_packet_() && receive_packet_()) {
      parse_power_();
    }
  }

  // 5. Extended info – once per hour (12th poll)
  extended_poll_count_++;
  if (extended_poll_count_ >= 12) {
    extended_poll_count_ = 0;
    build_short_packet_(0x30, 5);
    if (send_packet_() && receive_packet_()) {
      parse_extended_info_();
    }
  }

  publish_text_(last_status_, ok ? "OK" : "PARTIAL ERROR");
}

void MirtekCC1101::dump_config() {
  ESP_LOGCONFIG(TAG, "Mirtek CC1101 Gateway:");
  ESP_LOGCONFIG(TAG, "  Meter address: %u", meter_address_);
  ESP_LOGCONFIG(TAG, "  Poll interval: %u min", poll_interval_min_);
  LOG_PIN("  CS pin: ", cs_pin_);
  LOG_PIN("  GDO0 pin: ", gdo0_pin_);
}

}  // namespace mirtek_cc1101
}  // namespace esphome
