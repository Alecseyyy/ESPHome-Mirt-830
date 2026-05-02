#pragma once
/**
 * mirtek_cc1101.h  —  ESPHome компонент для счётчиков Миртек (Star 104/304)
 * Версия 2.1 — нативный ESPHome external_component с __init__.py
 *
 * Команды:
 *   0x01  Ping/Ident       — тип, версия ПО, реле, пломбы, статусы
 *   0x05  ReadEnergy       — активная/реактивная, 3 тарифа + сумма
 *   0x07  ReadAbonInfo     — данные абонента (лицевой счёт, ФИО)
 *   0x0A  ReadFactoryStr   — заводской №, дата пр-ва, наименование
 *   0x1C  ReadDateTime     — дата/время счётчика
 *   0x1E  ReadResBat       — ресурс батарейки
 *   0x2B  ReadInstantValue — U/I/P суммарные (0x00) + по фазам (0x10)
 *   0x30  GetInfo          — расш.инфо (время работы, версия платы)
 *   0x3A  OperateOverload  — управление реле
 *
 * CRC8: полином 0xA9. Байтстаффинг: 0x55→{0x73,0x11}, 0x73→{0x73,0x22}
 */
#include "esphome.h"
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <CRC8.h>
#include <cmath>
#include <string>

// ── RF-регистры CC1101 (47 байт, 433 МГц протокол Миртек) ──────────────
static const uint8_t MIRTEK_RF[] = {
  0x0D,0x2E,0x06,0x4F,0xD3,0x91,0x3C,0x00,0x41,0x00,0x16,
  0x0F,0x00,0x10,0x8B,0x54,0xD9,0x83,0x13,0xD2,0xAA,0x31,
  0x07,0x0C,0x08,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,
  0x56,0x10,0xE9,0x2A,0x00,0x1F,0x41,0x00,0x59,0x59,0x3F,
  0x81,0x35,0x09
};

// ── Коды команд ───────────────────────────────────────────────────────
static const uint8_t CMD_PING        = 0x01;
static const uint8_t CMD_ENERGY      = 0x05;
static const uint8_t CMD_ABON        = 0x07;
static const uint8_t CMD_FACTORY     = 0x0A;
static const uint8_t CMD_DATETIME    = 0x1C;
static const uint8_t CMD_BAT         = 0x1E;
static const uint8_t CMD_INSTANT     = 0x2B;
static const uint8_t CMD_GETINFO     = 0x30;
static const uint8_t CMD_RELAY       = 0x3A;

static const uint8_t SUB_E_ACT   = 0x04; // активная абс.
static const uint8_t SUB_E_REACT = 0x05; // реактивная абс.
static const uint8_t SUB_I_UI    = 0x00; // U,I,P,Q,f,cos
static const uint8_t SUB_I_PWR   = 0x10; // по фазам + T

// Типы счётчика
static const uint8_t MT_3P = 0xA8;
static const uint8_t MT_3P2= 0xA9;
static const uint8_t MT_1P = 0x98;
static const uint8_t MT_1P1= 0x90;

// ── Помощники декодирования ───────────────────────────────────────────
static inline float mk_s16(uint8_t lo, uint8_t hi, float d) {
  return (hi>=128)
    ? float(lo|((hi-128)<<8))/-d
    : float(lo|(hi<<8))/d;
}
static inline float mk_u32le(const uint8_t*b){
  return float(((uint32_t)b[3]<<24)|((uint32_t)b[2]<<16)|((uint32_t)b[1]<<8)|b[0]);
}
static inline float mk_u16le(const uint8_t*b){
  return float((uint16_t)b[1]<<8|b[0]);
}

namespace mirtek_cc1101 {

// ════════════════════════════════════════════════════════════════════════════
class MirtekCC1101 : public Component {
public:
  // ── Методы регистрации сенсоров (вызываются из __init__.py to_code) ──
  void set_sensor(int idx, sensor::Sensor *s)                   { if(idx<32) ss[idx]=s; }
  void set_text_sensor(int idx, text_sensor::TextSensor *s)     { if(idx<12) ts[idx]=s; }
  void set_binary_sensor(int idx, binary_sensor::BinarySensor*s){ if(idx<4)  bs[idx]=s; }

  int   addr_, cs_, gdo0_;
  bool  cc1101_ok_{false};
  int   poll_cnt_{0};
  CRC8  crc_;

  static const int BUF=64;
  uint8_t tx_[BUF]{}, raw_[BUF]{}, res_[BUF]{};
  int raw_len_{0}, res_len_{0};
  uint8_t my_crc_{0};

  // ── Данные ─────────────────────────────────────────────────────────
  float sum_{NAN},t1_{NAN},t2_{NAN},t3_{NAN};
  float sum_r_{NAN},t1_r_{NAN},t2_r_{NAN},t3_r_{NAN};
  float kw_{NAN},kvar_{NAN},freq_{NAN},cos_{NAN};
  float v1_{NAN},v2_{NAN},v3_{NAN};
  float i1_{NAN},i2_{NAN},i3_{NAN};
  float pa_{NAN},pb_{NAN},pc_{NAN};
  float qa_{NAN},qb_{NAN},qc_{NAN};
  float sa_{NAN},sb_{NAN},sc_{NAN};
  float ca_{NAN},cb_{NAN},cc_{NAN};
  float temp_{NAN};
  float bat_pct_{NAN};
  bool  three_phase_{true};
  uint8_t meter_type_{MT_3P};
  std::string tariff_{"—"}, relay_{"—"}, seal_{"—"}, type_name_{"—"};
  std::string fw_{"—"}, meter_date_{"—"}, meter_time_{"—"};
  std::string work_time_{"—"}, sync_time_{"—"};
  std::string serial_{"—"}, abon_{"—"}, status_{"Init"};

  // ── Сенсоры (задаются через set_sensors) ────────────────────────────
  sensor::Sensor *ss[34]{};  // 0..33 — плоский массив для простоты
  text_sensor::TextSensor *ts[12]{};
  binary_sensor::BinarySensor *bs[4]{};

  // Индексы в массиве ss[]
  enum SI {
    SUM,T1,T2,T3, SUM_R,T1_R,T2_R,T3_R,
    KW,KVAR,FREQ,COS,
    V1,V2,V3,I1,I2,I3,
    PA,PB,PC,QA,QB,QC,SA,SB,SC,CA,CB,CC,
    TEMP,BAT_PCT, _SCOUNT
  };
  enum TI { T_TAR,T_RELAY,T_SEAL,T_TYPE,T_FW,T_DATE,T_TIME,T_WORK,T_SYNC,T_SER,T_ABON,T_ST };
  enum BI { B_3P,B_REL,B_SEAL,B_CC };

  // ── Дисплей ─────────────────────────────────────────────────────────
  display::Display    *disp_{nullptr};
  display::Font       *font_b_{nullptr};
  display::Font       *font_s_{nullptr};
  uint8_t  pg_{0};
  uint32_t pg_t_{0};
  uint32_t pg_ms_{5000};

  // ── Конструктор ──────────────────────────────────────────────────────
  MirtekCC1101(int addr, int cs, int gdo0)
    : addr_(addr), cs_(cs), gdo0_(gdo0) {}

  void set_display(display::Display*d, display::Font*b, display::Font*s, uint32_t ms=5000){
    disp_=d; font_b_=b; font_s_=s; pg_ms_=ms;
  }

  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  // ── Инициализация CC1101 ────────────────────────────────────────────
  void setup() override {
    ESP_LOGI("mirtek","Init CC1101 cs=%d gdo0=%d addr=%d",cs_,gdo0_,addr_);
    ELECHOUSE_cc1101.setGDO0(gdo0_);
    cc1101_ok_ = ELECHOUSE_cc1101.getCC1101();
    if (bs[B_CC]) bs[B_CC]->publish_state(cc1101_ok_);
    if (!cc1101_ok_) {
      ESP_LOGE("mirtek","CC1101 SPI error!"); status_="CC1101 ERROR"; pub_text(); return;
    }
    ELECHOUSE_cc1101.SpiStrobe(0x30);
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x00,(byte*)MIRTEK_RF,0x2F);
    ELECHOUSE_cc1101.SpiStrobe(0x33); delay(1);
    ELECHOUSE_cc1101.SpiStrobe(0x3A); ELECHOUSE_cc1101.SpiStrobe(0x3B);
    ELECHOUSE_cc1101.SpiStrobe(0x34);
    ESP_LOGI("mirtek","CC1101 OK, RX mode");
    status_="Ready";
  }

  void loop() override {
    if (disp_ && millis()-pg_t_>pg_ms_) {
      pg_t_=millis();
      pg_=(pg_+1)%4;
    }
  }

  // ════════════════════════════════════════════════════════════════════
  // ПУБЛИЧНЫЙ API
  // ════════════════════════════════════════════════════════════════════
  void poll_all() {
    if (!cc1101_ok_) return;
    poll_cnt_++;
    ESP_LOGI("mirtek","=== Poll #%d addr=%d ===",poll_cnt_,addr_);
    // 1. Идентификация
    if (xact(CMD_PING,-1))       parse_ping();
    // 2. Дата/время
    if (xact(CMD_DATETIME,-1))   parse_datetime();
    // 3. Активная энергия (3 тарифа)
    if (xact(CMD_ENERGY,1,SUB_E_ACT))   parse_energy(false);
    // 4. Реактивная энергия (3 тарифа)
    if (xact(CMD_ENERGY,1,SUB_E_REACT)) parse_energy(true);
    // 5. Мгновенные U,I,P суммарные
    if (xact(CMD_INSTANT,1,SUB_I_UI))   parse_ui();
    // 6. Мгновенные по фазам + T
    if (three_phase_ && xact(CMD_INSTANT,1,SUB_I_PWR)) parse_pwr();
    // 7. Расширенная информация
    if (xact(CMD_GETINFO,-1))    parse_getinfo();
    // 8. Ресурс батарейки
    if (xact(CMD_BAT,-1))        parse_bat();
    // 9. Заводские данные (раз в опрос)
    if (xact(CMD_FACTORY,1,1))   parse_str(CMD_FACTORY,1);
    if (xact(CMD_FACTORY,1,4))   parse_str(CMD_FACTORY,4);
    // 10. Данные абонента
    if (xact(CMD_ABON,1,1))      parse_str(CMD_ABON,1);
    if (xact(CMD_ABON,1,6))      parse_str(CMD_ABON,6);

    pub_all();
    char st[32]; snprintf(st,32,"OK #%d",poll_cnt_);
    status_=st; pub_text();
    ESP_LOGI("mirtek","=== Done ===");
  }

  void relay_on()  { ctrl_relay(0x00); }
  void relay_off() { ctrl_relay(0x01); }
  void ctrl_relay(uint8_t cmd){
    ESP_LOGI("mirtek","Relay %s", cmd==0?"ON":"OFF");
    build(CMD_RELAY,2,0x00,cmd); send(); recv(600);
    relay_=(cmd==0)?"Вкл":"Выкл";
    if (ts[T_RELAY]) ts[T_RELAY]->publish_state(relay_);
    if (bs[B_REL])   bs[B_REL]->publish_state(cmd==0);
  }

  // ── Отрисовка дисплея ──────────────────────────────────────────────
  void draw(display::Display &d) {
    d.clear();
    if (!cc1101_ok_) { if(font_s_) d.printf(0,0,font_s_,"CC1101 ERROR"); return; }
    switch(pg_){
      case 0: draw_energy(d);  break;
      case 1: draw_ui(d);      break;
      case 2: draw_power(d);   break;
      case 3: draw_info(d);    break;
    }
    // страница
    if (font_s_) {
      char pg[5]; snprintf(pg,5,"%d/4",pg_+1);
      d.printf(d.get_width()-22, d.get_height()-10, font_s_, "%s", pg);
    }
  }

private:
  // ════════════════════════════════════════════════════════════════════
  // ПРОТОКОЛ
  // ════════════════════════════════════════════════════════════════════
  // sub_cnt: -1=нет данных, 1=1байт, 2=2байта
  void build(uint8_t cmd, int sub_cnt=- 1, uint8_t s1=0, uint8_t s2=0){
    int dlen=(sub_cnt<0)?0:sub_cnt;
    uint8_t tmp[44]{}; int n=0;
    tmp[n++]=0x73; tmp[n++]=0x55;
    tmp[n++]=uint8_t(0x20|(dlen&0x1F)); // params
    tmp[n++]=0x00;                       // reserve
    tmp[n++]=uint8_t(addr_&0xFF);
    tmp[n++]=uint8_t((addr_>>8)&0xFF);
    tmp[n++]=0xFE; tmp[n++]=0xFF;       // AddrSrc
    tmp[n++]=cmd;
    tmp[n++]=0; tmp[n++]=0; tmp[n++]=0; tmp[n++]=0; // PIN
    if(sub_cnt>=1) tmp[n++]=s1;
    if(sub_cnt>=2) tmp[n++]=s2;
    crc_.restart(); crc_.setPolynome(0xA9);
    for(int i=2;i<n;i++) crc_.add(tmp[i]);
    tmp[n++]=crc_.calc(); tmp[n++]=0x55;
    // байтстаффинг
    memset(tx_,0,BUF); int out=0;
    tx_[out++]=tmp[0]; tx_[out++]=tmp[1];
    for(int i=2;i<n-1;i++){
      if     (tmp[i]==0x55){tx_[out++]=0x73;tx_[out++]=0x11;}
      else if(tmp[i]==0x73){tx_[out++]=0x73;tx_[out++]=0x22;}
      else                  {tx_[out++]=tmp[i];}
    }
    tx_[out++]=0x55;
    memmove(tx_+1,tx_,out); tx_[0]=(uint8_t)out;
  }

  void send(){
    ELECHOUSE_cc1101.SpiStrobe(0x33); delay(1);
    ELECHOUSE_cc1101.SpiStrobe(0x3B); ELECHOUSE_cc1101.SpiStrobe(0x36);
    ELECHOUSE_cc1101.SpiWriteReg(0x3E,0xC4);
    ELECHOUSE_cc1101.SendData(tx_,tx_[0]+1);
    ELECHOUSE_cc1101.SpiStrobe(0x3A); ELECHOUSE_cc1101.SpiStrobe(0x34);
  }

  bool recv(uint32_t ms){
    memset(raw_,0,BUF); memset(res_,0,BUF);
    raw_len_=0; res_len_=0;
    uint8_t piece[BUF]{}; uint32_t t0=millis();
    while(millis()-t0<ms){
      if(ELECHOUSE_cc1101.CheckReceiveFlag()){
        int len=ELECHOUSE_cc1101.ReceiveData(piece);
        for(int i=1;i<len&&raw_len_<BUF-1;i++) raw_[raw_len_++]=piece[i];
        ELECHOUSE_cc1101.SpiStrobe(0x36); ELECHOUSE_cc1101.SpiStrobe(0x3A);
        ELECHOUSE_cc1101.SpiStrobe(0x3B); ELECHOUSE_cc1101.SpiStrobe(0x34);
      }
      yield();
    }
    if(!raw_len_) return false;
    // обратный байтстаффинг
    int j=0;
    for(int i=0;i<raw_len_;i++){
      res_[i-j]=raw_[i]; res_len_++;
      if(raw_[i]==0x73&&i+1<raw_len_){
        if     (raw_[i+1]==0x11){res_[i-j]=0x55;i++;j++;}
        else if(raw_[i+1]==0x22){i++;j++;}
      }
    }
    // CRC
    crc_.reset(); crc_.setPolynome(0xA9);
    int ce=res_len_-2;
    if(res_len_>3&&res_[res_len_-3]==0x76) ce=res_len_-3;
    for(int i=2;i<ce;i++) crc_.add(res_[i]);
    my_crc_=crc_.calc();
    return true;
  }

  bool valid(uint8_t cmd){
    if(res_len_<10) return false;
    if(res_[0]!=0x73||res_[1]!=0x55) return false;
    if(res_[6]!=(uint8_t)(addr_&0xFF)) return false;
    if(res_[8]!=cmd){ESP_LOGW("mirtek","cmd %02X!=%02X",res_[8],cmd);return false;}
    if(res_[res_len_-2]!=my_crc_){ESP_LOGW("mirtek","crc %02X!=%02X",my_crc_,res_[res_len_-2]);return false;}
    if(res_[res_len_-1]!=0x55) return false;
    return true;
  }

  bool xact(uint8_t cmd,int sub=-1,uint8_t s=0){
    build(cmd,sub,s,0); send();
    if(!recv(800)){ESP_LOGW("mirtek","timeout cmd=%02X",cmd);return false;}
    if(!valid(cmd)){ESP_LOGW("mirtek","invalid cmd=%02X",cmd);return false;}
    return true;
  }

  // ════════════════════════════════════════════════════════════════════
  // ПАРСЕРЫ
  // ════════════════════════════════════════════════════════════════════
  void parse_ping(){
    meter_type_=res_[9];
    if(meter_type_==MT_3P||meter_type_==MT_3P2){three_phase_=true;type_name_="3ф трансф.акт-реакт";}
    else if(meter_type_==MT_1P){three_phase_=false;type_name_="1ф 2-эл акт-реакт";}
    else if(meter_type_==MT_1P1){three_phase_=false;type_name_="1ф 1-эл";}
    else {three_phase_=true;char b[8];snprintf(b,8,"0x%02X",meter_type_);type_name_=b;}

    char fw[8]; snprintf(fw,8,"%02u.%02u",res_[14],res_[13]);
    fw_=fw;

    bool p3=(res_[10]>>3)&1,p2=(res_[10]>>2)&1,p1=(res_[10]>>1)&1;
    bool m1=(res_[10]>>4)&1,m2=(res_[10]>>5)&1;
    if(!p1&&!p2&&!p3&&!m1&&!m2){seal_="OK";}
    else{
      seal_="";
      if(p3)seal_+="Крышка модуля; ";
      if(p2)seal_+="Крышка корпуса; ";
      if(p1)seal_+="Клеммник; ";
      if(m1)seal_+="Пост.магн.; ";
      if(m2)seal_+="Перем.магн.; ";
    }
    bool r2=(res_[11]>>3)&1,r1=(res_[11]>>2)&1;
    bool relay_on=(res_[14]<5&&r1==0)?r2:!r2;
    relay_=relay_on?"Вкл":"Выкл";
    ESP_LOGI("mirtek","Type=%s FW=%s Relay=%s Seal=%s",
      type_name_.c_str(),fw_,relay_.c_str(),seal_.c_str());
  }

  void parse_datetime(){
    static const char*dow[]={"Вс","Пн","Вт","Ср","Чт","Пт","Сб"};
    char dt[24],tm[10];
    snprintf(dt,24,"%02u-%02u-20%02u %s",res_[17],res_[18],res_[19],dow[res_[16]%7]);
    snprintf(tm,10,"%02u:%02u:%02u",res_[15],res_[14],res_[13]);
    meter_date_=dt; meter_time_=tm;
    static const char*ts[]={"День","Ночь","Полупик","Спец."};
    tariff_=ts[(res_[10]>>2)&0x03];
  }

  void parse_energy(bool react){
    // SUM@[19..22], T1@[27..30], T2@[31..34], T3@[35..38]
    float s=mk_u32le(res_+19)/100.0f;
    float a=mk_u32le(res_+27)/100.0f;
    float b=mk_u32le(res_+31)/100.0f;
    float c=mk_u32le(res_+35)/100.0f;
    uint8_t ntariffs=((res_[14]>>6)&0x03)+1;
    // тариф из конф.байта (активная энергия)
    if(!react){
      sum_=s;t1_=a;t2_=b;t3_=(ntariffs>=3)?c:NAN;
      if(ntariffs<2)t2_=NAN;
      static const char*ts[]={"День","Ночь","Полупик","Спец."};
      tariff_=ts[(res_[14]>>2)&0x03];
    } else {
      sum_r_=s;t1_r_=a;t2_r_=b;t3_r_=(ntariffs>=3)?c:NAN;
      if(ntariffs<2)t2_r_=NAN;
    }
    ESP_LOGI("mirtek","%s SUM=%.2f T1=%.2f T2=%.2f T3=%.2f",
      react?"kVArh":"kWh",s,a,b,c);
  }

  void parse_ui(){
    const uint8_t*r=res_;
    if(three_phase_){
      kw_  =float(r[18]|(r[19]<<8)|(r[20]<<16))/1000.0f;
      kvar_=(r[23]>=128)?float(r[21]|(r[22]<<8)|((r[23]-128)<<16))/-1000.0f
                        :float(r[21]|(r[22]<<8)|(r[23]<<16))/1000.0f;
      freq_=mk_u16le(r+24)/100.0f;
      cos_ =mk_s16(r[26],r[27],1000.0f);
      v1_=mk_u16le(r+28)/100.0f; v2_=mk_u16le(r+30)/100.0f; v3_=mk_u16le(r+32)/100.0f;
      i1_=float(r[34]|(r[35]<<8)|(r[36]<<16))/1000.0f;
      i2_=float(r[37]|(r[38]<<8)|(r[39]<<16))/1000.0f;
      i3_=float(r[40]|(r[41]<<8)|(r[42]<<16))/1000.0f;
    } else {
      kw_  =mk_u16le(r+18)/1000.0f;
      kvar_=mk_s16(r[20],r[21],1000.0f);
      freq_=mk_u16le(r+22)/100.0f;
      cos_ =mk_s16(r[24],r[25],1000.0f);
      v1_=mk_u16le(r+26)/100.0f; v2_=NAN; v3_=NAN;
      i1_=float(r[32]|(r[33]<<8)|(r[34]<<16))/1000.0f; i2_=NAN; i3_=NAN;
    }
    ESP_LOGI("mirtek","U=%.1f/%.1f/%.1f I=%.3f/%.3f/%.3f P=%.3fkW f=%.2f cos=%.3f",
      v1_,v2_,v3_,i1_,i2_,i3_,kw_,freq_,cos_);
  }

  void parse_pwr(){
    const uint8_t*r=res_;
    ca_=mk_s16(r[18],r[19],1000.0f);
    cb_=mk_s16(r[20],r[21],1000.0f);
    cc_=mk_s16(r[22],r[23],1000.0f);
    pa_=mk_u16le(r+24); pb_=mk_u16le(r+26); pc_=mk_u16le(r+28);
    qa_=mk_s16(r[30],r[31],1.0f);
    qb_=mk_s16(r[32],r[33],1.0f);
    qc_=mk_s16(r[34],r[35],1.0f);
    sa_=mk_u16le(r+36); sb_=mk_u16le(r+38); sc_=mk_u16le(r+40);
    temp_=(r[42]>=128)?-(float)(r[42]-128):(float)r[42];
    ESP_LOGI("mirtek","Pa=%.0f Pb=%.0f Pc=%.0f T=%.0f",pa_,pb_,pc_,temp_);
  }

  void parse_getinfo(){
    uint32_t w=mk4le(res_+18), sl=mk4le(res_+22), sy=mk4le(res_+32);
    work_time_=fmt_sec(w); sync_time_=fmt_sec(sy);
    (void)sl;
  }

  void parse_bat(){
    if(res_len_<15) return;
    uint8_t tot=res_[13], rem=res_[14];
    bat_pct_=(tot>0)?(100.0f*rem/tot):NAN;
  }

  void parse_str(uint8_t cmd, uint8_t field){
    if(res_len_<14) return;
    char buf[31]{}; memcpy(buf,res_+14,30);
    for(int i=29;i>=0&&(buf[i]==' '||buf[i]==0);i--) buf[i]=0;
    if(cmd==CMD_FACTORY){
      if(field==1) serial_=buf;
    } else if(cmd==CMD_ABON){
      if(field==1) { abon_=buf; }
      else if(field==6&&strlen(buf)>0) { abon_+=" / "; abon_+=buf; }
    }
  }

  uint32_t mk4le(const uint8_t*b){
    return ((uint32_t)b[3]<<24)|((uint32_t)b[2]<<16)|((uint32_t)b[1]<<8)|b[0];
  }

  std::string fmt_sec(uint32_t s){
    uint32_t d=s/86400, h=(s%86400)/3600, m=(s%3600)/60;
    char b[24];
    if(d) snprintf(b,24,"%ud %02u:%02u",(unsigned)d,(unsigned)h,(unsigned)m);
    else  snprintf(b,24,"%02u:%02u",(unsigned)h,(unsigned)m);
    return b;
  }

  // ════════════════════════════════════════════════════════════════════
  // ПУБЛИКАЦИЯ
  // ════════════════════════════════════════════════════════════════════
  void pf(int idx, float v){ if(ss[idx]&&!std::isnan(v)) ss[idx]->publish_state(v); }

  void pub_all(){
    pf(SUM,sum_); pf(T1,t1_); pf(T2,t2_); pf(T3,t3_);
    pf(SUM_R,sum_r_); pf(T1_R,t1_r_); pf(T2_R,t2_r_); pf(T3_R,t3_r_);
    pf(KW,kw_); pf(KVAR,kvar_); pf(FREQ,freq_); pf(COS,cos_);
    pf(V1,v1_); pf(V2,v2_); pf(V3,v3_);
    pf(I1,i1_); pf(I2,i2_); pf(I3,i3_);
    pf(PA,pa_); pf(PB,pb_); pf(PC,pc_);
    pf(QA,qa_); pf(QB,qb_); pf(QC,qc_);
    pf(SA,sa_); pf(SB,sb_); pf(SC,sc_);
    pf(CA,ca_); pf(CB,cb_); pf(CC,cc_);
    pf(TEMP,temp_); pf(BAT_PCT,bat_pct_);
    pub_text();
  }

  void pub_text(){
    if(ts[T_TAR])  ts[T_TAR]->publish_state(tariff_);
    if(ts[T_RELAY])ts[T_RELAY]->publish_state(relay_);
    if(ts[T_SEAL]) ts[T_SEAL]->publish_state(seal_);
    if(ts[T_TYPE]) ts[T_TYPE]->publish_state(type_name_);
    if(ts[T_FW])   ts[T_FW]->publish_state(fw_);
    if(ts[T_DATE]) ts[T_DATE]->publish_state(meter_date_);
    if(ts[T_TIME]) ts[T_TIME]->publish_state(meter_time_);
    if(ts[T_WORK]) ts[T_WORK]->publish_state(work_time_);
    if(ts[T_SYNC]) ts[T_SYNC]->publish_state(sync_time_);
    if(ts[T_SER])  ts[T_SER]->publish_state(serial_);
    if(ts[T_ABON]) ts[T_ABON]->publish_state(abon_);
    if(ts[T_ST])   ts[T_ST]->publish_state(status_);
    if(bs[B_3P])   bs[B_3P]->publish_state(three_phase_);
    if(bs[B_REL])  bs[B_REL]->publish_state(relay_=="Вкл");
    if(bs[B_SEAL]) bs[B_SEAL]->publish_state(seal_=="OK");
  }

  // ════════════════════════════════════════════════════════════════════
  // ДИСПЛЕЙ
  // ════════════════════════════════════════════════════════════════════
  void draw_energy(display::Display &d){
    if(!font_b_||!font_s_) return;
    d.printf(0,0,font_s_,"kWh | %s",tariff_.c_str());
    if(!std::isnan(sum_)) d.printf(0,12,font_b_,"%.2f",sum_);
    char ln[40];
    snprintf(ln,40,"T1:%.2f  T2:%.2f",
      std::isnan(t1_)?0.f:t1_, std::isnan(t2_)?0.f:t2_);
    d.printf(0,31,font_s_,"%s",ln);
    if(!std::isnan(t3_)&&t3_>0) d.printf(0,42,font_s_,"T3: %.2f kWh",t3_);
    if(!std::isnan(sum_r_)) d.printf(0,53,font_s_,"kVArh: %.2f",sum_r_);
  }

  void draw_ui(display::Display &d){
    if(!font_s_) return;
    int y=0;
    if(!std::isnan(freq_)){d.printf(0,y,font_s_,"f=%.2fHz  cos=%.3f",freq_,cos_);y+=12;}
    if(!std::isnan(v1_)){d.printf(0,y,font_s_,"U1=%.1fV",v1_);y+=11;}
    if(three_phase_&&!std::isnan(v2_)){d.printf(0,y,font_s_,"U2=%.1f  U3=%.1f",v2_,v3_);y+=11;}
    if(!std::isnan(i1_)){d.printf(0,y,font_s_,"I1=%.3fA",i1_);y+=11;}
    if(three_phase_&&!std::isnan(i2_)){d.printf(0,y,font_s_,"I2=%.3f  I3=%.3f",i2_,i3_);y+=11;}
    if(!std::isnan(kw_)){d.printf(0,y,font_s_,"P=%.3fkW  Q=%.3fkvar",kw_,kvar_);}
  }

  void draw_power(display::Display &d){
    if(!font_b_||!font_s_) return;
    if(!std::isnan(kw_)) d.printf(0,0,font_b_,"%.3fkW",kw_);
    if(three_phase_&&!std::isnan(pa_))
      d.printf(0,18,font_s_,"Pa=%.0f Pb=%.0f Pc=%.0f W",pa_,pb_,pc_);
    if(three_phase_&&!std::isnan(ca_))
      d.printf(0,30,font_s_,"cosA=%.3f B=%.3f C=%.3f",ca_,cb_,cc_);
    if(!std::isnan(temp_)) d.printf(0,42,font_s_,"T=%.0f C",temp_);
    if(!std::isnan(bat_pct_)) d.printf(0,53,font_s_,"Бат=%.0f%%",bat_pct_);
  }

  void draw_info(display::Display &d){
    if(!font_s_) return;
    d.printf(0,0,font_s_,"%s",meter_date_.c_str());
    d.printf(0,11,font_s_,"%s  %s",meter_time_.c_str(),tariff_.c_str());
    d.printf(0,22,font_s_,"Реле: %s",relay_.c_str());
    d.printf(0,33,font_s_,"Пломба: %s",seal_.c_str());
    d.printf(0,44,font_s_,"Работа: %s",work_time_.c_str());
    if(!serial_.empty()&&serial_!="—")
      d.printf(0,55,font_s_,"SN:%s",serial_.c_str());
  }
};

}  // namespace mirtek_cc1101
