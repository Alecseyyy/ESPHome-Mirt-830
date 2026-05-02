"""
Mirtek CC1101 ESPHome Component
Счётчики Миртек Star 104/304 через радиомодуль CC1101 (433 МГц)
https://github.com/Alecseyyy/ESPHome-Mirt-830
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CONNECTIVITY,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_KILOWATT_HOURS,
    UNIT_KILOWATT,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_HERTZ,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

CODEOWNERS = ["@Alecseyyy"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

mirtek_ns = cg.esphome_ns.namespace("mirtek_cc1101")
MirtekCC1101Component = mirtek_ns.class_("MirtekCC1101", cg.Component)

CONF_METER_ADDRESS = "meter_address"
CONF_CS_PIN        = "cs_pin"
CONF_GDO0_PIN      = "gdo0_pin"

# ── Ключи конфигурации (совпадают с именами в mirtek.yaml) ────────────────
# Сенсоры ss[0..31] — порядок должен совпадать с enum SI{} в mirtek_cc1101.h
CONF_SUM    = "energy_sum"
CONF_T1     = "energy_t1"
CONF_T2     = "energy_t2"
CONF_T3     = "energy_t3"
CONF_SUM_R  = "reactive_sum"
CONF_T1_R   = "reactive_t1"
CONF_T2_R   = "reactive_t2"
CONF_T3_R   = "reactive_t3"
CONF_KW     = "power_active"
CONF_KVAR   = "power_reactive"
CONF_FREQ   = "frequency"
CONF_COS    = "cos_phi"
CONF_V1     = "voltage_1"
CONF_V2     = "voltage_2"
CONF_V3     = "voltage_3"
CONF_I1     = "current_1"
CONF_I2     = "current_2"
CONF_I3     = "current_3"
CONF_PA     = "power_a"
CONF_PB     = "power_b"
CONF_PC     = "power_c"
CONF_QA     = "reactive_a"
CONF_QB     = "reactive_b"
CONF_QC     = "reactive_c"
CONF_SA     = "apparent_a"
CONF_SB     = "apparent_b"
CONF_SC     = "apparent_c"
CONF_CA     = "cos_a"
CONF_CB     = "cos_b"
CONF_CC_SEN = "cos_c"
CONF_TEMP   = "temperature"
CONF_BAT    = "battery"

# Текст ts[0..11]
CONF_T_TARIFF = "tariff"
CONF_T_RELAY  = "relay_state"
CONF_T_SEAL   = "seal_state"
CONF_T_TYPE   = "meter_type"
CONF_T_FW     = "firmware"
CONF_T_DATE   = "meter_date"
CONF_T_TIME   = "meter_time"
CONF_T_WORK   = "uptime_meter"
CONF_T_SYNC   = "last_sync"
CONF_T_SERIAL = "serial_number"
CONF_T_ABON   = "subscriber"
CONF_T_STATUS = "status"

# Бинарные bs[0..3]
CONF_B_3PHASE = "three_phase"
CONF_B_RELAY  = "relay_on"
CONF_B_SEAL   = "seal_ok"
CONF_B_CC     = "cc1101_ok"

# ── Порядок ss[], ts[], bs[] ──────────────────────────────────────────────
_SS_KEYS = [
    CONF_SUM, CONF_T1, CONF_T2, CONF_T3,
    CONF_SUM_R, CONF_T1_R, CONF_T2_R, CONF_T3_R,
    CONF_KW, CONF_KVAR, CONF_FREQ, CONF_COS,
    CONF_V1, CONF_V2, CONF_V3,
    CONF_I1, CONF_I2, CONF_I3,
    CONF_PA, CONF_PB, CONF_PC,
    CONF_QA, CONF_QB, CONF_QC,
    CONF_SA, CONF_SB, CONF_SC,
    CONF_CA, CONF_CB, CONF_CC_SEN,
    CONF_TEMP, CONF_BAT,
]
_TS_KEYS = [
    CONF_T_TARIFF, CONF_T_RELAY, CONF_T_SEAL, CONF_T_TYPE,
    CONF_T_FW, CONF_T_DATE, CONF_T_TIME, CONF_T_WORK,
    CONF_T_SYNC, CONF_T_SERIAL, CONF_T_ABON, CONF_T_STATUS,
]
_BS_KEYS = [CONF_B_3PHASE, CONF_B_RELAY, CONF_B_SEAL, CONF_B_CC]

# ── Принудительные ASCII object_id для каждого сенсора ───────────────────
# Решает проблему: кириллические имена одной длины дают одинаковый slug.
# object_id = уникальный ASCII-идентификатор, не зависит от name.
_SS_OBJECT_IDS = {
    CONF_SUM:    "mirtek_energy_sum",
    CONF_T1:     "mirtek_energy_t1",
    CONF_T2:     "mirtek_energy_t2",
    CONF_T3:     "mirtek_energy_t3",
    CONF_SUM_R:  "mirtek_reactive_sum",
    CONF_T1_R:   "mirtek_reactive_t1",
    CONF_T2_R:   "mirtek_reactive_t2",
    CONF_T3_R:   "mirtek_reactive_t3",
    CONF_KW:     "mirtek_power_active",
    CONF_KVAR:   "mirtek_power_reactive",
    CONF_FREQ:   "mirtek_frequency",
    CONF_COS:    "mirtek_cos_phi",
    CONF_V1:     "mirtek_voltage_1",
    CONF_V2:     "mirtek_voltage_2",
    CONF_V3:     "mirtek_voltage_3",
    CONF_I1:     "mirtek_current_1",
    CONF_I2:     "mirtek_current_2",
    CONF_I3:     "mirtek_current_3",
    CONF_PA:     "mirtek_power_a",
    CONF_PB:     "mirtek_power_b",
    CONF_PC:     "mirtek_power_c",
    CONF_QA:     "mirtek_reactive_a",
    CONF_QB:     "mirtek_reactive_b",
    CONF_QC:     "mirtek_reactive_c",
    CONF_SA:     "mirtek_apparent_a",
    CONF_SB:     "mirtek_apparent_b",
    CONF_SC:     "mirtek_apparent_c",
    CONF_CA:     "mirtek_cos_a",
    CONF_CB:     "mirtek_cos_b",
    CONF_CC_SEN: "mirtek_cos_c",
    CONF_TEMP:   "mirtek_temperature",
    CONF_BAT:    "mirtek_battery",
}
_TS_OBJECT_IDS = {
    CONF_T_TARIFF: "mirtek_tariff",
    CONF_T_RELAY:  "mirtek_relay_state",
    CONF_T_SEAL:   "mirtek_seal_state",
    CONF_T_TYPE:   "mirtek_meter_type",
    CONF_T_FW:     "mirtek_firmware",
    CONF_T_DATE:   "mirtek_meter_date",
    CONF_T_TIME:   "mirtek_meter_time",
    CONF_T_WORK:   "mirtek_uptime",
    CONF_T_SYNC:   "mirtek_last_sync",
    CONF_T_SERIAL: "mirtek_serial",
    CONF_T_ABON:   "mirtek_subscriber",
    CONF_T_STATUS: "mirtek_status",
}
_BS_OBJECT_IDS = {
    CONF_B_3PHASE: "mirtek_three_phase",
    CONF_B_RELAY:  "mirtek_relay_on",
    CONF_B_SEAL:   "mirtek_seal_ok",
    CONF_B_CC:     "mirtek_cc1101_ok",
}


def _inject_object_id(schema, key, oid_map):
    """Добавить object_id по умолчанию в схему сенсора чтобы избежать slug-конфликтов."""
    return schema.extend({
        cv.Optional("object_id", default=oid_map[key]): cv.string,
    })


# ── Схема конфигурации ────────────────────────────────────────────────────
def _ss(key, **kwargs):
    return _inject_object_id(sensor.sensor_schema(**kwargs), key, _SS_OBJECT_IDS)

def _ts(key):
    return _inject_object_id(text_sensor.text_sensor_schema(), key, _TS_OBJECT_IDS)

def _bs(key, **kwargs):
    return _inject_object_id(binary_sensor.binary_sensor_schema(**kwargs), key, _BS_OBJECT_IDS)


CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(MirtekCC1101Component),
    cv.Required(CONF_METER_ADDRESS): cv.int_range(min=1, max=65000),
    cv.Required(CONF_CS_PIN):        cv.int_range(min=0, max=39),
    cv.Required(CONF_GDO0_PIN):      cv.int_range(min=0, max=39),

    # Активная энергия
    cv.Optional(CONF_SUM): _ss(CONF_SUM, unit_of_measurement=UNIT_KILOWATT_HOURS, device_class=DEVICE_CLASS_ENERGY,    state_class=STATE_CLASS_TOTAL_INCREASING, accuracy_decimals=2),
    cv.Optional(CONF_T1):  _ss(CONF_T1,  unit_of_measurement=UNIT_KILOWATT_HOURS, device_class=DEVICE_CLASS_ENERGY,    state_class=STATE_CLASS_TOTAL_INCREASING, accuracy_decimals=2),
    cv.Optional(CONF_T2):  _ss(CONF_T2,  unit_of_measurement=UNIT_KILOWATT_HOURS, device_class=DEVICE_CLASS_ENERGY,    state_class=STATE_CLASS_TOTAL_INCREASING, accuracy_decimals=2),
    cv.Optional(CONF_T3):  _ss(CONF_T3,  unit_of_measurement=UNIT_KILOWATT_HOURS, device_class=DEVICE_CLASS_ENERGY,    state_class=STATE_CLASS_TOTAL_INCREASING, accuracy_decimals=2),
    # Реактивная энергия
    cv.Optional(CONF_SUM_R): _ss(CONF_SUM_R, unit_of_measurement="kvarh", state_class=STATE_CLASS_TOTAL_INCREASING, accuracy_decimals=2),
    cv.Optional(CONF_T1_R):  _ss(CONF_T1_R,  unit_of_measurement="kvarh", state_class=STATE_CLASS_TOTAL_INCREASING, accuracy_decimals=2),
    cv.Optional(CONF_T2_R):  _ss(CONF_T2_R,  unit_of_measurement="kvarh", state_class=STATE_CLASS_TOTAL_INCREASING, accuracy_decimals=2),
    cv.Optional(CONF_T3_R):  _ss(CONF_T3_R,  unit_of_measurement="kvarh", state_class=STATE_CLASS_TOTAL_INCREASING, accuracy_decimals=2),
    # Мощность суммарная
    cv.Optional(CONF_KW):   _ss(CONF_KW,   unit_of_measurement=UNIT_KILOWATT, device_class=DEVICE_CLASS_POWER,     state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=3),
    cv.Optional(CONF_KVAR): _ss(CONF_KVAR, unit_of_measurement="kvar",        state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=3),
    cv.Optional(CONF_FREQ): _ss(CONF_FREQ, unit_of_measurement=UNIT_HERTZ,   device_class=DEVICE_CLASS_FREQUENCY,  state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=2),
    cv.Optional(CONF_COS):  _ss(CONF_COS,  unit_of_measurement="",            state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=3),
    # Напряжения
    cv.Optional(CONF_V1): _ss(CONF_V1, unit_of_measurement=UNIT_VOLT, device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=1),
    cv.Optional(CONF_V2): _ss(CONF_V2, unit_of_measurement=UNIT_VOLT, device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=1),
    cv.Optional(CONF_V3): _ss(CONF_V3, unit_of_measurement=UNIT_VOLT, device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=1),
    # Токи
    cv.Optional(CONF_I1): _ss(CONF_I1, unit_of_measurement=UNIT_AMPERE, device_class=DEVICE_CLASS_CURRENT, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=3),
    cv.Optional(CONF_I2): _ss(CONF_I2, unit_of_measurement=UNIT_AMPERE, device_class=DEVICE_CLASS_CURRENT, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=3),
    cv.Optional(CONF_I3): _ss(CONF_I3, unit_of_measurement=UNIT_AMPERE, device_class=DEVICE_CLASS_CURRENT, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=3),
    # Активная мощность по фазам
    cv.Optional(CONF_PA): _ss(CONF_PA, unit_of_measurement="W", device_class=DEVICE_CLASS_POWER, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    cv.Optional(CONF_PB): _ss(CONF_PB, unit_of_measurement="W", device_class=DEVICE_CLASS_POWER, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    cv.Optional(CONF_PC): _ss(CONF_PC, unit_of_measurement="W", device_class=DEVICE_CLASS_POWER, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    # Реактивная мощность по фазам
    cv.Optional(CONF_QA): _ss(CONF_QA, unit_of_measurement="var", state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    cv.Optional(CONF_QB): _ss(CONF_QB, unit_of_measurement="var", state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    cv.Optional(CONF_QC): _ss(CONF_QC, unit_of_measurement="var", state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    # Полная мощность по фазам
    cv.Optional(CONF_SA): _ss(CONF_SA, unit_of_measurement="VA", state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    cv.Optional(CONF_SB): _ss(CONF_SB, unit_of_measurement="VA", state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    cv.Optional(CONF_SC): _ss(CONF_SC, unit_of_measurement="VA", state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    # Косинусы по фазам
    cv.Optional(CONF_CA):     _ss(CONF_CA,     unit_of_measurement="", state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=3),
    cv.Optional(CONF_CB):     _ss(CONF_CB,     unit_of_measurement="", state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=3),
    cv.Optional(CONF_CC_SEN): _ss(CONF_CC_SEN, unit_of_measurement="", state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=3),
    # Прочие
    cv.Optional(CONF_TEMP): _ss(CONF_TEMP, unit_of_measurement=UNIT_CELSIUS, device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),
    cv.Optional(CONF_BAT):  _ss(CONF_BAT,  unit_of_measurement=UNIT_PERCENT,  device_class=DEVICE_CLASS_BATTERY,    state_class=STATE_CLASS_MEASUREMENT, accuracy_decimals=0),

    # Текстовые сенсоры
    cv.Optional(CONF_T_TARIFF): _ts(CONF_T_TARIFF),
    cv.Optional(CONF_T_RELAY):  _ts(CONF_T_RELAY),
    cv.Optional(CONF_T_SEAL):   _ts(CONF_T_SEAL),
    cv.Optional(CONF_T_TYPE):   _ts(CONF_T_TYPE),
    cv.Optional(CONF_T_FW):     _ts(CONF_T_FW),
    cv.Optional(CONF_T_DATE):   _ts(CONF_T_DATE),
    cv.Optional(CONF_T_TIME):   _ts(CONF_T_TIME),
    cv.Optional(CONF_T_WORK):   _ts(CONF_T_WORK),
    cv.Optional(CONF_T_SYNC):   _ts(CONF_T_SYNC),
    cv.Optional(CONF_T_SERIAL): _ts(CONF_T_SERIAL),
    cv.Optional(CONF_T_ABON):   _ts(CONF_T_ABON),
    cv.Optional(CONF_T_STATUS): _ts(CONF_T_STATUS),

    # Бинарные сенсоры
    cv.Optional(CONF_B_3PHASE): _bs(CONF_B_3PHASE),
    cv.Optional(CONF_B_RELAY):  _bs(CONF_B_RELAY,  device_class=DEVICE_CLASS_POWER),
    cv.Optional(CONF_B_SEAL):   _bs(CONF_B_SEAL),
    cv.Optional(CONF_B_CC):     _bs(CONF_B_CC,     device_class=DEVICE_CLASS_CONNECTIVITY),
}).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
        config[CONF_METER_ADDRESS],
        config[CONF_CS_PIN],
        config[CONF_GDO0_PIN],
    )
    await cg.register_component(var, config)

    # Сенсоры ss[]
    for i, key in enumerate(_SS_KEYS):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(var.set_sensor(i, sens))

    # Текстовые ts[]
    for i, key in enumerate(_TS_KEYS):
        if key in config:
            ts = await text_sensor.new_text_sensor(config[key])
            cg.add(var.set_text_sensor(i, ts))

    # Бинарные bs[]
    for i, key in enumerate(_BS_KEYS):
        if key in config:
            bs = await binary_sensor.new_binary_sensor(config[key])
            cg.add(var.set_binary_sensor(i, bs))
