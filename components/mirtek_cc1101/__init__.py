"""Mirtek CC1101 — ESPHome 2026.4.x external component."""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_SAFETY,
    DEVICE_CLASS_TAMPER,
    DEVICE_CLASS_PROBLEM,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_KILOWATT_HOURS,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_HERTZ,
    UNIT_CELSIUS,
    UNIT_EMPTY,
    UNIT_PERCENT,
    UNIT_KILOVOLT_AMPS_REACTIVE,
    UNIT_KILOVOLT_AMPS_REACTIVE_HOURS,
    UNIT_VOLT_AMPS,
    ICON_FLASH,
    ICON_THERMOMETER,
)

CODEOWNERS = []

# ── config keys ───────────────────────────────────────────────────────────────
CONF_CS_PIN       = "cs_pin"
CONF_GDO0_PIN     = "gdo0_pin"
CONF_METER_ADDR   = "meter_address"

# sensors
CONF_S_SUM    = "energy_sum"
CONF_S_T1     = "energy_t1"
CONF_S_T2     = "energy_t2"
CONF_S_T3     = "energy_t3"
CONF_S_SUM_R  = "reactive_sum"
CONF_S_T1_R   = "reactive_t1"
CONF_S_T2_R   = "reactive_t2"
CONF_S_T3_R   = "reactive_t3"
CONF_S_KW     = "power_active"
CONF_S_KVAR   = "power_reactive"
CONF_S_FREQ   = "frequency"
CONF_S_COS    = "power_factor"
CONF_S_V1     = "voltage_1"
CONF_S_V2     = "voltage_2"
CONF_S_V3     = "voltage_3"
CONF_S_I1     = "current_1"
CONF_S_I2     = "current_2"
CONF_S_I3     = "current_3"
CONF_S_PA     = "power_a"
CONF_S_PB     = "power_b"
CONF_S_PC     = "power_c"
CONF_S_QA     = "reactive_a"
CONF_S_QB     = "reactive_b"
CONF_S_QC     = "reactive_c"
CONF_S_SA     = "apparent_a"
CONF_S_SB     = "apparent_b"
CONF_S_SC     = "apparent_c"
CONF_S_CA     = "pf_a"
CONF_S_CB     = "pf_b"
CONF_S_CC     = "pf_c"
CONF_S_TEMP   = "temperature"
CONF_S_BAT    = "battery"

# text sensors
CONF_T_TARIFF = "tariff"
CONF_T_RELAY  = "relay_state"
CONF_T_SEAL   = "seal_state"
CONF_T_TYPE   = "meter_type"
CONF_T_FW     = "fw_version"
CONF_T_DATE   = "meter_date"
CONF_T_TIME   = "meter_time"
CONF_T_WORK   = "uptime_meter"
CONF_T_SYNC   = "last_sync"
CONF_T_SERIAL = "serial_number"
CONF_T_ABON   = "subscriber"
CONF_T_STATUS = "status"

# binary sensors
CONF_B_3PHASE  = "three_phase"
CONF_B_RELAY   = "relay_on"
CONF_B_SEAL_OK = "seal_ok"
CONF_B_CC1101  = "cc1101_ok"

# ── C++ class ─────────────────────────────────────────────────────────────────
mirtek_ns  = cg.esphome_ns.namespace("mirtek_cc1101")
MirtekCC1101 = mirtek_ns.class_("MirtekCC1101", cg.PollingComponent)

# ── helpers ───────────────────────────────────────────────────────────────────
def _sens(unit, decimals, dc=None, sc=STATE_CLASS_MEASUREMENT, icon=None):
    kwargs = dict(unit_of_measurement=unit, accuracy_decimals=decimals, state_class=sc)
    if dc:   kwargs["device_class"] = dc
    if icon: kwargs["icon"] = icon
    return sensor.sensor_schema(**kwargs)

def _energy(unit=UNIT_KILOWATT_HOURS, icon=ICON_FLASH):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        icon=icon,
    )

# ── CONFIG_SCHEMA ─────────────────────────────────────────────────────────────
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID():                cv.declare_id(MirtekCC1101),
    cv.Required(CONF_CS_PIN):       pins.gpio_output_pin_schema,
    cv.Required(CONF_GDO0_PIN):     pins.gpio_input_pin_schema,
    cv.Required(CONF_METER_ADDR):   cv.int_range(min=1, max=65000),

    # active energy
    cv.Optional(CONF_S_SUM):   _energy(),
    cv.Optional(CONF_S_T1):    _energy(icon="mdi:weather-sunny"),
    cv.Optional(CONF_S_T2):    _energy(icon="mdi:weather-night"),
    cv.Optional(CONF_S_T3):    _energy(icon="mdi:weather-sunset"),
    # reactive energy
    cv.Optional(CONF_S_SUM_R): _energy(unit=UNIT_KILOVOLT_AMPS_REACTIVE_HOURS),
    cv.Optional(CONF_S_T1_R):  _energy(unit=UNIT_KILOVOLT_AMPS_REACTIVE_HOURS),
    cv.Optional(CONF_S_T2_R):  _energy(unit=UNIT_KILOVOLT_AMPS_REACTIVE_HOURS),
    cv.Optional(CONF_S_T3_R):  _energy(unit=UNIT_KILOVOLT_AMPS_REACTIVE_HOURS),
    # instantaneous
    cv.Optional(CONF_S_KW):   _sens(UNIT_WATT,    0, DEVICE_CLASS_POWER,       icon=ICON_FLASH),
    cv.Optional(CONF_S_KVAR): _sens(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
    cv.Optional(CONF_S_FREQ): _sens(UNIT_HERTZ,   2, DEVICE_CLASS_FREQUENCY),
    cv.Optional(CONF_S_COS):  _sens(UNIT_EMPTY,   3, DEVICE_CLASS_POWER_FACTOR),
    # voltage
    cv.Optional(CONF_S_V1):   _sens(UNIT_VOLT,  1, DEVICE_CLASS_VOLTAGE),
    cv.Optional(CONF_S_V2):   _sens(UNIT_VOLT,  1, DEVICE_CLASS_VOLTAGE),
    cv.Optional(CONF_S_V3):   _sens(UNIT_VOLT,  1, DEVICE_CLASS_VOLTAGE),
    # current
    cv.Optional(CONF_S_I1):   _sens(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
    cv.Optional(CONF_S_I2):   _sens(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
    cv.Optional(CONF_S_I3):   _sens(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
    # per-phase active
    cv.Optional(CONF_S_PA):   _sens(UNIT_WATT, 0, DEVICE_CLASS_POWER),
    cv.Optional(CONF_S_PB):   _sens(UNIT_WATT, 0, DEVICE_CLASS_POWER),
    cv.Optional(CONF_S_PC):   _sens(UNIT_WATT, 0, DEVICE_CLASS_POWER),
    # per-phase reactive
    cv.Optional(CONF_S_QA):   _sens(UNIT_KILOVOLT_AMPS_REACTIVE, 0),
    cv.Optional(CONF_S_QB):   _sens(UNIT_KILOVOLT_AMPS_REACTIVE, 0),
    cv.Optional(CONF_S_QC):   _sens(UNIT_KILOVOLT_AMPS_REACTIVE, 0),
    # per-phase apparent
    cv.Optional(CONF_S_SA):   _sens(UNIT_VOLT_AMPS, 0),
    cv.Optional(CONF_S_SB):   _sens(UNIT_VOLT_AMPS, 0),
    cv.Optional(CONF_S_SC):   _sens(UNIT_VOLT_AMPS, 0),
    # per-phase cos
    cv.Optional(CONF_S_CA):   _sens(UNIT_EMPTY, 3),
    cv.Optional(CONF_S_CB):   _sens(UNIT_EMPTY, 3),
    cv.Optional(CONF_S_CC):   _sens(UNIT_EMPTY, 3),
    # misc
    cv.Optional(CONF_S_TEMP): _sens(UNIT_CELSIUS, 0, DEVICE_CLASS_TEMPERATURE, icon=ICON_THERMOMETER),
    cv.Optional(CONF_S_BAT):  _sens(UNIT_PERCENT, 0, DEVICE_CLASS_BATTERY),

    # text sensors
    cv.Optional(CONF_T_TARIFF): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_RELAY):  text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_SEAL):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_TYPE):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_FW):     text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_DATE):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_TIME):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_WORK):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_SYNC):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_SERIAL): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_ABON):   text_sensor.text_sensor_schema(),
    cv.Optional(CONF_T_STATUS): text_sensor.text_sensor_schema(),

    # binary sensors
    cv.Optional(CONF_B_3PHASE):  binary_sensor.binary_sensor_schema(),
    cv.Optional(CONF_B_RELAY):   binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_POWER),
    cv.Optional(CONF_B_SEAL_OK): binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_SAFETY),
    cv.Optional(CONF_B_CC1101):  binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_CONNECTIVITY),
}).extend(cv.polling_component_schema("300s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cs   = await cg.gpio_pin_expression(config[CONF_CS_PIN])
    gdo0 = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_cs_pin(cs))
    cg.add(var.set_gdo0_pin(gdo0))
    cg.add(var.set_meter_address(config[CONF_METER_ADDR]))

    SENSORS = [
        CONF_S_SUM, CONF_S_T1, CONF_S_T2, CONF_S_T3,
        CONF_S_SUM_R, CONF_S_T1_R, CONF_S_T2_R, CONF_S_T3_R,
        CONF_S_KW, CONF_S_KVAR, CONF_S_FREQ, CONF_S_COS,
        CONF_S_V1, CONF_S_V2, CONF_S_V3,
        CONF_S_I1, CONF_S_I2, CONF_S_I3,
        CONF_S_PA, CONF_S_PB, CONF_S_PC,
        CONF_S_QA, CONF_S_QB, CONF_S_QC,
        CONF_S_SA, CONF_S_SB, CONF_S_SC,
        CONF_S_CA, CONF_S_CB, CONF_S_CC,
        CONF_S_TEMP, CONF_S_BAT,
    ]
    for idx, key in enumerate(SENSORS):
        if cfg := config.get(key):
            s = await sensor.new_sensor(cfg)
            cg.add(var.set_sensor(idx, s))

    TEXT_SENSORS = [
        CONF_T_TARIFF, CONF_T_RELAY, CONF_T_SEAL, CONF_T_TYPE,
        CONF_T_FW, CONF_T_DATE, CONF_T_TIME, CONF_T_WORK,
        CONF_T_SYNC, CONF_T_SERIAL, CONF_T_ABON, CONF_T_STATUS,
    ]
    for idx, key in enumerate(TEXT_SENSORS):
        if cfg := config.get(key):
            ts = await text_sensor.new_text_sensor(cfg)
            cg.add(var.set_text_sensor(idx, ts))

    BINARY_SENSORS = [CONF_B_3PHASE, CONF_B_RELAY, CONF_B_SEAL_OK, CONF_B_CC1101]
    for idx, key in enumerate(BINARY_SENSORS):
        if cfg := config.get(key):
            bs = await binary_sensor.new_binary_sensor(cfg)
            cg.add(var.set_binary_sensor(idx, bs))
