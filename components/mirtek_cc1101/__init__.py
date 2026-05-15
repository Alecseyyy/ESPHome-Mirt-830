"""Mirtek CC1101 ESPHome external component — ESPHome 2026.4.x compatible."""
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
    UNIT_KILOVOLT_AMPS_REACTIVE,
    UNIT_VOLT_AMPS,
    ICON_FLASH,
    ICON_THERMOMETER,
)

CODEOWNERS = []

CONF_CS_PIN = "cs_pin"
CONF_GDO0_PIN = "gdo0_pin"
CONF_METER_ADDRESS = "meter_address"
CONF_ENERGY_SUM = "energy_sum"
CONF_ENERGY_T1 = "energy_t1"
CONF_ENERGY_T2 = "energy_t2"
CONF_ENERGY_T3 = "energy_t3"
CONF_VOLTAGE_A = "voltage_a"
CONF_VOLTAGE_B = "voltage_b"
CONF_VOLTAGE_C = "voltage_c"
CONF_CURRENT_A = "current_a"
CONF_CURRENT_B = "current_b"
CONF_CURRENT_C = "current_c"
CONF_POWER_ACTIVE = "power_active"
CONF_POWER_REACTIVE = "power_reactive"
CONF_FREQUENCY = "frequency"
CONF_POWER_FACTOR = "power_factor"
CONF_TEMPERATURE = "temperature"
CONF_POWER_A = "power_a"
CONF_POWER_B = "power_b"
CONF_POWER_C = "power_c"
CONF_REACTIVE_A = "reactive_a"
CONF_REACTIVE_B = "reactive_b"
CONF_REACTIVE_C = "reactive_c"
CONF_APPARENT_A = "apparent_a"
CONF_APPARENT_B = "apparent_b"
CONF_APPARENT_C = "apparent_c"
CONF_PF_A = "pf_a"
CONF_PF_B = "pf_b"
CONF_PF_C = "pf_c"
CONF_TARIFF_TEXT = "tariff"
CONF_RELAY_STATE = "relay_state"
CONF_SEAL_STATE = "seal_state"
CONF_HW_VERSION = "hw_version"
CONF_SW_VERSION = "sw_version"
CONF_METER_TYPE = "meter_type"
CONF_METER_DATETIME = "meter_datetime"
CONF_LAST_STATUS = "last_status"
CONF_SEAL_COVER = "seal_cover"
CONF_SEAL_TERMINAL = "seal_terminal"
CONF_SEAL_MODULE = "seal_module"
CONF_MAGNET_DC = "magnet_dc"
CONF_MAGNET_AC = "magnet_ac"
CONF_RELAY_ON = "relay_on"
CONF_CURRENT_IMBALANCE = "current_imbalance"
CONF_TIME_SYNCED = "time_synced"

mirtek_ns = cg.esphome_ns.namespace("mirtek_cc1101")
MirtekCC1101 = mirtek_ns.class_("MirtekCC1101", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MirtekCC1101),
        cv.Required(CONF_CS_PIN): pins.gpio_output_pin_schema,
        cv.Required(CONF_GDO0_PIN): pins.gpio_input_pin_schema,
        cv.Required(CONF_METER_ADDRESS): cv.int_range(min=1, max=65000),
        cv.Optional(CONF_ENERGY_SUM): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon=ICON_FLASH,
        ),
        cv.Optional(CONF_ENERGY_T1): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon=ICON_FLASH,
        ),
        cv.Optional(CONF_ENERGY_T2): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon=ICON_FLASH,
        ),
        cv.Optional(CONF_ENERGY_T3): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOWATT_HOURS,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_ENERGY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
            icon=ICON_FLASH,
        ),
        cv.Optional(CONF_VOLTAGE_A): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VOLTAGE_B): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VOLTAGE_C): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT_A): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT_B): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT_C): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER_ACTIVE): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER_REACTIVE): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOVOLT_AMPS_REACTIVE,
            accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_FREQUENCY): sensor.sensor_schema(
            unit_of_measurement=UNIT_HERTZ,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_FREQUENCY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER_FACTOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_POWER_FACTOR,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            icon=ICON_THERMOMETER,
        ),
        cv.Optional(CONF_POWER_A): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER_B): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER_C): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_REACTIVE_A): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOVOLT_AMPS_REACTIVE,
            accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_REACTIVE_B): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOVOLT_AMPS_REACTIVE,
            accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_REACTIVE_C): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOVOLT_AMPS_REACTIVE,
            accuracy_decimals=3,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_APPARENT_A): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_APPARENT_B): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_APPARENT_C): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT_AMPS,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PF_A): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PF_B): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PF_C): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TARIFF_TEXT): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_RELAY_STATE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SEAL_STATE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_HW_VERSION): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SW_VERSION): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_METER_TYPE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_METER_DATETIME): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_LAST_STATUS): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SEAL_COVER): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_TAMPER,
        ),
        cv.Optional(CONF_SEAL_TERMINAL): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_TAMPER,
        ),
        cv.Optional(CONF_SEAL_MODULE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_TAMPER,
        ),
        cv.Optional(CONF_MAGNET_DC): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
        ),
        cv.Optional(CONF_MAGNET_AC): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
        ),
        cv.Optional(CONF_RELAY_ON): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_CURRENT_IMBALANCE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
        ),
        cv.Optional(CONF_TIME_SYNCED): binary_sensor.binary_sensor_schema(),
    }
).extend(cv.polling_component_schema("300s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cs_pin = await cg.gpio_pin_expression(config[CONF_CS_PIN])
    cg.add(var.set_cs_pin(cs_pin))

    gdo0_pin = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_gdo0_pin(gdo0_pin))

    cg.add(var.set_meter_address(config[CONF_METER_ADDRESS]))

    sensor_map = {
        CONF_ENERGY_SUM: "set_energy_sum",
        CONF_ENERGY_T1: "set_energy_t1",
        CONF_ENERGY_T2: "set_energy_t2",
        CONF_ENERGY_T3: "set_energy_t3",
        CONF_VOLTAGE_A: "set_voltage_a",
        CONF_VOLTAGE_B: "set_voltage_b",
        CONF_VOLTAGE_C: "set_voltage_c",
        CONF_CURRENT_A: "set_current_a",
        CONF_CURRENT_B: "set_current_b",
        CONF_CURRENT_C: "set_current_c",
        CONF_POWER_ACTIVE: "set_power_active",
        CONF_POWER_REACTIVE: "set_power_reactive",
        CONF_FREQUENCY: "set_frequency",
        CONF_POWER_FACTOR: "set_power_factor",
        CONF_TEMPERATURE: "set_temperature",
        CONF_POWER_A: "set_power_a",
        CONF_POWER_B: "set_power_b",
        CONF_POWER_C: "set_power_c",
        CONF_REACTIVE_A: "set_reactive_a",
        CONF_REACTIVE_B: "set_reactive_b",
        CONF_REACTIVE_C: "set_reactive_c",
        CONF_APPARENT_A: "set_apparent_a",
        CONF_APPARENT_B: "set_apparent_b",
        CONF_APPARENT_C: "set_apparent_c",
        CONF_PF_A: "set_pf_a",
        CONF_PF_B: "set_pf_b",
        CONF_PF_C: "set_pf_c",
    }
    for conf_key, method in sensor_map.items():
        if sensor_config := config.get(conf_key):
            sens = await sensor.new_sensor(sensor_config)
            cg.add(getattr(var, method)(sens))

    text_map = {
        CONF_TARIFF_TEXT: "set_tariff_text",
        CONF_RELAY_STATE: "set_relay_state",
        CONF_SEAL_STATE: "set_seal_state",
        CONF_HW_VERSION: "set_hw_version",
        CONF_SW_VERSION: "set_sw_version",
        CONF_METER_TYPE: "set_meter_type",
        CONF_METER_DATETIME: "set_meter_datetime",
        CONF_LAST_STATUS: "set_last_status",
    }
    for conf_key, method in text_map.items():
        if ts_config := config.get(conf_key):
            ts = await text_sensor.new_text_sensor(ts_config)
            cg.add(getattr(var, method)(ts))

    binary_map = {
        CONF_SEAL_COVER: "set_seal_cover",
        CONF_SEAL_TERMINAL: "set_seal_terminal",
        CONF_SEAL_MODULE: "set_seal_module",
        CONF_MAGNET_DC: "set_magnet_dc",
        CONF_MAGNET_AC: "set_magnet_ac",
        CONF_RELAY_ON: "set_relay_on",
        CONF_CURRENT_IMBALANCE: "set_current_imbalance",
        CONF_TIME_SYNCED: "set_time_synced",
    }
    for conf_key, method in binary_map.items():
        if bs_config := config.get(conf_key):
            bs = await binary_sensor.new_binary_sensor(bs_config)
            cg.add(getattr(var, method)(bs))
