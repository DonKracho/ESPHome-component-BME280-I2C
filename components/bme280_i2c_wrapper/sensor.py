import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome import pins
from esphome.const import (
    CONF_HUMIDITY,
    CONF_ID,
    CONF_IIR_FILTER,
    CONF_OVERSAMPLING,
    CONF_ALTITUDE,
    CONF_PRESSURE,
    CONF_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_PRESSURE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_HECTOPASCAL,
    UNIT_PERCENT,
)

CODEOWNERS = ["@DonKracho"]

cg.add_library(
    name="BME280_SensorAPI",
    repository="https://github.com/boschsensortec/BME280_SensorAPI.git",
    version=None,
)

CONF_POWER_PIN = "power_pin"
CONF_PRESSURE_RAW = "pressure_raw"

DEPENDENCIES = ["i2c"]

bme280_i2c_ns = cg.esphome_ns.namespace("bme280_i2c_wrapper")
BMX280I2CWrapper = bme280_i2c_ns.class_("BME280I2CWrapper", cg.PollingComponent, i2c.I2CDevice)

BME280Oversampling = bme280_i2c_ns.enum("EnumOversampling")

OVERSAMPLING_OPTIONS = {
    "NONE": BME280Oversampling.OVERSAMPLING_NONE,
    "1X":   BME280Oversampling.OVERSAMPLING_1X,
    "2X":   BME280Oversampling.OVERSAMPLING_2X,
    "4X":   BME280Oversampling.OVERSAMPLING_4X,
    "8X":   BME280Oversampling.OVERSAMPLING_8X,
    "16X":  BME280Oversampling.OVERSAMPLING_16X,
}

BME280IIRFilter = bme280_i2c_ns.enum("EnumIIRFilter")
IIR_FILTER_OPTIONS = {
    "OFF": BME280IIRFilter.IIR_FILTER_OFF,
    "2X":  BME280IIRFilter.IIR_FILTER_2X,
    "4X":  BME280IIRFilter.IIR_FILTER_4X,
    "8X":  BME280IIRFilter.IIR_FILTER_8X,
    "16X": BME280IIRFilter.IIR_FILTER_16X,
}

CONFIG_SCHEMA_BASE = cv.Schema(
    {
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(
            {
                cv.Optional(CONF_OVERSAMPLING, default="16X"): cv.enum(OVERSAMPLING_OPTIONS, upper=True),
            }
        ),
        cv.Optional(CONF_PRESSURE_RAW): sensor.sensor_schema(
            unit_of_measurement=UNIT_HECTOPASCAL,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_PRESSURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(
            {
                cv.Optional(CONF_OVERSAMPLING, default="16X"): cv.enum(OVERSAMPLING_OPTIONS, upper=True),
            }
        ),
        cv.Optional(CONF_PRESSURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_HECTOPASCAL,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_PRESSURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(
            {
                cv.Optional(CONF_OVERSAMPLING, default="16X"): cv.enum(OVERSAMPLING_OPTIONS, upper=True),
            }
        ),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(
            {
                cv.Optional(CONF_OVERSAMPLING, default="16X"): cv.enum(OVERSAMPLING_OPTIONS, upper=True),
            }
        ),
        cv.Optional(CONF_IIR_FILTER, default="OFF"): cv.enum(IIR_FILTER_OPTIONS, upper=True),
        cv.Optional(CONF_ALTITUDE, default="0"): cv.All(float),
        cv.Optional(CONF_POWER_PIN): pins.gpio_output_pin_schema,
    }
).extend(cv.polling_component_schema("60s"))


async def to_code_base(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if temperature_config := config.get(CONF_TEMPERATURE):
        sens = await sensor.new_sensor(temperature_config)
        cg.add(var.set_temperature_sensor(sens))
        cg.add(var.set_temperature_oversampling(temperature_config[CONF_OVERSAMPLING]))

    if pressure_config := config.get(CONF_PRESSURE):
        sens = await sensor.new_sensor(pressure_config)
        cg.add(var.set_pressure_sensor(sens))
        cg.add(var.set_pressure_oversampling(pressure_config[CONF_OVERSAMPLING]))

    if humidity_config := config.get(CONF_HUMIDITY):
        sens = await sensor.new_sensor(humidity_config)
        cg.add(var.set_humidity_sensor(sens))
        cg.add(var.set_humidity_oversampling(humidity_config[CONF_OVERSAMPLING]))

    if CONF_POWER_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_POWER_PIN])
        cg.add(var.set_power_pin(pin))

    cg.add(var.set_iir_filter(config[CONF_IIR_FILTER]))
    cg.add(var.set_altitude(config[CONF_ALTITUDE]))

    return var


CONFIG_SCHEMA = CONFIG_SCHEMA_BASE.extend(
    i2c.i2c_device_schema(default_address=0x77)
).extend({cv.GenerateID(): cv.declare_id(BMX280I2CWrapper)})


async def to_code(config):
    var = await to_code_base(config)
    await i2c.register_i2c_device(var, config)
