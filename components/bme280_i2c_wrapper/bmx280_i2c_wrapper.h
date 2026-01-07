#pragma once

#include <Arduino.h>
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/gpio.h"
#include <bme280.h>

namespace esphome {
namespace bme280_i2c_wrapper {

/** Enum listing all Oversampling values for the BME280.
 *
 * Oversampling basically means measuring a condition multiple times. Higher oversampling
 * values therefore increase the time required to read sensor values but increase accuracy.
*/
 enum EnumOversampling {
  OVERSAMPLING_NONE = BME280_NO_OVERSAMPLING,
  OVERSAMPLING_1X = BME280_OVERSAMPLING_1X,
  OVERSAMPLING_2X = BME280_OVERSAMPLING_2X,
  OVERSAMPLING_4X = BME280_OVERSAMPLING_4X,
  OVERSAMPLING_8X = BME280_OVERSAMPLING_8X,
  OVERSAMPLING_16X = BME280_OVERSAMPLING_16X,
};

/** Enum listing all Infinite Impulse Filter values for the BME280.
 *
 * Higher values increase accuracy, but decrease response time.
*/
 enum EnumIIRFilter {
  IIR_FILTER_OFF = BME280_FILTER_COEFF_OFF,
  IIR_FILTER_2X = BME280_FILTER_COEFF_2,
  IIR_FILTER_4X = BME280_FILTER_COEFF_4,
  IIR_FILTER_8X = BME280_FILTER_COEFF_8,
  IIR_FILTER_16X = BME280_FILTER_COEFF_16,
};

/// This class implements support for the BME280 Temperature+Pressure+Humidity sensor.
class BME280I2CWrapper : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_temperature_sensor(sensor::Sensor *sensor) { temperature_sensor_ = sensor; }
  void set_humidity_sensor(sensor::Sensor *sensor) { humidity_sensor_ = sensor; }
  void set_pressure_sensor(sensor::Sensor *sensor) { pressure_sensor_ = sensor; }
  void set_dew_point_sensor(sensor::Sensor *sensor) { dew_point_sensor_ = sensor; }
  void set_humidity_absolute_sensor(sensor::Sensor *sensor) { humidity_absolute_sensor_ = sensor; }
  void set_pressure_sealevel_sensor(sensor::Sensor *sensor) { pressure_sealevel_sensor_ = sensor; }
  void set_power_pin(InternalGPIOPin *pin) { power_pin_ = pin; }
  void set_altitude(float altitude) { altitude_ = altitude; }

  /// Set the oversampling value for the temperature sensor. Default is 16x.
  void set_temperature_oversampling(EnumOversampling oversampling) { settings_.osr_t = oversampling; };
  /// Set the oversampling value for the humidity sensor. Default is 16x.
  void set_humidity_oversampling(EnumOversampling oversampling)  { settings_.osr_h = oversampling; };
  /// Set the oversampling value for the pressure sensor. Default is 16x.
  void set_pressure_oversampling(EnumOversampling oversampling)  { settings_.osr_p = oversampling; };
  /// Set the IIR Filter used to increase accuracy, defaults to no IIR Filter.
  void set_iir_filter(EnumIIRFilter iir_filter)  { settings_.filter = iir_filter; };

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

protected:
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *pressure_sensor_{nullptr};
  sensor::Sensor *dew_point_sensor_{nullptr};
  sensor::Sensor *humidity_absolute_sensor_{nullptr};
  sensor::Sensor *pressure_sealevel_sensor_{nullptr};

  InternalGPIOPin *power_pin_{nullptr};
  float altitude_{0};

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    WRONG_CHIP_ID,
  } error_code_{NONE};

private:
  int8_t begin(int i2c_addr);

  bool   forceMeasure();
  bool   startMeasure();
  bool   readSensorData();

  double getTemperature() { return comp_data_.temperature; };
  double getHumidity() { return comp_data_.humidity; };
  double getPressure() { return comp_data_.pressure / 100.0; };
  double getDewPoint();
  double getHumidityAbsolute();
  double getPressureSeaLevel();

  uint8_t dev_addr_;
  uint32_t measure_delay_;
  struct bme280_dev dev_;
  struct bme280_settings settings_;
  struct bme280_data comp_data_;
};

}  // namespace bme280_i2c_wrapper
}  // namespace esphome
