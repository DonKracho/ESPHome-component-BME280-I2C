#include <cmath>
#include <cstdint>

#include "bmx280_i2c_wrapper.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <esphome/components/sensor/sensor.h>
#include <esphome/core/component.h>

#define BME280_ERROR_WRONG_CHIP_ID "Wrong chip ID"
#undef LOG_STR
#define LOG_STR(x) x

namespace esphome {
namespace bme280_i2c_wrapper {

static const char *const TAG = "bme280wrapper";

static BME280I2CWrapper *i2c = nullptr;

inline uint16_t combine_bytes(uint8_t msb, uint8_t lsb) { return ((msb & 0xFF) << 8) | (lsb & 0xFF); }

const char *iir_filter_to_str(uint8_t filter) {  // NOLINT
  switch (filter) {
    case IIR_FILTER_OFF:
      return "OFF";
    case IIR_FILTER_2X:
      return "2x";
    case IIR_FILTER_4X:
      return "4x";
    case IIR_FILTER_8X:
      return "8x";
    case IIR_FILTER_16X:
      return "16x";
    default:
      return "UNKNOWN";
  }
}

const char *oversampling_to_str(uint8_t oversampling) {  // NOLINT
  switch (oversampling) {
    case OVERSAMPLING_NONE:
      return "None";
    case OVERSAMPLING_1X:
      return "1x";
    case OVERSAMPLING_2X:
      return "2x";
    case OVERSAMPLING_4X:
      return "4x";
    case OVERSAMPLING_8X:
      return "8x";
    case OVERSAMPLING_16X:
      return "16x";
    default:
      return "UNKNOWN";
  }
}

void user_delay_us(uint32_t period, void *intf_ptr) {
  delayMicroseconds(period);
}

int8_t user_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  // Return 0 for Success, non-zero for failure
  int8_t ret = 1;
  if (i2c != nullptr) {
    return !i2c->I2CDevice::read_bytes(reg_addr, reg_data, len);
  }
  return ret;
}

int8_t user_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
  // Return 0 for Success, non-zero for failure 
  int8_t ret = 1;
  if (i2c != nullptr) {
    ret = !i2c->I2CDevice::write_bytes(reg_addr, reg_data, len);
  }
  return ret;
}

void BME280I2CWrapper::setup() {
  i2c = this; // required for read / write functions
  int8_t res;

  // if a power_pin is configured supply power to the device now 
  if (power_pin_ != nullptr) {
    power_pin_->setup();
    power_pin_->digital_write(HIGH);
    delay(10);
  }

  // Mark as not failed before initializing. Some devices will turn off sensors to save on batteries
  // and when they come back on, the COMPONENT_STATE_FAILED bit must be unset on the component.
  if (this->is_failed()) {
    this->reset_to_construction_state();
  }

  res = begin(this->I2CDevice::get_i2c_address()); 

  if (res != BME280_OK) {
    ESP_LOGE(TAG, "init error %d", res);
    if (res == BME280_E_DEV_NOT_FOUND) {
      this->error_code_ = WRONG_CHIP_ID;
      this->mark_failed(LOG_STR(BME280_ERROR_WRONG_CHIP_ID));
    } else {
      this->error_code_ = COMMUNICATION_FAILED;
      this->mark_failed(LOG_STR(ESP_LOG_MSG_COMM_FAIL));
    }
  } else {
    ESP_LOGD(TAG, "chip_id=%0x02x", dev_.chip_id);
  }

  // Recommended mode of operation: Indoor navigation
  // settings_.osr_t  = BME280_OVERSAMPLING_16X;
  // settings_.osr_h  = BME280_OVERSAMPLING_16X;
  // settings_.osr_p  = BME280_OVERSAMPLING_16X;
  // settings_.filter = BME280_FILTER_COEFF_16;
  settings_.standby_time = BME280_STANDBY_TIME_500_MS;

  uint8_t settings_sel = BME280_SEL_OSR_PRESS | BME280_SEL_OSR_TEMP | BME280_SEL_OSR_HUM | BME280_SEL_FILTER;

  res = bme280_set_sensor_settings(settings_sel, &settings_, &dev_);

  if (res != BME280_OK) {
    ESP_LOGE(TAG, "settings error %d", res);
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed(LOG_STR(ESP_LOG_MSG_COMM_FAIL));
  }

  // Calculate the minimum delay required between consecutive measurement based upon the sensor enabled  and the oversampling configuration.
  res = bme280_cal_meas_delay(&measure_delay_, &settings_);

  if (res != BME280_OK) {
    ESP_LOGE(TAG, "neasure delay calc error %d", res);
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed(LOG_STR(ESP_LOG_MSG_COMM_FAIL));
  }
  else {
    ESP_LOGD(TAG, "measure delay=%d", measure_delay_);
  }

  // setup done power supply can be switched off
  if (power_pin_ != nullptr) {
    power_pin_->digital_write(LOW);
  }
}

void BME280I2CWrapper::dump_config() {
  ESP_LOGCONFIG(TAG, "BME280I2CWrapper:");
  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGE(TAG, ESP_LOG_MSG_COMM_FAIL);
      break;
    case WRONG_CHIP_ID:
      ESP_LOGE(TAG, BME280_ERROR_WRONG_CHIP_ID);
      break;
    case NONE:
    default:
      break;
  }
  ESP_LOGCONFIG(TAG, "  IIR Filter: %s", iir_filter_to_str(this->settings_.filter));
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  address: 0x%02x", this->I2CDevice::get_i2c_address());
  ESP_LOGCONFIG(TAG, "  altitude: %f", this->altitude_);
  if (power_pin_ != nullptr) {
    LOG_PIN("  power_pin: ", power_pin_);
  }
  if (this->temperature_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
    ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->settings_.osr_t));
  }
  if (this->humidity_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
    ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->settings_.osr_h));
  }
  if (this->pressure_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
    ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->settings_.osr_p));
  }
}

float BME280I2CWrapper::get_setup_priority() const {
  return setup_priority::DATA;
}

void BME280I2CWrapper::update() {
  if (power_pin_ != nullptr) {
    power_pin_->digital_write(HIGH);
    delay(200);
  }

  if (forceMeasure()) {
    if (temperature_sensor_ != nullptr) {
      temperature_sensor_->publish_state(getTemperature());
    }
    if (humidity_sensor_ != nullptr) {
      humidity_sensor_->publish_state(getHumidity());
    }
    if (pressure_sensor_ != nullptr) {
      pressure_sensor_->publish_state(getPressure());
    }
    if (dew_point_sensor_ != nullptr) {
      dew_point_sensor_->publish_state(getDewPoint());
    }
    if (humidity_absolute_sensor_ != nullptr) {
      humidity_absolute_sensor_->publish_state(getHumidityAbsolute());
    }
    if (pressure_sealevel_sensor_ != nullptr) {
      pressure_sealevel_sensor_->publish_state(getPressureSeaLevel());
    }
  } else {
    ESP_LOGE(TAG, "BME measure failded");
  }

  if (power_pin_ != nullptr) {
    power_pin_->digital_write(LOW);
  }
}

double BME280I2CWrapper::getDewPoint() {
  // a and b are coefficients.
  double a = 17.62;
  double b = 243.12;
  double alpha = log(getHumidity() / 100) + a * getTemperature() / (getTemperature() + b);
  return b * alpha / (a - alpha); 
}

double BME280I2CWrapper::getHumidityAbsolute() {
      const double temperature = getTemperature();
      const double humidity = getHumidity();
      const double mw = 18.01534;    // molar mass of water g/mol
      const double r = 8.31447215;   // Universal gas constant J/mol/K
      return (6.112 * powf(2.718281828, (17.67 * temperature) /
        (temperature + 243.5)) * humidity * mw) /
        ((273.15 + temperature) * r);
}

double BME280I2CWrapper::getPressureSeaLevel() {
  return getPressure() / powf(1 - (altitude_/44330), 5.255); // refer to bmp180 documentation
}

bool BME280I2CWrapper::readSensorData() {
  return bme280_get_sensor_data(BME280_ALL, &comp_data_, &dev_) == 0;
}

bool BME280I2CWrapper::startMeasure() {
  uint8_t stat = 0;
  int8_t rslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &dev_);
  do {
    dev_.delay_us(measure_delay_, &dev_.intf_ptr);   // wait for the measurement to complete
    rslt = bme280_get_regs(BME280_REG_STATUS, &stat, 1, &dev_);
  } while (rslt == 0 && (stat & 0x08) != 0);
  return rslt == 0;
}

bool BME280I2CWrapper::forceMeasure() {
  uint8_t stat = 0;
  int8_t rslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, &dev_);
  do {
    dev_.delay_us(measure_delay_, &dev_.intf_ptr);   // wait for the measurement to complete
    rslt = bme280_get_regs(BME280_REG_STATUS, &stat, 1, &dev_);
  } while (rslt == 0 && (stat & 0x08) != 0);
  if (rslt == 0) {
    rslt = bme280_get_sensor_data(BME280_ALL, &comp_data_, &dev_);
  };
  return rslt == 0 && comp_data_.pressure > 90000.0;
}

int8_t BME280I2CWrapper::begin(int addr) {
  uint8_t dev_addr = addr;

  dev_.intf_ptr = &dev_addr;
  dev_.intf = BME280_I2C_INTF;
  dev_.read = user_i2c_read;
  dev_.write = user_i2c_write;
  dev_.delay_us = user_delay_us;
  
  bme280_init(&dev_);

  return dev_.intf_rslt;
}

}  // namespace bme280_i2c_wrapper
}  // namespace esphome
