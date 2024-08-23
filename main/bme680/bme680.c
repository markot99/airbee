/*

Library for the BME680 sensor

Adjusted from original library by Adafruit:
https://github.com/adafruit/Adafruit_BME680

*/

#define BME680_ADDRESS 0x77

#include "bme680.h"

#include <math.h>

#include "../i2c/i2c.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

//! @brief begin reading data from the sensor
//! @return milliseconds to wait until the data is ready
uint32_t beginReading();

//! @brief set the temperature oversampling
//! @param os Oversampling setting
//! @return True if successful, false if there was an error
bool setTemperatureOversampling(uint8_t oversample);

//! @brief set the pressure oversampling
//! @param os Oversampling setting
//! @return True if successful, false if there was an error
bool setPressureOversampling(uint8_t oversample);

//! @brief set the humidity oversampling
//! @param os Oversampling setting
//! @return True if successful, false if there was an error
bool setHumidityOversampling(uint8_t oversample);

//! @brief set the IIR filter size
//! @param filtersize Filter size
//! @return True if successful, false if there was an error
bool setIIRFilterSize(uint8_t filtersize);

//! @brief set the gas measurement interval
//! @param interval Interval in milliseconds
//! @return True if successful, false if there was an error
bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime);

//! @brief get the remaining time in milliseconds until the measurement is
//! complete
//! @return milliseconds to wait until the data is ready
int remainingReadingMillis();

//! Function to read from the I2C bus
//! @note function is passed to the BME68X library
//! @param reg_addr Register address
//! @param reg_data Pointer to the data to be read
//! @param len Number of bytes to read
//! @param intf_ptr Pointer to the I2C interface
//! @return 0 if successful, non-zero otherwise
static int8_t readI2C(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                      void *interface);

//! Function to write to the I2C bus
//! @note function is passed to the BME68X library
//! @param reg_addr Register address
//! @param reg_data Pointer to the data to be written
//! @param len Number of bytes to write
//! @param intf_ptr Pointer to the I2C interface
//! @return 0 if successful, non-zero otherwise
static int8_t writeI2C(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                       void *interface);

//! Function to delay execution
//! @note function is passed to the BME68X library
//! @param period Time in microseconds to delay
//! @param intf_ptr Pointer to the I2C interface
//! @return 0 if successful, non-zero otherwise
static void delayMicroseconds(uint32_t us, void *intf_ptr);

//! Sensor ID
int32_t _sensorID;

//! Timestamp of the start of the measurement
uint32_t m_meas_start = 0;

//! Measurement period in milliseconds
uint16_t m_meas_period = 0;

//! Sensor configuration
struct bme68x_dev gas_sensor;

//! Sensor configuration
struct bme68x_conf gas_conf;

//! Heater configuration
struct bme68x_heatr_conf gas_heatr_conf;

//! cached temperature in degrees C
float temperature;

//! cached pressure in Pascals
uint32_t pressure;

//! cached humidity in RH %
float humidity;

//! cached gas in ohms
uint32_t gas_resistance;

//! Flag to indicate that the sensor reading has been started
static const int reading_not_started = -1;

//! Flag to indicate that the sensor reading is completed
static const int reading_complete = 0;

//! Tag for logging
static const char *TAG = "BME680";

esp_err_t bme680_init() {
  ESP_LOGI(TAG, "Initializing sensor");
  gas_sensor.chip_id = BME680_ADDRESS;
  gas_sensor.intf = BME68X_I2C_INTF;
  // gas_sensor.intf_ptr = (void *)m_i2c;
  gas_sensor.read = &readI2C;
  gas_sensor.write = &writeI2C;
  gas_sensor.amb_temp = 25;
  gas_sensor.delay_us = delayMicroseconds;

  int8_t rslt = bme68x_init(&gas_sensor);

  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to initialize sensor");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Sensor initialized");

  // default settings
  setTemperatureOversampling(BME68X_OS_8X);
  setHumidityOversampling(BME68X_OS_2X);
  setPressureOversampling(BME68X_OS_4X);
  setIIRFilterSize(BME68X_FILTER_SIZE_3);
  setGasHeater(320, 150);

  rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);

  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to set sensor mode");
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Sensor mode set");
  return ESP_OK;
}

esp_err_t bme680_readData(bme680_data *air_quality_data_p) {
  uint32_t meas_end = beginReading();

  if (meas_end == 0) {
    return ESP_FAIL;
  }

  int remaining_millis = remainingReadingMillis();

  if (remaining_millis > 0) {
    /* Delay till the measurement is ready */
    delayMicroseconds(remaining_millis * 2 * 1000, NULL);
  }
  m_meas_start = 0; /* Allow new measurement to begin */
  m_meas_period = 0;

  struct bme68x_data data;
  uint8_t n_fields;

  int8_t rslt =
      bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &gas_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to get sensor data. Error code %d", rslt);
    return ESP_FAIL;
  }

  if (n_fields) {
    if (data.status & (BME68X_HEAT_STAB_MSK | BME68X_GASM_VALID_MSK)) {
      air_quality_data_p->temperature = data.temperature;
      air_quality_data_p->humidity = data.humidity;
      // convert from Pa to hPa
      air_quality_data_p->pressure = data.pressure / 100;
      air_quality_data_p->gas = data.gas_resistance;

      ESP_LOGI(TAG, "Measured Temperature: %.2f C", data.temperature);
      ESP_LOGI(TAG, "Measured Pressure: %.2f hPa", data.pressure / 100);
      ESP_LOGI(TAG, "Measured Humidity: %.2f %%", data.humidity);
      ESP_LOGI(TAG, "Measured Gas: %d ohms", (int)data.gas_resistance);
      return ESP_OK;
    }
  }

  return ESP_FAIL;
}

uint32_t beginReading(void) {
  if (m_meas_start != 0) {
    /* A measurement is already in progress */
    return m_meas_start + m_meas_period;
  }

  int8_t rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &gas_sensor);
  if (rslt != BME68X_OK) {
    ESP_LOGE(TAG, "Failed to set sensor mode");
    return false;
  }

  /* Calculate delay period in microseconds */
  uint32_t delayus_period = (uint32_t)bme68x_get_meas_dur(
                                BME68X_FORCED_MODE, &gas_conf, &gas_sensor) +
                            ((uint32_t)gas_heatr_conf.heatr_dur * 1000);

  m_meas_start = esp_timer_get_time();
  m_meas_period = delayus_period / 1000;

  return m_meas_start + m_meas_period;
}

int remainingReadingMillis(void) {
  if (m_meas_start != 0) {
    /* A measurement is already in progress */
    int remaining_time =
        (int)m_meas_period - (esp_timer_get_time() - m_meas_start);
    return remaining_time < 0 ? reading_complete : remaining_time;
  }
  return reading_not_started;
}

bool setGasHeater(uint16_t heaterTemp, uint16_t heaterTime) {
  if ((heaterTemp == 0) || (heaterTime == 0)) {
    gas_heatr_conf.enable = BME68X_DISABLE;
  } else {
    gas_heatr_conf.enable = BME68X_ENABLE;
    gas_heatr_conf.heatr_temp = heaterTemp;
    gas_heatr_conf.heatr_dur = heaterTime;
  }

  int8_t rslt =
      bme68x_set_heatr_conf(BME68X_FORCED_MODE, &gas_heatr_conf, &gas_sensor);
  return rslt == 0;
}

bool setTemperatureOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X) return false;

  gas_conf.os_temp = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
  return rslt == 0;
}

bool setHumidityOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X) return false;

  gas_conf.os_hum = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
  return rslt == 0;
}

bool setPressureOversampling(uint8_t oversample) {
  if (oversample > BME68X_OS_16X) return false;

  gas_conf.os_pres = oversample;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
  return rslt == 0;
}

bool setIIRFilterSize(uint8_t filtersize) {
  if (filtersize > BME68X_FILTER_SIZE_127) return false;
  gas_conf.filter = filtersize;

  int8_t rslt = bme68x_set_conf(&gas_conf, &gas_sensor);
  return rslt == 0;
}

int8_t readI2C(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
               void *intf_ptr) {
  i2c_read(BME680_ADDRESS, reg_addr, reg_data, len);

  return 0;
}

int8_t writeI2C(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                void *intf_ptr) {
  i2c_write(BME680_ADDRESS, reg_addr, reg_data, len);
  return 0;
}

void delayMicroseconds(uint32_t us, void *intf_ptr) {
  uint64_t start = esp_timer_get_time();
  while (esp_timer_get_time() - start < us) {
  }
}