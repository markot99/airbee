/*

Library for the BME680 sensor

Adjusted from original library by Adafruit:
https://github.com/adafruit/Adafruit_BME680

*/

#pragma once

#include "../i2c/i2c.h"
#include "./libs/bme68x.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define BME68X_DEFAULT_ADDRESS (0x77)

//! struct for the data from the BME680 sensor
typedef struct bme680_data {
  //! Temperature in degrees C
  float temperature;

  //! Pressure in hektopascals
  float pressure;

  //! Humidity in RH %
  float humidity;

  //! VOC in ohms
  uint32_t gas;
} bme680_data;

//! Initialize the BME680 sensor
esp_err_t bme680_init();

//! @brief Start a reading and store the results in class variables
//! @param[out] air_quality_data_p The data from the sensor
//! @return True if successful, false if there was an error
esp_err_t bme680_readData(bme680_data *air_quality_data_p);
