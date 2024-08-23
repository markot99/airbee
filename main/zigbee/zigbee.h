#pragma once
#include <stdint.h>

#include "../bme680/bme680.h"

//! @brief register the device with the clusters and start the device
//! @note This function starts a task and returns immediately
void zigbee_start();

//! @brief update the air quality data
//! @param[in] air_quality_data_p The data from the sensor
void zigbee_updateAirQualityData(bme680_data *air_quality_data_p);

//! @brief update the battery level
//! @param[in] battery_level Battery level in percentage
void zigbee_updateBatteryLevel(uint8_t battery_level);
