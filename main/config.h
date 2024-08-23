#pragma once

//! @brief Interval in seconds to update the air quality data
#define AIRQUALITY_DATA_INTERVAL_SEC 300

//! @brief Interval in seconds to update the battery level
#define BATTERY_LEVEL_INTERVAL_SEC 86400

/*

General Zigbee Configuration

*/

//! @brief Manufacturer name for the Zigbee device
//! @note String must contain the length of the string as the first byte
#define ZIGBEE_DEVICE_MANUFACTURER_NAME \
  "\x06"                                \
  "AirBee" /* Customized manufacturer name */

//! @brief Model identifier for the Zigbee device
//! @note String must contain the length of the string as the first byte
#define ZIGBEE_DEVICE_MODEL \
  "\x06"                    \
  "Sensor" /* Customized model identifier */

/*

Temperature Zigbee Configuration

*/

//! @brief Minimum reporting interval for temperature in seconds
//! @note 0 means no minimum reporting interval
#define TEMPERATURE_MIN_REPORTING_INTERVAL_SEC 3600

//! @brief Offset for temperature reporting in degrees
#define TEMPERATURE_INSTANT_REPORTING_OFFSET_DEGREE 0.5

/*

Humidity Zigbee Configuration

*/

//! @brief Minimum reporting interval for humidity in seconds
//! @note 0 means no minimum reporting interval
#define HUMIDITY_MIN_REPORTING_INTERVAL_SEC 3600

//! @brief Offset for humidity reporting in percentage
#define HUMIDITY_INSTANT_REPORTING_OFFSET_PERCENT 5

/*

Pressure Zigbee Configuration

*/

//! @brief Minimum reporting interval for pressure in seconds
//! @note 0 means no minimum reporting interval
#define PRESSURE_MIN_REPORTING_INTERVAL_SEC 3600

//! @brief Offset for pressure reporting in hPa
#define PRESSURE_INSTANT_REPORTING_OFFSET_HPA 25

/*

Gas Zigbee Configuration

*/

//! @brief Minimum reporting interval for gas in seconds
//! @note 0 means no minimum reporting interval
#define GAS_MIN_REPORTING_INTERVAL_SEC 3600

//! @brief Offset for gas reporting in ohms
#define GAS_INSTANT_REPORTING_OFFSET_OHMS 10000

/*

Battery Zigbee Configuration

*/

//! @brief Minimum reporting interval for battery in seconds
//! @note 0 means no minimum reporting interval
#define BATTERY_MIN_REPORTING_INTERVAL_SEC 3600

//! @brief Offset for battery reporting in percentage
#define BATTERY_INSTANT_REPORTING_OFFSET_PERCENTAGE 5