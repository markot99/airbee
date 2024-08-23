#include "zigbee.h"

#include <math.h>

#include "../battery/battery.h"
#include "../config.h"
#include "../dataupdater/dataupdater.h"
#include "../power/power.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "nvs_flash.h"
#include "zboss_api.h"
#include "zcl/esp_zigbee_zcl_power_config.h"

//! @brief Start the top level commissioning
//! @param mode_mask The commissioning mode mask
//! @note This function is called by the Zigbee stack
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);

//! @brief Handle the Zigbee signals
//! @param signal_struct The signal structure
//! @note This function is called by the Zigbee stack
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);

//! @brief Attribute handler for Zigbee
//! @param message The message to handle
//! @note This function is called by the Zigbee stack
//! @return ESP_OK on success, else error code
static esp_err_t zb_attribute_handler(
    const esp_zb_zcl_set_attr_value_message_t *message);

//! @brief Action handler for Zigbee
//! @param callback_id The callback ID
//! @param message The message to handle
//! @note This function is called by the Zigbee stack
//! @return ESP_OK on success, else error code
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message);

//! @brief Task for the Zigbee stack
//! @param pvParameters The task parameters
static void esp_zb_task(void *pvParameters);

static const char *TAG = "ZIGBEE";

void set_connected_task(void *pvParameters);

#define CUSTOM_CLUSTER_ID 0xFFF2
#define CUSTROM_ATTRIBUTE_1_ID 0x0001U

#define INSTALLCODE_POLICY_ENABLE false
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE 3000
#define HA_ESP_SENSOR_ENDPOINT 10
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

#define ESP_ZB_ZED_CONFIG()                             \
  {                                                     \
      .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,             \
      .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
      .nwk_cfg =                                        \
          {                                             \
              .zed_cfg =                                \
                  {                                     \
                      .ed_timeout = ED_AGING_TIMEOUT,   \
                      .keep_alive = ED_KEEP_ALIVE,      \
                  },                                    \
          },                                            \
  }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
  {                                                             \
    .radio_mode = ZB_RADIO_MODE_NATIVE, .radio_uart_config = {} \
  }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                      \
  {                                                       \
    .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    .host_uart_config = {}                                \
  }

static EventGroupHandle_t event_group;
static bool connected = false;

void zigbee_start() {
  event_group = xEventGroupCreate();
  esp_zb_platform_config_t config = {
      .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
      .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
  };
  ESP_ERROR_CHECK(esp_zb_platform_config(&config));

  xTaskCreate(set_connected_task, "Connected_task", 4096, NULL, 5, NULL);
  xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}

void report_attr_cb(uint16_t attributeID, uint16_t clusterID) {
  ESP_LOGI(TAG, "Report attribute callback");

  esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
  report_attr_cmd.attributeID = attributeID;
  report_attr_cmd.clusterID = clusterID;
  report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
  report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
  report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_SENSOR_ENDPOINT;

  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
  esp_zb_lock_release();
}

static float reported_temperature = 0.0f;
static int64_t last_temperature_report_time = 0;

static float reported_pressure = 0.0f;
static int64_t last_pressure_report_time = 0;

static float reported_humidity = 0.0f;
static int64_t last_humidity_report_time = 0;

static uint32_t reported_gas = 0;
static int64_t last_gas_report_time = 0;

static uint8_t reported_battery = 0;
static int64_t last_battery_report_time = 0;

void zigbee_updateAirQualityData(bme680_data *air_quality_data_p) {
  // convert data to the format that Zigbee expects
  int32_t z_temperature = (int32_t)(air_quality_data_p->temperature * 100);
  int32_t z_humidity = (int32_t)(air_quality_data_p->humidity * 100);
  int32_t z_pressure = (int32_t)(air_quality_data_p->pressure);
  float_t z_gas = (float_t)(air_quality_data_p->gas / 1000000.0);

  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_set_attribute_val(
      HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
      &z_temperature, false);

  esp_zb_zcl_set_attribute_val(
      HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &z_humidity, false);

  esp_zb_zcl_set_attribute_val(
      HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &z_pressure, false);

  esp_zb_zcl_set_attribute_val(
      HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &z_gas,
      false);
  esp_zb_lock_release();

  if (fabs(reported_temperature - air_quality_data_p->temperature) >=
      TEMPERATURE_INSTANT_REPORTING_OFFSET_DEGREE) {
    ESP_LOGI(TAG, "Temperature changed by %f. Reporting!",
             fabs(reported_temperature - air_quality_data_p->temperature));
    report_attr_cb(ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    reported_temperature = air_quality_data_p->temperature;
    last_temperature_report_time = esp_timer_get_time();
  } else if ((esp_timer_get_time() - last_temperature_report_time) / 1000000 >=
             TEMPERATURE_MIN_REPORTING_INTERVAL_SEC) {
    ESP_LOGI(
        TAG, "time since last temperature report: %d",
        (int)((esp_timer_get_time() - last_temperature_report_time) / 1000000));
    ESP_LOGI(TAG, "Minimum reporting interval: %d",
             TEMPERATURE_MIN_REPORTING_INTERVAL_SEC);
    ESP_LOGI(TAG, "Temperature reporting interval reached. Reporting!");
    report_attr_cb(ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
    reported_temperature = air_quality_data_p->temperature;
    last_temperature_report_time = esp_timer_get_time();
  } else {
    ESP_LOGI(TAG,
             "Temperature reporting interval not reached, seconds until next "
             "report: %d",
             (TEMPERATURE_MIN_REPORTING_INTERVAL_SEC -
              (int)((esp_timer_get_time() - last_temperature_report_time)) /
                  1000000));
  }

  if (fabs(reported_humidity - air_quality_data_p->humidity) >=
      HUMIDITY_INSTANT_REPORTING_OFFSET_PERCENT) {
    ESP_LOGI(TAG, "Humidity changed by %f. Reporting!",
             fabs(reported_humidity - air_quality_data_p->humidity));
    report_attr_cb(ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    reported_humidity = air_quality_data_p->humidity;
    last_humidity_report_time = esp_timer_get_time();
  } else if ((esp_timer_get_time() - last_humidity_report_time) / 1000000 >=
             HUMIDITY_MIN_REPORTING_INTERVAL_SEC) {
    ESP_LOGI(TAG, "Humidity reporting interval reached. Reporting!");
    report_attr_cb(ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
    reported_humidity = air_quality_data_p->humidity;
    last_humidity_report_time = esp_timer_get_time();
  }

  if (fabs(reported_pressure - air_quality_data_p->pressure) >=
      PRESSURE_INSTANT_REPORTING_OFFSET_HPA) {
    ESP_LOGI(TAG, "Pressure changed by %f. Reporting!",
             fabs(reported_pressure - air_quality_data_p->pressure));
    report_attr_cb(ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    reported_pressure = air_quality_data_p->pressure;
    last_pressure_report_time = esp_timer_get_time();
  } else if ((esp_timer_get_time() - last_pressure_report_time) / 1000000 >=
             PRESSURE_MIN_REPORTING_INTERVAL_SEC) {
    ESP_LOGI(TAG, "Pressure reporting interval reached. Reporting!");
    report_attr_cb(ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
    reported_pressure = air_quality_data_p->pressure;
    last_pressure_report_time = esp_timer_get_time();
  }

  if (abs(reported_gas - air_quality_data_p->gas) >=
      GAS_INSTANT_REPORTING_OFFSET_OHMS) {
    ESP_LOGI(TAG, "Gas changed by %d. Reporting!",
             abs(reported_gas - air_quality_data_p->gas));
    report_attr_cb(ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT);
    reported_gas = air_quality_data_p->gas;
    last_gas_report_time = esp_timer_get_time();
  } else if ((esp_timer_get_time() - last_gas_report_time) / 1000000 >=
             GAS_MIN_REPORTING_INTERVAL_SEC) {
    ESP_LOGI(TAG, "Gas reporting interval reached. Reporting!");
    report_attr_cb(ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT);
    reported_gas = air_quality_data_p->gas;
    last_gas_report_time = esp_timer_get_time();
  }
}

void zigbee_updateBatteryLevel(uint8_t battery_level) {
  // convert data to the format that Zigbee expects
  int8_t z_remaining_battery_percentage = battery_level * 2;

  esp_zb_lock_acquire(portMAX_DELAY);
  esp_zb_zcl_set_attribute_val(
      HA_ESP_SENSOR_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
      &z_remaining_battery_percentage, false);
  esp_zb_lock_release();

  if (abs(reported_battery - battery_level) >=
      BATTERY_INSTANT_REPORTING_OFFSET_PERCENTAGE) {
    ESP_LOGI(TAG, "Battery changed by %d. Reporting!",
             abs(reported_battery - battery_level));
    report_attr_cb(ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    reported_battery = battery_level;
    last_battery_report_time = esp_timer_get_time();
  } else if ((esp_timer_get_time() - last_battery_report_time) / 1000000 >=
             BATTERY_MIN_REPORTING_INTERVAL_SEC) {
    ESP_LOGI(TAG, "Battery reporting interval reached. Reporting!");
    report_attr_cb(ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                   ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    reported_battery = battery_level;
    last_battery_report_time = esp_timer_get_time();
  }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
  ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
  uint32_t *p_sg_p = signal_struct->p_app_signal;
  esp_err_t err_status = signal_struct->esp_err_status;
  esp_zb_app_signal_type_t sig_type = *p_sg_p;
  switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
      ESP_LOGI(TAG, "Zigbee stack initialized");
      esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
      break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
      if (err_status == ESP_OK) {
        ESP_LOGI(TAG, "Device started up in %s factory-reset mode",
                 esp_zb_bdb_is_factory_new() ? "" : "non");
        if (esp_zb_bdb_is_factory_new()) {
          ESP_LOGI(TAG, "Start network steering");
          esp_zb_bdb_start_top_level_commissioning(
              ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
          ESP_LOGI(TAG, "Device rebooted");
          xEventGroupSetBits(event_group, BIT0);
        }
      } else {
        /* commissioning failed */
        ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)",
                 esp_err_to_name(err_status));
      }
      break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
      if (err_status == ESP_OK) {
        esp_zb_ieee_addr_t extended_pan_id;
        esp_zb_get_extended_pan_id(extended_pan_id);
        ESP_LOGI(TAG,
                 "Joined network successfully (Extended PAN ID: "
                 "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, "
                 "Channel:%d, Short Address: 0x%04hx)",
                 extended_pan_id[7], extended_pan_id[6], extended_pan_id[5],
                 extended_pan_id[4], extended_pan_id[3], extended_pan_id[2],
                 extended_pan_id[1], extended_pan_id[0], esp_zb_get_pan_id(),
                 esp_zb_get_current_channel(), esp_zb_get_short_address());
        xEventGroupSetBits(event_group, BIT0);
      } else {
        ESP_LOGI(TAG, "Network steering was not successful (status: %s)",
                 esp_err_to_name(err_status));
        esp_zb_scheduler_alarm(
            (esp_zb_callback_t)bdb_start_top_level_commissioning_cb,
            ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
      }
      break;
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:

      if (connected) {
        ESP_LOGD(TAG, "Going to sleep");
        esp_zb_sleep_now();
        ESP_LOGD(TAG, "Woke up from sleep");
      }

      break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
      ESP_LOGI(TAG, "ESP has been removed from the network");
      break;
    default:
      ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s",
               esp_zb_zdo_signal_to_string(sig_type), sig_type,
               esp_err_to_name(err_status));
      break;
  }
}

static esp_err_t zb_attribute_handler(
    const esp_zb_zcl_set_attr_value_message_t *message) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
  ESP_RETURN_ON_FALSE(
      message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG,
      TAG, "Received message: error status(%d)", message->info.status);
  ESP_LOGI(TAG,
           "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), "
           "data size(%d)",
           message->info.dst_endpoint, message->info.cluster,
           message->attribute.id, message->attribute.data.size);
  return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id,
                                   const void *message) {
  esp_err_t ret = ESP_OK;
  switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
      ret =
          zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
      break;
    case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
      ESP_LOGI(TAG, "Received identify effect callback");
      break;
    default:
      ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
      break;
  }
  return ret;
}

void register_zb_device() {
  esp_zb_attribute_list_t *basic_cluster =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
      basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
      (void *)ZIGBEE_DEVICE_MANUFACTURER_NAME));
  ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(
      basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
      (void *)ZIGBEE_DEVICE_MODEL));

  // add temperature cluster
  uint16_t tempValue = 0;
  uint16_t tempMin = 0;
  uint16_t tempMax = 50;
  esp_zb_attribute_list_t *esp_zb_temperature_cluster =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);
  esp_zb_temperature_meas_cluster_add_attr(
      esp_zb_temperature_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
      &tempValue);
  esp_zb_temperature_meas_cluster_add_attr(
      esp_zb_temperature_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID,
      &tempMin);
  esp_zb_temperature_meas_cluster_add_attr(
      esp_zb_temperature_cluster, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID,
      &tempMax);

  // add humidity cluster
  uint16_t humidityValue = 0;
  uint16_t humidityMin = 0;
  uint16_t humidityMax = 100;
  esp_zb_attribute_list_t *esp_zb_humidity_cluster =
      esp_zb_zcl_attr_list_create(
          ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);
  esp_zb_humidity_meas_cluster_add_attr(
      esp_zb_humidity_cluster,
      ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &humidityValue);
  esp_zb_humidity_meas_cluster_add_attr(
      esp_zb_humidity_cluster,
      ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &humidityMin);
  esp_zb_humidity_meas_cluster_add_attr(
      esp_zb_humidity_cluster,
      ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &humidityMax);

  // add pressure cluster
  uint16_t pressureValue = 0;
  uint16_t pressureMin = 0;
  uint16_t pressureMax = 100;
  esp_zb_attribute_list_t *esp_zb_pressure_cluster =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);
  esp_zb_pressure_meas_cluster_add_attr(
      esp_zb_pressure_cluster, ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
      &pressureValue);
  esp_zb_pressure_meas_cluster_add_attr(
      esp_zb_pressure_cluster,
      ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_ID, &pressureMin);
  esp_zb_pressure_meas_cluster_add_attr(
      esp_zb_pressure_cluster,
      ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_ID, &pressureMax);

  // add gas cluster
  float_t gasValue = 0.0;
  float_t gasMin = 0.0;
  float_t gasMax = 1.0;
  esp_zb_attribute_list_t *esp_zb_gas_cluster = esp_zb_zcl_attr_list_create(
      ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT);
  esp_zb_carbon_dioxide_measurement_cluster_add_attr(
      esp_zb_gas_cluster,
      ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID, &gasValue);
  esp_zb_carbon_dioxide_measurement_cluster_add_attr(
      esp_zb_gas_cluster,
      ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MIN_MEASURED_VALUE_ID,
      &gasMin);
  esp_zb_carbon_dioxide_measurement_cluster_add_attr(
      esp_zb_gas_cluster,
      ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MAX_MEASURED_VALUE_ID,
      &gasMax);

  // add battery cluster
  uint16_t battery_remaining = 0;
  esp_zb_attribute_list_t *esp_zb_battery_cluster =
      esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
  esp_zb_power_config_cluster_add_attr(
      esp_zb_battery_cluster,
      ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
      &battery_remaining);

  esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(
      cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(
      cluster_list, esp_zb_temperature_cluster,
      ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(
      cluster_list, esp_zb_humidity_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_pressure_meas_cluster(
      cluster_list, esp_zb_pressure_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(
      cluster_list, esp_zb_gas_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
  ESP_ERROR_CHECK(esp_zb_cluster_list_add_power_config_cluster(
      cluster_list, esp_zb_battery_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

  esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

  esp_zb_endpoint_config_t endpoint_config = {
      .endpoint = HA_ESP_SENSOR_ENDPOINT,
      .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
      .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
      .app_device_version = 0};

  esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);

  esp_zb_device_register(ep_list);
}

void configure_attribute_reporting() {
  esp_zb_zcl_attr_location_info_t temp_info = {
      .endpoint_id = HA_ESP_SENSOR_ENDPOINT,
      .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
      .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
      .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
  };
  ESP_ERROR_CHECK(esp_zb_zcl_stop_attr_reporting(temp_info));

  esp_zb_zcl_attr_location_info_t humidity_info = {
      .endpoint_id = HA_ESP_SENSOR_ENDPOINT,
      .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
      .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
      .attr_id = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
  };
  ESP_ERROR_CHECK(esp_zb_zcl_stop_attr_reporting(humidity_info));

  esp_zb_zcl_attr_location_info_t pressure_info = {
      .endpoint_id = HA_ESP_SENSOR_ENDPOINT,
      .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
      .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
      .attr_id = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
  };
  ESP_ERROR_CHECK(esp_zb_zcl_stop_attr_reporting(pressure_info));

  esp_zb_zcl_attr_location_info_t gas_info = {
      .endpoint_id = HA_ESP_SENSOR_ENDPOINT,
      .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
      .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
      .attr_id = ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
  };
  ESP_ERROR_CHECK(esp_zb_zcl_stop_attr_reporting(gas_info));

  esp_zb_zcl_attr_location_info_t battery_info = {
      .endpoint_id = HA_ESP_SENSOR_ENDPOINT,
      .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG,
      .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
      .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
      .attr_id = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
  };
  ESP_ERROR_CHECK(esp_zb_zcl_stop_attr_reporting(battery_info));
}

static void esp_zb_task(void *pvParameters) {
  esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();

  esp_zb_sleep_enable(true);
  esp_zb_init(&zb_nwk_cfg);

  register_zb_device();

  esp_zb_core_action_handler_register(zb_action_handler);

  configure_attribute_reporting();

  // set the data in the zigbee stack
  bme680_data air_quality_data;
  i2c_init();
  bme680_init();
  ESP_ERROR_CHECK(bme680_readData(&air_quality_data));
  uint8_t remaining_battery_percentage = battery_getRemainingPercentage();
  zigbee_updateAirQualityData(&air_quality_data);
  zigbee_updateBatteryLevel(remaining_battery_percentage);

  // start the data updater
  dataupdater_start();

  esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

  ESP_ERROR_CHECK(esp_zb_start(false));
  esp_zb_main_loop_iteration();
}

void set_connected_task(void *pvParameters) {
  xEventGroupWaitBits(event_group, BIT0, pdTRUE, pdFALSE, portMAX_DELAY);
  ESP_LOGI(TAG, "Connected task started");

  // waiting 30 seconds to finish the commissioning
  vTaskDelay(30000 / portTICK_PERIOD_MS);
  zb_zdo_pim_set_long_poll_interval(3600000);
  powersaving_unlock();
  connected = true;
  ESP_LOGI(TAG, "Connected task finished");
  vTaskDelete(NULL);
}