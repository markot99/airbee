#include "dataupdater.h"

#include "../bme680/bme680.h"
#include "../config.h"
#include "../i2c/i2c.h"
#include "../power/power.h"
#include "./battery/battery.h"
#include "./zigbee/zigbee.h"
#include "esp_log.h"

//! @brief Tag for the log messages
static const char *TAG = "DATAUPDATER";

//! event groups to wait until the zigbee stack is ready
static EventGroupHandle_t update_air_quality_hold;
static EventGroupHandle_t update_battery_data_hold;

//! @brief Task to update the air quality data
//! @param[in] pvParameters Unused
void update_air_quality_data_task(void *pvParameters);

//! @brief Task to update the battery level
//! @param[in] pvParameters Unused
void update_battery_level_task(void *pvParameters);

void dataupdater_init() {
  update_air_quality_hold = xEventGroupCreate();
  update_battery_data_hold = xEventGroupCreate();
  xTaskCreate(update_air_quality_data_task, "update_air_quality_data_task",
              4096, NULL, 6, NULL);
  xTaskCreate(update_battery_level_task, "update_battery_level_task", 4096,
              NULL, 6, NULL);
}

void dataupdater_start() {
  xEventGroupSetBits(update_air_quality_hold, BIT0);
  xEventGroupSetBits(update_battery_data_hold, BIT0);
}

void update_air_quality_data_task(void *pvParameters) {
  ESP_LOGI(TAG,
           "Update air quality data task waiting for zigbee stack to be ready");
  xEventGroupWaitBits(update_air_quality_hold, BIT0, pdTRUE, pdFALSE,
                      portMAX_DELAY);

  ESP_LOGI(TAG, "Update air quality data task started, zigbee stack is ready");
  while (1) {
    vTaskDelay(AIRQUALITY_DATA_INTERVAL_SEC * 1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Updating air quality data");

    bme680_data air_quality_data;

    // esp loses i2c connection after light sleep, so we need to reinit it
    i2c_init();
    bme680_init();
    ESP_ERROR_CHECK(bme680_readData(&air_quality_data));

    zigbee_updateAirQualityData(&air_quality_data);
  }
}

void update_battery_level_task(void *pvParameters) {
  ESP_LOGI(TAG,
           "Update battery level task waiting for zigbee stack to get ready");
  xEventGroupWaitBits(update_battery_data_hold, BIT0, pdTRUE, pdFALSE,
                      portMAX_DELAY);

  ESP_LOGI(TAG, "Update battery level task started, zigbee stack is ready");
  while (1) {
    vTaskDelay(BATTERY_LEVEL_INTERVAL_SEC * 1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Updating battery level");

    uint8_t remaining_battery_percentage = battery_getRemainingPercentage();

    zigbee_updateBatteryLevel(remaining_battery_percentage);
  }
}