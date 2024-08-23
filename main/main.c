#include "./dataupdater/dataupdater.h"
#include "./power/power.h"
#include "./zigbee/zigbee.h"
#include "esp_check.h"
#include "esp_ieee802154.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "esp_zigbee_core.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "zcl/esp_zigbee_zcl_power_config.h"

//! Tag for logging
static const char *TAG = "MAIN";

//! @brief Main function
void app_main(void) {
  ESP_LOGI(TAG, "Starting the application");
  nvs_flash_init();
  ESP_ERROR_CHECK(nvs_flash_erase());
  ESP_ERROR_CHECK(nvs_flash_init());

  powersaving_init();
  powersaving_lock();

  dataupdater_init();
  zigbee_start();
  ESP_LOGI(TAG, "Application started");
}
