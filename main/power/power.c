#include "power.h"

#include "esp_log.h"
#include "esp_pm.h"

//! power lock handle
static esp_pm_lock_handle_t pm_lock;

static const char *TAG = "POWER";

esp_err_t powersaving_init() {
  ESP_LOGI(TAG, "Initializing power saving");
  esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
  int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
  esp_pm_config_t pm_config = {.max_freq_mhz = cur_cpu_freq_mhz,
                               .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
                               .light_sleep_enable = true
#endif
  };
  rc = esp_pm_configure(&pm_config);
#endif
  ESP_LOGI(TAG, "Power saving initialized");
  return rc;
}

void powersaving_lock() {
  ESP_LOGI(TAG, "Locking power saving");
  esp_pm_lock_acquire(pm_lock);
}

void powersaving_unlock() {
  ESP_LOGI(TAG, "Unlocking power saving");
  esp_pm_lock_release(pm_lock);
}