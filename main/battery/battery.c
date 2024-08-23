#include "battery.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "hal/adc_types.h"
#include "stdint.h"

static const char *TAG = "BATTERY";

//! @brief Size of the conversion table
#define TABLE_SIZE 101

//! @brief Conversion table from voltage to percentage (index is percentage)
//! @note values from https://github.com/danilopinotti/Battery18650Stats
const float conversion_table[TABLE_SIZE] = {
    3.200, 3.250, 3.300, 3.350, 3.400, 3.450, 3.500, 3.550, 3.600, 3.650, 3.700,
    3.703, 3.706, 3.710, 3.713, 3.716, 3.719, 3.723, 3.726, 3.729, 3.732, 3.735,
    3.739, 3.742, 3.745, 3.748, 3.752, 3.755, 3.758, 3.761, 3.765, 3.768, 3.771,
    3.774, 3.777, 3.781, 3.784, 3.787, 3.790, 3.794, 3.797, 3.800, 3.805, 3.811,
    3.816, 3.821, 3.826, 3.832, 3.837, 3.842, 3.847, 3.853, 3.858, 3.863, 3.868,
    3.874, 3.879, 3.884, 3.889, 3.895, 3.900, 3.906, 3.911, 3.917, 3.922, 3.928,
    3.933, 3.939, 3.944, 3.950, 3.956, 3.961, 3.967, 3.972, 3.978, 3.983, 3.989,
    3.994, 4.000, 4.008, 4.015, 4.023, 4.031, 4.038, 4.046, 4.054, 4.062, 4.069,
    4.077, 4.085, 4.092, 4.100, 4.111, 4.122, 4.133, 4.144, 4.156, 4.167, 4.178,
    4.189, 4.200};

//! @brief Get the current battery voltage
//! @return Battery voltage in volts
double getVoltage();

double getVoltage() {
  int adc_read0;
  int mv_output;

  adc_oneshot_unit_handle_t handle = NULL;

  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE,
  };

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &handle));

  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  ESP_ERROR_CHECK(adc_oneshot_config_channel(handle, ADC_CHANNEL_0, &config));

  adc_cali_handle_t cali_handle = NULL;

  adc_cali_curve_fitting_config_t cali_config = {
      .unit_id = ADC_UNIT_1,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  ESP_ERROR_CHECK(
      adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));

  ESP_ERROR_CHECK(adc_oneshot_read(handle, ADC_CHANNEL_0, &adc_read0));
  adc_cali_raw_to_voltage(cali_handle, adc_read0, &mv_output);

  ESP_ERROR_CHECK(adc_oneshot_del_unit(handle));
  ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(cali_handle));

  return (mv_output * 2) / 1000.0;
}

uint8_t battery_getRemainingPercentage() {
  double voltage = getVoltage();

  // binary search faster than iterating through the array
  int low = 0;
  int high = TABLE_SIZE - 1;
  int mid;

  if (voltage <= conversion_table[0]) {
    return 0;
  }

  while (low <= high) {
    mid = (low + high) / 2;

    if (conversion_table[mid] == voltage) {
      ESP_LOGI(TAG, "Measured Voltage: %f, Percentage: %d", voltage, mid);
      return mid;
    } else if (conversion_table[mid] < voltage) {
      low = mid + 1;
    } else {
      high = mid - 1;
    }
  }
  ESP_LOGI(TAG, "Measured Voltage: %f, Percentage: %d", voltage, high);
  return high;
}
