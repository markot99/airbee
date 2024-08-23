#include "i2c.h"

#include <string.h>

#include "freertos/FreeRTOS.h"

#define MASTER_TIMEOUT_MS 1000
#define SDA_PIN 19
#define SCL_PIN 20

//! @brief The I2C configuration.
const i2c_config_t m_conf = {.mode = I2C_MODE_MASTER,
                             .sda_io_num = SDA_PIN,
                             .scl_io_num = SCL_PIN,
                             .sda_pullup_en = GPIO_PULLUP_ENABLE,
                             .scl_pullup_en = GPIO_PULLUP_ENABLE,
                             .master = {.clk_speed = 100000},
                             .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL};

void i2c_init() {
  i2c_param_config(I2C_NUM_0, &m_conf);
  i2c_driver_install(I2C_NUM_0, m_conf.mode, 0, 0, 0);
}

void i2c_read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data,
              size_t len) {
  i2c_master_write_read_device(I2C_NUM_0, device_addr, &reg_addr, 1, data, len,
                               MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void i2c_write(uint8_t device_addr, uint8_t reg_addr, const uint8_t *data,
               size_t len) {
  uint8_t write_buf[len + 1];
  write_buf[0] = reg_addr;
  memcpy(write_buf + 1, data, len);

  i2c_master_write_to_device(I2C_NUM_0, device_addr, write_buf,
                             sizeof(write_buf),
                             MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
