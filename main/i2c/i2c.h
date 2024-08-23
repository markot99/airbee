#pragma once

#include "driver/i2c.h"

//! @brief Initialize the I2C
void i2c_init();

//! @brief Read data from the I2C.
//! @param device_addr The address of the device to read from.
//! @param reg_addr The address of the register to read from.
//! @param data The pointer to store the data in.
//! @param len The length of the data to read.
void i2c_read(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, size_t len);

//! @brief Write data to the I2C.
//! @param device_addr The address of the device to write to.
//! @param reg_addr The address of the register to write to.
//! @param data The data to write.
//! @param len The length of the data to write.
void i2c_write(uint8_t device_addr, uint8_t reg_addr, const uint8_t *data,
               size_t len);
