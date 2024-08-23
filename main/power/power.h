#pragma once

#include "esp_err.h"

//! @brief Initialize the power saving of the device
esp_err_t powersaving_init();

//! @brief Stop and lock the power saving of the device
void powersaving_lock();

//! @brief Unlock the power saving of the device
void powersaving_unlock();