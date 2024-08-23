#pragma once

//! @brief Initialize the data updater
void dataupdater_init();

//! @brief Start the data updater
//! @note Should only be called after the zigbee stack has been initialized!
void dataupdater_start();
