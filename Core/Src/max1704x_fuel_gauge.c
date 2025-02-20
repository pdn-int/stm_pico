#include "logger.h"
#include "max1704x_fuel_gauge.h"

#ifdef STM32_UMOTE
	extern I2C_HandleTypeDef hi2c1;
	#define max1704x_i2c hi2c1
#else //Pico
	extern I2C_HandleTypeDef hi2c3;
	#define max1704x_i2c hi2c3
#endif


/*************************************************************************
 * Initialization
 *************************************************************************/
extern IC3Daemon ic3;

bool max1704x_init(void) {
	uint8_t id;

	id = max1704x_get_id();
	if(id != MAX1704X_ID) {
//		L_ERROR("Error: Failed to read MAX1704x Fuel gauge sensor - incorrect MAX1704X ID (%d)", id);
		return false;
	}

  // Configure fuel gauge interrupts
  if(ic3.fgauge_int_enabled) {
//    L_INFO("Fuel gauge interrupt enabled. Setting configurations for interrupt...");

    // Configure GPIO for external interrupt
    __HAL_RCC_GPIOI_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = MAX1704X_INT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(MAX1704X_INT_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(MAX1704X_EXTI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(MAX1704X_EXTI_IRQn);

    // clear STATUS alerts
    uint16_t status = max1704x_get_status();
    max1704x_clear_status_alerts(MAX1704X_STATUS_RI_BIT + MAX1704X_STATUS_VH_BIT +
            MAX1704X_STATUS_VL_BIT + MAX1704X_STATUS_VR_BIT +
            MAX1704X_STATUS_HD_BIT + MAX1704X_STATUS_SC_BIT);

    // Enable STATUS.EnVr (battery removal/reinsertion detection)
    max1704x_set_status_envr(true);

    // Set VALRT.MIN/MAX (volage low/high)
    max1704x_set_valrt_min(ic3.fgauge_valrt_min);
    max1704x_set_valrt_max(ic3.fgauge_valrt_max);

    // Set CONFIG.ATHD (SOC low)
    uint8_t athd = 32 - ic3.fgauge_soc_low;
    max1704x_set_config_athd(athd);

    // Enable CONFIG.ALSC (SOC change alert)
    max1704x_set_config_alsc(ic3.fgauge_soc_change_enabled);

    // Clear CONFIG.ALRT bit
    max1704x_clear_config_alrt();

  }

//	L_INFO("Fuel Gauge initialization completed");

	return true;
}

/*************************************************************************
 * Sleep Operations
 *************************************************************************/
void max1704x_enable_sleep(bool sleep) {
  // put to sleep by
  //  1. MODE.EnSleep = 1
  //  2. CONFIG.SLEEP = 1
  // wake up from sleep by CONFIG.SLEEP = 0

  if(sleep) {
    max1704x_set_mode_ensleep(true);
    max1704x_set_config_sleep(true);
//    L_INFO("Fuel Gauge entering sleep");
  } else {
    max1704x_set_config_sleep(false);
//    L_INFO("Fuel Gauge exiting sleep");
  }
}

/*************************************************************************
 * ID
 *************************************************************************/
uint8_t max1704x_get_id(void) {
	uint8_t rData[2] = {0};

	HAL_I2C_Mem_Read(&max1704x_i2c, MAX1704X_I2C_ADDR, MAX1704X_VRESET_ID_ADDR, 1, rData, 2, 10);
//	L_DEBUG("Vreset/Id = %02X/%02X", rData[0], rData[1]);
	return rData[1];
}

/*************************************************************************
 * Version
 *************************************************************************/
uint8_t max1704x_get_version(void) {
	uint8_t rData[2] = {0};

	HAL_I2C_Mem_Read(&max1704x_i2c, MAX1704X_I2C_ADDR, MAX1704X_VERSION_ADDR, 1, rData, 2, 10);
//	L_DEBUG("Version = %04X", (uint16_t)(rData[0]<<8) + (uint16_t)rData[1]);
	return (uint16_t)(rData[0]<<8) + (uint16_t)rData[1];
}

/*************************************************************************
 * VCELL
 *************************************************************************/
void max1704x_get_vcell(COT_DATA *data) {
	uint8_t rData[2] = {0};
	uint8_t wData[1] = {MAX1704X_VCELL_ADDR};

	// Problem using HAL_I2C_Mem_Read() where HAL_ERROR returned after 5 mins
	HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 1, 100);
	HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(&max1704x_i2c, MAX1704X_I2C_ADDR, rData, 2, 100);
	if(hal_status != HAL_OK) {
//		L_ERROR("ERROR: Cannot read VCELL from MAX17048 Fuel Gauge sensor %d", hal_status);
		data->fgauge.vcell = 0;
	} else {
    data->fgauge.vcell = (((uint16_t)rData[0]<<8) + (uint16_t)rData[1]) * 78.125 / 1000000;
//    L_DEBUG("VCELL = %f V (%02X, %02X)", data->fgauge.vcell, rData[0], rData[1]);
  }
}

/*************************************************************************
 * SOC
 *************************************************************************/
void max1704x_get_soc(COT_DATA *data) {
	uint8_t rData[2] = {0};
	uint8_t wData[1] = {MAX1704X_SOC_ADDR};

	HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 1, 100);
	HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(&max1704x_i2c, MAX1704X_I2C_ADDR, rData, 2, 100);
	if(hal_status != HAL_OK) {
//		L_ERROR("ERROR: Cannot read SOC from MAX17048 Fuel Gauge sensor %d", hal_status);
		data->fgauge.soc = 0;
	} else {
    data->fgauge.soc = (((uint16_t)rData[0]<<8) + (uint16_t)rData[1]) / 256.0;
//    L_DEBUG("SOC = %f %% (%02X, %02X)", data->fgauge.soc, rData[0], rData[1]);
  }
}

/*************************************************************************
 * CRATE
 *************************************************************************/
void max1704x_get_crate(COT_DATA *data) {
	uint8_t rData[2];
	uint8_t wData[1] = {MAX1704X_CRATE_ADDR};

	HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 1, 100);
	HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(&max1704x_i2c, MAX1704X_I2C_ADDR, rData, 2, 100);
	if(hal_status != HAL_OK) {
//		L_ERROR("ERROR: Cannot read CRATE from MAX17048 Fuel Gauge sensor %d", hal_status);
		data->fgauge.crate = 0;
	} else {
    data->fgauge.crate = (int16_t)(((uint16_t)rData[0]<<8) + (uint16_t)rData[1]) * 0.208;
//    L_DEBUG("CRATE = %f %%/hr (%02X, %02X)", data->fgauge.crate, rData[0], rData[1]);
  }
}

/*************************************************************************
 * MODE
 *************************************************************************/
uint16_t max1704x_get_mode(void) {
  uint8_t rData[2];
  uint8_t wData[1] = {MAX1704X_MODE_ADDR};
  uint16_t mode = 0x0000;

	HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 1, 100);
	HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(&max1704x_i2c, MAX1704X_I2C_ADDR, rData, 2, 100);
	if(hal_status != HAL_OK) {
//		L_ERROR("ERROR: Cannot read MODE from MAX17048 Fuel Gauge sensor %d", hal_status);
		mode = 0;
	} else {
    mode = ((uint16_t)rData[0]<<8) + (uint16_t)rData[1];
//    L_DEBUG("MODE = %04X (%02X, %02X)", mode, rData[0], rData[1]);
  }
  return mode;
}

bool max1704x_set_mode_ensleep(bool enSleep) {
  uint16_t mode = (max1704x_get_mode())&0xFF00;
  mode &= ~0x2000; // clear enSleep bit
  mode |= (enSleep<<13); // set enSleep bit
	uint8_t wData[3] = {MAX1704X_MODE_ADDR, (uint8_t)(mode>>8), 0x00};

//  L_DEBUG("MODE: 0x%02X%02X", wData[1], wData[2]);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to set MODE.EnSleep to %04X", mode);
		return false;
	}
	return true;
}

/*************************************************************************
 * HIBRT
 *************************************************************************/
bool max1704x_hibernate(bool hib) {
  uint8_t thres = (hib)? 0xFF:0x00;
  uint8_t wData[3] = {MAX1704X_HIBRT_ADDR, thres, thres};

  if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
    L_ERROR("ERROR: Failed to %s hibernate mode.", (hib)? "enter":"exit");
		return false;
	}
	return true;

}

/*************************************************************************
 * CONFIG
 *************************************************************************/
uint16_t max1704x_get_config(void) {
  uint8_t rData[2];
  uint8_t wData[1] = {MAX1704X_CONFIG_ADDR};
  uint16_t config = 0x0000;

	HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 1, 100);
	HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(&max1704x_i2c, MAX1704X_I2C_ADDR, rData, 2, 100);
	if(hal_status != HAL_OK) {
//		L_ERROR("ERROR: Cannot read CONFIG from MAX17048 Fuel Gauge sensor %d", hal_status);
		config = 0;
	} else {
    config = ((uint16_t)rData[0]<<8) + (uint16_t)rData[1];
//    L_DEBUG("CONFIG = %04X (%02X, %02X)", config, rData[0], rData[1]);
  }
  return config;
}

// for best performance, measure battery temperature at least once per minute
// Note: thermistor was temporarily replaced by 10k Ohm resistor, use temperature from PTH sensor instead
bool max1704x_set_rcomp(COT_DATA *data) {
  uint8_t rcomp = 0;
  uint8_t rcomp0 = 0x97;
  uint16_t config = (max1704x_get_config())&0x00FF;
	uint8_t wData[3] = {MAX1704X_CONFIG_ADDR, rcomp, (uint8_t)(config&0x00FF)};
  float temp = data->pth.temperature;

  if(temp > 20) {
    rcomp = rcomp0 + (temp - 20) * -0.5;
  } else {
    rcomp = rcomp0 + (temp - 20) * -5.0;
  }
  wData[1] = rcomp;

//  L_DEBUG("CONFIG: 0x%02X%02X", wData[1], wData[2]);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to set CONFIG.RCOMP to %04X", config);
		return false;
	}
	return true;
}

bool max1704x_set_config_sleep(bool sleep) {
  uint16_t config = max1704x_get_config();
  config &= ~0x0080; // clear sleep bit
  config |= (sleep<<7); // set sleep bit
	uint8_t wData[3] = {MAX1704X_CONFIG_ADDR, (uint8_t)(config>>8), (uint8_t)(config&0x00FF)};

//  L_DEBUG("CONFIG: 0x%02X%02X", wData[1], wData[2]);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to set CONFIG.SLEEP to %04X", config);
		return false;
	}
	return true;
}

bool max1704x_set_config_alsc(bool alsc) {
  uint16_t config = max1704x_get_config();
  config &= ~0x0040; // clear alsc bit
  config |= (alsc<<6); // set alsc bit
	uint8_t wData[3] = {MAX1704X_CONFIG_ADDR, (uint8_t)(config>>8), (uint8_t)(config&0x00FF)};

//  L_DEBUG("CONFIG: 0x%02X%02X", wData[1], wData[2]);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to set CONFIG.ALSC to %04X", config);
		return false;
	}
	return true;
}

bool max1704x_set_config_athd(uint8_t athd) {
  uint16_t config = max1704x_get_config();
  config &= ~0x001F; // clear athd bits
  config |= athd&0x1F; // set athd bits
	uint8_t wData[3] = {MAX1704X_CONFIG_ADDR, (uint8_t)(config>>8), (uint8_t)(config&0x00FF)};

//  L_DEBUG("CONFIG: 0x%02X%02X", wData[1], wData[2]);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to set CONFIG.ATHD to %04X", config);
		return false;
	}
	return true;
}

#if 0
uint8_t max1704x_get_config_athd_percentage(void) {
	uint16_t config;
	uint8_t percent = 0;

	config = max1704x_get_config();
	percent = 32 - ((uint8_t)config & 0x1F);
	return percent;
}
#endif

bool max1704x_clear_config_alrt(void) {
  uint16_t config = max1704x_get_config();
  config &= ~0x0020; // clear alrt bit
	uint8_t wData[3] = {MAX1704X_CONFIG_ADDR, (uint8_t)(config>>8), (uint8_t)(config&0x00FF)};

//  L_DEBUG("CONFIG: 0x%02X%02X", wData[1], wData[2]);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to clear CONFIG.ALRT to %04X", config);
		return false;
	}
	return true;
}

/*************************************************************************
 * VALRT (Read/Write)
 *************************************************************************/
uint16_t max1704x_get_vlrt(void) {
  uint8_t rData[2];
  uint8_t wData[1] = {MAX1704X_VALRT_ADDR};
  uint16_t vlrt = 0x0000;

	HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 1, 100);
	HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(&max1704x_i2c, MAX1704X_I2C_ADDR, rData, 2, 100);
	if(hal_status != HAL_OK) {
//		L_ERROR("ERROR: Cannot read VALRT from MAX17048 Fuel Gauge sensor %d", hal_status);
		vlrt = 0;
	} else {
    vlrt = ((uint16_t)rData[0]<<8) + (uint16_t)rData[1];
//    L_DEBUG("VALRT = %04X (%02X, %02X)", vlrt, rData[0], rData[1]);
  }
  return vlrt;
}

bool max1704x_set_valrt_max(uint8_t max) {
  uint16_t valrt = max1704x_get_vlrt();
  valrt &= ~0x00FF; // clear max bits
  valrt |= (uint16_t)max; // set max bits
	uint8_t wData[3] = {MAX1704X_VALRT_ADDR, (uint8_t)(valrt>>8), (uint8_t)(valrt&0x00FF)};

//  L_DEBUG("VALRT: 0x%02X%02X", wData[1], wData[2]);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to set VALRT.MAX to %04X", valrt);
		return false;
	}
	return true;
}

bool max1704x_set_valrt_min(uint8_t min) {
  uint16_t valrt = max1704x_get_vlrt();
  valrt &= ~0xFF00; // clear min bits
  valrt |= (((uint16_t)min)<<8); // set min bits
	uint8_t wData[3] = {MAX1704X_VALRT_ADDR, (uint8_t)(valrt>>8), (uint8_t)(valrt&0x00FF)};

//  L_DEBUG("VALRT: 0x%02X%02X", wData[1], wData[2]);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to set VALRT.MAX to %04X", valrt);
		return false;
	}
	return true;
}

/*************************************************************************
 * STATUS
 *************************************************************************/
uint16_t max1704x_get_status(void) {
  uint8_t rData[2];
  uint8_t wData[1] = {MAX1704X_STATUS_ADDR};
  uint16_t status = 0x0000;

	HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 1, 100);
	HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(&max1704x_i2c, MAX1704X_I2C_ADDR, rData, 2, 100);
	if(hal_status != HAL_OK) {
//		L_ERROR("ERROR: Cannot read STATUS from MAX17048 Fuel Gauge sensor %d", hal_status);
		status = 0;
	} else {
    status = ((uint16_t)rData[0]<<8) + (uint16_t)rData[1];
//    L_DEBUG("STATUS = %04X (%02X, %02X)", status, rData[0], rData[1]);
  }
  return status;
}

// just for clearing alerts
bool max1704x_clear_status_alerts(uint16_t alerts) {
  uint16_t status = max1704x_get_status();
  status &= ~(alerts&0x3F00);

	uint8_t wData[3] = {MAX1704X_STATUS_ADDR, (uint8_t)(status>>8), 0x00};

//  L_DEBUG("STATUS: 0x%02X%02X", (uint8_t)(status>>8), (uint8_t)status);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to set STATUS to %04X", status);
		return false;
	}
	return true;
}

bool max1704x_set_status_envr(bool envr) {
  uint16_t status = max1704x_get_status();
  status &= ~0x4000; // clear envr bit
  status |= (envr<<14); // set envr bit
	uint8_t wData[3] = {MAX1704X_STATUS_ADDR, (uint8_t)(status>>8), 0x00};

//  L_DEBUG("STATUS: 0x%02X%02X", wData[1], wData[2]);

	if(HAL_I2C_Master_Transmit(&max1704x_i2c, MAX1704X_I2C_ADDR, wData, 3, 10) != HAL_OK) {
//		L_ERROR("Error: Failed to set STATUS.EnVR to %04X", status);
		return false;
	}
	return true;
}
