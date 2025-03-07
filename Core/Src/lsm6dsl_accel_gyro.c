#include "lsm6dsl_accel_gyro.h"
// #include "cmsis_os2.h"
#include "logger.h"
#include <math.h>
#include "daemon.h"

/* Note: On micromote (I2C), CS is connected to 3V3DC enabling I2C mode
 *       On pico (SPI), CS is connected to SPI_NSS
 * This sensor is compatible with SPI mode 0 and 3
 */
#if defined(STM32_UMOTE2)
  #include "stm32h7xx_hal.h"
	extern I2C_HandleTypeDef hi2c1;
	#define lsm6dsl_i2c hi2c1
#elif defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
	extern SPI_HandleTypeDef hspi5;
	#define lsm6dsl_spi hspi5
  #define LSM6DSL_NSS_PORT  GPIOK
  #define LSM6DSL_NSS_PIN   GPIO_PIN_1
#endif

#define LSM6DSL_ODR_XL          LSM6DSL_ODR_52HZ_XL
#define LSM6DSL_ODR_XL_TIME_MS  LSM6DSL_ODR_52HZ_XL_TIME_MS
#define LSM6DSL_FS_XL			      LSM6DSL_FS_2G_XL
#define LSM6DSL_ODR_G			      LSM6DSL_ODR_12_5HZ_G
#define LSM6DSL_FS_G			      LSM6DSL_FS_250DPS_G

extern IC3Daemon ic3;

float sensitivity_g = 0;
float sensitivity_xl = 0;
CALIB_TypeDef offset = {0};

// odr/fs values
float odr_xl[14] = {0, 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660, 0, 0, 1.6};
float fs_xl[4] = {2, 16, 4, 8};

/**************************************************
 * Initialization
 **************************************************/
bool lsm6dsl_init(void)
{
	uint8_t data;
	uint8_t deviceID;
	
	// 15ms for boot procedure to load
	osDelay(15);
	
	// WHO_AM_I (0x0F)
	deviceID = lsm6dsl_get_who_am_i();
//	printf("LSM6DSL Device ID = %02X\r\n", deviceID);
	if(deviceID != LSM6DSL_WHO_AMI_I) {
//		printf("Incorrect LSM6DSL Device ID [%02X], retrying\r\n", deviceID);
    
    // try reading WHO_AM_I again
    osDelay(10);
    deviceID = lsm6dsl_get_who_am_i();
    if(deviceID != LSM6DSL_WHO_AMI_I) {
#if defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
//      printf("Incorrect LSM6DSL Device ID [%02X], changing SPI mode\r\n", deviceID);
      
      // try a different SPI mode
      HAL_SPI_DeInit(&hspi5);
      osDelay(10);
      hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
      hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
      if (HAL_SPI_Init(&hspi5) != HAL_OK) {
        Error_Handler();
      }
      deviceID = lsm6dsl_get_who_am_i();
      if(deviceID != LSM6DSL_WHO_AMI_I) {
#endif
        printf("Incorrect LSM6DSL Device ID [%02X], SPI5 failed to initialize\r\n", deviceID);
        return false;

#if defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
      }
#endif
    }
	}
  
  data = LSM6DSL_ODR_XL << 4;
	lsm6dsl_write(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	data = 0x10; // Gyrcosope: ODR=12.5 Hz, FS=250dps
	lsm6dsl_write(LSM6DSL_CTRL2_G_ADDR, &data, 1);
	data = 0x44; // BDU=update outut after read; IF_INC=autoincrement during multi-byte access
	lsm6dsl_write(LSM6DSL_CTRL3_C_ADDR, &data, 1);
	data = 0x02; // SPI only
	lsm6dsl_write(LSM6DSL_CTRL4_C_ADDR, &data, 1);
	data = 0x00;
	lsm6dsl_write(LSM6DSL_CTRL5_C_ADDR, &data, 1);
  // XL_HM_MODE = 1 = high-performance disabled
  // USR_OFF_W = 0 = weight of XLR user offset set to 2^-10 g/LSB
  data = 0x10;
	lsm6dsl_write(LSM6DSL_CTRL6_C_ADDR, &data, 1);
  data = 0x00;
	lsm6dsl_write(LSM6DSL_CTRL7_G_ADDR, &data, 1);
	lsm6dsl_write(LSM6DSL_CTRL8_XL_ADDR, &data, 1);
	lsm6dsl_write(LSM6DSL_CTRL9_XL_ADDR, &data, 1);
	lsm6dsl_write(LSM6DSL_CTRL10_C_ADDR, &data, 1);

	lsm6dsl_set_sensitivity_g();
	lsm6dsl_set_sensitivity_xl();

  // Set interrupts
  if(ic3.xlg_int_enabled) {
    printf("Accelerometer/Gyroscope interrupt enabled. Setting configurations for interrupt...\r\n");
    
    // Configure GPIO for external interrupt
    __HAL_RCC_GPIOI_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
#if defined(STM32_UMOTE2)
    GPIO_InitStruct.Pin = LSM6DSL_INT_PIN;
#elif defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
    GPIO_InitStruct.Pin = LSM6DSL_INT_PIN1|LSM6DSL_INT_PIN2;
#endif
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LSM6DSL_INT_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(LSM6DSL_EXTI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(LSM6DSL_EXTI_IRQn);
    
    // enable activity/inactivity recognition
		lsm6dsl_inact_recog_int(ic3.xlg_wake_dur, ic3.xlg_wake_thres);
    
  }
  
  // get calibration offsets
//  offset = lsm6dsl_get_offset_xl_g();
  
	printf("Accelerometer/Gyroscope initialization completed\r\n");
	return true;
}

/*****************************************************************
 * WHO_AM_I
 *****************************************************************/
uint8_t lsm6dsl_get_who_am_i(void)
{
	uint8_t rData;
//	printf("&rData = %p\r\n",&rData);
	lsm6dsl_read(LSM6DSL_WHO_AM_I_ADDR, &rData, 1);
//	printf("rData = %X\r\n", rData);
	return rData;
}
/*****************************************************************
 * STATUS Register
 *****************************************************************/
uint8_t lsm6dsl_get_status_reg(void)
{
	uint8_t rData;
	lsm6dsl_read(LSM6DSL_STATUS_REG_ADDR, &rData, 1);
//	L_DEBUG("STATUS REG[%d] = %02X", LSM6DSL_STATUS_REG_ADDR, rData);
	return rData;
}

/*****************************************************************
 * Get both acceleration and angular velocity
 *****************************************************************/
bool lsm6dsl_get_out_xl_g(COT_DATA *data)
{
  lsm6dsl_get_out_g(data);
  lsm6dsl_get_out_xl(data);
  
  return true;
//  float temperature = lsm6dsl_get_out_temp();
//  
//  return temperature;
}

/*****************************************************************
 * Temperature
 *****************************************************************/
float lsm6dsl_get_out_temp(void)
{
  
  if (!(lsm6dsl_get_status_reg()&0x04)) {
//		L_DEBUG("Temperature old data");
		return false;
	}
    
	uint8_t rData[2];
	lsm6dsl_read(LSM6DSL_OUT_TEMP_L_ADDR, rData, 2);
	int16_t temp_raw = (uint16_t)(rData[1]<<8) + (uint16_t)rData[0];
	
  // temperature sensitivity 256 LSB/degC
  // zero-level corresponds to 25 degC
  float temperature = (float)temp_raw / 256.0f + 25; 
  
	L_DEBUG("xlg temperature = %d (%04X)\r\n", temperature, temp_raw);
  
  return temperature;
}

/*****************************************************************
 * Angular Velocity (Gyroscope)
 *****************************************************************/
bool lsm6dsl_get_out_g(COT_DATA *data)
{
	uint8_t rData[6];
	int16_t pData[3] = {0};
	
	// Read Status
	// If STATUS_REG[GDA] = 0 (old data), then do not update
	if (!((lsm6dsl_get_status_reg()&0x02)>>1)) {
    // inactivity will cause gyroscope to power-down if set
//			L_DEBUG("Gyroscope in power-down mode");
    data->xlgyr.gx = 0;
    data->xlgyr.gy = 0;
    data->xlgyr.gz = 0;
		return false;
	}
	// Read OUT(axis)_L/H_XL data
	lsm6dsl_read(LSM6DSL_OUTX_L_G_ADDR, rData, 6);
	pData[2] = (uint16_t)(rData[1]<<8) + (uint16_t)rData[0]; // x
	pData[1] = (uint16_t)(rData[3]<<8) + (uint16_t)rData[2]; // y
	pData[0] = (uint16_t)(rData[5]<<8) + (uint16_t)rData[4]; // z
	
	data->xlgyr.gx = (pData[2] * sensitivity_g) - offset.gx;
	data->xlgyr.gy = (pData[1] * sensitivity_g) - offset.gy;
	data->xlgyr.gz = (pData[0] * sensitivity_g) - offset.gz;
//  L_DEBUG("gyro = %.2f, %.2f, %.2f", data->xlgyr.gx, data->xlgyr.gy, data->xlgyr.gz);
  
	return true;
}

/*****************************************************************
 * Acceleration (Accelerometer)
 *****************************************************************/
bool lsm6dsl_get_out_xl(COT_DATA *data)
{
	uint8_t rData[6];
	int16_t pData[3] = {0};
	
	// Read Status
	// If STATUS_REG[XLDA] = 0, then do not update
	if (!(lsm6dsl_get_status_reg()&0x01)) {
//		L_DEBUG("Accelerometer old data");
		return false;
	}
	
	// Read OUT(axis)_L/H_G data
	lsm6dsl_read(LSM6DSL_OUTX_L_XL_ADDR, rData, 6);
	pData[2] = (uint16_t)(rData[1]<<8) + (uint16_t)rData[0]; // x
	pData[1] = (uint16_t)(rData[3]<<8) + (uint16_t)rData[2]; // y
	pData[0] = (uint16_t)(rData[5]<<8) + (uint16_t)rData[4]; // z
	
	data->xlgyr.xlx = (pData[2] * sensitivity_xl) - offset.xlx;
	data->xlgyr.xly = (pData[1] * sensitivity_xl) - offset.xly;
	data->xlgyr.xlz = (pData[0] * sensitivity_xl) - offset.xlz;
//	L_DEBUG("accel = %.2f, %.2f, %.2f", data->xlgyr.xlx, data->xlgyr.xly, data->xlgyr.xlz);
  
	return true;
}

/*****************************************************************
 * Sensitivity
 *****************************************************************/
void lsm6dsl_set_sensitivity_g(void)
{
	switch(LSM6DSL_FS_G) {
		case LSM6DSL_FS_250DPS_G:  // 250 dps
			sensitivity_g = LSM6DSL_SENSITIVITY_FS_250DPS_G;
			break;
		case LSM6DSL_FS_125DPS_G:  // 125 dps
			sensitivity_g = LSM6DSL_SENSITIVITY_FS_125DPS_G;
			break;
		case LSM6DSL_FS_500DPS_G:	 // 500 dps
			sensitivity_g = LSM6DSL_SENSITIVITY_FS_500DPS_G;
			break;
		case LSM6DSL_FS_1000DPS_G: // 1000 dps
			sensitivity_g = LSM6DSL_SENSITIVITY_FS_1000DPS_G;
			break;
		case LSM6DSL_FS_2000DPS_G: // 2000 dps
			sensitivity_g = LSM6DSL_SENSITIVITY_FS_2000DPS_G;
			break;
		default: // default 00 - 250 dps
			sensitivity_g = LSM6DSL_SENSITIVITY_FS_250DPS_G;
			break;
	}
}

void lsm6dsl_set_sensitivity_xl(void)
{
	switch(LSM6DSL_FS_XL) {
		case LSM6DSL_FS_2G_XL:  // 2g
			sensitivity_xl = LSM6DSL_SENSITIVITY_FS_2G_XL;
			break;
		case LSM6DSL_FS_16G_XL: // 16g
			sensitivity_xl = LSM6DSL_SENSITIVITY_FS_16G_XL;
			break;
		case LSM6DSL_FS_4G_XL:  // 4g
			sensitivity_xl = LSM6DSL_SENSITIVITY_FS_4G_XL;
			break;
		case LSM6DSL_FS_8G_XL:  // 8g
			sensitivity_xl = LSM6DSL_SENSITIVITY_FS_8G_XL;
			break;
		default:								// default 00 - 2g
			sensitivity_xl = LSM6DSL_SENSITIVITY_FS_2G_XL;
			break;
	}
}

/*****************************************************************
 * Offset Correction to Accelerometer Output Data
 *****************************************************************/
void lsm6dsl_set_offset_xl(CALIB_TypeDef calib)
{
  uint8_t offset[3] = {0};
  
  // 1000 mg * 2^-10 g/LSB = 0.9765625 ~= 1
  offset[0] = abs((int8_t)floor(calib.xlx));
  offset[1] = abs((int8_t)floor(calib.xly));
  offset[2] = abs((int8_t)floor(calib.xlz));
  
	L_DEBUG("Write: Offset = [%d, %d, %d]", offset[0], offset[1], offset[2]);
	lsm6dsl_write(LSM6DSL_X_OFS_USR_ADDR, (uint8_t *)offset, 3);
  
}

#if 0
/*****************************************************************
 * Free-Fall Interrupt
 * Acceleration measured along axes goes to zero
 *****************************************************************/
void lsm6dsl_free_fall_int(uint8_t duration, uint8_t threshold)
{
	uint8_t data;

	L_TRACE("============== Free-Fall Interrupt Configurations ==============");
	
	// Set ODR_XL to 26 Hz
	lsm6dsl_read(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	data = 0x20;
	lsm6dsl_write(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	
	// Enable interrupts and latched interrupt
	lsm6dsl_read(LSM6DSL_TAP_CFG_ADDR, &data, 1);
	data = 0x81;
	lsm6dsl_write(LSM6DSL_TAP_CFG_ADDR, &data, 1);

	//Set free-fall duration and threshold
	lsm6dsl_read(LSM6DSL_WAKE_UP_DUR_ADDR, &data, 1);
	data = (duration & 0x20) << 2;
	lsm6dsl_write(LSM6DSL_WAKE_UP_DUR_ADDR, &data, 1);
	
	lsm6dsl_read(LSM6DSL_FREE_FALL_ADDR, &data, 1);
	data = ((duration & 0x1F) << 3) + threshold;
	lsm6dsl_write(LSM6DSL_FREE_FALL_ADDR, &data, 1);

	// Drive free-fall interrupt to INT1
	// Drive tilt interrupt to INT1
	lsm6dsl_read(LSM6DSL_MD1_CFG_ADDR, &data, 1);
	data = 0x10;
	lsm6dsl_write(LSM6DSL_MD1_CFG_ADDR, &data, 1);
}
#endif

#if 0
/*****************************************************************
 * 6D Orientation Interrupt
 *****************************************************************/
void lsm6dsl_6d_orientation_int(void)
{
	L_TRACE("============ 6D Orientation Interrupt Configurations ===========");
}
#endif

#if 0
/*****************************************************************
 * Wake-Up Interrupt
 *****************************************************************/
void lsm6dsl_wake_up_int(uint8_t duration, uint8_t threshold)
{
	uint8_t data;
	uint8_t delay;
	
	L_TRACE("Wake-Up Interrupt Activated");
	
	// Set ODR_XL to 26 Hz
//	lsm6dsl_read(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	data = 0x20;
	lsm6dsl_write(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	
	// Enable interrupts, apply slope filter, and latched disabled
//	lsm6dsl_read(LSM6DSL_TAP_CFG_ADDR, &data, 1);
	data = 0x90;
	lsm6dsl_write(LSM6DSL_TAP_CFG_ADDR, &data, 1);

	// Wait for a certain amount of time to ignore unneccessary wake-up
	delay = (1/LSM6DSL_ODR_26HZ_G)*1000 + 10;
	osDelay(delay);

	// Set wake-up duration
//	lsm6dsl_read(LSM6DSL_WAKE_UP_DUR_ADDR, &data, 1);
	data = (duration&0x3)<<5;
	lsm6dsl_write(LSM6DSL_WAKE_UP_DUR_ADDR, &data, 1);
	
	// Set wake-up threshold
//	lsm6dsl_read(LSM6DSL_WAKE_UP_THS_ADDR, &data, 1);
	data = threshold&0x3F;
	lsm6dsl_write(LSM6DSL_WAKE_UP_THS_ADDR, &data, 1);

	// Drive wake-up interrupt to INT1
//	lsm6dsl_read(LSM6DSL_MD1_CFG_ADDR, &data, 1);
	data = 0x20;
	lsm6dsl_write(LSM6DSL_MD1_CFG_ADDR, &data, 1);
}
#endif

/*****************************************************************
 * Inactivity/Activity Recognition Interrupt
 *****************************************************************/
void lsm6dsl_inact_recog_int(uint8_t duration, uint8_t threshold)
{
	uint8_t data = 0, rData = 0;
	float durValue, thresValue;
	
	durValue = duration * 512 / odr_xl[LSM6DSL_ODR_XL];
	thresValue = threshold * fs_xl[LSM6DSL_FS_XL] / 64;
	
//	L_TRACE("Inactivity/Activity Recognition Interrupt Activated (thres = %.4f, dur = %.2f)", thresValue, durValue);

	// Set wake_up duration (in lsm6dsl_wake_up_int)
//	lsm6dsl_read(LSM6DSL_WAKE_UP_DUR_ADDR, &data, 1);
	data = duration&0x0F;
	lsm6dsl_write(LSM6DSL_WAKE_UP_DUR_ADDR, &data, 1);

	// Set wake_up threshold (in lsm6dsl_wake_up_int)
//	lsm6dsl_read(LSM6DSL_WAKE_UP_THS_ADDR, &data, 1);
	data = threshold&0x3F;
	lsm6dsl_write(LSM6DSL_WAKE_UP_THS_ADDR, &data, 1);
	
	// Enable interrupts, slope filter, latch interrupts, and set gyroscope option to power-down
	// Set gyro to not change, sleep, power-down with 0xA1, 0xC1, 0xE1, respectively
//	lsm6dsl_read(LSM6DSL_TAP_CFG_ADDR, &data, 1);
	data = 0xE1;
	lsm6dsl_write(LSM6DSL_TAP_CFG_ADDR, &data, 1);
//  osDelay(1);
  lsm6dsl_read(LSM6DSL_TAP_CFG_ADDR, &rData, 1);
  if(rData != 0xE1) {
    data = 0xE1;
    lsm6dsl_write(LSM6DSL_TAP_CFG_ADDR, &data, 1);
  }
	
	// Drive inactivity/activity interrupt to INT2
//	lsm6dsl_read(LSM6DSL_MD2_CFG_ADDR, &data, 1);
	data = 0x80;
	lsm6dsl_write(LSM6DSL_MD2_CFG_ADDR, &data, 1);
//  osDelay(1);
  lsm6dsl_read(LSM6DSL_MD2_CFG_ADDR, &rData, 1);
  if(rData != 0x80) {
    data = 0x80;
    lsm6dsl_write(LSM6DSL_MD2_CFG_ADDR, &data, 1);
  }
    
  // verify interrupts are enabled
}
#if 0
/*****************************************************************
 * Relative Tilt Interrupt
 * Detect tilt event if object is tilted more than 35 degrees from
 * start position (or position of last interrupt detection) for 
 * period of 2 seconds
 *****************************************************************/
void lsm6dsl_tilt_int(void)
{
	uint8_t data;

	L_TRACE("============ Relative Tilt Interrupt Configurations ============");
	
	// Set ODR_XL to 26 Hz
	lsm6dsl_read(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	data = 0x20;
	lsm6dsl_write(LSM6DSL_CTRL1_XL_ADDR, &data, 1);

	// Enable embedded functions and tilt detection
	lsm6dsl_read(LSM6DSL_CTRL10_C_ADDR, &data, 1);
	data = 0x0C;
	lsm6dsl_write(LSM6DSL_CTRL10_C_ADDR, &data, 1);
	
	// Drive tilt interrupt to INT1
	lsm6dsl_read(LSM6DSL_MD1_CFG_ADDR, &data, 1);
	data = 0x02;
	lsm6dsl_write(LSM6DSL_MD1_CFG_ADDR, &data, 1);
}
#endif

#if 0
/*****************************************************************
 * Abosolut Wrist Tilt Interrupt
 * Detects if angle between selectable accelerometer semi-axis and
 * horizontal plane becomes higher than specific threshold
 *****************************************************************/

uint8_t lsm6dsl_awt_int(uint8_t latency, uint8_t threshold, uint8_t mask)
{
	uint8_t data;
	
	L_TRACE("================= AWT Interrupt Configurations =================");
	
	// Set ODR_XL to 26 Hz
	lsm6dsl_read(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	data = 0x20;
	lsm6dsl_write(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	
	// Enable embedded functions and latched interrupts
	lsm6dsl_read(LSM6DSL_CTRL10_C_ADDR, &data, 1);
	data = 0x04;
	lsm6dsl_write(LSM6DSL_CTRL10_C_ADDR, &data, 1);

	osDelay(50);
	
	// Disable embedded functions
	lsm6dsl_read(LSM6DSL_CTRL10_C_ADDR, &data, 1);
	data = 0x00;
	lsm6dsl_write(LSM6DSL_CTRL10_C_ADDR, &data, 1);

	// Enable embedded functions access for bank B
	lsm6dsl_read(LSM6DSL_FUNC_CFG_ACCESS_ADDR, &data, 1);
	data = 0xA0;
	lsm6dsl_write(LSM6DSL_FUNC_CFG_ACCESS_ADDR, &data, 1);

	// Set latency
	lsm6dsl_read(LSM6DSL_A_WRIST_TILT_LAT_ADDR, &data, 1);
	data = latency;
	lsm6dsl_write(LSM6DSL_A_WRIST_TILT_LAT_ADDR, &data, 1);
	
	// Set threshold
	lsm6dsl_read(LSM6DSL_A_WRIST_TILT_THS_ADDR, &data, 1);
	data = threshold;
	lsm6dsl_write(LSM6DSL_A_WRIST_TILT_THS_ADDR, &data, 1);
	
	// Set mask
	lsm6dsl_read(LSM6DSL_A_WRIST_TILT_MASK_ADDR, &data, 1);
	data = mask;
	lsm6dsl_write(LSM6DSL_A_WRIST_TILT_MASK_ADDR, &data, 1);
	
	// Disable embedded functions acces for bank B
	lsm6dsl_read(LSM6DSL_FUNC_CFG_ACCESS_ADDR, &data, 1);
	data = 0x00;
	lsm6dsl_write(LSM6DSL_FUNC_CFG_ACCESS_ADDR, &data, 1);

	// Enable embedded funtions and AWT detecion
	lsm6dsl_read(LSM6DSL_CTRL10_C_ADDR, &data, 1);
	data = 0x84;
	lsm6dsl_write(LSM6DSL_CTRL10_C_ADDR, &data, 1);

	// Drive AWT interrupt to INT2
	lsm6dsl_read(LSM6DSL_DRDY_PULSE_CFG_G_ADDR, &data, 1);
	data = 0x01;
	lsm6dsl_write(LSM6DSL_DRDY_PULSE_CFG_G_ADDR, &data, 1);
}
#endif

#if 0
/*****************************************************************
 * Significant Motion Detection Interrupt
 *****************************************************************/
void lsm5dsl_sig_motion_int(uint8_t thres)
{
	uint8_t data;
	
	L_TRACE("========= Significant Motion Interrupt Configurations ==========");
	
	// Enable embedded functions access for bank A
	lsm6dsl_read(LSM6DSL_FUNC_CFG_ACCESS_ADDR, &data, 1);
	data = 0x80;
	lsm6dsl_write(LSM6DSL_FUNC_CFG_ACCESS_ADDR, &data, 1);
	
	// Set significant mostion threshold
	lsm6dsl_read(LSM6DSL_SM_THS_ADDR, &data, 1);
	data = thres;
	lsm6dsl_write(LSM6DSL_SM_THS_ADDR, &data, 1);
	
	// Disable embedded functions access for bank A
	lsm6dsl_read(LSM6DSL_FUNC_CFG_ACCESS_ADDR, &data, 1);
	data = 0x00;
	lsm6dsl_write(LSM6DSL_FUNC_CFG_ACCESS_ADDR, &data, 1);

	// Set ODR_XL to 26 Hz
	lsm6dsl_read(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	data = 0x20;
	lsm6dsl_write(LSM6DSL_CTRL1_XL_ADDR, &data, 1);

	// Enable embedded functions and significant motion detection
	lsm6dsl_read(LSM6DSL_CTRL10_C_ADDR, &data, 1);
	data = 0x05;
	lsm6dsl_write(LSM6DSL_CTRL10_C_ADDR, &data, 1);
	
	// Drive significant motion interrupt to INT1
	lsm6dsl_read(LSM6DSL_INT1_CTRL_ADDR, &data, 1);
	data = 0x40;
	lsm6dsl_write(LSM6DSL_INT1_CTRL_ADDR, &data, 1);
}
#endif

/*****************************************************************
 * Disable/Enable Interrupts
 *****************************************************************/
void lsm6dsl_interrupt_en(bool enable)
{
	uint8_t data;
	
	lsm6dsl_read(LSM6DSL_TAP_CFG_ADDR, &data, 1);
	if(enable)
		data |= 0x80; //enable interrupt bits
	else
		data &= ~(0x00); //clears interrupt bits
	lsm6dsl_write(LSM6DSL_TAP_CFG_ADDR, &data, 1);
}

/*****************************************************************
 * Set Gyroscope Function During Inactivity
 *****************************************************************/
void lsm6dsl_gyro_inact(uint8_t gyro_func)
{
	uint8_t data;
	
	lsm6dsl_read(LSM6DSL_TAP_CFG_ADDR, &data, 1);
	data |= (gyro_func & 0x03) << 5;
	lsm6dsl_write(LSM6DSL_TAP_CFG_ADDR, &data, 1);
}

/*****************************************************************
 * Set to Power-Down Mode for Gyroscope
 *****************************************************************/
void lsm6dsl_set_powerdown_g(void)
{
	uint8_t data;
	
	// Disable high-performance
	lsm6dsl_read(LSM6DSL_CTRL7_G_ADDR, &data, 1);
	data |= 0x80;
	lsm6dsl_write(LSM6DSL_CTRL7_G_ADDR, &data, 1);
	
	// Clear ODR_G[3:0] to 0 for power-down mode
	lsm6dsl_read(LSM6DSL_CTRL2_G_ADDR, &data, 1);
	data &= ~(0xF0);
	lsm6dsl_write(LSM6DSL_CTRL2_G_ADDR, &data, 1);
	
	osDelay(1);
}

/*****************************************************************
 * Set to Power-Down Mode for Accelerometer
 *****************************************************************/
void lsm6dsl_set_powerdown_xl(void)
{
	uint8_t data;
	
	// Disable high-performance
	lsm6dsl_read(LSM6DSL_CTRL6_C_ADDR, &data, 1);
	data |= 0x10;
	lsm6dsl_write(LSM6DSL_CTRL6_C_ADDR, &data, 1);
	
	// Clear ODR_G[3:0] to 0 for power-down mode
	lsm6dsl_read(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	data &= ~(0xF0);
	lsm6dsl_write(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	
	osDelay(1);
}

/*****************************************************************
 * Gyroscope Sleep Mode
 *****************************************************************/
void lsm6dsl_gyro_sleep_mode(bool sleep)
{
	uint8_t data;
	
	// Set sleep
	lsm6dsl_read(LSM6DSL_CTRL4_C_ADDR, &data, 1);
	data &= ~(0x40);
  data |= sleep<<6;
	lsm6dsl_write(LSM6DSL_CTRL4_C_ADDR, &data, 1);
	
	osDelay(70); // longest time to go into sleep mode (power-down to sleep)
}

/*****************************************************************
 * Software Reset or Reboot
 * Sofware Reset:	Reset default values of control registers
 * Reboot: Used to re-load trimming parameters
 * Note: Do not set SW_RESET and BOOT at the same time
 * reset_boot: 0 for software reset; 1 for reboot
 *****************************************************************/
void lsm6dsl_sw_reset_boot(bool reset_boot)
{
	uint8_t data;
	uint8_t delay;
	
	// Set gyroscope in power-down mode
	// requires 1us (XL in PD) or 300us (XL not in PD) delay for PD
	lsm6dsl_read(LSM6DSL_CTRL2_G_ADDR, &data, 1);
	data &= ~(0xF0);
	lsm6dsl_write(LSM6DSL_CTRL2_G_ADDR, &data, 1);
	// Set accelerometer in high-performance mode
	// requires 1us delay when switching ODR_XL and sample discard
	lsm6dsl_read(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	data = (data & ~(0xF0)) | (LSM6DSL_ODR_416HZ_XL << 4);
	lsm6dsl_write(LSM6DSL_CTRL1_XL_ADDR, &data, 1);
	osDelay(1);
	
	// Set BOOT or SW_RESET bit in CTRL3_C register
	lsm6dsl_read(LSM6DSL_CTRL3_C_ADDR, &data, 1);
	if(reset_boot){
//		L_TRACE("LSM6DSL Reboot");
		data = 0x14;
		delay = 15; // 15ms wait
	} else {
//		L_TRACE("LSM6DSL SW Reset");
		data = 0x05;
		delay = 1; // 50us wait or check CRTL3_C[SW_RESET] if retuned to 0
	}
	
	lsm6dsl_write(LSM6DSL_CTRL3_C_ADDR, &data, 1);
	
	// Wait delayed amount
	osDelay(delay);
}

/*****************************************************************
 * LSM6DSL Callibration
 * If callibration is required, please have accel/gyro laying flat
 *****************************************************************/
CALIB_TypeDef lsm6dsl_get_offset_xl_g(void)
{
  CALIB_TypeDef calib = {0};
  CALIB_TypeDef sum = {0};
  CALIB_TypeDef gravity = {0};
  COT_DATA sample = {0};
  
  // reset user offset to 0 before calibration
  uint8_t offset[3] = {0};
  lsm6dsl_write(LSM6DSL_X_OFS_USR_ADDR, (uint8_t *)offset, 3);
  
  // compute the sum
  int amount = ceil(10000/LSM6DSL_ODR_XL_TIME_MS);
  for(int i = 0; i < amount+10; i++) {
    
    // get samples
    lsm6dsl_get_out_g(&sample);
    lsm6dsl_get_out_xl(&sample);
    
    // first few samples gives weird values
    if(i < 10) {
        continue;
    }  
    // running averages
    sum.xlx = sum.xlx + sample.xlgyr.xlx;
    sum.xly = sum.xly + sample.xlgyr.xly;
    sum.xlz = sum.xlz + sample.xlgyr.xlz;
    sum.gx = sum.gx + sample.xlgyr.gx;
    sum.gy = sum.gy + sample.xlgyr.gy;
    sum.gz = sum.gz + sample.xlgyr.gz;
    
    osDelay(LSM6DSL_ODR_XL_TIME_MS);
  }
  
  // find which axis is point up to find offset without gravity
  lsm6dsl_read(LSM6DSL_D6D_SRC_ADDR, (uint8_t *)offset, 1);
  printf("axis 0x%02X\r\n", offset[0]);
  switch(offset[0] & 0x3F) {
    case 0b00000001: // -X
      gravity.xlx = -1000;
      break;
    case 0b00000010: // +X
      gravity.xlx = 1000;
      break;
    case 0b00000100: // -Y
      gravity.xly = -1000;
      break;
    case 0b00001000: // +Y
      gravity.xly = 1000;
      break;
    case 0b00010000: // -Z
      gravity.xlz = -1000;
      break;
    case 0b00100000: // +Z
      gravity.xlz = 1000;
      break;
    default:
      break;
  }
  
  
  calib.xlx = sum.xlx / amount - gravity.xlx;
  calib.xly = sum.xly / amount - gravity.xly;
  calib.xlz = sum.xlz / amount - gravity.xlz;
  calib.gx = sum.gx / amount - gravity.gx;
  calib.gy = sum.gy / amount - gravity.gy;
  calib.gz = sum.gz / amount - gravity.gz;
  
  // verify offsets from calibration
  L_DEBUG("offsets acclerometer [%.2f, %.2f, %.2f] and gyroscope [%.2f, %.2f, %.2f]", 
          calib.xlx, calib.xly, calib.xlz, calib.gx, calib.gy, calib.gz);
  
  return calib;
}

/*****************************************************************
 * LSM6DSL Read/Write Functions
 * Can either be SPI or I2C, selected by LSM6DSL_I2C_SPI
 * Note: I2C have not been implemented yet
 *****************************************************************/
void lsm6dsl_write(uint8_t reg, uint8_t *pData, uint8_t len)
{
	uint8_t *wData = malloc((len+1)*sizeof(uint8_t));
	memcpy(wData, &reg, sizeof(uint8_t));
	memcpy(wData+sizeof(uint8_t), pData, len*sizeof(uint8_t));
	
#if defined(STM32_UMOTE2) // I2C
	if(HAL_I2C_Master_Transmit(&lsm6dsl_i2c, LSM6DSL_I2C_ADDR, wData, len+1, 10) != HAL_OK) {
    printf("I2C XLG Transmit error...\r\n");
  }
#elif defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B) // SPI
  HAL_GPIO_WritePin(LSM6DSL_NSS_PORT, LSM6DSL_NSS_PIN, GPIO_PIN_RESET); // Pull CS
  if(HAL_SPI_Transmit(&lsm6dsl_spi, wData, len+1, 100) != HAL_OK) {
    printf("SPI XLG Transmit error...\r\n");
  }
  HAL_GPIO_WritePin(LSM6DSL_NSS_PORT, LSM6DSL_NSS_PIN, GPIO_PIN_SET); // De-assert CS
#endif
  free(wData);
}

void lsm6dsl_read(uint8_t reg, uint8_t *pData, uint8_t len)
{

#if defined(STM32_UMOTE2) // I2C
	if(HAL_I2C_Mem_Read(&lsm6dsl_i2c, LSM6DSL_I2C_ADDR, reg, 1, pData, len, 10) != HAL_OK) {
      printf("I2C XLG Receiver error\r\n");
  }
#elif defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B) // SPI
  // bit 8 is R/W where R = 1 and W = 0
  uint8_t wData = 0x80 | reg;
//  printf("wData = %4X\r\n", wData);

  HAL_GPIO_WritePin(LSM6DSL_NSS_PORT, LSM6DSL_NSS_PIN, GPIO_PIN_RESET); // Pull CS
//  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, GPIO_PIN_RESET);
  osDelay(10);
  if(HAL_SPI_Transmit(&lsm6dsl_spi, &wData, 1, 100) != HAL_OK){
	  printf("SPI XLG Transmit error\r\n");
  }
  if(HAL_SPI_Receive(&lsm6dsl_spi, pData, len, 100) != HAL_OK) {
      printf("SPI XLG Receiver error\r\n");
  }
//  printf("pData = %4X\r\n", *pData);
  HAL_GPIO_WritePin(LSM6DSL_NSS_PORT, LSM6DSL_NSS_PIN, GPIO_PIN_SET); // De-assert CS
//  HAL_GPIO_WritePin(GPIOK, GPIO_PIN_1, GPIO_PIN_SET);

#endif
}
