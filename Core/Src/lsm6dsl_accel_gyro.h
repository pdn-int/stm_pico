#ifndef _LSM6DSL_ACCEL_GYRO_H_
#define _LSM6DSL_ACCEL_GYRO_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cotfactory.h"

/********************************************************
 * Interrupt Pin
 ********************************************************/
#if defined(STM32_UMOTE2)
  #define LSM6DSL_INT_PORT  GPIOI
  #define LSM6DSL_INT_PIN   GPIO_PIN_12
  #define LSM6DSL_EXTI_IRQn EXTI15_10_IRQn
#elif defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
  #define LSM6DSL_INT_PORT  GPIOI
  #define LSM6DSL_INT_PIN1  GPIO_PIN_10
  #define LSM6DSL_INT_PIN2  GPIO_PIN_11
  #define LSM6DSL_EXTI_IRQn EXTI15_10_IRQn
#endif
 
/**************************************************
 * WHO_AM_I
 **************************************************/
#define LSM6DSL_WHO_AMI_I		(uint8_t)0x6A

/**************************************************
 * Modes
 **************************************************/
#define LSM6DSL_MODE1 	1	// I2C/SPI serial interface only
#define LSM6DSL_MODE2 	2	// I2C/SPI serial interface and I2C master for external sensor

/**************************************************
 * I2C Slave Address
 * In Micromote, SDO/SA0 is grounded giving 7-bit slave address 0x6A
 * If SDO/SA0 was connected to supply voltage, 7-bit slave address is 0x6B
 **************************************************/
#define LSM6DSL_I2C_ADDR		(uint8_t)0x6A<<1

/**************************************************
 * Register Addresses
 * Note: Register address are 7-bits
 **************************************************/
#define LSM6DSL_FUNC_CFG_ACCESS_ADDR						(uint8_t)0x01
#define LSM6DSL_SENSOR_SYNC_TIME_FRAME_ADDR		(uint8_t)0x04
#define LSM6DSL_SENSOR_SYNC_RES_RATIO_ADDR		(uint8_t)0x05
#define LSM6DSL_FIFO_CTRL1_ADDR								(uint8_t)0x06
#define LSM6DSL_FIFO_CTRL2_ADDR								(uint8_t)0x07
#define LSM6DSL_FIFO_CTRL3_ADDR								(uint8_t)0x08
#define LSM6DSL_FIFO_CTRL4_ADDR								(uint8_t)0x09
#define LSM6DSL_FIFO_CTRL5_ADDR								(uint8_t)0x0A
#define LSM6DSL_DRDY_PULSE_CFG_G_ADDR					(uint8_t)0x0B
#define LSM6DSL_INT1_CTRL_ADDR								(uint8_t)0x0D
#define LSM6DSL_INT2_CTRL_ADDR								(uint8_t)0x0E
#define LSM6DSL_WHO_AM_I_ADDR									(uint8_t)0x0F
#define LSM6DSL_CTRL1_XL_ADDR									(uint8_t)0x10
#define LSM6DSL_CTRL2_G_ADDR									(uint8_t)0x11
#define LSM6DSL_CTRL3_C_ADDR									(uint8_t)0x12
#define LSM6DSL_CTRL4_C_ADDR									(uint8_t)0x13
#define LSM6DSL_CTRL5_C_ADDR									(uint8_t)0x14
#define LSM6DSL_CTRL6_C_ADDR									(uint8_t)0x15
#define LSM6DSL_CTRL7_G_ADDR									(uint8_t)0x16
#define LSM6DSL_CTRL8_XL_ADDR									(uint8_t)0x17
#define LSM6DSL_CTRL9_XL_ADDR									(uint8_t)0x18
#define LSM6DSL_CTRL10_C_ADDR									(uint8_t)0x19
#define LSM6DSL_MASTER_CONFIG_ADDR						(uint8_t)0x1A
#define LSM6DSL_WAKE_UP_SRC_ADDR							(uint8_t)0x1B
#define LSM6DSL_TAP_SRC_ADDR									(uint8_t)0x1C
#define LSM6DSL_D6D_SRC_ADDR									(uint8_t)0x1D
#define LSM6DSL_STATUS_REG_ADDR								(uint8_t)0x1E
#define LSM6DSL_OUT_TEMP_L_ADDR								(uint8_t)0x20
#define LSM6DSL_OUT_TEMP_H_ADDR								(uint8_t)0x21
#define LSM6DSL_OUTX_L_G_ADDR									(uint8_t)0x22
#define LSM6DSL_OUTX_H_G_ADDR									(uint8_t)0x23
#define LSM6DSL_OUTY_L_G_ADDR									(uint8_t)0x24
#define LSM6DSL_OUTY_H_G_ADDR									(uint8_t)0x25
#define LSM6DSL_OUTZ_L_G_ADDR									(uint8_t)0x26
#define LSM6DSL_OUTZ_H_G_ADDR									(uint8_t)0x27
#define LSM6DSL_OUTX_L_XL_ADDR								(uint8_t)0x28
#define LSM6DSL_OUTX_H_XL_ADDR								(uint8_t)0x29
#define LSM6DSL_OUTY_L_XL_ADDR								(uint8_t)0x2A
#define LSM6DSL_OUTY_H_XL_ADDR								(uint8_t)0x2B
#define LSM6DSL_OUTZ_L_XL_ADDR								(uint8_t)0x2C
#define LSM6DSL_OUTZ_H_XL_ADDR								(uint8_t)0x2D
#define LSM6DSL_SENSORHUB1_REG_ADDR						(uint8_t)0x2E
#define LSM6DSL_SENSORHUB2_REG_ADDR						(uint8_t)0x2F
#define LSM6DSL_SENSORHUB3_REG_ADDR						(uint8_t)0x30
#define LSM6DSL_SENSORHUB4_REG_ADDR						(uint8_t)0x31
#define LSM6DSL_SENSORHUB5_REG_ADDR						(uint8_t)0x32
#define LSM6DSL_SENSORHUB6_REG_ADDR						(uint8_t)0x33
#define LSM6DSL_SENSORHUB7_REG_ADDR						(uint8_t)0x34
#define LSM6DSL_SENSORHUB8_REG_ADDR						(uint8_t)0x35
#define LSM6DSL_SENSORHUB9_REG_ADDR						(uint8_t)0x36
#define LSM6DSL_SENSORHUB10_REG_ADDR					(uint8_t)0x37
#define LSM6DSL_SENSORHUB11_REG_ADDR					(uint8_t)0x38
#define LSM6DSL_SENSORHUB12_REG_ADDR					(uint8_t)0x39
#define LSM6DSL_FIFO_STATUS1_ADDR							(uint8_t)0x3A
#define LSM6DSL_FIFO_STATUS2_ADDR							(uint8_t)0x3B
#define LSM6DSL_FIFO_STATUS3_ADDR							(uint8_t)0x3C
#define LSM6DSL_FIFO_STATUS4_ADDR							(uint8_t)0x3D
#define LSM6DSL_FIFO_DATA_OUT_L_ADDR					(uint8_t)0x3E
#define LSM6DSL_FIFO_DATA_OUT_H_ADDR					(uint8_t)0x3F
#define LSM6DSL_TIMESTAMP0_REG_ADDR						(uint8_t)0x40
#define LSM6DSL_TIMESTAMP1_REG_ADDR						(uint8_t)0x41
#define LSM6DSL_TIMESTAMP2_REG_ADDR						(uint8_t)0x42
#define LSM6DSL_STEP_TIMESTAMP_L_ADDR					(uint8_t)0x49
#define LSM6DSL_STEP_TIMESTAMP_h_ADDR					(uint8_t)0x4A
#define LSM6DSL_STEP_COUNTER_L_ADDR						(uint8_t)0x4B
#define LSM6DSL_STEP_COUNTER_H_ADDR						(uint8_t)0x4C
#define LSM6DSL_SENSORHUB13_REG_ADDR					(uint8_t)0x4D
#define LSM6DSL_SENSORHUB14_REG_ADDR					(uint8_t)0x4E
#define LSM6DSL_SENSORHUB15_REG_ADDR					(uint8_t)0x4F
#define LSM6DSL_SENSORHUB16_REG_ADDR					(uint8_t)0x50
#define LSM6DSL_SENSORHUB17_REG_ADDR					(uint8_t)0x51
#define LSM6DSL_SENSORHUB18_REG_ADDR					(uint8_t)0x52
#define LSM6DSL_FUNC_SRC1_ADDR								(uint8_t)0x53
#define LSM6DSL_FUNC_SRC2_ADDR								(uint8_t)0x54
#define LSM6DSL_WRIST_TILT_IA_ADDR						(uint8_t)0x55
#define LSM6DSL_TAP_CFG_ADDR									(uint8_t)0x58
#define LSM6DSL_TAP_THS_6D_ADDR								(uint8_t)0x59
#define LSM6DSL_INT_DUR2_ADDR									(uint8_t)0x5A
#define LSM6DSL_WAKE_UP_THS_ADDR							(uint8_t)0x5B
#define LSM6DSL_WAKE_UP_DUR_ADDR							(uint8_t)0x5C
#define LSM6DSL_FREE_FALL_ADDR								(uint8_t)0x5D
#define LSM6DSL_MD1_CFG_ADDR									(uint8_t)0x5E
#define LSM6DSL_MD2_CFG_ADDR									(uint8_t)0x5F
#define LSM6DSL_MASTER_CMD_CODE_ADDR					(uint8_t)0x60
#define LSM6DSL_SENS_SYNC_SPI_ERROR_CODE_ADDR	(uint8_t)0x61
#define LSM6DSL_OUT_MAG_RAW_X_L_ADDR					(uint8_t)0x66
#define LSM6DSL_OUT_MAG_RAW_X_H_ADDR					(uint8_t)0x67
#define LSM6DSL_OUT_MAG_RAW_Y_L_ADDR					(uint8_t)0x68
#define LSM6DSL_OUT_MAG_RAW_Y_H_ADDR					(uint8_t)0x69
#define LSM6DSL_OUT_MAG_RAW_Z_L_ADDR					(uint8_t)0x6A
#define LSM6DSL_OUT_MAG_RAW_Z_H_ADDR					(uint8_t)0x6B
#define LSM6DSL_X_OFS_USR_ADDR								(uint8_t)0x73
#define LSM6DSL_Y_OFS_USR_ADDR								(uint8_t)0x74
#define LSM6DSL_Z_OFS_USR_ADDR								(uint8_t)0x75

#if 0 // not used
/**************************************************
 * Embedded Functions Register Mapping
 * Note: all modifications on these registers must be performed
 * with device in power-down mode
 **************************************************/
// BANK A
#define LSM6DSL_SLV0_ADD_ADDR											(uint8_t)0x02
#define LSM6DSL_SLV0_SUBADD_ADDR									(uint8_t)0x03
#define LSM6DSL_SLAVE0_CONFIG_ADDR								(uint8_t)0x04
#define LSM6DSL_SLV1_ADD_ADDR											(uint8_t)0x05
#define LSM6DSL_SLV1_SUBADD_ADDR									(uint8_t)0x06
#define LSM6DSL_SLAVE1_CONFIG_ADDR								(uint8_t)0x07
#define LSM6DSL_SLV2_ADD_ADDR											(uint8_t)0x08
#define LSM6DSL_SLV2_SUBADD_ADDR									(uint8_t)0x09
#define LSM6DSL_SLAVE2_CONFIG_ADDR								(uint8_t)0x0A
#define LSM6DSL_SLV3_ADD_ADDR											(uint8_t)0x0B
#define LSM6DSL_SLV3_SUBADD_ADDR									(uint8_t)0x0C
#define LSM6DSL_SLAVE3_CONFIG_ADDR								(uint8_t)0x0D
#define LSM6DSL_DATAWRITE_SRC_MODE_SUB_SLV0_ADDR	(uint8_t)0x0E
#define LSM6DSL_CONFIG_PEDO_THS_MIN_ADDR					(uint8_t)0x0F
#define LSM6DSL_SM_THS_ADDR												(uint8_t)0x13
#define LSM6DSL_PEDO_DEB_REG_ADDR									(uint8_t)0x14
#define LSM6DSL_STEP_COUNT_DELTA_ADDR							(uint8_t)0x15
#define LSM6DSL_MAG_SI_XX_ADDR										(uint8_t)0x24
#define LSM6DSL_MAG_SI_XY_ADDR										(uint8_t)0x25
#define LSM6DSL_MAG_SI_XZ_ADDR										(uint8_t)0x26
#define LSM6DSL_MAG_SI_YX_ADDR										(uint8_t)0x27
#define LSM6DSL_MAG_SI_YY_ADDR										(uint8_t)0x28
#define LSM6DSL_MAG_SI_YZ_ADDR										(uint8_t)0x29
#define LSM6DSL_MAG_SI_ZX_ADDR										(uint8_t)0x2A
#define LSM6DSL_MAG_SI_ZY_ADDR										(uint8_t)0x2B
#define LSM6DSL_MAG_SI_ZZ_ADDR										(uint8_t)0x2C
#define LSM6DSL_MAG_OFFX_L_ADDR										(uint8_t)0x2D
#define LSM6DSL_MAG_OFFX_H_ADDR										(uint8_t)0x2E
#define LSM6DSL_MAG_OFFY_L_ADDR										(uint8_t)0x2F
#define LSM6DSL_MAG_OFFY_H_ADDR										(uint8_t)0x30
#define LSM6DSL_MAG_OFFZ_L_ADDR										(uint8_t)0x31
#define LSM6DSL_MAG_OFFZ_H_ADDR										(uint8_t)0x32
// BANK B
#define LSM6DSL_A_WRIST_TILT_LAT_ADDR							(uint8_t)0x50
#define LSM6DSL_A_WRIST_TILT_THS_ADDR							(uint8_t)0x54
#define LSM6DSL_A_WRIST_TILT_MASK_ADDR						(uint8_t)0x59
#endif

/**************************************************
 * CTRL1_XL
 **************************************************/
// ODR_XL
#define LSM6DSL_ODR_POWER_DOWN_XL						0b0000
#define LSM6DSL_ODR_LP_1_6HZ_HP_12_5HZ_XL		0b1011
#define LSM6DSL_ODR_12_5HZ_XL								0b0001
#define LSM6DSL_ODR_26HZ_XL									0b0010
#define LSM6DSL_ODR_52HZ_XL									0b0011
#define LSM6DSL_ODR_104HZ_XL								0b0100
#define LSM6DSL_ODR_208HZ_XL								0b0101
#define LSM6DSL_ODR_416HZ_XL								0b0110
#define LSM6DSL_ODR_833HZ_XL								0b0111
#define LSM6DSL_ODR_1_66KHZ_XL							0b1000
#define LSM6DSL_ODR_3_33KHZ_XL							0b1001
#define LSM6DSL_ODR_6_66KHZ_XL							0b1010
// ODR_XL one sample time
#define LSM6DSL_ODR_1_6HZ_XL_TIME_MS        700
#define LSM6DSL_ODR_12_5HZ_XL_TIME_MS       80
#define LSM6DSL_ODR_26HZ_XL_TIME_MS         40
#define LSM6DSL_ODR_52HZ_XL_TIME_MS         20
#define LSM6DSL_ODR_104HZ_XL_TIME_MS        10
#define LSM6DSL_ODR_208HZ_XL_TIME_MS        5
#define LSM6DSL_ODR_416HZ_XL_TIME_MS        3
#define LSM6DSL_ODR_833HZ_XL_TIME_MS        2
#define LSM6DSL_ODR_1_66KHZ_XL_TIME_MS      1
#define LSM6DSL_ODR_3_33KHZ_XL_TIME_MS      1
#define LSM6DSL_ODR_6_66KHZ_XL_TIME_MS      1
// FS_XL
#define LSM6DSL_FS_2G_XL		0b00
#define LSM6DSL_FS_16G_XL		0b01
#define LSM6DSL_FS_4G_XL		0b10
#define LSM6DSL_FS_8G_XL		0b11
// BW0_XL
#define LSM6DSL_BW0_1_5KHZ_XL		0b0
#define LSM6DSL_BW0_400HZ_XL		0b1

// Sensitivity
#define LSM6DSL_SENSITIVITY_FS_2G_XL			(float)0.061
#define LSM6DSL_SENSITIVITY_FS_4G_XL			(float)0.122
#define LSM6DSL_SENSITIVITY_FS_8G_XL			(float)0.244
#define LSM6DSL_SENSITIVITY_FS_16G_XL			(float)0.488

/**************************************************
 * CTRL2_G
 **************************************************/
 // ODR_G
#define LSM6DSL_ODR_POWER_DOWN_G		0b0000
#define LSM6DSL_ODR_12_5HZ_G				0b0001
#define LSM6DSL_ODR_26HZ_G					0b0010
#define LSM6DSL_ODR_52HZ_G					0b0011
#define LSM6DSL_ODR_104HZ_G					0b0100
#define LSM6DSL_ODR_208HZ_G					0b0101
#define LSM6DSL_ODR_416HZ_G					0b0110
#define LSM6DSL_ODR_833HZ_G					0b0111
#define LSM6DSL_ODR_1_66KHZ_G				0b1000
#define LSM6DSL_ODR_3_33KHZ_G				0b1001
#define LSM6DSL_ODR_6_66KHZ_G				0b1010
// FS_G
#define LSM6DSL_FS_250DPS_G			0b000
#define LSM6DSL_FS_125DPS_G			0b001
#define LSM6DSL_FS_500DPS_G			0b010
#define LSM6DSL_FS_1000DPS_G		  0b100
#define LSM6DSL_FS_2000DPS_G	  	0b110
// FS_125
#define LSM6DSL_FS_125_DISABLE	0b0
#define LSM6DSL_FS_125_ENABLE		0b1
// Sensitivity
#define LSM6DSL_SENSITIVITY_FS_125DPS_G		(float)4.375
#define LSM6DSL_SENSITIVITY_FS_250DPS_G		(float)8.75
#define LSM6DSL_SENSITIVITY_FS_500DPS_G		(float)15.50
#define LSM6DSL_SENSITIVITY_FS_1000DPS_G	(float)35
#define LSM6DSL_SENSITIVITY_FS_2000DPS_G	(float)70

/**************************************************
 * CTRL3_C
 **************************************************/
#define LSM6DSL_BOOT_NORMAL_MODE							0
#define LSM6DSL_BOOT_REBOOT										1
#define LSM6DSL_BDU_CONTINUOUS_UPDATE					0
#define LSM6DSL_BDU_READ_FIRST_BEFORE_UPDATE	1
#define LSM6DSL_H_LACTIVE_ACTIVE_HIGH_INT			0
#define LSM6DSL_H_LACTIVE_ACTIVE_LOW_INT			1
#define LSM6DSL_PP_OD_PUSH_PULL								0
#define LSM6DSL_PP_OD_OPEN_DRAIN							1
#define LSM6DSL_SIM_4_WIRE										0
#define LSM6DSL_SIM_3_WIRE										1
#define LSM6DSL_IF_INC_DISABLED								0
#define LSM6DSL_IF_INC_ENABLED								1
#define LSM6DSL_BLE_LSB_AT_LOWER_ADDR					0
#define LSM6DSL_BLE_MSB_AT_LOWER_ADDR					1
#define LSM6DSL_SW_RESET_NORMAL_MODE					0
#define LSM6DSL_SW_RESET_RESET								1

/**************************************************
 * CTRL4_C
 **************************************************/
// Enable I2C
#define LSM6DSL_CTRL4_C_I2C_SPI_ENABLE    0
#define LSM6DSL_CTRL4_C_SPI_ENABLE_ONLY   1

/**************************************************
 * CTRL6_C
 **************************************************/
//High performance mode
#define LSM6DSL_CTRL6_C_HM_ENABLE   0
#define LSM6DSL_CTRL6_C_HM_DISABLE  1
//Trigger mode
#define LSM6DSL_CTRL6_C_EDGE_TRIGGER    0b100
#define LSM6DSL_CTRL6_C_LEVEL_TRIGGER   0b010
#define LSM6DSL_CTRL6_C_LEVEL_LATCH     0b011
#define LSM6DSL_CTRL6_C_LEVEL_FIFO      0b110

/**************************************************
 * CTRL7_C
 **************************************************/
#define LSM6DSL_CTRL7_G_HM_ENABLE   0
#define LSM6DSL_CTRL7_G_HM_DISABLE  1

/**************************************************
 * Gyroscope Inactivity Mode
 **************************************************/
#define LSM6DSL_INACT_EN_GYRO_DSIABLE					0
#define LSM6DSL_INACT_EN_GYRO_NO_CHANGE				1
#define LSM6DSL_INACT_EN_GYRO_SLEEP						2
#define LSM6DSL_INACT_EN_GYRO_POWER_DOWN			3
#define LSM6DSL_INACT_DUR_TIME_SEC  ((ic3.xlg_wake_dur << 9) / 
#define XLG_ACTIVE    0x01
#define XLG_INACTIVE  0x02
#define XLG_DETECT_ACTIVITY()         ((xlg_activity&XLG_ACTIVE) && ic3.xlg_int_enabled)
#define XLG_DETECT_INACTIVITY()       ((xlg_activity&XLG_INACTIVE) && ic3.xlg_int_enabled)

/**************************************************
 * Calibration
 **************************************************/
typedef struct calib {
  float xlx;
  float xly;
  float xlz;
  float gx;
  float gy;
  float gz;
} CALIB_TypeDef;

bool lsm6dsl_init(void);
uint8_t lsm6dsl_get_who_am_i(void);
uint8_t lsm6dsl_get_status_reg(void);
float lsm6dsl_get_out_temp(void);
bool lsm6dsl_get_out_xl_g(COT_DATA *data);
bool lsm6dsl_get_out_g(COT_DATA *data);
bool lsm6dsl_get_out_xl(COT_DATA *data);
void lsm6dsl_set_sensitivity_g(void);
void lsm6dsl_set_sensitivity_xl(void);
void lsm6dsl_set_offset_xl(CALIB_TypeDef calib);

// interrupt configuration functions
//void lsm6dsl_free_fall_int(uint8_t duration, uint8_t threshold);
//void lsm6dsl_6d_orientation_int(void);
void lsm6dsl_wake_up_int(uint8_t duration, uint8_t threshold);
void lsm6dsl_inact_recog_int(uint8_t threshold, uint8_t duration);
//void lsm5dsl_sig_motion_int(uint8_t thres);
//void lsm6dsl_tilt_int(void);
//uint8_t lsm6dsl_awt_int(uint8_t latency, uint8_t threshold, uint8_t mask);
void lsm6dsl_interrupt_en(bool enable);
void lsm6dsl_set_powerdown_g(void);
void lsm6dsl_set_powerdown_xl(void);
void lsm6dsl_gyro_sleep_mode(bool sleep);

void lsm6dsl_write(uint8_t reg, uint8_t *pData, uint8_t len);
void lsm6dsl_read(uint8_t reg, uint8_t *pData, uint8_t len);

void lsm6dsl_sw_reset_boot(bool reset_boot);
void lsm6dsl_read_registers(void);

CALIB_TypeDef lsm6dsl_get_offset_xl_g(void);

#endif
