#ifndef _MS8607_PHT_H_
#define _MS8607_PHT_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "cotfactory.h"

#define MS8607_PT_I2C_ADDR		(uint8_t)0x76<<1
#define MS8607_RH_I2C_ADDR		(uint8_t)0x40<<1

/***************************************************
 * Commands For Pressure and Temperature
 *	1. Reset
 *	2. Read PROM P&T (112-bit of calibration words)
 *	3. D1 Conversion
 *	4. D2 Conversion
 *	5. Read ADC (24-bit pressure/temperature)
 ***************************************************/
#define MS8607_PT_RESET_CMD								(uint8_t)0x1E
#define MS8607_PT_CVRT_D1_0SR_256_CMD			(uint8_t)0x40
#define MS8607_PT_CVRT_D1_0SR_512_CMD			(uint8_t)0x42
#define MS8607_PT_CVRT_D1_0SR_1024_CMD		(uint8_t)0x44
#define MS8607_PT_CVRT_D1_0SR_2048_CMD		(uint8_t)0x46
#define MS8607_PT_CVRT_D1_0SR_4096_CMD		(uint8_t)0x48
#define MS8607_PT_CVRT_D1_0SR_8192_CMD		(uint8_t)0x4A
#define MS8607_PT_CVRT_D2_0SR_256_CMD			(uint8_t)0x50
#define MS8607_PT_CVRT_D2_0SR_512_CMD			(uint8_t)0x52
#define MS8607_PT_CVRT_D2_0SR_1024_CMD		(uint8_t)0x54
#define MS8607_PT_CVRT_D2_0SR_2048_CMD		(uint8_t)0x56
#define MS8607_PT_CVRT_D2_0SR_4096_CMD		(uint8_t)0x58
#define MS8607_PT_CVRT_D2_0SR_8192_CMD		(uint8_t)0x5A
#define MS8607_PT_ADC_READ_CMD						(uint8_t)0x00
#define MS8607_PT_PROM_READ_A0_CMD				(uint8_t)0xA0
#define MS8607_PT_PROM_READ_A2_CMD				(uint8_t)0xA2
#define MS8607_PT_PROM_READ_A4_CMD				(uint8_t)0xA4
#define MS8607_PT_PROM_READ_A6_CMD				(uint8_t)0xA6
#define MS8607_PT_PROM_READ_A8_CMD				(uint8_t)0xA8
#define MS8607_PT_PROM_READ_AA_CMD				(uint8_t)0xAA
#define MS8607_PT_PROM_READ_AC_CMD				(uint8_t)0xAC

#define MS8607_PT_OSR_256_D1_CMD      0x40
#define MS8607_PT_OSR_256_D2_CMD      0x50
#define MS8607_PT_OSR_512_D1_CMD      0x42
#define MS8607_PT_OSR_512_D2_CMD      0x52
#define MS8607_PT_OSR_1024_D1_CMD     0x44
#define MS8607_PT_OSR_1024_D2_CMD     0x54
#define MS8607_PT_OSR_2048_D1_CMD     0x46
#define MS8607_PT_OSR_2048_D2_CMD     0x56
#define MS8607_PT_OSR_4096_D1_CMD     0x48
#define MS8607_PT_OSR_4096_D2_CMD     0x58
#define MS8607_PT_OSR_8192_D1_CMD     0x4A
#define MS8607_PT_OSR_8192_D2_CMD     0x5A

/***************************************************
 * PT Max ADC Conversion times (ms)
 ***************************************************/
#define MS8607_PT_CONVERT_TIME_OSR_8192		18
#define MS8607_PT_CONVERT_TIME_OSR_4096		9
#define MS8607_PT_CONVERT_TIME_OSR_2048		5
#define MS8607_PT_CONVERT_TIME_OSR_1024		3
#define MS8607_PT_CONVERT_TIME_OSR_512		2
#define MS8607_PT_CONVERT_TIME_OSR_256		1

/***************************************************
 * PT PROM Calibration values
 ***************************************************/
typedef struct {
	uint16_t crc;
	uint16_t sens;
	uint16_t off;
	uint16_t tcs;
	uint16_t tco;
	uint16_t tref;
	uint16_t tempsens;
} ms8607_pt_calib_t;


#define	MS8607_PT_OSR_256		0
#define	MS8607_PT_OSR_512		1
#define	MS8607_PT_OSR_1024	2
#define	MS8607_PT_OSR_2048	3
#define	MS8607_PT_OSR_4096	4
#define	MS8607_PT_OSR_8192	5

/***************************************************
 * Commands For Relative Humidity
 *	1. Reset
 *	2. Write User Register
 *	3. Read User Register
 *	4. Measure RH (Hold Master)
 *	5. Measure RH (No Hold Master)
 *	6. PROM read RH
 ***************************************************/
#define MS8607_RH_RESET_CMD								(uint8_t)0xFE
#define MS8607_RH_WRTIE_USER_REG_CMD			(uint8_t)0xE6
#define MS8607_RH_READ_USER_REG_CMD				(uint8_t)0xE7
#define MS8607_RH_MEAS_HOLD_CMD						(uint8_t)0xE5
#define MS8607_RH_MEAS_NO_HOLD_CMD				(uint8_t)0xF5
#define MS8607_RH_PROM_READ_A0_CMD				(uint8_t)0xA0
#define MS8607_RH_PROM_READ_A2_CMD				(uint8_t)0xA2
#define MS8607_RH_PROM_READ_A4_CMD				(uint8_t)0xA4
#define MS8607_RH_PROM_READ_A6_CMD				(uint8_t)0xA6
#define MS8607_RH_PROM_READ_A8_CMD				(uint8_t)0xA8
#define MS8607_RH_PROM_READ_AA_CMD				(uint8_t)0xAA
#define MS8607_RH_PROM_READ_AC_CMD				(uint8_t)0xAC

/***************************************************
 * RH Max ADC Conversion times (ms)
 ***************************************************/
#define MS8607_RH_CONVERT_TIME_OSR_4096		16
#define MS8607_RH_CONVERT_TIME_OSR_2048		9
#define MS8607_RH_CONVERT_TIME_OSR_1024		5
#define MS8607_RH_CONVERT_TIME_OSR_256		5 //3

/***************************************************
 * USER REGISTER
 ***************************************************/
// Note: Datasheet is inconsistent with OSR and conversion time.
// Hence, using other code from sparkfun/adafruit to figure out these values
#define	MS8607_RH_OSR_4096					0b00000000 //default
#define	MS8607_RH_OSR_2048					0b10000001
#define	MS8607_RH_OSR_1024					0b10000000
#define	MS8607_RH_OSR_256						0b00000001

#define	MS8607_RH_VDD_MORE_2_25 		0b00000000 //default
#define	MS8607_RH_VDD_LESS_2_25			0b01000000

#define	MS8607_RH_HEATER_DISABLED		0b00000000 //default
#define	MS8607_RH_HEATER_ENABLED		0b00000010

/***************************************************
 * Functions
 ***************************************************/
bool ms8607_init(void);
// Pressure and Temperature
void ms8607_reset_pt(void);
uint16_t ms8607_prom_read_one_addr_pt(uint8_t prom_addr);
bool ms8607_prom_read_all_addr_pt(ms8607_pt_calib_t *calib);
uint32_t ms8607_read_adc_pt(uint8_t convert_cmd);
bool ms8607_get_pt(COT_DATA *data);
float ms8607_degC_to_degF(float degC);

// Humidity
void ms8607_reset_rh(void);
void ms8607_write_usr_reg_rh(uint8_t res, uint8_t batt, uint8_t heater);
uint8_t ms8607_read_usr_reg_rh(void);
 bool ms8607_read_adc_rh(uint16_t *adc_rh);
uint16_t ms8607_prom_read_rh(uint8_t prom_addr);
bool ms8607_get_rh(COT_DATA *data);

// CRC checking
uint8_t ms8607_crc4_pt(uint16_t *n_prom);
//uint8_t ms8607_crc4_rh(uint16_t *n_prom);
uint8_t ms8607_checksum_rh(uint16_t rh);
#endif
