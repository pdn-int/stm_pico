#ifndef _APDS9250_LIGHT_H_
#define _APDS9250_LIGHT_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "daemon.h"
#include "cotfactory.h"

/********************************************************
 * Interrupt Pin
 ********************************************************/
#define APDS9250_INT_PORT   GPIOI
#define APDS9250_INT_PIN    GPIO_PIN_8
#define APDS9250_EXTI_IRQn  EXTI9_5_IRQn

/********************************************************
 * Slave Address
 * 7-bit address = 0x52
 ********************************************************/
#define APDS9250_I2C_ADDR	(uint8_t)0x52<<1

/********************************************************
 * Register Addresses
 ********************************************************/
#define APDS9250_MAIN_CTRL_ADDR					(uint8_t)0x00
#define APDS9250_LS_MEAS_RATE_ADDR			(uint8_t)0x04
#define APDS9250_LS_GAIN_ADDR						(uint8_t)0x05
#define APDS9250_PART_ID_ADDR						(uint8_t)0x06
#define APDS9250_MAIN_STATUS_ADDR				(uint8_t)0x07
#define APDS9250_LS_DATA_IR_0_ADDR			(uint8_t)0x0A
#define APDS9250_LS_DATA_IR_1_ADDR			(uint8_t)0x0B
#define APDS9250_LS_DATA_IR_2_ADDR			(uint8_t)0x0C
#define APDS9250_LS_DATA_GREEN_0_ADDR		(uint8_t)0x0D
#define APDS9250_LS_DATA_GREEN_1_ADDR		(uint8_t)0x0E
#define APDS9250_LS_DATA_GREEN_2_ADDR		(uint8_t)0x0F
#define APDS9250_LS_DATA_BLUE_0_ADDR		(uint8_t)0x10
#define APDS9250_LS_DATA_BLUE_1_ADDR		(uint8_t)0x11
#define APDS9250_LS_DATA_BLUE_2_ADDR		(uint8_t)0x12
#define APDS9250_LS_DATA_RED_0_ADDR			(uint8_t)0x13
#define APDS9250_LS_DATA_RED_1_ADDR			(uint8_t)0x14
#define APDS9250_LS_DATA_RED_2_ADDR			(uint8_t)0x15
#define APDS9250_INT_CFG_ADDR						(uint8_t)0x19
#define APDS9250_INT_PERSISTENCE_ADDR		(uint8_t)0x1A
#define APDS9250_LS_THRES_UP_0_ADDR			(uint8_t)0x21
#define APDS9250_LS_THRES_UP_1_ADDR			(uint8_t)0x22
#define APDS9250_LS_THRES_UP_2_ADDR			(uint8_t)0x23
#define APDS9250_LS_THRES_LOW_0_ADDR		(uint8_t)0x24
#define APDS9250_LS_THRES_LOW_1_ADDR		(uint8_t)0x25
#define APDS9250_LS_THRES_LOW_2_ADDR		(uint8_t)0x26
#define APDS9250_THRES_VAR_ADDR					(uint8_t)0x27

/********************************************************
 * MAIN_CTRL
 ********************************************************/
enum {
	ALS_IR_COMP_ACTIVATED 			= 0,
	ALL_RGB_IR_COMP_ACTIVATED		= 1
};

enum {
	LS_STANDBY	= 0,
	LS_ACTIVE		= 1
};

/********************************************************
 * LS_MEAS_RATE
 ********************************************************/
enum {
	BIT20_400MS		=	0,
	BIT19_200MS 	= 1,
	BIT18_100MS 	= 2, //default
	BIT17_50MS		= 3,
	BIT16_25MS		= 4,
	BIT13_3_125MS = 5
};

enum {
	RATE_25MS		= 0,
	RATE_50MS		= 1,
	RATE_100MS	= 2, //default
	RATE_200MS	= 3,
	RATE_500MS	= 4,
	RATE_1000MS	= 5,
	RATE_2000MS = 6 //7 is 2000 ms
};

/********************************************************
 * LS_GAIN
 ********************************************************/
enum {
	GAIN1		= 0,
	GAIN3		= 1,
	GAIN6		= 2,
	GAIN9		= 3,
	GAIN18	= 4
};

/********************************************************
 * PART_ID (Read Only)
 ********************************************************/
#define APDS9250_PART_ID	0xB0
#define APDS9250_REV_ID		0x05

/********************************************************
 * MAIN_STATUS (Read Only)
 ********************************************************/

/********************************************************
 * LS_DATA_IR/GREEN/BLUE/RED (0/1/2) (Read Only)
 ********************************************************/

/********************************************************
 * INT_CFG
 ********************************************************/
#define LIGHT_INT_DISABLED  0
#define LIGHT_INT_THRESHOLD 1
#define LIGHT_INT_VARIANCE  2

enum {
	LS_INT_DISABLED	= 0, //default
	LS_INT_ENABLED	= 1
};

enum {
	LS_THRES_INT_MODE	= 0, //default
	LS_VAR_INT_MODE		= 1
};

enum {
	IR_CHANNEL				= 0,
	ALS_GREEN_CHANNEL	= 1, //default
	RED_CHANNEL				= 2,
	BLUE_CHANNEL			= 3
};

/********************************************************
 * INT_PERSISTENCE
 ********************************************************/
enum {
	ASSERT_INT_EVERY_1	= 0, //default
	ASSERT_INT_EVERY_2	= 1,
	ASSERT_INT_EVERY_3	= 2,
	ASSERT_INT_EVERY_4	= 3,
	ASSERT_INT_EVERY_5	= 4,
	ASSERT_INT_EVERY_6	= 5,
	ASSERT_INT_EVERY_7	= 6,
	ASSERT_INT_EVERY_8	= 7,
	ASSERT_INT_EVERY_9	= 8,
	ASSERT_INT_EVERY_10	= 9,
	ASSERT_INT_EVERY_11	= 10,
	ASSERT_INT_EVERY_12	= 11,
	ASSERT_INT_EVERY_13	= 12,
	ASSERT_INT_EVERY_14	= 13,
	ASSERT_INT_EVERY_15	= 14,
	ASSERT_INT_EVERY_16	= 15
};

/********************************************************
 * LS_THRES_UP (0/1/2)
 ********************************************************/

/********************************************************
 * LS_THRES_LOW (0/1/2)
 ********************************************************/

/********************************************************
 * LS_THRES_VAR
 ********************************************************/
enum {
	VARY_BY_8_FROM_PREV			= 0,
	VARY_BY_16_FROM_PREV		= 1,
	VARY_BY_32_FROM_PREV		= 2,
	VARY_BY_64_FROM_PREV		= 3,
	VARY_BY_128_FROM_PREV		= 4,
	VARY_BY_256_FROM_PREV		= 5,
	VARY_BY_512_FROM_PREV		= 6,
	VARY_BY_1024_FROM_PREV	= 7
};

bool apds9250_init(void);
void apds9250_set_main_ctrl(bool sw_reset, bool cs_mode, bool ls_en);
uint8_t apds9250_get_main_ctrl(void);
void apds9250_set_meas_rate(uint8_t resolution, uint8_t rate);
uint8_t apds9250_get_meas_rate(void);
void apds9250_set_gain(uint8_t gain);
uint8_t apds9250_get_gain(void);
bool apds9250_get_part_id(void);
uint8_t apds9250_get_main_status(void);
void apds9250_get_rgb(COT_DATA *data);
void apds9250_get_ir(COT_DATA *data);
void apds9250_get_green(COT_DATA *data);
void apds9250_get_blue(COT_DATA *data);
void apds9250_get_red(COT_DATA *data);

void apds9250_set_int_cfg(uint8_t sel, bool mode, bool en);
uint8_t apds9250_get_int_cfg(void);
void apds9250_set_int_sel(uint8_t sel);
void apds9250_set_var_mode(bool mode);
void apds9250_set_int_en(bool en);
void apds9250_set_int_persistence(uint8_t persist);
uint8_t apds9250_get_int_persistence(void);
void apds9250_set_ls_thres_up(uint32_t thres);
uint32_t apds9250_get_ls_thres_up(void);
void apds9250_set_ls_thres_low(uint32_t thres);
uint32_t apds9250_get_ls_thres_low(void);
void apds9250_set_ls_thres_var(uint8_t vary);
uint8_t apds9250_get_ls_thres_var(void);
#endif
