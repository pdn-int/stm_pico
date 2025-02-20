#ifndef _MAX1704X_FUEL_GAUGE_H_
#define _MAX1704X_FUEL_GAUGE_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "daemon.h"
#include "cotfactory.h"

/********************************************************
 * Interrupt Pin
 ********************************************************/
#ifdef STM32_UMOTE
  #define MAX1704X_INT_PORT   GPIOC
  #define MAX1704X_INT_PIN    GPIO_PIN_3
  #define MAX1704X_EXTI_IRQn  EXTI3_IRQn
#else //Pico
  #define MAX1704X_INT_PORT   GPIOI
  #define MAX1704X_INT_PIN    GPIO_PIN_13
  #define MAX1704X_EXTI_IRQn  EXTI15_10_IRQn
#endif

 
// Note: micromote uses max17049 and pico uses max17048
// max17049 is for 2 cells in series
// max17048 is for 1 cell
/***************************************************
 * I2C Slave Address
 ***************************************************/
#define MAX1704X_I2C_ADDR				(uint8_t)0x36<<1

/***************************************************
 * Register Addresses
 * Note: Read and Write at 2 bytes since registers are 16-bits
 *       Register value = (16-bit word) * (Unit or LSB value)
 ***************************************************/
#define MAX1704X_VCELL_ADDR			(uint8_t)0x02
#define MAX1704X_SOC_ADDR				(uint8_t)0x04
#define MAX1704X_MODE_ADDR			(uint8_t)0x06 // default: 0x0000
#define MAX1704X_VERSION_ADDR		(uint8_t)0x08 // default: 0x001_
#define MAX1704X_HIBRT_ADDR			(uint8_t)0x0A // default: 0x8030
#define MAX1704X_CONFIG_ADDR		(uint8_t)0x0C // default: 0x971C
#define MAX1704X_VALRT_ADDR			(uint8_t)0x14 // default: 0x00FF
#define MAX1704X_CRATE_ADDR			(uint8_t)0x16
#define MAX1704X_VRESET_ID_ADDR	(uint8_t)0x18 // default: 0x96__
#define MAX1704X_STATUS_ADDR		(uint8_t)0x1A // default: 0x01__
#define MAX1704X_TABLE_ADDR			(uint8_t)0x40
#define MAX1704X_CMD_ADDR				(uint8_t)0xFE // default: 0xFFFF

#define MAX1704X_ID								0x0C
#define MAX1704X_DIS_COMP_DISABLE	1
#define MAX1704X_DIS_COMP_ENABLE	0

bool max1704x_init(void);
uint8_t max1704x_get_version(void);
void max1704x_get_vcell(COT_DATA *data);
void max1704x_get_soc(COT_DATA *data);
void max1704x_get_crate(COT_DATA *data);
void max1704x_enable_sleep(bool sleep);

/***************************************************
 * MODE (Write Only)
 * 1) Quick-Start --> MODE[6]
 * 2) EnSleep --> MODE[5]
 * 3) HibStat --> MODE[4]
 ***************************************************/
uint16_t max1704x_get_mode(void);
bool max1704x_set_mode_ensleep(bool enSleep);

/***************************************************
 * HIBRT (Read/Write)
 * - Disable hibernate mode
 * - Enable hibernate mode
 * - Active Threshold (ActThr) --> HIBRT[15:8]
 *		+ If |OCV-CELL| > ActThr ==> exits hibernate mode
 *		+ Unit: 1.25mV
 * - Hibernate Threshold (HibThr) --> HIBRT[7:0]
 *		+ If |CRATE| < HibThr for + 6min ==> enters hibernate mode
 *		+ Unit: 0.208%/hr
 ***************************************************/
bool max1704x_hibernate(bool hib);

/***************************************************
 * CONFIG (Read/Write)
 * - RCOMP
 *		+ Adjustable 8-bit value to optimize IC
 *		+ POR value = 0x97 
 * - SLEEP
 *		+ forces IC in/out of sleep if EnSleep set
 *		+ Enter sleep = 1 & exit sleep = 0
 *		+ POR value = 0
 * - ALSC
 *		+ SOC change alert
 *		+ Enables alerting when SOC changes by at leas 1%
 *		+ Remains unitl STATUS.SC is clearted
 *		+ Do not use use this to accumulate cahnges in SOC
 * - ALRT
 *		+ Alert status bit
 *		+ Set by IC when alert occurs
 *		+ ALRTn pin asserts (goes low) when bit is set
 *		+ Clear this bit to service and deassert the ALRTn pin
 *		+ STATUS register specifies why ALRTn pin was asserted
 * - ATHD
 *		+ Empty alert threshold
 *		+ sets SOC threshold to generate interrupt on ALRTn pin
 *		+ Can be programed to be 1% to 32%
 *		+ threshold = (32-ATHD)%
 *		+ alert only on falling edge past this threshold
 *		+ POR value = 0x1C = 4%
 ***************************************************/
uint16_t max1704x_get_config(void);
bool max1704x_set_config_sleep(bool sleep);
bool max1704x_set_config_alsc(bool alsc);
bool max1704x_set_config_athd(uint8_t athd);
bool max1704x_set_rcomp(COT_DATA *data);
bool max1704x_clear_config_alrt(void);

/***************************************************
 * VALRT (Read/Write)
 * - Two threshold
 *		1) VALRT.MAX --> VALRT[7:0]
 *		2) VALRT.MIN --> VALRT[15:8]
 * - Alerts when 
 *		1) VCELL > VALRT.MAX
 *		2) VCELL < VALRT.MIN
 * - Unit: 20mV
 ***************************************************/
uint16_t max1704x_get_vlrt(void);
bool max1704x_set_valrt_max(uint8_t max);
bool max1704x_set_valrt_min(uint8_t min);

/***************************************************
 * VRESET_ID (Read/Write)
 * - ID --> VRESET_ID[7:1]
 *		+ 8-bit read only device ID
 * - VRESET --> VRESET_ID[15:9]
 *		+ adjusts analog comparator and digital ADC threshold
 *		  to detect battery removal and reinsertion
 *		+ captie battery		<= 2.5V
 *		+ removable battery	<= desired_voltage - 300mV
 *		+ if comparator enabled, IC resets 1ms after VCELL > threshold
 *		+ otherwise, resets 250ms after VCELL > threshold
 *		+ Unit: 40mV
 * - DIS --> VRESET_ID[8]
 *		+ Disable analog comparator in hibernate mode when = 1
 ***************************************************/
uint8_t max1704x_get_id(void);

/***************************************************
 * STATUS (Read/Write)
 * - Defines alert
 * - Clear corresponding bit after servicing alert
 * - RI (Reset Indicator)
 *		+ Set on POR indicating IC not configured
 *		+ Requires modle loaded and this bit cleared
 * - VH (Voltage High)
 * - VL (Voltage Low)
 * - VR (Voltage Reset)
 * - HD (SOC Low)
 * - SC (1% SOC Change)
 * - EnVR (Enable Votlage Reset Alert)
 ***************************************************/
#define MAX1704X_STATUS_RI_BIT    0x0100
#define MAX1704X_STATUS_VH_BIT    0x0200
#define MAX1704X_STATUS_VL_BIT    0x0400
#define MAX1704X_STATUS_VR_BIT    0x0800
#define MAX1704X_STATUS_HD_BIT    0x1000
#define MAX1704X_STATUS_SC_BIT    0x2000
#define MAX1704X_STATUS_ENVR_BIT  0x4000
uint16_t max1704x_get_status(void);
bool max1704x_clear_status_alerts(uint16_t alerts);
bool max1704x_set_status_envr(bool envr);

#endif
