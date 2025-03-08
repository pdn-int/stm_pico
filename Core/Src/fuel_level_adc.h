#ifndef _FUEL_LEVEL_ADC_H_
#define _FUEL_LEVEL_ADC_H_

#include <stdint.h>
#include "cotfactory.h"

#define ADS1115_ADDR (0x48<<1) // ADS1115 I2C address when ADDR connected to GND
//#define FILL_H 6.8f // max fill height in inches
//#define HEIGHT 14.8f // distance from sensor to bottom of tank in inches
//#define RESISTOR 100 // shunt resistor value
#define FSR 6.144f // ADS1115 full scale range (FSR)
#define ADC_MAX_READING 32768.0f

#define FLEVEL_SINGLE_SHOT_MODE   0
#define FLEVEL_CONTINUOUS_MODE    1

// output current between 4 mA to 22 mA (+/- 1mA)
#define FLEVEL_CURRENT_MIN        0.003f
#define FLEVEL_CURRENT_MAX        0.023f // current can output up to 22 mA for fail-safe

#define WDEPTH_CURRENT_MIN        0.004f
#define WDEPTH_CURRENT_MAX        0.022f // current can output up to 22 mA for fail-safe

enum FLEVEL_REG_ADDR {
	CONV_REG_ADDR = 0x00,
	CONF_REG_ADDR = 0x01,
	LO_THRES_REG_ADDR = 0x02,
	HI_THRES_REG_ADDR = 0x03
};

void flevel_get_data(COT_DATA *data);
float flevel_get_voltage(void);
float flevel_get_current(float voltage);
float flevel_get_liquid(float current);
float flevel_get_air(float liquid);

void wdepth_get_data(COT_DATA *data);
float wdepth_get_voltage(void);
float wdepth_get_current(float voltage);
float wdepth_get_depth(float current);

void ads1115_start_single_shot(void);
bool Sensor_I2C1_Init(void);

#endif
