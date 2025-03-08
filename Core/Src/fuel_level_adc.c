#ifdef STM32_UMOTE2
#include "stm32h7xx_hal.h"
#endif

#include <stdint.h>
#include "fuel_level_adc.h"
#include "stm32h7xx_hal.h"
#include "daemon.h"
//#include "stats.h"

#define FLEVEL_INCREMENTS 5.0f

#define FLEVEL_LIQUID_MAX               100.0f
#define FLEVEL_LIQUID_SLOPE             6250.0f
#define FLEVEL_LIQUID_Y_INTERCEPT       25.0f

#define WDEPTH_DEPTH_MAX               5000.0f //5 m = 5000 mm



void flevel_get_data(COT_DATA *data) {
  data->flevel.voltage = flevel_get_voltage();
  data->flevel.current = flevel_get_current(data->flevel.voltage);
  data->flevel.liquid = flevel_get_liquid(data->flevel.current);
  data->flevel.air = flevel_get_air(data->flevel.liquid);
}

// Using Ohm's law to return current across the 100 Ohm resistor
float flevel_get_current(float voltage){
  float current = voltage / ic3.flevel_resistor;
  
	if((voltage < 0) || (current < FLEVEL_CURRENT_MIN) || (current > FLEVEL_CURRENT_MAX)) {
		current = -1;
	}
  
  return current;
}

float flevel_get_liquid(float current){
  float liquid = 0;
  
  if(current < 0) {
    liquid = -1;
  } else {
    liquid = FLEVEL_LIQUID_SLOPE * current - FLEVEL_LIQUID_Y_INTERCEPT;
    if(liquid > FLEVEL_LIQUID_MAX) {
      liquid = FLEVEL_LIQUID_MAX;
    }if(liquid < 0) {
      liquid = 0;
    }
  }
  
  // if reporting liquid by increments
  //liquid = round(liquid / FLEVEL_INCREMENTS) * FLEVEL_INCREMENTS;
    
  return liquid;
}

float flevel_get_air(float liquid){
  if((liquid < 0) || (liquid > FLEVEL_LIQUID_MAX)) {
    return -1;
  } else {
    return FLEVEL_LIQUID_MAX - liquid;
  }
}

void wdepth_get_data(COT_DATA *data) {
  data->wdepth.voltage = wdepth_get_voltage();
  data->wdepth.current = wdepth_get_current(data->wdepth.voltage);
  data->wdepth.depth = wdepth_get_depth(data->wdepth.current);
}

// Using Ohm's law to return current across the 230 Ohm resistor
float wdepth_get_current(float voltage){
	// Check for invalid voltage
	if (voltage < 0.0f) {
		return -1.0f;
	}

	// Prevent division by zero
	if (ic3.wdepth_resistor == 0.0f) {
		return -1.0f;
	}

	// Calculate current using Ohm's law: I = V / R
	float current = voltage / ic3.wdepth_resistor;

	// Define a tolerance to allow small measurement variations (e.g., 0.0001 A)
	const float tolerance = 0.0001f;

	// Check if current is within the acceptable range (including tolerance)
	if ((current + tolerance) < WDEPTH_CURRENT_MIN || (current - tolerance) > WDEPTH_CURRENT_MAX) {
		return -1.0f;
	}

	return current;
}

float wdepth_get_depth(float current){
	// If current is invalid, return an error indicator (-1)
	if (current < 0.0f) {
		return -1.0f;
	}

	// Calculate the water depth using linear interpolation:
	// Depth = ((current - I_min) / (I_max - I_min)) * Depth_max
	float depth = ((current - WDEPTH_CURRENT_MIN) / (WDEPTH_CURRENT_MAX - WDEPTH_CURRENT_MIN)) * WDEPTH_DEPTH_MAX;

	// Clamp the depth to the valid range [0, WDEPTH_DEPTH_MAX]
	if (depth < 0.0f) {
		depth = 0.0f;
	}
	if (depth > WDEPTH_DEPTH_MAX) {
		depth = WDEPTH_DEPTH_MAX;
	}

	    return depth;
}

//********************************************************
// ADS1115 ADC Module
//********************************************************

//unsigned char ADSwrite[3]; // data written to ADS115
//unsigned char ADSread[2]; // data read from ADS1115
//int16_t reading; // I2C readings from ADC module

extern I2C_HandleTypeDef hi2c1;
#define hi2c hi2c1
#define I2C I2C1

static void ads1115_write(uint8_t reg, uint8_t *data, int len);
static void ads1115_read(uint8_t reg, uint8_t *data, int len);

// converts analog data read from ADC to voltage
// Full-Scale Range (FSR) using 6.144V conversion scale (Config Register [11:9] = PGA[2:0])
// why 32768? 2^15 (16-bit where MSB = sign bit)
const float voltageConv = FSR / ADC_MAX_READING;

// returns the voltage across the 100 ohm resistor
// Note: with R =100, the voltage range [0.4V, 2.0V]
float flevel_get_voltage(void){
  uint8_t data[2] = {0};
  float voltage = 0;

  ads1115_start_single_shot();
//  osDelay(10); // requires some time for conversion
  HAL_Delay(10);

  ads1115_read(CONV_REG_ADDR, data, 2);

  int16_t reading = (data[0] << 8 | data[1]);
//  printf(" reading value inside flevel_getvoltage: %d \r\n", reading);
  voltage = reading * voltageConv;

	if((voltage < (ic3.flevel_resistor * FLEVEL_CURRENT_MIN)) || (voltage > (ic3.flevel_resistor * FLEVEL_CURRENT_MAX))) {
    voltage = -1;
  }

  return voltage;
}

// converts analog data read from ADC to voltage
// Full-Scale Range (FSR) using 6.144V conversion scale (Config Register [11:9] = PGA[2:0])
// why 32768? 2^15 (16-bit where MSB = sign bit)

// returns the voltage across the 230 ohm resistor
float wdepth_get_voltage(void){
  uint8_t data[2] = {0};
  float voltage = 0;

  ads1115_start_single_shot();
//  osDelay(10); // requires some time for conversion
  HAL_Delay(10);

  ads1115_read(CONV_REG_ADDR, data, 2);
//  printf("data[0] = %d\r\n",data[0]);
//  printf("data[1] = %d\r\n",data[1]);
  int16_t reading = (data[0] << 8 | data[1]);
//  printf("reading = %d\r\n",reading);
//  printf(" reading value inside wdepth_getvoltage: %d \r\n", reading);
  voltage = reading * voltageConv;

  if((voltage < (ic3.wdepth_resistor * WDEPTH_CURRENT_MIN)) || (voltage > (ic3.wdepth_resistor * WDEPTH_CURRENT_MAX))) {
    voltage = -1;
  }

  return voltage;
}

void ads1115_start_single_shot(void) {
    uint8_t data[2] = {0};
    
    // single-shot configuration and read
    // Config Register settings
    // [15] = begin single conversion
    // [14:12] = read between AIN0 and GND
    // [11:9] = 6.144V FSR (Analog input must not go over this value)
    // [8] = single shot
    // [7:5] = 8 samples per second (SPS) (DEFAULT)
    // [4] = traditional comparator (DEFAULT)
    // [3] = active low polarity (DEFAULT)
    // [2] = nonlatching comparator (DEFAULT)
    // [1:0] = disable comparator and set ALRT/RDY pin to hi-z (DEFAULT)
    data[0] = 0xC1; // Config Reg [15:8] = 1_100_000_1
    data[1] = 0x83; // Config Reg [7:0]  = 000_0_0_0_11
    ads1115_write(CONF_REG_ADDR, data, 2);
}

void ads1115_write(uint8_t reg, uint8_t *data, int len) {
  uint8_t *wData = malloc((len+1)*sizeof(uint8_t));
  memcpy(wData, &reg, sizeof(uint8_t));
  memcpy(wData+sizeof(uint8_t), data, len*sizeof(uint8_t));
  
  if(HAL_I2C_IsDeviceReady(&hi2c, ADS1115_ADDR, 2, 100) == HAL_OK) {
    if(HAL_I2C_Master_Transmit(&hi2c, ADS1115_ADDR, wData, len+1, 100) != HAL_OK) {
      printf("I2C FLEVEL Transmit error...");
    }
  } else {
    printf("I2C1 device not ready for write");
	}
  
  free(wData);
}

void ads1115_read(uint8_t reg, uint8_t *data, int len) {
  if(HAL_I2C_IsDeviceReady(&hi2c, ADS1115_ADDR, 2, 100) == HAL_OK) {
    HAL_I2C_Master_Transmit(&hi2c, ADS1115_ADDR, &reg, 1, 100);
    HAL_I2C_Master_Receive(&hi2c, ADS1115_ADDR, data, len, 100);
  } else {
    printf("I2C1 device not ready for read");
	}
}

bool Sensor_I2C1_Init(void) {
  hi2c.Instance = I2C;
  hi2c.Init.Timing = 0x00C0EAFF;
  hi2c.Init.OwnAddress1 = 0;
  hi2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c.Init.OwnAddress2 = 0;
  hi2c.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c) != HAL_OK) {
    printf("Failed to initialize I2C");
    return false;
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
//    L_INFO("Failed to configure I2C analog filter");
    return false;
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c, 0) != HAL_OK) {
    printf("Failed to configure I2C digital filter");
    return false;
  }
  
  printf("I2C1 for Fuel Level Sensor initialization completed\r\n");
  return true;
}

void Sensor_I2C1_MspInit(I2C_HandleTypeDef* hi2c, GPIO_InitTypeDef *gpio, RCC_PeriphCLKInitTypeDef *periph) {
  periph->PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  periph->I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(periph) != HAL_OK) {
    printf("Failed to initilized peripheral clock for I2C1");
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  /**I2C1 GPIO Configuration
  PB6     ------> I2C1_SCL
  PB7     ------> I2C1_SDA
  */
  gpio->Pin = GPIO_PIN_6|GPIO_PIN_7;
  gpio->Mode = GPIO_MODE_AF_OD;
  gpio->Pull = GPIO_PULLUP;
  gpio->Speed = GPIO_SPEED_FREQ_LOW;
  gpio->Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, gpio);

  __HAL_RCC_I2C1_CLK_ENABLE();
}

void Sensor_I2C1_MspDeInit(I2C_HandleTypeDef* hi2c) {
  __HAL_RCC_I2C1_CLK_DISABLE();

  /**I2C1 GPIO Configuration
  PB6     ------> I2C1_SCL
  PB7     ------> I2C1_SDA
  */
  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
}
