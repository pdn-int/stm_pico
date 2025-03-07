#include "logger.h"
#include "apds9250_light.h"

#if defined(STM32_UMOTE2)
  #include "stm32h7xx_hal.h"
  extern I2C_HandleTypeDef hi2c4;
  #define apds9250_i2c hi2c4
#elif defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
  extern I2C_HandleTypeDef hi2c3;
  #define apds9250_i2c hi2c3
#endif

/********************************************************
 * Initialization
 * Initialize:
 *   1. Resolution/Bit Width
 *   2. Measurement Rate
 *   3. Gain
 *   4. CS Mode
 * Then, enable the light sensor
 ********************************************************/

//extern IC3Daemon ic3;

char colors[4][4] = {"IR", "GRN", "RED", "BLU"};

bool apds9250_init(void)
{
	// Check Device ID
	if(!apds9250_get_part_id()) {
		return false;
	}
  
	// Read MAIN_STATUS Register to clear Power-On Status
	apds9250_get_main_status();
	
	// Note: These are set to default for now
	// Set Resolution/Bit Width and Meausrement Rate
  apds9250_set_meas_rate(BIT18_100MS, RATE_100MS);

	
	// Set Gain Range
  apds9250_set_gain(GAIN1);

	// Select Threshold Interrupt Mode
  if(ic3.light_sensor_int_mode > LIGHT_INT_DISABLED) {
    //L_INFO("Light sensor interrupt enabled. Setting configurations for interrupt...");
    
    // Set Interrupt
    apds9250_set_int_en(LS_INT_ENABLED);
    
    // Configure GPIO for external interrupt
    __HAL_RCC_GPIOI_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = APDS9250_INT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(APDS9250_INT_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(APDS9250_EXTI_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(APDS9250_EXTI_IRQn);
    
    // Select Red Channels for Interrupt
    //L_INFO("Setting light sensor interrupt channel to %s", colors[ic3.light_sensor_int_ch]);
    apds9250_set_int_sel(ic3.light_sensor_int_ch);
    
    if(ic3.light_sensor_int_mode == LIGHT_INT_VARIANCE) {
      //L_INFO("Light sensor interrupt set to variance mode.");
      
      // Set interrupt to variance mode
      apds9250_set_var_mode(LS_VAR_INT_MODE);
      
      // Set variance threshold
      apds9250_set_ls_thres_var(ic3.light_sensor_var_thres);
      
    } else {
      //L_INFO("Light sensor interrupt set to threshold mode.");
      
      // Set interrupt to threshold mode
      apds9250_set_var_mode(LS_THRES_INT_MODE);
	
      // Set Upper Threshold
      apds9250_set_ls_thres_up(ic3.light_sensor_upper_thres);
    
      // Set Lower Threshold
      apds9250_set_ls_thres_low(ic3.light_sensor_lower_thres);
    }
  }
  
	// Set CS Mode to RGB and enable Light Sensor
  apds9250_set_main_ctrl(0, ALL_RGB_IR_COMP_ACTIVATED, LS_ACTIVE);

	printf("Light Sensor initialization completed\r\n");
	return true;
}

/********************************************************
 * MAIN_CTRL
 ********************************************************/
void apds9250_set_main_ctrl(bool sw_reset, bool cs_mode, bool ls_en)
{
  uint8_t main_ctrl = ((uint8_t)sw_reset<<4) + ((uint8_t)cs_mode<<2) + ((uint8_t)ls_en<<1);
	uint8_t wData[2] = {APDS9250_MAIN_CTRL_ADDR, main_ctrl};
  
	//L_DEBUG("main_ctrl = %02X", wData[1]);
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 2, 100);
}

uint8_t apds9250_get_main_ctrl(void)
{
	uint8_t rData;
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_MAIN_CTRL_ADDR, 1, &rData, 1, 100);
	return rData;
}

/********************************************************
 * MEAS_RATE
 ********************************************************/
void apds9250_set_meas_rate(uint8_t resolution, uint8_t rate)
{
  uint8_t meas_rate = (resolution<<4) + rate;
	uint8_t wData[2] = {APDS9250_LS_MEAS_RATE_ADDR, meas_rate};
  
	//L_DEBUG("main_ctrl = %02X", wData[1]);
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 2, 100);
}

uint8_t apds9250_get_meas_rate(void)
{
	uint8_t rData;
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_MEAS_RATE_ADDR, 1, &rData, 1, 100);
	return rData;
}

/********************************************************
 * GAIN
 ********************************************************/
void apds9250_set_gain(uint8_t gain)
{
	uint8_t wData[2] = {APDS9250_LS_GAIN_ADDR, gain};
  
	//L_DEBUG("main_ctrl = %02X", wData[1]);
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 2, 100);
}

uint8_t apds9250_get_gain(void)
{
	uint8_t rData;
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_GAIN_ADDR, 1, &rData, 1, 100);
	return rData;
}

/********************************************************
 * PART_ID (Read Only)
 ********************************************************/
bool apds9250_get_part_id(void)
{
	uint8_t reg;
	if(HAL_I2C_IsDeviceReady(&apds9250_i2c, APDS9250_I2C_ADDR, 2, 0xFFFF) != HAL_OK) {
		//L_DEBUG("Light Sensor Reading Part ID ERROR");
		return false;
	} else {
		HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_PART_ID_ADDR, 1, &reg, 1, 100);
		if((reg & 0xF0) == APDS9250_PART_ID) {
//			//L_DEBUG("APDS9250 Part ID = %X (valid)", reg);
			return true;
		} else {
			//L_DEBUG("APDS9250 Part ID = %X (invalid)", reg);
			return false;
		}
	}
	
}

/********************************************************
 * MAIN_STATUS (Read Only)
 * Note read the register after it is powered on to clear the power-on status
 * If power-on status is HIGH after it has been cleared, there is a power supply disturbance
 ********************************************************/
uint8_t apds9250_get_main_status(void)
{
	uint8_t reg;
	
	// Read MAIN_STATUS register
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_MAIN_STATUS_ADDR, 1, &reg, 1, 100);
	return reg;
}

/********************************************************
 * LS_DATA_IR/GREEN/BLUE/RED (Read Only)
 ********************************************************/
// Get all four color (ir, green, blue, red)
void apds9250_get_rgb(COT_DATA *data)
{
	uint8_t rData[12];
	
	if(HAL_I2C_IsDeviceReady(&apds9250_i2c, APDS9250_I2C_ADDR, 2, 0xFFFF) != HAL_OK) {
		data->light.ir = 0;
		data->light.green = 0;
		data->light.blue = 0;
		data->light.red = 0;
		//L_DEBUG("Light Sensor Reading IR-RGB ERROR");
	} else {
		HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_DATA_IR_0_ADDR, 1, rData, 12, 100);
		data->light.ir		= ((uint32_t)rData[2]<<16) + ((uint32_t)rData[1]<<8) + (uint32_t)rData[0];
		data->light.green	= ((uint32_t)rData[5]<<16) + ((uint32_t)rData[4]<<8) + (uint32_t)rData[3];
		data->light.blue	= ((uint32_t)rData[8]<<16) + ((uint32_t)rData[7]<<8) + (uint32_t)rData[6];
		data->light.red		= ((uint32_t)rData[11]<<16) + ((uint32_t)rData[10]<<8) + (uint32_t)rData[9];
	}
//	//L_DEBUG("IR-RGB = [%d, %d, %d, %d]", data->light.ir, data->light.green, data->light.blue, data->light.red);
}

void apds9250_get_ir(COT_DATA *data)
{
	uint8_t rData[3];
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_DATA_IR_0_ADDR, 1, rData, 3, 100);
	data->light.ir =  (((uint32_t)rData[2])<<16) + (((uint32_t)rData[1])<<8) + (uint32_t)rData[0];
}

void apds9250_get_green(COT_DATA *data)
{
	uint8_t rData[3];
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_DATA_GREEN_0_ADDR, 1, rData, 3, 100);
	data->light.green =  (((uint32_t)rData[2])<<16) + (((uint32_t)rData[1])<<8) + (uint32_t)rData[0];
}

void apds9250_get_blue(COT_DATA *data)
{
	uint8_t rData[3];
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_DATA_BLUE_0_ADDR, 1, rData, 3, 100);
	data->light.blue = (((uint32_t)rData[2])<<16) + (((uint32_t)rData[1])<<8) + (uint32_t)rData[0];
}

void apds9250_get_red(COT_DATA *data)
{
	uint8_t rData[3];
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_DATA_RED_0_ADDR, 1, rData, 3, 100);
	data->light.red =  (((uint32_t)rData[2])<<16) + (((uint32_t)rData[1])<<8) + (uint32_t)rData[0];
}

/********************************************************
 * INT_CFG
 ********************************************************/
// Configure the whole INT_CFG register
void apds9250_set_int_cfg(uint8_t sel, bool mode, bool en)
{
	uint8_t int_cfg_reg;
	uint8_t wData[2] = {APDS9250_INT_CFG_ADDR, 0};
		
	int_cfg_reg = (en << 2);
	int_cfg_reg |= (mode << 3);
	int_cfg_reg |= ((sel & 0x3) << 4);
	wData[1] = int_cfg_reg;
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 2, 100);
}

uint8_t apds9250_get_int_cfg(void)
{
	uint8_t rData;
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_INT_CFG_ADDR, 1, &rData, 1, 100);
	return rData;
}

void apds9250_set_int_sel(uint8_t sel)
{
	uint8_t int_cfg_reg;
	uint8_t wData[2] = {APDS9250_INT_CFG_ADDR, 0};
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_INT_CFG_ADDR, 1, &int_cfg_reg, 1, 100);
	int_cfg_reg = (int_cfg_reg & ~0x30) | (sel << 4);
	wData[1] = int_cfg_reg;
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 2, 100);
}

void apds9250_set_var_mode(bool mode)
{
	uint8_t int_cfg_reg;
	uint8_t wData[2] = {APDS9250_INT_CFG_ADDR, 0};
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_INT_CFG_ADDR, 1, &int_cfg_reg, 1, 100);
	int_cfg_reg = (int_cfg_reg & ~0x08) | (mode << 3);
	wData[1] = int_cfg_reg;
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 2, 100);
}

void apds9250_set_int_en(bool en)
{
	uint8_t int_cfg_reg;
	uint8_t wData[2] = {APDS9250_INT_CFG_ADDR, 0};
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_INT_CFG_ADDR, 1, &int_cfg_reg, 1, 100);
	int_cfg_reg = (int_cfg_reg & ~0x04) | (en << 2);
	wData[1] = int_cfg_reg;
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 2, 100);
}

/********************************************************
 * INT_PERSISTENCE
 ********************************************************/
void apds9250_set_int_persistence(uint8_t persist)
{
	uint8_t wData[2] = {APDS9250_INT_PERSISTENCE_ADDR, persist};
  
	//L_DEBUG("persist = %02X", wData[1]);
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 2, 100);
}

uint8_t apds9250_get_int_persistence(void)
{
	uint8_t rData;
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_INT_PERSISTENCE_ADDR, 1, &rData, 1, 100);
	return rData;
}

/********************************************************
 * LS_THRES_UP (0/1/2)
 * max 20-bits unsigned integer
 ********************************************************/
void apds9250_set_ls_thres_up(uint32_t thres)
{
	uint8_t wData[4] = {APDS9250_LS_THRES_UP_0_ADDR, thres&0xFF, (thres>>8)&0xFF, (thres>>16)&0x0F};
	
	//L_DEBUG("thres_up = [%02X, %02X, %02X]", wData[3], wData[2], wData[1]);
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 4, 100);
}

uint32_t apds9250_get_ls_thres_up(void)
{
	uint8_t rData[3];
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_THRES_UP_0_ADDR, 1, rData, 3, 100);
	return (rData[2]<<16) + (rData[1]<<8) +rData[0];
}

/********************************************************
 * LS_THRES_LOW (0/1/2)
 * max 20-bits unsigned integer
 ********************************************************/
void apds9250_set_ls_thres_low(uint32_t thres)
{
	uint8_t wData[4] = {APDS9250_LS_THRES_LOW_0_ADDR, thres&0xFF, (thres>>8)&0xFF, (thres>>16)&0x0F};
	
	//L_DEBUG("thres_low = [%X, %X, %X]", wData[3], wData[2], wData[1]);
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 4, 100);
}

uint32_t apds9250_get_ls_thres_low(void)
{
	uint8_t rData[3];
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_THRES_LOW_0_ADDR, 1, rData, 3, 100);
	return (rData[2]<<16) + (rData[1]<<8) +rData[0];
}

/********************************************************
 * LS_THRES_VAR
 * INT set when difference between previous and current
 * is above variance threshold (LS_THRES_VAR) for specified number of
 * consecutive mesurements (INT_PERSISTENCE)
 ********************************************************/
void apds9250_set_ls_thres_var(uint8_t vary)
{
	uint8_t wData[2] = {APDS9250_THRES_VAR_ADDR, vary&0x07};
	HAL_I2C_Master_Transmit(&apds9250_i2c, APDS9250_I2C_ADDR, wData, 2, 100);
}

uint8_t apds9250_get_ls_thres_var(void)
{
	uint8_t thres_var_reg;
	
	HAL_I2C_Mem_Read(&apds9250_i2c, APDS9250_I2C_ADDR, APDS9250_LS_THRES_LOW_0_ADDR, 1, &thres_var_reg, 1, 100);
	return (thres_var_reg & 0x07);
}
