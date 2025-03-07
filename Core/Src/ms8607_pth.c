#include "ms8607_pth.h"
#include "logger.c"
// #include "cmsis_os2.h"

#if defined(picoSTM32H753) || defined(PICOSTM_COMBO) || defined(PICOSTM_COMBO_REV_B)
  extern I2C_HandleTypeDef hi2c3;
  #define ms8607_i2c hi2c3
#endif

// Modify the below defines to change the resolution and its corresponding delay
#define MS8607_PT_OSR   MS8607_PT_OSR_256
#define MS8607_RH_OSR   MS8607_RH_OSR_256

#if (MS8607_PT_OSR == MS8607_PT_OSR_8192)
  #define MS8607_PT_CONVERT_DELAY 	MS8607_PT_CONVERT_TIME_OSR_8192
  #define MS8607_PT_OSR_D1_CMD      MS8607_PT_OSR_8192_D1_CMD
  #define MS8607_PT_OSR_D2_CMD      MS8607_PT_OSR_8192_D2_CMD
#elif (MS8607_PT_OSR == MS8607_PT_OSR_4096)
  #define MS8607_PT_CONVERT_DELAY 	MS8607_PT_CONVERT_TIME_OSR_4096
  #define MS8607_PT_OSR_D1_CMD      MS8607_PT_OSR_4096_D1_CMD
  #define MS8607_PT_OSR_D2_CMD      MS8607_PT_OSR_4096_D2_CMD
#elif (MS8607_PT_OSR == MS8607_PT_OSR_2048)
  #define MS8607_PT_CONVERT_DELAY 	MS8607_PT_CONVERT_TIME_OSR_2048
  #define MS8607_PT_OSR_D1_CMD      MS8607_PT_OSR_2048_D1_CMD
  #define MS8607_PT_OSR_D2_CMD      MS8607_PT_OSR_2048_D2_CMD
#elif (MS8607_PT_OSR == MS8607_PT_OSR_1024)
  #define MS8607_PT_CONVERT_DELAY 	MS8607_PT_CONVERT_TIME_OSR_1024
  #define MS8607_PT_OSR_D1_CMD      MS8607_PT_OSR_1024_D1_CMD
  #define MS8607_PT_OSR_D2_CMD      MS8607_PT_OSR_1024_D2_CMD
#elif (MS8607_PT_OSR == MS8607_PT_OSR_512)
  #define MS8607_PT_CONVERT_DELAY 	MS8607_PT_CONVERT_TIME_OSR_512
  #define MS8607_PT_OSR_D1_CMD      MS8607_PT_OSR_512_D1_CMD
  #define MS8607_PT_OSR_D2_CMD      MS8607_PT_OSR_512_D2_CMD
#else // set to lowest resolution if 256 or none of the above
  #define MS8607_PT_CONVERT_DELAY 	MS8607_PT_CONVERT_TIME_OSR_256
  #define MS8607_PT_OSR_D1_CMD      MS8607_PT_OSR_256_D1_CMD
  #define MS8607_PT_OSR_D2_CMD      MS8607_PT_OSR_256_D2_CMD
#endif


#if (MS8607_RH_OSR == MS8607_RH_OSR_4096)
  #define MS8607_RH_CONVERT_DELAY 	MS8607_RH_CONVERT_TIME_OSR_4096
#elif (MS8607_RH_OSR == MS8607_RH_OSR_2048)
  #define MS8607_RH_CONVERT_DELAY 	MS8607_RH_CONVERT_TIME_OSR_2048
#elif (MS8607_RH_OSR == MS8607_RH_OSR_1024)
  #define MS8607_RH_CONVERT_DELAY 	MS8607_RH_CONVERT_TIME_OSR_1024
#else // set to lowest resolution if 256 or none of the above
  #define MS8607_RH_CONVERT_DELAY 	MS8607_RH_CONVERT_TIME_OSR_256
#endif

#define MS8607_RH_BATT_STATE			MS8607_RH_VDD_MORE_2_25
#define MS8607_RH_HEATER					MS8607_RH_HEATER_DISABLED

ms8607_pt_calib_t calib;

bool ms8607_init(void)
{
  ms8607_reset_pt(); // set once after power-on to make sure calibration PROM is loaded
  if(HAL_I2C_IsDeviceReady(&ms8607_i2c, MS8607_PT_I2C_ADDR, 2, 1000) != HAL_OK){
		printf("ERROR: Cannot initialize PTH Sensor. Device not ready (PT)");
		return false;
	}
  if(HAL_I2C_IsDeviceReady(&ms8607_i2c, MS8607_RH_I2C_ADDR, 2, 1000) != HAL_OK){
		printf("ERROR: Cannot initialize PTH Sensor. Device not ready (RH)");
		return false;
	}
  
	// Read calibration data from PROM after power-on or reset
	if(!ms8607_prom_read_all_addr_pt(&calib)) {
		printf("ERROR: Failed to read PT PROM\r\n");
		return false;
	}
//	log_debug(__func__, "crc & FD= %04X", calib.crc);
//	log_debug(__func__, "c1 = %04X -> SENS", calib.sens);
//	log_debug(__func__, "c2 = %04X -> OFF", calib.off);
//	log_debug(__func__, "c3 = %04X -> TCS", calib.tcs);
//	log_debug(__func__, "c4 = %04X -> TCO", calib.tco);
//	printf("INFO: c5 = %04X -> TREF\r\n", calib.tref);
//	log_debug(__func__, "c6 = %04X -> TEMPSENS", calib.tempsens);
  
  // Configure User Register
	uint8_t user_reg = ms8607_read_usr_reg_rh();
//  log_debug(__func__, "user_reg = %02X", user_reg);
	ms8607_write_usr_reg_rh(MS8607_RH_OSR, MS8607_RH_BATT_STATE, MS8607_RH_HEATER);

	printf("Pressure/Temperature/Humidity sensor initialization completed\r\n");
  return true;
}

//void ms8607_get_pth(COT_DATA *data)
//{
//}

/********************************************************************
 *                    PRESSURE AND TEMPERAUTRE
 ********************************************************************/
/********************************************************************
 * Reset Sequence
 ********************************************************************/
void ms8607_reset_pt(void)
{
  //L_INFO("Resetting the Pressure/Temperature sensor");
	uint8_t wData = MS8607_PT_RESET_CMD;
	HAL_I2C_Master_Transmit(&ms8607_i2c, MS8607_PT_I2C_ADDR, &wData, 1, 100);
//	printf("%d",HAL_I2C_Master_Transmit(&ms8607_i2c, MS8607_PT_I2C_ADDR, &wData, 1, 100));
}

/********************************************************************
 * PROM Read P&T Sequence
 ********************************************************************/
uint16_t ms8607_prom_read_one_addr_pt(uint8_t prom_addr)
{
	//uint8_t wData;
	uint8_t rData[2];
	
	HAL_I2C_Master_Transmit(&ms8607_i2c, MS8607_PT_I2C_ADDR, &prom_addr, 1, 100);
	HAL_I2C_Master_Receive(&ms8607_i2c, MS8607_PT_I2C_ADDR, rData, 2, 100);
//	log_debug(__func__, "rData[0] = %02X | ", rData[0]);
//	log_debug(__func__, "rData[1] = %02X | ", rData[1]);
//	log_debug(__func__, "PROM[ %02X ] = %04X", prom_addr, (uint16_t)(rData[0]<<8) + (uint16_t)rData[1]);
	return (uint16_t)(rData[0]<<8) + (uint16_t)rData[1];
}

bool ms8607_prom_read_all_addr_pt(ms8607_pt_calib_t *calib)
{
	uint16_t temp[8];
	uint8_t crc_check;
	uint8_t crc;
	
	temp[0] = ms8607_prom_read_one_addr_pt(0xA0);
	temp[1] = ms8607_prom_read_one_addr_pt(0xA2);
	temp[2] = ms8607_prom_read_one_addr_pt(0xA4);
	temp[3] = ms8607_prom_read_one_addr_pt(0xA6);
	temp[4] = ms8607_prom_read_one_addr_pt(0xA8);
	temp[5] = ms8607_prom_read_one_addr_pt(0xAA);
	temp[6] = ms8607_prom_read_one_addr_pt(0xAC);
//	printf("%04X\r\n%04X\r\n%04X\r\n%04X\r\n%04X\r\n%04X\r\n%04X\r\n",temp[0],temp[1],temp[2],
//			temp[3],temp[4],temp[5],temp[6]);
	
	// check CRC
	crc = ((temp[0] & 0xF000)>>12);
//	printf("real crc= %4X\r\n",crc);

	crc_check = ms8607_crc4_pt(temp);

//	printf("old crc= %4X\r\n",crc);

//	printf("PT CRC (%02X) =? remainder (%02X)\r\n", crc, crc_check);
	if(crc_check != crc) {
		printf("ERROR: PT CRC (%02X) does not match remainder (%02X)\r\n", crc, crc_check);
		return false;
	} 
	memcpy(calib, temp, sizeof(temp));
	return true;
}

/********************************************************************
 * Conversion Sequence
 ********************************************************************/
uint32_t ms8607_read_adc_pt(uint8_t convert_cmd)
{
	uint8_t wData;
	uint8_t rData[3];
	
	// Send command to start conversion
	HAL_I2C_Master_Transmit(&ms8607_i2c, MS8607_PT_I2C_ADDR, &convert_cmd, 1, 100);
	// Wait for conversion to finish
	osDelay(MS8607_PT_CONVERT_DELAY);
	// Read ADC
	wData = MS8607_PT_ADC_READ_CMD;
	HAL_I2C_Master_Transmit(&ms8607_i2c, MS8607_PT_I2C_ADDR, &wData, 1, 100);
	HAL_I2C_Master_Receive(&ms8607_i2c, MS8607_PT_I2C_ADDR, rData, 3, 100);
	
	return ((uint32_t)rData[0]<<16) + ((uint32_t)rData[1]<<8)+ (uint32_t)rData[2];
}

bool ms8607_get_pt(COT_DATA *data)
{
//	ms8607_pt_calib_t calib;
	uint32_t d1, d2;
	int32_t dT, T2;
	int64_t off, sens, off2, sens2;
	int32_t temp, pres;
	
	if(HAL_I2C_IsDeviceReady(&ms8607_i2c, MS8607_PT_I2C_ADDR, 2, 1000) != HAL_OK){
		printf("ERROR: PTH Sensor not ready (PT)");
		return false;
	}
	
	// Read digital pressure and temperature data
	d1 = ms8607_read_adc_pt(MS8607_PT_OSR_D1_CMD);
	d2 = ms8607_read_adc_pt(MS8607_PT_OSR_D2_CMD);
//	printf("P_ADC = %X\r\n", d1);
//	printf("T_ADC = %X\r\n", d2);
	
	// Calculate temperature
	dT = (int32_t)d2 - ((int32_t)calib.tref << 8);
//	printf("dT = %d\r\n", dT);
	temp = 2000 + (((int64_t)dT * (int64_t)calib.tempsens) >> 23);
	
//	data->pth.temperature  = (float)temp / 100;
//	log_debug(__func__, "temperature = %f degC", data->pth.temperature );
	
	// Calculate temperature compensated pressure (first order)
	off = ((int64_t)calib.off << 17) + ((int64_t)(calib.tco * dT) >> 6);
	sens = ((int64_t)calib.sens << 16) + ((int64_t)(calib.tcs * dT) >> 7);
//	log_debug(__func__, "off = %lld", off);
//	log_debug(__func__, "sens = %lld", sens);
	
	// Calculate temperature compensated pressure (second order)
	if(temp < 2000) { // 20 degC
		T2 = (3 * (int64_t)dT * (int64_t)dT) >> 33;
		off2 = (61 * (int64_t)(temp - 2000) * (int64_t)(temp - 2000)) >> 4;
		sens2 = (29 * (int64_t)(temp - 2000) * (int64_t)(temp- 2000)) >> 4;
//		log_debug(__func__, "<20 : T2 = %d", T2);
//		log_debug(__func__, "<20 : OFF2 = %lld", off2);
//		log_debug(__func__, "<20 : SENS2 = %lld", sens2);
		if(temp < -1500) { // -15 degC
			off2 += (17 * (int64_t)(temp + 1500) * (int64_t)(temp + 1500));
			sens2 += (9 * (int64_t)(temp + 1500) * (int64_t)(temp + 1500));
//			log_debug(__func__, "<-15 : OFF2 = %lld", off2);
//			log_debug(__func__, "<-15 : SENS2 = %lld", sens2);
		}
	} else {
		T2 = (5 * (int64_t)dT * (int64_t)dT) >> 38;
//		log_debug(__func__, ">20 : T2 = %d", T2);
		off2 = 0;
		sens2 = 0;
	}
//	log_debug(__func__, "T2 = %d", T2);
//	log_debug(__func__, "OFF2 = %lld", off2);
//	log_debug(__func__, "SENS2 = %lld", sens2);
	
	temp -= T2;
	off -= off2;
	sens -= sens2;
	
	data->pth.temperature = (float)temp/ 100;
//	log_debug(__func__, "temperature (2nd order) = %f (%d)", data->pth.temperature , temp);
	
	pres = (((int64_t)(d1 * sens) >> 21) - off) >> 15;
	data->pth.pressure = (float)pres / 100;
//	log_debug(__func__, "pressure = %f (%d)", data->pth.pressure, pres);
	// atmospheric pressure ~ 1013.25 mbar
	
	return true;
}

float ms8607_degC_to_degF(float degC)
{
	return degC * 9/5 + 32;
}

/********************************************************************
 *                        RELATIVE HUMIDITY
 ********************************************************************/
/********************************************************************
 * Reset Sequence
 ********************************************************************/
void ms8607_reset_rh(void)
{
  //L_INFO("Resetting the Humidity sensor");
	uint8_t wData = MS8607_RH_RESET_CMD;
	HAL_I2C_Master_Transmit(&ms8607_i2c, MS8607_RH_I2C_ADDR, &wData, 1, 100);
	osDelay(15);
}

/********************************************************************
 * Read and Wrtie to User Register Sequence
 ********************************************************************/
void ms8607_write_usr_reg_rh(uint8_t res, uint8_t batt, uint8_t heat)
{
	uint8_t rData;
	uint8_t usr_reg;
	uint8_t wData[2] = {MS8607_RH_WRTIE_USER_REG_CMD, 0};
	
	// Read
	HAL_I2C_Mem_Read(&ms8607_i2c, MS8607_RH_I2C_ADDR, MS8607_RH_READ_USER_REG_CMD, 1, &rData, 1, 100);
	wData[0] = MS8607_RH_WRTIE_USER_REG_CMD;
	usr_reg = res | batt | heat;
	wData[1] = usr_reg;
//	log_debug(__func__, "res = %X, batt = %X, heat = %X, user_reg = %X", res, batt, heat, wData[1]);
	
	// Write
	HAL_I2C_Master_Transmit(&ms8607_i2c, MS8607_RH_I2C_ADDR, wData, 2, 100);
}

uint8_t ms8607_read_usr_reg_rh(void)
{
	uint8_t rData;
	
	HAL_I2C_Mem_Read(&ms8607_i2c, MS8607_RH_I2C_ADDR, MS8607_RH_READ_USER_REG_CMD, 1, &rData, 1, 100);
//	log_debug(__func__, "user_reg = %02X", rData);
	return rData;
}

/********************************************************************
 * Measure RH Hold/No Hold Sequence (using no hold)
 ********************************************************************/
 bool ms8607_read_adc_rh(uint16_t *adc_rh)
{
	uint8_t wData = MS8607_RH_MEAS_NO_HOLD_CMD;
	uint8_t rData[3];
	uint8_t checksum, check;
	
	HAL_I2C_Master_Transmit(&ms8607_i2c, MS8607_RH_I2C_ADDR, &wData, 1, 100);
	osDelay(MS8607_RH_CONVERT_DELAY);
	HAL_I2C_Master_Receive(&ms8607_i2c, MS8607_RH_I2C_ADDR, rData, 3, 1000);
	
	*adc_rh = (uint16_t)(rData[0]<<8) + (uint16_t)rData[1];
	checksum = rData[2];
//	log_debug(__func__, "checksum = %02X", checksum);
//	log_debug(__func__, "adc_rh = %04X", *adc_rh);
//	
	check = ms8607_checksum_rh(*adc_rh);
//	log_debug(__func__, "check = %02X", check);
	if(check != checksum) {
		//L_ERROR("ERROR: RH CRC (%02X) does not match remainder (%02X)", checksum, check);;
		return false;
	}
	return true;
}

bool ms8607_get_rh(COT_DATA *data)
{
	uint16_t d3;
	
	if(HAL_I2C_IsDeviceReady(&ms8607_i2c, MS8607_RH_I2C_ADDR, 2, 1000) != HAL_OK){
		//L_ERROR("ERROR: PTH Sensor not ready (RH)");
		return false;
	}
	
	// Read digital relative humidity data
	if(!ms8607_read_adc_rh(&d3)){
		//L_ERROR("ERROR: Failed to read RH ADC");
		return false;
	}
	
	// Note: adc_humidity is 14-bits where 2 LSB are status bits
//	log_debug(__func__, "D3 = %X", d3);
//	uint8_t status = d3 & 0x0003;
	//d3 >>= 2;
//	log_debug(__func__, "H_ADC = %X", d3);
//	log_debug(__func__, "status = %X", status);
	
	// Calculate relative humidity (in %RH)
	data->pth.humidity = ((float)(125 * d3) / 65536) - 6;
//	log_debug(__func__, "humidity = %f &&", data->pth.humidity);
	
	// Calculate compensated relative humidity
	data->pth.humidity += (20 - (data->pth.temperature)) * (-0.18);
	
//	log_debug(__func__, "humidity (compensated) = %f %%", data->pth.humidity);
	
	return true;
}

/********************************************************************
 * PROM Read RH Sequence
 ********************************************************************/
uint16_t ms8607_prom_read_rh(uint8_t prom_addr)
{
	uint8_t rData[2];
	
	HAL_I2C_Master_Transmit(&ms8607_i2c, MS8607_RH_I2C_ADDR, &prom_addr, 1, 100);
	HAL_I2C_Master_Receive(&ms8607_i2c, MS8607_RH_I2C_ADDR, rData, 2, 100);
	return (uint16_t)(rData[0]<<8) + (uint16_t)rData[1];
}

/********************************************************************
 * CRC (Given in datasheet)
 ********************************************************************/
uint8_t ms8607_crc4_pt(uint16_t *n_prom) 	// n_prom defined as 8x unsigned int (n_prom[8])
{ 
	uint8_t cnt; 																// simple counter
	uint16_t n_rem=0; 										// crc remainder
	uint8_t n_bit;
	uint16_t p0 = n_prom[0];
	n_prom[0]=((n_prom[0]) & 0x0FFF); 						// CRC byte is replaced by 0
	n_prom[7]=0;    										// Subsidiary value, set to 0
	for (cnt = 0; cnt < 16; cnt++) 							// operation is performed on bytes
	{ 														// choose LSB or MSB
		if (cnt%2==1) 
				n_rem ^= ((n_prom[cnt>>1]) & 0x00FF);
		else 
				n_rem ^= (n_prom[cnt>>1]>>8);
		for (n_bit = 8; n_bit > 0; n_bit--) 
		{ 
			if (n_rem & (0x8000)) 
					n_rem = (n_rem << 1) ^ 0x3000;
			else 
					n_rem = (n_rem << 1);
		} 
	} 
	n_rem = ((n_rem >> 12) & 0x000F); 					// final 4-bit remainder is CRC code
	n_prom[0] = p0;															// restore n_prom[0]
	return (n_rem ^ 0x00);
}

#if 0
// Note: Reading RH PROM is not required to calculate RH
uint8_t ms8607_crc4_rh(uint16_t *n_prom) 	// n_prom defined as 8x unsigned int (n_prom[8])
{ 
	uint8_t cnt; 																// simple counter
	uint16_t n_rem=0; 													// crc remainder
	uint8_t n_bit;
	uint16_t p6 = n_prom[6];
	
	n_prom[6]=((n_prom[6]) & 0xFFF0); 					// CRC byte is replaced by 0 
	n_prom[7]=0; 																// Subsidiary value, set to 0
	for (cnt = 0; cnt < 16; cnt++) 							// operation is performed on bytes
	{ 																					// choose LSB or MSB
		if (cnt%2==1) 
			n_rem ^= ((n_prom[cnt>>1]) & 0x00FF); 
		else 
			n_rem ^= (n_prom[cnt>>1]>>8); 
		for (n_bit = 8; n_bit > 0; n_bit--) 
		{ 
			if (n_rem & (0x8000)) 
				n_rem = (n_rem << 1) ^ 0x3000; 
			else 
				n_rem = (n_rem << 1); 
		} 
	} 
	n_rem= ((n_rem >> 12) & 0x000F); 						// final 4-bit remainder is CRC code
	n_prom[0] = p6;															// restore n_prom[6]
	return (n_rem ^ 0x00);
}
#endif

// Note: Was not in datasheet, but found in TE drivers in
//			 https://github.com/TEConnectivity/MS8607_Generic_C_Driver
uint8_t ms8607_checksum_rh(uint16_t rh)
{
	uint32_t polynom = 0x988000;
	uint32_t msb     = 0x800000;
	uint32_t mask    = 0xFF8000;
	uint32_t result  = (uint32_t)rh<<8;
	
	while( msb != 0x80 ) {
		
		// Check if msb of current value is 1 and apply XOR mask
		if( result & msb )
			result = ((result ^ polynom) & mask) | ( result & ~mask);
			
		msb >>= 1;
		mask >>= 1;
		polynom >>=1;
	}
	return (uint8_t)result;
}
