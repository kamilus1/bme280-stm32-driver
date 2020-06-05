/*
 * bme280.c
 *
 *  Created on: May 15, 2020
 *      Author: KamilB
 */

#include "bme280.h"

void BME_Write8(uint8_t val, uint8_t reg, bme280 bme ){
	reg = reg & ~0x80;
	uint8_t tx_buf[] = {reg, val};
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, tx_buf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_SET);
}
uint8_t BME_read8(uint8_t reg,  bme280 bme ){
	uint8_t value;
	reg = reg |0x80;
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, &value, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_SET);
	return value;
}
uint16_t BME_read16(uint8_t reg, bme280 bme){
	uint16_t value;
	uint8_t byte;
	reg = reg |0x80;
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, &byte, 1, HAL_MAX_DELAY);
	value = byte;
	value <<= 8;
	HAL_SPI_Receive(&hspi2, &byte, 1, HAL_MAX_DELAY);
	value |= byte;
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_SET);
	return value;
}
uint32_t BME_read24(uint8_t reg, bme280 bme){
	uint32_t value;
	uint8_t byte;
	reg = reg |0x80;
				HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_RESET);
				HAL_SPI_Transmit(&hspi2, &reg, 1, HAL_MAX_DELAY);
				HAL_SPI_Receive(&hspi2, &byte, 1, HAL_MAX_DELAY);
				value = byte;
				value <<= 8;
				HAL_SPI_Receive(&hspi2, &byte, 1, HAL_MAX_DELAY);
				value |= byte;
				value <<= 8;
				HAL_SPI_Receive(&hspi2, &byte, 1, HAL_MAX_DELAY);
				value |= byte;
				HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_SET);

	return value;
}
bool BME_begin(bme280 bme){
	uint8_t rslt;
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_RESET);
	BME_Write8(MODE_SOFT_RESET_CODE, REGISTER_RESET, bme);
	HAL_Delay(500);
	if(BME_read8(REGISTER_CHIPID, bme)!=	CHIPID){
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_SET);
							return false;
						}
	HAL_GPIO_WritePin(bme.GPIO_PORT, bme.GPIO_PIN, GPIO_PIN_SET);

	return true;
}
void BME_sample(bme280 bme){
	/*filer and standby set */
	uint8_t config_value=0x00, control_value=0x01;
	config_value <<=5;
	config_value += (0x04<<2);
	BME_Write8(config_value, REGISTER_CONFIG, bme);
	// osrs_h config
		control_value = 0b100;
		BME_Write8(control_value, REGISTER_CONTROL_H, bme);
	/* osrs_t osrs_p and mode config */
	control_value <<= 5;
	control_value += (0x03<<2);
	control_value += 0x03;
	BME_Write8(control_value, REGISTER_CONTROL, bme);

}


uint16_t BME_read16_LE(uint8_t reg, bme280 bme){
	uint16_t temp = BME_read16(reg, bme);
	return (temp>>8)|(temp<<8);
}
int16_t BME_readS16_LE(uint8_t reg, bme280 bme){
	int16_t temp = BME_read16_LE(reg, bme);
	return temp;
}
void BME_read_coefficients(bme280 *bme){
		int32_t x, y;
		bme->calib_data[0] = BME_read16_LE(REGISTER_DIG_T1, *bme);
		bme->calib_data[1] = BME_readS16_LE(REGISTER_DIG_T2, *bme);
		bme->calib_data[2] = BME_readS16_LE(REGISTER_DIG_T3, *bme);

		bme->calib_data[3] = BME_read16_LE(REGISTER_DIG_P1, *bme);
		bme->calib_data[4] = BME_readS16_LE(REGISTER_DIG_P2, *bme);
		bme->calib_data[5] = BME_readS16_LE(REGISTER_DIG_P3, *bme);
		bme->calib_data[6] = BME_readS16_LE(REGISTER_DIG_P4, *bme);
		bme->calib_data[7] = BME_readS16_LE(REGISTER_DIG_P5, *bme);
		bme->calib_data[8] = BME_readS16_LE(REGISTER_DIG_P6, *bme);
		bme->calib_data[9] = BME_readS16_LE(REGISTER_DIG_P7, *bme);
		bme->calib_data[10] = BME_readS16_LE(REGISTER_DIG_P8, *bme);
		bme->calib_data[11] = BME_readS16_LE(REGISTER_DIG_P9, *bme);
		bme->calib_data[12] = BME_read8(REGISTER_DIG_H1, *bme);
		bme->calib_data[13] = BME_readS16_LE(REGISTER_DIG_H2, *bme);
		bme->calib_data[14] = BME_read8(REGISTER_DIG_H3, *bme);
		x = BME_read8(REGISTER_DIG_H4, *bme);
		y = BME_read8(REGISTER_DIG_H5, *bme);
		bme->calib_data[15] = (x<<4)|(y&0xF);
		x =BME_read8(REGISTER_DIG_H5_2, *bme);
		bme->calib_data[16] = (x<<4)|(y>>4);
		bme->calib_data[17] = BME_read8(REGISTER_DIG_H6, *bme);
}
int BME_read_temp(bme280 *bme){

	int var1, var2;
	int adc_t = BME_read24(REGISTER_TEMPDATA, *bme);

	adc_t >>= 4;
	var1 =((((adc_t>>3)-((int32_t)bme->calib_data[0]<<1)))*((int32_t)bme->calib_data[1])) >>11;
	var2 = (((((adc_t >> 4) - ((int32_t)bme->calib_data[0])) *
	            ((adc_t >> 4) - ((int32_t)bme->calib_data[0]))) >>
	           12) *
	          ((int32_t)bme->calib_data[2])) >>
	         14;
	  bme->t_fine = var1 + var2;

	  float T = (bme->t_fine * 5 + 128) >> 8;
	  return T/100;
}
int BME_read_press(bme280 *bme){
	int64_t var1, var2, p;

	  // Must be done first to get the t_fine variable set up
	  BME_read_temp(bme);

	  int32_t adc_P = BME_read24(REGISTER_PRESSDATA, *bme);
	  adc_P >>= 4;

	  var1 = ((int64_t)bme->t_fine) - 128000;
		  var2 = var1 * var1 * (int64_t)bme->calib_data[8];
		  var2 = var2 + ((var1 * (int64_t)bme->calib_data[7]) << 17);
		  var2 = var2 + (((int64_t)bme->calib_data[6]) << 35);
		  var1 = ((var1 * var1 * (int64_t)bme->calib_data[5]) >> 8) +
				 ((var1 * (int64_t)bme->calib_data[4]) << 12);
		  var1 =
			  (((((int64_t)1) << 47) + var1)) * ((int64_t)bme->calib_data[3]) >> 33;

		  if (var1 == 0) {
			return 0; // avoid exception caused by division by zero
		  }
		  p = 1048576 - adc_P;
		  p = (((p << 31) - var2) * 3125) / var1;
		  var1 = (((int64_t)bme->calib_data[11]) * (p >> 13) * (p >> 13)) >> 25;
		  var2 = (((int64_t)bme->calib_data[10]) * p) >> 19;

		  p = ((p + var1 + var2) >> 8) + (((int64_t)bme->calib_data[9]) << 4);
	  return p / 256;
}
int BME_read_hum(bme280 *bme){
	int32_t adc_h, v_x1_u32r, h;

	BME_read_temp(bme);
	adc_h = BME_read16(REGISTER_HUMDATA, *bme);
	if(adc_h==0x8000){
		return -1;
	}
	v_x1_u32r = (bme->t_fine-((int32_t)76800));
	v_x1_u32r = (((((adc_h<<14)-(((int32_t)bme->calib_data[15])<<20)-(((int32_t)bme->calib_data[16])*v_x1_u32r))
			+((int32_t)16384))>>15)*(((((((v_x1_u32r*((int32_t)bme->calib_data[17]))>>10)*(((v_x1_u32r*((int32_t)bme->calib_data[14]))>>11)
					+((int32_t)32768)))>>10)+((int32_t)2097152))*((int32_t)bme->calib_data[13])+8192)>>14));
	v_x1_u32r = (v_x1_u32r -(((((v_x1_u32r>>15)*(v_x1_u32r>>15))>>7)*((int32_t)bme->calib_data[12]))>>4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0: v_x1_u32r);
	v_x1_u32r = (v_x1_u32r>419430400 ? 41930400: v_x1_u32r);
	h = v_x1_u32r>>12;
	h /= 1024;
	return h;

}
