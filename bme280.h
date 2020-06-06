/*
 * bme280.h
 *
 *  Created on: May 15, 2020
 *      Author: KamilB
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include <stdio.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

//gpio pin var is cs pin. sensors walks only in spi mode. Gonna update it for i2c soon(or maybe not so soon, cause I am actually pretty busy)
typedef struct BME280{
	int32_t calib_data[18];
	int t_fine;
	uint16_t GPIO_PIN;
	GPIO_TypeDef * GPIO_PORT;
} bme280;


void BME_write8(uint8_t val, uint8_t reg, bme280 bme);
uint8_t BME_read8(uint8_t reg, bme280 bme);
uint16_t BME_read16(uint8_t reg, bme280 bme);
uint16_t BME_read16_LE(uint8_t reg, bme280 bme);
int16_t BME_readS16_LE(uint8_t reg, bme280 bme);
uint32_t BME_read24(uint8_t reg, bme280 bme);
int BME_read_temp(bme280 *bme);
int BME_read_press(bme280 *bme);
int BME_read_hum(bme280 *bme);
bool BME_begin(bme280 bme);
void BME_sample(bme280 bme);
void BME_read_coefficients(bme280 *bme);
extern SPI_HandleTypeDef hspi2;
#define SPI_HANDLE &hspi2;
#define CS_PORT GPIOC
#define CS_PIN_LIST_SIZE 4
#define CS_PIN GPIO_PIN_9
#define CS_PIN_2 GPIO_PIN_10
#define CS_PIN_3 GPIO_PIN_11
#define CS_PIN_4 GPIO_PIN_12

#define MODE_SLEEP  0x00
#define MODE_FORCED  0x01;
#define MODE_NORMAL 0x03
#define MODE_SOFT_RESET_CODE 0xB6
#define REGISTER_RESET 0xE0
#define REGISTER_CONTROL_H 0xF2
#define REGISTER_CONFIG 0xF5
#define REGISTER_CONTROL 0xF4
#define REGISTER_CHIPID  0xD0
#define REGISTER_TEMPDATA 0xFA
#define REGISTER_HUMDATA 0xFD
#define REGISTER_PRESSDATA 0xF7
#define REGISTER_DIG_T1  0x88
#define REGISTER_DIG_T2  0x8A
#define REGISTER_DIG_T3  0x8C
#define REGISTER_DIG_P1 0x8E
#define REGISTER_DIG_P2 0x90
#define REGISTER_DIG_P3 0x92
#define REGISTER_DIG_P4 0x94
#define REGISTER_DIG_P5 0x96
#define REGISTER_DIG_P6 0x98
#define REGISTER_DIG_P7 0x9A
#define REGISTER_DIG_P8 0x9C
#define REGISTER_DIG_P9 0x9E
#define REGISTER_DIG_H1 0xA1
#define REGISTER_DIG_H2 0xE1
#define REGISTER_DIG_H3 0xE3
#define REGISTER_DIG_H4 0xE4
#define REGISTER_DIG_H5 0xE5
#define REGISTER_DIG_H5_2 0xE6
#define REGISTER_DIG_H6 0xE7



#define CHIPID 0x60

#endif /* INC_BME280_H_ */
