#ifndef __TOF_MIDDLEWARE_H__
#define __TOF_MIDDLEWARE_H__

#include "stm32f4xx_hal.h"
#include "tof_driver.h"

extern I2C_HandleTypeDef hi2c2;

uint8_t tof_IIC_read_single_reg_by_id(uint8_t device_id, uint8_t reg);
void tof_IIC_write_single_reg_by_id(uint8_t device_id, uint8_t reg, uint8_t data);
void tof_IIC_read_multi_reg_by_id(uint8_t device_id, uint8_t reg, uint8_t *buf, uint8_t len);
void tof_IIC_write_multi_reg_by_id(uint8_t device_id, uint8_t reg, uint8_t *data, uint8_t len);
void tof_delay_ms(uint16_t ms);
uint8_t tof_get_i2c_address(uint8_t device_id);

#endif