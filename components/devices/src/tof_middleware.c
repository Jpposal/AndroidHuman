#include "tof_middleware.h"
#include "ist8310driver_middleware.h"

// 设备ID到I2C地址的映射
uint8_t tof_get_i2c_address(uint8_t device_id)
{
    uint8_t addr = TOF_BASE_ADDR + device_id;
    return addr << 1;
}

uint8_t tof_IIC_read_single_reg_by_id(uint8_t device_id, uint8_t reg)
{
    uint8_t res = 0xFF;
    uint8_t i2c_addr = tof_get_i2c_address(device_id);
    HAL_I2C_Mem_Read(&hi2c2, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 100);
    return res;
}

void tof_IIC_write_single_reg_by_id(uint8_t device_id, uint8_t reg, uint8_t data)
{
    uint8_t i2c_addr = tof_get_i2c_address(device_id);
    HAL_I2C_Mem_Write(&hi2c2, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void tof_IIC_read_multi_reg_by_id(uint8_t device_id, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i2c_addr = tof_get_i2c_address(device_id);
    for(int i = 0; i < len; i++) {
        buf[i] = 0xFF;
    }
    HAL_I2C_Mem_Read(&hi2c2, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

void tof_IIC_write_multi_reg_by_id(uint8_t device_id, uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i2c_addr = tof_get_i2c_address(device_id);
    HAL_I2C_Mem_Write(&hi2c2, i2c_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

void tof_delay_ms(uint16_t ms)
{
    ist8310_delay_ms(ms);
}