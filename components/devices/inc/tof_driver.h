#ifndef __TOF_DRIVER_H__
#define __TOF_DRIVER_H__

#include "stm32f4xx_hal.h"
#include "stdint.h"

// 支持的TOF设备数量和地址
#define TOF_MAX_DEVICES             4       // 最大支持4个TOF设备
#define TOF_BASE_ADDR               0x08    // 基础7位从机地址
#define TOF_REGISTER_TOTAL_SIZE     48      // 寄存器总长度

// TOF设备ID枚举
typedef enum {
    TOF_DEVICE_0 = 0,    // 地址: 0x08
    TOF_DEVICE_1 = 1,    // 地址: 0x09  
    TOF_DEVICE_2 = 2,    // 地址: 0x10
    TOF_DEVICE_3 = 3,    // 地址: 0x11
} tof_device_id_t;

// 寄存器地址定义（相对地址，适用于所有设备）
#define TOF_ADDR_MODE               0x0c    
#define TOF_SIZE_MODE               1       
#define TOF_ADDR_ID                 0x0d    
#define TOF_SIZE_ID                 1       
#define TOF_ADDR_UART_BAUDRATE      0x10    
#define TOF_SIZE_UART_BAUDRATE      4       
#define TOF_ADDR_SYSTEM_TIME        0x20    
#define TOF_SIZE_SYSTEM_TIME        4       
#define TOF_ADDR_DIS                0x24    
#define TOF_SIZE_DIS                4       
#define TOF_ADDR_DIS_STATUS         0x28    
#define TOF_SIZE_DIS_STATUS         2       
#define TOF_ADDR_SIGNAL_STRENGTH    0x2a    
#define TOF_SIZE_SIGNAL_STRENGTH    2       
#define TOF_ADDR_RANGE_PRECISION    0x2c    
#define TOF_SIZE_RANGE_PRECISION    1       
#define IIC_CHANGE_TO_UART_DATA     0x00    

// TOF数据结构
typedef struct {
    uint8_t device_id;          // 设备ID
    uint8_t i2c_address;        // I2C地址
    uint32_t system_time;       // 系统时间，单位：ms
    uint32_t distance;          // 距离，单位:mm
    uint16_t status;            // 距离状态指示:0为无效,1为有效
    uint16_t signal_strength;   // 信号强度
    uint8_t range_precision;    // 测距精度，单位:cm
    uint8_t interface_mode;     // 通讯接口模式
    uint32_t uart_baudrate;     // 串口波特率
    uint8_t id;                 // 模块ID
    uint8_t is_online;          // 设备在线状态
} tof_data_t;

// 多TOF管理器结构
typedef struct {
    tof_data_t devices[TOF_MAX_DEVICES];    // 设备数组
    uint8_t device_count;                   // 有效设备数量
    uint8_t scan_complete;                  // 扫描完成标志
} tof_manager_t;

// 函数声明
uint8_t tof_init_device(uint8_t device_id);
uint8_t tof_scan_all_devices(void);
void tof_read_all_data_by_id(uint8_t device_id, tof_data_t *tof_data);
void tof_read_all_devices_data(void);
uint32_t tof_read_distance_by_id(uint8_t device_id);
uint8_t tof_get_device_count(void);
tof_data_t* tof_get_device_data(uint8_t device_index);
tof_data_t* tof_get_device_data_by_id(uint8_t device_id);  // 新增：按设备ID获取数据
uint8_t tof_is_device_online(uint8_t device_id);

#endif