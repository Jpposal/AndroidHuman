#include "tof_driver.h"
#include "tof_middleware.h"

static tof_manager_t tof_manager = {0};

uint8_t tof_init_device(uint8_t device_id)
{
    static const uint8_t sleepTime = 511;

    tof_delay_ms(sleepTime);
    uint8_t id = tof_IIC_read_single_reg_by_id(device_id, TOF_ADDR_ID);
    
    if (id == 0xFF) {
        return 1; // 通信失败
    }
    uint8_t mode = tof_IIC_read_single_reg_by_id(device_id, TOF_ADDR_MODE);
    if (mode == 0xFF) {
        return 1; // 通信失败
    }
    
    return 0;
}

uint8_t tof_scan_all_devices(void)
{
    uint8_t device_ids[] = {0, 1, 2, 3};
    tof_manager.device_count = 0;

    for (int i = 0; i < TOF_MAX_DEVICES; i++) {
        tof_manager.devices[i].is_online = 0;
    }
    
    for (int i = 0; i < TOF_MAX_DEVICES; i++) {
        uint8_t device_id = device_ids[i];
        if (tof_init_device(device_id) == 0) {
            tof_manager.devices[device_id].device_id = device_id;
            tof_manager.devices[device_id].i2c_address = tof_get_i2c_address(device_id);
            tof_manager.devices[device_id].is_online = 1;
            tof_manager.device_count++;
        }
    }
    
    tof_manager.scan_complete = 1;
    return tof_manager.device_count;
}

void tof_read_all_data_by_id(uint8_t device_id, tof_data_t *tof_data)
{
    uint8_t pdata[TOF_REGISTER_TOTAL_SIZE];
    memset(pdata,0,sizeof(pdata));
    // 分两次读取所有寄存器数据
    tof_IIC_read_multi_reg_by_id(device_id, 0x00, &pdata[0], TOF_REGISTER_TOTAL_SIZE / 2);
    tof_IIC_read_multi_reg_by_id(device_id, 0x18, &pdata[24], TOF_REGISTER_TOTAL_SIZE / 2);
    
    // 设置设备信息
    tof_data->device_id = device_id;
    tof_data->i2c_address = tof_get_i2c_address(device_id);
    
    // 解析数据
    tof_data->interface_mode = pdata[TOF_ADDR_MODE] & 0x07;
    tof_data->id = pdata[TOF_ADDR_ID];
    
    // 解析4字节数据（小端序）
    tof_data->uart_baudrate = (uint32_t)(pdata[TOF_ADDR_UART_BAUDRATE]) |
                              (uint32_t)(pdata[TOF_ADDR_UART_BAUDRATE + 1] << 8) |
                              (uint32_t)(pdata[TOF_ADDR_UART_BAUDRATE + 2] << 16) |
                              (uint32_t)(pdata[TOF_ADDR_UART_BAUDRATE + 3] << 24);
    
    tof_data->system_time = (uint32_t)(pdata[TOF_ADDR_SYSTEM_TIME]) |
                           (uint32_t)(pdata[TOF_ADDR_SYSTEM_TIME + 1] << 8) |
                           (uint32_t)(pdata[TOF_ADDR_SYSTEM_TIME + 2] << 16) |
                           (uint32_t)(pdata[TOF_ADDR_SYSTEM_TIME + 3] << 24);
    
    tof_data->distance = (uint32_t)(pdata[TOF_ADDR_DIS]) |
                        (uint32_t)(pdata[TOF_ADDR_DIS + 1] << 8) |
                        (uint32_t)(pdata[TOF_ADDR_DIS + 2] << 16) |
                        (uint32_t)(pdata[TOF_ADDR_DIS + 3] << 24);
    
    // 解析2字节数据
    tof_data->status = (uint16_t)(pdata[TOF_ADDR_DIS_STATUS]) |
                      (uint16_t)(pdata[TOF_ADDR_DIS_STATUS + 1] << 8);
    
    tof_data->signal_strength = (uint16_t)(pdata[TOF_ADDR_SIGNAL_STRENGTH]) |
                               (uint16_t)(pdata[TOF_ADDR_SIGNAL_STRENGTH + 1] << 8);
    
    // 解析1字节数据
    tof_data->range_precision = pdata[TOF_ADDR_RANGE_PRECISION];
    
    // 更新在线状态
    tof_data->is_online = 1;
}

void tof_read_all_devices_data(void)
{
    for (uint8_t device_id = 0; device_id < TOF_MAX_DEVICES; device_id++) {
        if (tof_is_device_online(device_id) || 1) {
            tof_read_all_data_by_id(device_id, &tof_manager.devices[device_id]);
        }
    }
}

uint32_t tof_read_distance_by_id(uint8_t device_id)
{
    uint8_t buf[4];
    tof_IIC_read_multi_reg_by_id(device_id, TOF_ADDR_DIS, buf, 4);
    return (uint32_t)(buf[0]) |
           (uint32_t)(buf[1] << 8) |
           (uint32_t)(buf[2] << 16) |
           (uint32_t)(buf[3] << 24);
}

uint8_t tof_get_device_count(void)
{
    return tof_manager.device_count;
}

tof_data_t* tof_get_device_data(uint8_t device_index)
{
    if (device_index < TOF_MAX_DEVICES) {
        if (tof_manager.devices[device_index].is_online) {
            return &tof_manager.devices[device_index];
        }
    }
    return NULL;
}

tof_data_t* tof_get_device_data_by_id(uint8_t device_id)
{
    if (device_id < TOF_MAX_DEVICES && tof_manager.devices[device_id].is_online) {
        return &tof_manager.devices[device_id];
    }
    return NULL;
}

uint8_t tof_is_device_online(uint8_t device_id)
{
    if (device_id < TOF_MAX_DEVICES) {
        return tof_manager.devices[device_id].is_online;
    }
    return 0;
}