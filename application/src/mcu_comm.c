/* mcu_comm 已移除；保留此文件为兼容占位实现，避免未删除引用导致链接错误。
   所有实际逻辑已迁移到 `pc_serial.c` 与 `pc_serial.h`。 */

#include "mcu_comm.h"

void MCUComm_Init(void) { }

size_t MCUComm_ParseBytes(const uint8_t *buf, size_t len) { (void)buf; (void)len; return 0; }

void MCUComm_RegisterChassisControlCb(void *cb) { (void)cb; }

void MCUComm_RegisterSendCb(void *cb) { (void)cb; }

int MCUComm_SendChassisStatus(const void *st) { (void)st; return -1; }

int MCUComm_SendBatteryInfo(const void *bat) { (void)bat; return -1; }
