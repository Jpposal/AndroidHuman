#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "algorithmOfCRC.h"
#include "ins_task.h"
#include "robot_config.h"

// 协议及常量补充（移除 mcu_comm / Gimbal 之后的一些缺失定义）
#define FRAME_HEAD 0x5A
#define FRAME_TAIL 0xA5

#define TYPE_CHASSIS_CONTROL 0x10
#define TYPE_CHASSIS_STATUS  0x11
#define TYPE_BATTERY_INFO    0x12

// 用于 PCSendData.aim_request 的默认值（原来在 Gimbal.h 或相关头中定义）
#define NOT_USE_AIM 0

// 从已删除的 mcu_comm.h 迁移的数据类型（文档定义）
#pragma pack(push,1)
typedef struct {
    uint8_t control_type; // 1=VELOCITY,2=FOLLOW,3=SWING,4=SPIN
    float vx; // m/s
    float vy; // m/s
    float wz; // rad/s
} ChassisControlData; // 1 + 4 + 4 + 4 =13 bytes

typedef struct {
    float vx;
    float vy;
    float wz;
    float x;
    float y;
    float theta;
} ChassisStatusData; // 6*4 = 24 bytes

typedef struct {
    float voltage;
    float current;
    float percentage;
    uint8_t status; // 0=not charging,1=charging,2=full
    uint8_t reserved; // 保持长度为14字节，与文档一致
} BatteryInfoData; // 4+4+4+1+1 =14
#pragma pack(pop)

enum AUTOAIM_MODE
{
    AUTO_AIM = 0,
    SMALL_BUFF,
    BIG_BUFF
};

/* 发送/接收数据定义 - 保持字节对齐和顺序一致以便直接 memcpy */
#pragma pack(push, 1)
typedef struct PCSendData // 数据顺序不能变, 注意对齐
{
    int8_t start_flag;        // 起始标志, e.g. '!'
    uint8_t mode_want;        // 模式请求: 0/1/2
    float pitch_now;          // 当前 pitch 角度
    float yaw_now;            // 当前 yaw 角度
    float roll_now;           // roll（保留，当前为 0）
    float actual_bullet_speed;// 当前弹速/估计速度
    uint8_t shoot_avaiable;   // 是否可射
    uint8_t aim_request;      // 目标瞄准请求（AUTOAIM_MODE 或 NOT_USE_AIM）
    uint8_t number_want;      // 需要的弹数/序号（保留）
    uint8_t enemy_color;      // 敌方颜色标志
    uint16_t crc16;           // CRC16 校验
} PCSendData;

typedef struct PCRecvData
{
    int8_t start_flag;    // 起始标志
    float pitch;          // 目标 pitch
    float yaw;            // 目标 yaw
    uint8_t is_valid;     // 数据是否有效（非 0 表示有效）
    uint8_t should_shoot; // 是否射击请求
    uint16_t crc16;       // CRC16 校验
} PCRecvData;
#pragma pack(pop)

#define PC_SENDBUF_SIZE sizeof(PCSendData)
#define PC_RECVBUF_SIZE sizeof(PCRecvData)

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

void PCReceive(unsigned char *PCbuffer, uint32_t len);
void PCReceive_float(unsigned char PCReceivebuffer[]);
void SendtoPC(void);
void SendtoPCPack_float(void);

// 发送文档定义的数据结构到上位机（帧 TYPE 0x11 / 0x12）
int PC_SendChassisStatus(const ChassisStatusData *st);
int PC_SendBatteryInfo(const BatteryInfoData *bat);

extern PCRecvData pc_recv_data;
extern PCSendData pc_send_data;

#endif // _PC_SERIAL_H
