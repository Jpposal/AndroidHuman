#ifndef _PC_SERIAL_H
#define _PC_SERIAL_H

#include "algorithmOfCRC.h"
#include "ins_task.h"

#include "robot_config.h"

enum AUTOAIM_MODE
{
    AUTO_AIM = 0,
    SMALL_BUFF,
    BIG_BUFF
};

/*   发送数据定义 */
#pragma pack(push, 1)     // 不进行字节对齐
typedef struct PCSendData // 数据顺序不能变,注意32字节对齐
{
    int8_t start_flag; // 一样
    float pitch_now;
    float yaw_now;
    uint16_t crc16;
} PCSendData;

typedef struct PCRecvData
{
    int8_t start_flag; 
    float pitch;
    float yaw;
    int8_t is_valid; 
    int8_t should_shoot; 
    uint16_t crc16;
} PCRecvData;
#pragma pack(pop) // 不进行字节对齐

#define PC_SENDBUF_SIZE sizeof(PCSendData)  // 动态计算大小
#define PC_RECVBUF_SIZE sizeof(PCRecvData)  // 动态计算大小

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

void PCReceive(unsigned char *PCbuffer);
void PCReceive_float(unsigned char PCReceivebuffer[]);
void SendtoPC(void);
void SendtoPCPack_float(void);

extern PCRecvData pc_recv_data;
extern PCSendData pc_send_data;

extern unsigned char PCbuffer[PC_RECVBUF_SIZE];
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];
#endif // !_PC_SERIAL_H
