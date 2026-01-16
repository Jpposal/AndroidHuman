/**
 ******************************************************************************
 * @file    pc_uart.c
 * @brief   serial数据接发
 ******************************************************************************
 * @attention
 ******************************************************************************
 */

#include "pc_serial.h"
#include "GimbalTask.h"
#include "arm_atan2_f32.h"
#include "debug.h"

#include "SignalGenerator.h"

unsigned char PCbuffer[PC_RECVBUF_SIZE];
unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

PCRecvData pc_recv_data;
PCSendData pc_send_data;

void PCSolve(void)
{
    LossUpdate(&global_debugger.pc_receive_debugger, 0.02);
}

void PCReceive(unsigned char *PCbuffer)
{
    if (PCbuffer[0] == '!'  && Verify_CRC16_Check_Sum(PCbuffer, PC_RECVBUF_SIZE))
    {
        memcpy(&pc_recv_data, PCbuffer, PC_RECVBUF_SIZE);
        PCSolve();
    }	
}

/**
 * @brief 在这里写发送数据的封装
 * @param[in] void
 */
void SendtoPCPack(unsigned char *buff)
{
		pc_send_data.start_flag = '!';
//		pc_send_data.pitch_now = gimbal_controller.pitch_info.angle;
//		pc_send_data.yaw_now = gimbal_controller.gyro_yaw_angle;
        fmodf(pc_send_data.pitch_now, 360.0f); // 确保角度在0-360度范围内 
		Append_CRC16_Check_Sum((uint8_t *)(&pc_send_data), PC_SENDBUF_SIZE);
		
		memcpy(buff, (void *)&pc_send_data, PC_SENDBUF_SIZE);
}

/**
 * @brief 发送数据调用
 * @param[in] void
 */
void SendtoPC(void)
{
    SendtoPCPack(SendToPC_Buff);

    CDC_Transmit_FS(SendToPC_Buff, PC_SENDBUF_SIZE); // 通过USB_CDC发送
}
