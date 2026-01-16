#ifndef _MOTOR_CONFIG_H
#define _MOTOR_CONFIG_H

#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal_conf.h"

#include "can.h"
#include "robot_config.h"

// 电机接收配置(注意不会影响发送)



#if ROBOT==GONG_XUN
// 轮毂电机排序
#define DJI_3508_MOTORS_1 0x201
#define DJI_3508_MOTORS_2 0x202
#define DJI_3508_MOTORS_3 0x203
#define DJI_3508_MOTORS_4 0x204


#define DJI_WHEELS_CAN CAN1

// FIFO 0 接收ID
#define CAN1_FIFO0_ID0 DJI_3508_MOTORS_1
#define CAN1_FIFO0_ID1 DJI_3508_MOTORS_2
#define CAN1_FIFO0_ID2 DJI_3508_MOTORS_3
#define CAN1_FIFO0_ID3 DJI_3508_MOTORS_4


#define DJI_6020_MOTORS_1 0x206
#define DJI_2006_MOTORS_1 0x205

#define CAN1_FIFO1_ID0 DJI_6020_MOTORS_1
#define CAN1_FIFO1_ID1 DJI_2006_MOTORS_1
//某种意义上可以叫做舵
//#define DJI_STEERS_CAN CAN1

//// FIFO 0 接收ID
//#define CAN1_FIFO0_ID0 DJI_3508_MOTORS_1
//#define CAN1_FIFO0_ID1 DJI_3508_MOTORS_2
//#define CAN1_FIFO0_ID2 DJI_3508_MOTORS_3
//#define CAN1_FIFO0_ID3 DJI_3508_MOTORS_4

//// FIFO 1 接收ID
//#define CAN1_FIFO1_ID0 DJI_6020_MOTORS_1


#endif



#endif // !_MOTOR_CONFIG_H
