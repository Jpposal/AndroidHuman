#ifndef _MOTOR_TYPEDEF
#define _MOTOR_TYPEDEF

#include "struct_typedef.h"
#include "can.h"

typedef enum MOTOR_ID_TYPE
{
    DJI_0x1FF,
    DJI_0x1FE,
    DJI_0x2FF,
    DJI_0x2FE,
    DJI_0x200,
    DM_MOTOR_1,
		DM_MOTOR_2,
    MOTOR_MF9025,
    SEND_ID_NUMS // 一定放到最后,指代所有可能发送的电机ID
} MOTOR_ID_TYPE;

typedef enum MOTOR_TYPE
{
    GM6020,
    M3508,
    M2006,
    DM_MOTOR
} MOTOR_TYPE;

typedef enum MOTOR_APP_TYPE // 电机应用类型
{
    M3508_1,
    M3508_2,
		M3508_3,
    M3508_4,
    GM6020_0,
    YAW_MOTOR,
    TOGGLE_MOTOR,
		BAY_MOTOR,
    MOTOR_APP_NUMS
} MOTOR_APP_TYPE;

// 电机通信结构体
typedef struct Motor_Communication
{
    CAN_TypeDef *can;            // 发送CAN选择
    uint32_t std_id;             // 发送帧ID
    uint32_t motor_id;           // 电机ID 0x201-0x208
    MOTOR_ID_TYPE motor_id_type; // 电机ID类型
    MOTOR_TYPE motor_type;		// 电机类型
	  float gyro_yaw_angle;
	  float gyro_last_yaw_angle;
		float gyro_yaw_speed;
		float v[2];
		float last_v[2];
		float x[2];
		float last_x[2];//0代表x轴方向，1代表y轴方向
		uint32_t last_cnt;
		float delta_t;
    float control;               // 电机控制电流/电压
} Motor_Communication;

extern uint32_t MOTOR_STD_ID_LIST[SEND_ID_NUMS];
extern uint32_t motor_wait_time[2][SEND_ID_NUMS];
extern CAN_TxHeaderTypeDef motor_tx_header[4];

#endif