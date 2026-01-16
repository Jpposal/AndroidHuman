#ifndef _ACTION_TASK_H
#define _ACTION_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "GimbalController.h"
#include "bsp_PWM.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "gpio.h"

#include "ChassisSolver.h"
#include "bsp_dwt.h"
#include "ChasisController.h"

typedef enum CHASSIS_ACT_TYPE // 电机应用类型
{
		CHASSIS_POWERDOWN,
    FORWARD,
    LEFT,
		RIGHT,
		WAIT,
		STOP,
		BACK,
		CHASSIS_PC,
		GRASP,
		LAY_DOWN,
		TURN_L,
		TURN_R,
		START,
		STARTTURN,
		RELOCATE
} CHASSIS_ACT_TYPE;

typedef enum GIMBAL_ACT_TYPE // 电机应用类型
{
    MYGIMBAL_POWERDOWN,
		GRASP1,
		GRASP2,
		LAY_DOWN1,
		LAY_DOWN2,
		GIMBAL_ST,
		PC_CONTROL,
		GIMBAL_STOP,
		GIMBAL_WAIT
} GIMBAL_ACT_TYPE;

typedef struct GimbalState
{
	uint8_t GimbalAct;
	uint8_t last_act;
	uint32_t last_cnt;
	float act_time;
} GimbalState;
typedef struct GraspState
{
	uint8_t GraspAct;
	uint8_t last_act;
	uint32_t last_cnt;
	float act_time;
	float i;
} GraspState;
typedef struct ChassiState
{
	uint8_t ChassisAct;
	uint8_t last_act;
	uint32_t last_cnt;
	float32_t act_time;
} ChassisState;


void ActionTask(void *pvParameters);
void Graph_Decode();
float IMU_Yaw_Change(float yaw_angle);
float Drive_Line_Change(uint16_t PC_Receive_distance);
#endif // !_ACTION_TASK_H
