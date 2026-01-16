#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "pid.h"
#include "GM6020.h"
#include "ins_task.h"
#include "DM_Motor.h"
#include "M2006.h"
#include "tim.h"

#include "my_filter.h"
#include "TD.h"
#include "bsp_dwt.h"

#include "gimbal_config.h"


typedef struct
{
    float PCPitch;
    float PCYaw;
    short ReceiveFromTx2BullectCnt;
    short FrictionWheel_speed;
    short DisConnect;
    float PCdistance;
} PC_Receive_t;




typedef struct GimbalController
{
  // 弹舱盖
  PID_t bay_pos_pid;
  PID_t bay_speed_pid;
  M2006_Recv bay_recv;
  M2006_Info bay_info;
  // Pitch 轴
  PID_t pitch_current_pid;           // 电流环
  PID_t pitch_speed_pid;             // 速度环
  PID_t pitch_angle_pid;             // 角度环
  Feedforward_t pitch_speed_forward; // 速度环前馈
  Feedforward_t pitch_angle_forward; // 角度环前馈

  M2006_Recv pitch_recv;
  M2006_Info pitch_info;
  GM6020_Recv yaw_recv;
  GM6020_Info yaw_info;

  float set_pitch_speed;
  float set_pitch_current;
  float set_pitch_angle;
  float set_pitch_vol;
  float comp_pitch_current; // 重力补偿

  // 陀螺仪信息及其解算
  float gyro_pitch_speed;
  float gyro_pitch_angle;
  float gyro_last_pitch_angle;
  uint32_t last_cnt;
  float delta_t; // 两帧计算之间的时间差

  float target_pitch_angle; // 设定的角度值

  // Yaw在底盘控制
  // // Yaw 轴
  PID_t yaw_current_pid;           // 电流环
  PID_t yaw_speed_pid;             // 速度环
  PID_t yaw_angle_pid;             // 角度环
  Feedforward_t yaw_speed_forward; // 速度环前馈
  Feedforward_t yaw_angle_forward; // 角度环前馈

  float set_yaw_speed;
  float set_yaw_current;
  float set_yaw_angle;
  float set_yaw_vol;

  // 陀螺仪信息及其解算
  float gyro_yaw_speed;
  float gyro_yaw_angle;
  float gyro_last_yaw_angle;

  float target_yaw_angle;
  float last_target_yaw_angle; // 设定的角度值

  TD_t pos_yaw_td; // 位置跟踪微分器
  TD_t speed_yaw_td;

  // pitch 限位计算
  float pitch_max_gyro_angle;
  float pitch_min_gyro_angle;
	
  float control;
	int8_t gimbal_send_data[8];
	int8_t gimbal_send_data1[8];

} GimbalController;






void PWM_Init(void);

void GimbalPidInit(void);
void GimbalClear(void);
void updateGyro(void);

// pitch
void limitPitchAngle(void);
float GimbalPitchComp(void);
float Gimbal_Pitch_Calculate(float set_point);

// Yaw
float Gimbal_Yaw_Calculate(float set_point);
float Gimbal_Speed_Calculate(float set_point);
float GimbalFrictionModel(void);

void GimbalTask(void *pvParameters);

#endif // !_GIMBAL_H
