#ifndef _OMNI_H
#define _OMNI_H

#include "accel.h"
#include "math.h"
#include "tools.h"

//  ?????

/*  ??CAN ID? ?????????? ,????,????????(??????LEFT_UP_OMNI_WHEEL????0,1,2,3) */
/*
||0x201  ||0x202
                -->  ?
||0x203  ||0x204

*/

// ????0??0x201??
#define LEFT_UP_OMNI_WHEEL 0    // ????
#define RIGHT_UP_OMNI_WHEEL 1   // ????
#define LEFT_DOWN_OMNI_WHEEL 3  // ????
#define RIGHT_DOWN_OMNI_WHEEL 2 // ????

#define OMNI_WHEEL_RADIUS 0.054f
#define OMNI_SPEED_TO_DEGEREE_S (180.0f / OMNI_WHEEL_RADIUS / PI) // ??/s???/s??
#define OMNI_DEGEREE_S_TO_MS (PI * OMNI_WHEEL_RADIUS / 180.0f)    // ??/s??m/s
#define OMNI_RADIUS 0.1425f

// ????
void omni_pos_kinematics(void);

// ????
void omni_inv_kinematics(void);

// 世界坐标系逆运动学
void omni_inv_kinematics_world_coordinate(float world_x_v, float world_y_v, float yaw_v, float chassis_angle);
// ??????
void omni_follow_control(void);
void omni_follow_control_without_chassis(void);
// ???????
void omni_chassis_control(void);

// ??????
void omni_rotate_control(void);

// 底盘自转同时沿世界坐标系平移
void omni_rotate_with_world_coordinate(void);

void omni_pid_init(void);

#endif
