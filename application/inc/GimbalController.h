#ifndef _GIMBALCONTROLLER_H
#define _GIMBALCONTROLLER_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "remote_control.h"
#include "GimbalTask.h"

#include "can_config.h"

#include "M3508.h"
#include "ins_task.h"
#include "Offline_Task.h"

#include "SystemIdentification.h"
#include "SignalGenerator.h"

#include "pc_serial.h"

#include "DM_Motor.h"
#include "can.h"
#include "Motor_Typdef.h"
#include "bsp_can.h"
#include "bsp_dwt.h"

#include "can_send_config.h"

void GimbalupdataSensors();
void gimbal_turn(GimbalController *gimbal_controller);
void execute_gimbal(GimbalController *gimbal_controller);
void STEP_Init();
void gimbal_init(GimbalController *gimbal_controller);


#endif
