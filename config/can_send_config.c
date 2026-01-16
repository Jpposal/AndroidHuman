#include "can_send_config.h"

// 发送结构体配置(注意不影响接收)
Motor_Communication motor_communication[8];

void Motor_Config_Init()
{

#if ROBOT == NIU_MO_SON
    motor_communication[M3508_1].can = CAN2;
    motor_communication[M3508_1].motor_id = 0x201;
    motor_communication[M3508_1].motor_id_type = DJI_0x200;
    motor_communication[M3508_1].motor_type = M3508;
    motor_communication[M3508_1].std_id = MOTOR_STD_ID_LIST[DJI_0x200]; // 发送ID

    motor_communication[RIGHT_UP_FRICTION_WHEEL_MOTOR].can = CAN2;
    motor_communication[RIGHT_UP_FRICTION_WHEEL_MOTOR].motor_id = 0x202;
    motor_communication[RIGHT_UP_FRICTION_WHEEL_MOTOR].motor_id_type = DJI_0x200;
    motor_communication[RIGHT_UP_FRICTION_WHEEL_MOTOR].motor_type = M3508;
    motor_communication[RIGHT_UP_FRICTION_WHEEL_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200];
		
		motor_communication[LEFT_DOWN_FRICTION_WHEEL_MOTOR].can = CAN2;
    motor_communication[LEFT_DOWN_FRICTION_WHEEL_MOTOR].motor_id = 0x203;
    motor_communication[LEFT_DOWN_FRICTION_WHEEL_MOTOR].motor_id_type = DJI_0x200;
    motor_communication[LEFT_DOWN_FRICTION_WHEEL_MOTOR].motor_type = M3508;
    motor_communication[LEFT_DOWN_FRICTION_WHEEL_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200]; // 发送ID

    motor_communication[RIGHT_DOWN_FRICTION_WHEEL_MOTOR].can = CAN2;
    motor_communication[RIGHT_DOWN_FRICTION_WHEEL_MOTOR].motor_id = 0x204;
    motor_communication[RIGHT_DOWN_FRICTION_WHEEL_MOTOR].motor_id_type = DJI_0x200;
    motor_communication[RIGHT_DOWN_FRICTION_WHEEL_MOTOR].motor_type = M3508;
    motor_communication[RIGHT_DOWN_FRICTION_WHEEL_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200];

    motor_communication[PITCH_MOTOR].can = CAN2;
    motor_communication[PITCH_MOTOR].motor_id = 0x206;
    motor_communication[PITCH_MOTOR].motor_id_type = DJI_0x1FF;
    motor_communication[PITCH_MOTOR].motor_type = GM6020;
    motor_communication[PITCH_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x1FF];

    motor_communication[YAW_MOTOR].can = CAN1;
    motor_communication[YAW_MOTOR].motor_id = 0x10;
    motor_communication[YAW_MOTOR].motor_id_type = DM_MOTOR_1;
    motor_communication[YAW_MOTOR].motor_type = DM_MOTOR;
    motor_communication[YAW_MOTOR].std_id = MOTOR_STD_ID_LIST[DM_MOTOR_1];

//    motor_communication[TOGGLE_MOTOR].can = CAN1;
//    motor_communication[TOGGLE_MOTOR].motor_id = 0x204;
//    motor_communication[TOGGLE_MOTOR].motor_id_type = DJI_0x200;
//    motor_communication[TOGGLE_MOTOR].motor_type = M2006;
//    motor_communication[TOGGLE_MOTOR].std_id = MOTOR_STD_ID_LIST[DJI_0x200];


#elif ROBOT == GONG_XUN

		motor_communication[M3508_1].can = CAN1;
    motor_communication[M3508_1].motor_id = 0x201;
    motor_communication[M3508_1].motor_id_type = DJI_0x200;
    motor_communication[M3508_1].motor_type = M3508;
    motor_communication[M3508_1].std_id = MOTOR_STD_ID_LIST[DJI_0x200]; // 发送ID
		
		motor_communication[M3508_2].can = CAN1;
    motor_communication[M3508_2].motor_id = 0x202;
    motor_communication[M3508_2].motor_id_type = DJI_0x200;
    motor_communication[M3508_2].motor_type = M3508;
    motor_communication[M3508_2].std_id = MOTOR_STD_ID_LIST[DJI_0x200]; // 发送ID
		
		motor_communication[M3508_3].can = CAN1;
    motor_communication[M3508_3].motor_id = 0x203;
    motor_communication[M3508_3].motor_id_type = DJI_0x200;
    motor_communication[M3508_3].motor_type = M3508;
    motor_communication[M3508_3].std_id = MOTOR_STD_ID_LIST[DJI_0x200]; // 发送ID
		
		motor_communication[M3508_4].can = CAN1;
    motor_communication[M3508_4].motor_id = 0x204;
    motor_communication[M3508_4].motor_id_type = DJI_0x200;
    motor_communication[M3508_4].motor_type = M3508;
    motor_communication[M3508_4].std_id = MOTOR_STD_ID_LIST[DJI_0x200]; // 发送ID
		
		motor_communication[GM6020_0].can = CAN1;
    motor_communication[M3508_2].motor_id = 0x1FE;
    motor_communication[M3508_2].motor_id_type = DJI_0x1FE;//舵电机，要用电流控制
    motor_communication[M3508_2].motor_type = GM6020;
    motor_communication[M3508_2].std_id = MOTOR_STD_ID_LIST[DJI_0x1FE]; // 发送ID




#endif
}