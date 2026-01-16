#include "testTask.h"
#include "cmsis_os.h"
#include "can.h"
#include "ZLA_Motor.h"
#include "kinematics.h"
#include "robot_config.h"

extern CAN_HandleTypeDef hcan1;
ZLA_Motor front_drive;  // node 1: front-left & front-right
ZLA_Motor rear_drive;   // node 2: rear-left & rear-right
int test_debug_mode = 0;
                float vx = 0.0f;    // m/s
                float omega = 0.0f; // rad/s
								int32_t left_dev, right_dev;
void TestTask_SetDebug(int enable)
{
    test_debug_mode = enable ? 1 : 0;
}

typedef enum {
    TS_INIT = 0,
    TS_DEBUG,
    TS_NORMAL,
} TestTaskState;

void test_task(void const * argument)
{
    // 加载配置参数
    RobotConfig_Load();
    
    // 初始化驱动节点（使用配置中的节点 ID）
    ZLA_Motor_Init(&front_drive, &hcan1, robot_config.node_front);
    ZLA_Motor_Init(&rear_drive, &hcan1, robot_config.node_rear);

    vTaskDelay(1000);

    front_drive.last_error = ZLA_FullInit_VelocityMode(&front_drive);
    rear_drive.last_error = ZLA_FullInit_VelocityMode(&rear_drive);

    // 配置 TPDO（若需要）
    if (front_drive.last_error == ZLA_OK) ZLA_ConfigureTPDO1_606C_Timed(&front_drive, 200);
    if (rear_drive.last_error == ZLA_OK) ZLA_ConfigureTPDO1_606C_Timed(&rear_drive, 200);

    // 初始化运动学（使用配置中的参数）
    KIN_Init(robot_config.wheel_radius_m, robot_config.half_track_m, robot_config.max_rpm);

    TestTaskState state = TS_NORMAL;
    for(;;)
    {
        switch (state)
        {
            case TS_INIT:
                if (test_debug_mode)
                    state = TS_DEBUG;
                else
                    state = TS_NORMAL;
                break;

            case TS_DEBUG:
            {
                // 调试模式：保留原先的行为（手动写入目标速度并使用 TPDO 上报）
                if (front_drive.last_error == ZLA_OK)
                {
                    uint32_t combined_speed = (uint16_t)front_drive.target_left_rpm | ((uint32_t)(uint16_t)front_drive.target_right_rpm << 16);
                    ZLA_SDO_Write32(&front_drive, OD_TARGET_VELOCITY, SUBIDX_BOTH_MOTORS, combined_speed);
                }
                if (rear_drive.last_error == ZLA_OK)
                {
                    uint32_t combined_speed_r = (uint16_t)rear_drive.target_left_rpm | ((uint32_t)(uint16_t)rear_drive.target_right_rpm << 16);
                    ZLA_SDO_Write32(&rear_drive, OD_TARGET_VELOCITY, SUBIDX_BOTH_MOTORS, combined_speed_r);
                }
                vTaskDelay(100);
            }
            break;

            case TS_NORMAL:
            {

                KIN_ComputeLeftRight(vx, omega, &left_dev, &right_dev);
                if (front_drive.last_error == ZLA_OK)
                {
                    // front_drive -> front-left (FL) & front-right (FR)
                    uint16_t fl16 = (uint16_t)(left_dev);
                    uint16_t fr16 = (uint16_t)(-right_dev);
                    uint32_t combined_front = ((uint32_t)fl16) | (((uint32_t)fr16) << 16);
                    ZLA_SDO_Write32(&front_drive, OD_TARGET_VELOCITY, SUBIDX_BOTH_MOTORS, (int32_t)combined_front);
                }
                if (rear_drive.last_error == ZLA_OK)
                {
                    // rear_drive -> rear-left (RL) & rear-right (RR)
                    uint16_t rl16 = (uint16_t)(left_dev);
                    uint16_t rr16 = (uint16_t)(-right_dev);
                    uint32_t combined_rear = ((uint32_t)rr16) | (((uint32_t)rl16) << 16);
                    ZLA_SDO_Write32(&rear_drive, OD_TARGET_VELOCITY, SUBIDX_BOTH_MOTORS, (int32_t)combined_rear);
                }

                vTaskDelay(100);
            }
            break;

            default:
                state = TS_INIT;
                break;
        }
    }
}
