#include "testTask.h"
#include "cmsis_os.h"
#include "ZLA_Motor.h"
#include "kinematics.h"
#include "robot_config.h"
#include <stdbool.h>
#include "pc_serial.h"
extern CAN_HandleTypeDef hcan;
ZLA_Motor front_drive;  // node 1: front-left & front-right
ZLA_Motor rear_drive;   // node 2: rear-left & rear-right
int test_debug_mode = 0;
float vx = 0.0f;    // m/s
float omega = 0.0f; // rad/s
float vy = 0.0f;    // m/s
int32_t left_dev, right_dev;

// 通信周期（ms），根据需求文档调整此值
#ifndef PC_COMM_PERIOD_MS
#define PC_COMM_PERIOD_MS 20
#endif
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
    ZLA_Motor_Init(&front_drive, &hcan, robot_config.node_front);
    ZLA_Motor_Init(&rear_drive, &hcan, robot_config.node_rear);

    vTaskDelay(1000);

    front_drive.last_error = ZLA_FullInit_VelocityMode(&front_drive);
    rear_drive.last_error = ZLA_FullInit_VelocityMode(&rear_drive);

    // 配置 TPDO（若需要）
    if (front_drive.last_error == ZLA_OK) ZLA_ConfigureTPDO1_606C_Timed(&front_drive, 200);
    if (rear_drive.last_error == ZLA_OK) ZLA_ConfigureTPDO1_606C_Timed(&rear_drive, 200);

    // 初始化运动学（使用配置中的参数）
    KIN_Init(robot_config.wheel_radius_m, robot_config.half_track_m, robot_config.max_rpm);

    TestTaskState state = TS_NORMAL;
    portTickType xLastWakeTime = xTaskGetTickCount();
    const portTickType xFrequency = PC_COMM_PERIOD_MS; // 通信周期

    uint32_t send_tick = 0; // ms 计数，用于 battery 周期
    // 驱动重试计时（ms）：当检测到驱动处于错误/未就绪时，周期性重试初始化
    const uint32_t REINIT_PERIOD_MS = 2000;
    uint32_t front_reinit_timer = 0;
    uint32_t rear_reinit_timer = 0;
    uint32_t pc_last_recv_tick = 0; // 上次接收 PC 数据的 tick

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
                break;
            }

            case TS_NORMAL:
            {

                // 仅当两个驱动器都就绪（ZLA_OK）时，才启动并发送目标速度
                bool both_ok = (front_drive.last_error == ZLA_OK) && (rear_drive.last_error == ZLA_OK);
                if (both_ok)
                {
                    KIN_ComputeLeftRight(vx, omega, &left_dev, &right_dev);

                    // front_drive -> front-left (FL) & front-right (FR)
                    uint16_t fl16 = (uint16_t)(left_dev);
                    uint16_t fr16 = (uint16_t)(-right_dev);
                    uint32_t combined_front = ((uint32_t)fl16) | (((uint32_t)fr16) << 16);
                    ZLA_SDO_Write32(&front_drive, OD_TARGET_VELOCITY, SUBIDX_BOTH_MOTORS, (int32_t)combined_front);

                    // rear_drive -> rear-left (RL) & rear-right (RR)
                    uint16_t rl16 = (uint16_t)(left_dev);
                    uint16_t rr16 = (uint16_t)(-right_dev);
                    uint32_t combined_rear = ((uint32_t)rr16) | (((uint32_t)rl16) << 16);
                    ZLA_SDO_Write32(&rear_drive, OD_TARGET_VELOCITY, SUBIDX_BOTH_MOTORS, (int32_t)combined_rear);
                }

                    // 如果驱动处于错误/未就绪状态，按周期重试初始化
                    if (front_drive.last_error != ZLA_OK)
                    {
                        front_reinit_timer += xFrequency;
                        if (front_reinit_timer >= REINIT_PERIOD_MS)
                        {
                            front_reinit_timer = 0;
                            front_drive.last_error = ZLA_FullInit_VelocityMode(&front_drive);
                            if (front_drive.last_error == ZLA_OK)
                            {
                                ZLA_ConfigureTPDO1_606C_Timed(&front_drive, 200);
                            }
                        }
                    }
                    else
                    {
                        front_reinit_timer = 0;
                    }

                    if (rear_drive.last_error != ZLA_OK)
                    {
                        rear_reinit_timer += xFrequency;
                        if (rear_reinit_timer >= REINIT_PERIOD_MS)
                        {
                            rear_reinit_timer = 0;
                            rear_drive.last_error = ZLA_FullInit_VelocityMode(&rear_drive);
                            if (rear_drive.last_error == ZLA_OK)
                            {
                                ZLA_ConfigureTPDO1_606C_Timed(&rear_drive, 200);
                            }
                        }
                    }
                    else
                    {
                        rear_reinit_timer = 0;
                    }

                // 处理从 PC/上位机接收的数据（pc_recv_data 在 pc_serial.h 中定义）
                if (pc_recv_data.is_valid)
                {
                    // 收到上位机速度指令，更新本地速度并记录接收时间（tick）
                    vx = pc_recv_data.vx;    // 使用 pc_serial 中的 vx 字段作为前向速度（m/s）
                    omega = pc_recv_data.yaw;   // yaw 字段用于角速度（rad/s）
                    // 记录 tick（用于超时检测）
                    pc_last_recv_tick = xTaskGetTickCount();
                    // 使用完成后清除有效位，避免重复处理
                    pc_recv_data.is_valid = 0;
                }

                // 使用 FreeRTOS tick 检查上次接收时间，若超过 1000 ms 则清零速度
                if (pc_last_recv_tick != 0)
                {
                    TickType_t now_tick = xTaskGetTickCount();
                    uint32_t elapsed_ms = (uint32_t)((now_tick - pc_last_recv_tick) * portTICK_PERIOD_MS);
                    if (elapsed_ms >= 1000U)
                    {
                        vx = 0.0f;
                        omega = 0.0f;
                    }
                }

                break;
            }

            default:
                state = TS_INIT;
                break;
        }

        /* 发送底盘状态，建议 50Hz */
        ChassisStatusData st;
        st.vx = vx;
        st.vy = vy;
        st.wz = omega;
        st.x = 0.0f; st.y = 0.0f; st.theta = 0.0f; // 填充示例位置/航向
        PC_SendChassisStatus(&st);

        /* 每 100ms 发送电池信息 */
        send_tick += xFrequency;
//        if (send_tick >= BATTERY_INFO_PERIOD_MS) {
//            send_tick = 0;
//            BatteryInfoData bat;
//            bat.voltage = 12.0f; bat.current = 0.1f; bat.percentage = 80.0f; bat.status = 2; bat.reserved = 0;
//            PC_SendBatteryInfo(&bat);
//        }

        /*  延时，保持通信周期 */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
