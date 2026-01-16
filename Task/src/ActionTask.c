#include "ActionTask.h"
#include "stm32f407xx.h"  // 直接包含设备头文件
float bright = 0.0f;
static uint8_t button_pressed = 0;
static uint32_t button_press_start_time = 0;
uint8_t system_started = 0;

void ActionTask(void *pvParameters)
{
    portTickType xLastWakeTime;
    const portTickType xFrequency = 4; // 250HZ

    vTaskDelay(2000);
    DWT_GetDeltaT(&chassis_solver.last_cnt); // 时间初始化
    DWT_GetDeltaT(&chassis_controller.last_cnt);

    omni_pid_init();
    buzzer_init();

    InfantryInit(&chassis_controller);
    vTaskDelay(1);

    while (1)
    {
        xLastWakeTime = xTaskGetTickCount();
        chassis_controller.ActionTask_Time = DWT_GetTimeline_s();

        chassis_solver.delta_t = DWT_GetDeltaT(&chassis_solver.last_cnt);
        chassis_controller.delta_t = DWT_GetDeltaT(&chassis_controller.last_cnt);

        get_sensors_info(&chassis_controller.sensors_info);

        // 按键检测 - PA0低电平表示按下，长按3秒启动
        const uint32_t LONG_PRESS_TIME = 500; // 3秒长按时间        
        if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) // PA0为低电平
        {
            if (!button_pressed) // 按键刚被按下
            {
                button_pressed = 1;
                button_press_start_time = HAL_GetTick();

                // 开始长按，设置初始LED亮度
                bright = 10.0f;
            }
            else // 按键持续按下
            {
                uint32_t press_duration = HAL_GetTick() - button_press_start_time;

                if (press_duration < LONG_PRESS_TIME && !system_started)
                {
                    // 长按过程中，LED亮度逐渐降低从1.0到0.1
                    float brightness_ratio = 1.0f - (float)press_duration / LONG_PRESS_TIME;
                    bright = 0.1f + (10.0f - 0.1f) * brightness_ratio;
                }
                else if (press_duration >= LONG_PRESS_TIME && !system_started)
                {
                    // 长按3秒完成，等待松开按钮
                    bright = 0.1f; // 保持最低亮度，提示可以松开
                }
            }
        }
        else // PA0为高电平，按键释放
        {
            if (button_pressed)
            {
                uint32_t press_duration = HAL_GetTick() - button_press_start_time;

                if (press_duration < LONG_PRESS_TIME && !system_started)
                {
                    // 长按时间不足3秒就释放，取消启动
                    bright = 0.0f;
                }
                else if (press_duration >= LONG_PRESS_TIME && !system_started)
                {
                    // 长按3秒完成且松开按钮，系统正式启动
                    system_started = 1;
                    bright = 0.0f; // 关闭LED表示启动完成
                }

                button_pressed = 0;
            }
        }

        get_control_info(&chassis_solver);
        // 设置LED亮度
        // if(!system_started) 
        //     Set_LED_Brightness(bright);

        main_control(&chassis_controller);

        /* 执行控制 */
        execute_control(&chassis_controller.excute_info);
        //  execute_gimbal(&gimbal_controller);
        /*  延时  */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
