/**
  ******************************************************************************
  * @file	 bsp_PWM.c
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/3/1
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "bsp_PWM.h"

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;
        break;
    }
}

/**
 * @brief 蜂鸣器开启函数
 * @param psc 预分频值 (决定频率)
 * @param pwm PWM占空比值 (决定音量，0-htim4.Init.Period)
 */
void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_SET_PRESCALER(&htim4, psc);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

/**
 * @brief 蜂鸣器关闭函数
 */
void buzzer_off(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}

/**
 * @brief 蜂鸣器初始化函数
 * @note 在使用蜂鸣器前必须先调用此函数
 */
void buzzer_init(void)
{
    MX_TIM4_Init();
}

/**
 * @brief 蜂鸣器播放指定频率和音量
 * @param freq 频率 (Hz)
 * @param volume 音量 (0-100)
 * @param duration 持续时间 (ms)
 */
void buzzer_beep(uint16_t freq, uint8_t volume)
{
    if(freq == 0 || volume == 0) {
        buzzer_off();
        return;
    }
    
    // 计算预分频值来获得目标频率
    // 84MHz / (psc + 1) / (period + 1) = freq
    // 假设period = 1000，则 psc = 84MHz / freq / 1000 - 1
    uint16_t psc = (84000000 / freq / 1000) - 1;
    uint16_t pwm = (1000 * volume) / 100; // 占空比
    
    buzzer_on(psc, pwm);
}

/* USER CODE BEGIN 1 */
void Set_LED_Brightness(float brightness_percent)
{
    static float Arr = 16800;
    
    // 限制亮度范围在0-100%之间
    if (brightness_percent < 0) brightness_percent = 0;
    if (brightness_percent > 100) brightness_percent = 100;
    
    // 计算PWM占空比值 (0表示关闭，Arr表示最亮)
    uint16_t Crr = (uint16_t)(brightness_percent / 100.0f * Arr);
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Crr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Crr);
}
void Set_LED_Off(void)
{
    // 将PWM占空比设置为0来关闭LED
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}