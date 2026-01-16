/**
  ******************************************************************************
  * @file	 bsp_PWM.h
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/3/1
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __BSP_IMU_PWM_H
#define __BSP_IMU_PWM_H

#include "stdint.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);

// ·äÃùÆ÷¿ØÖÆº¯ÊýÉùÃ÷
void buzzer_init(void);
void buzzer_on(uint16_t psc, uint16_t pwm);
void buzzer_off(void);
void buzzer_beep(uint16_t freq, uint8_t volume);
void Set_LED_Brightness(float brightness_percent);
void Set_LED_Off(void);
#endif
