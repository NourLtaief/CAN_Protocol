/*
 * PWM_Input.c
 *
 *  Created on: Aug 26, 2024
 *      Author: DELL
 */

#include"PWM_Input.h"
#include "stm32f4xx_hal.h"
#define Bad_parameter 0xFEU
extern TIM_HandleTypeDef htim4;






PWM_Measurement pwm = {0, 0};
uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;

PWM_Measurement Handle_TIM_IC_Capture()
{
        PWM_Measurement pwm = {0, 0};
        uint32_t IC_Value1 = 0;
        uint32_t IC_Value2 = 0;

        // Capture de la période (TIM_CHANNEL_1)
        IC_Value1 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
        // Capture du temps haut (TIM_CHANNEL_2)
        IC_Value2 = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2);

        if (IC_Value1 != 0)
        {
            // Get the timer clock frequency
            uint32_t timerClock = HAL_RCC_GetPCLK1Freq() / (htim4.Init.Prescaler + 1);

            // Calculate duty cycle: high_time / period * 100
            pwm.dutyCycle = (IC_Value2 * 100.0f) / IC_Value1;

            // Calculate frequency: timerClock / period
            pwm.frequency = timerClock / IC_Value1;
        }
        else
        {
            // If period is zero, set error values
            pwm.frequency = Bad_parameter;
            pwm.dutyCycle = Bad_parameter;
        }


    return pwm;
}
