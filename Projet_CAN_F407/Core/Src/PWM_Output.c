/*
 * PWM_Output.c
 *
 *  Created on: Aug 15, 2024
 *      Author: DELL
 */

#include"PWM_Output.h"
#include "stm32f4xx_hal.h"

#define Driver_error 0xFBU
#define Bad_parameter 0xFEU
#define pwm_channel_max 4U
#define duty_cycle_max 0x2710U   //duty cycle =100% equivalent to 10000(HEX 2710)  //precision=0.01%

uint8_t Set_PWM_output(uint32_t channel, uint16_t duty_cycle, uint16_t frequency)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    uint8_t ReturnStatus = 0U;

    if ((channel > pwm_channel_max ) || (duty_cycle >duty_cycle_max ) || (frequency == 0U))
    {
        return Bad_parameter ;
    }
    else
    {
        uint32_t PCLK1 = HAL_RCC_GetPCLK1Freq();
        htim2.Init.Prescaler = (PCLK1 / (1000U * 1000U)) - 1U;  //PCLK1 clock in MHZ  //prescaler=39
        htim2.Init.Period = (1000000/frequency) - 1U;     //ARR=999
        if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
        {
            return Driver_error;
        }
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = ((duty_cycle*htim2.Init.Period)/100)/100; //duty cycle in %
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
          uint32_t tim_channel;
          switch (channel)
           {
               case 1: tim_channel = TIM_CHANNEL_1; break;
               case 2: tim_channel = TIM_CHANNEL_2; break;
               case 3: tim_channel = TIM_CHANNEL_3; break;
               case 4: tim_channel = TIM_CHANNEL_4; break;
               default: return Bad_parameter; // Channel invalid
           }
           if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, tim_channel) != HAL_OK)
           {
               return Driver_error;
           }

           if (HAL_TIM_PWM_Start(&htim2, tim_channel) != HAL_OK)
           {
               return Driver_error;
           }
           return ReturnStatus; // success
    }
}


//check set_PWM_output values
uint8_t Get_PWM_output(uint32_t channel, uint16_t *duty_cycle, uint16_t *frequency)  //use pointer to return 2 values : duty cycle and frequency
{
    if ((channel >  pwm_channel_max) || (duty_cycle == NULL) || (frequency == NULL))
    {
        return Bad_parameter;
    }
    uint32_t PCLK1 = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = htim2.Init.Prescaler + 1U;
    uint32_t period = htim2.Init.Period + 1U;
    uint32_t compare_value = 0U;                    // Compare register default  value CCR
    switch (channel)
    {
        case 1: compare_value = htim2.Instance->CCR1; break;
        case 2: compare_value = htim2.Instance->CCR2; break;
        case 3: compare_value = htim2.Instance->CCR3; break;
        case 4: compare_value = htim2.Instance->CCR4; break;
        default: return Bad_parameter;       // Invalid channel
    }
    if (period != 0)   //check the PWM frequency
    {
        *frequency = (PCLK1 / prescaler) / period;
        *duty_cycle = ((compare_value * 100U) / period)*100;
    }
    else
    {
        *frequency = 0U;
        *duty_cycle = 0U;
    }
    return 0U; // Success
}

