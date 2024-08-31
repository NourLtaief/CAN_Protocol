/*
 * PWM_Output.h
 *
 *  Created on: Aug 15, 2024
 *      Author: DELL
 */
#include "stm32f4xx_hal.h"

#ifndef INC_PWM_OUTPUT_H_
#define INC_PWM_OUTPUT_H_

extern TIM_HandleTypeDef htim2;
#define pwm_channel_max 4U
#define duty_cycle_max 0x2710U   //duty cycle =100% equivalent to 10000(HEX 2710)  //precision=0.01%
uint8_t Set_PWM_output(uint32_t channel, uint16_t duty_cycle, uint16_t frequency);
uint8_t Get_PWM_output(uint32_t channel, uint16_t *duty_cycle, uint16_t *frequency) ;



#endif /* INC_PWM_OUTPUT_H_ */
