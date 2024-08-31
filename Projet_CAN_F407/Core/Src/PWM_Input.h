/*
 * PWM_Input.h
 *
 *  Created on: Aug 26, 2024
 *      Author: DELL
 */
#include "stm32f4xx_hal.h"
#ifndef SRC_PWM_INPUT_H_
#define SRC_PWM_INPUT_H_
typedef struct
{
    uint16_t dutyCycle;
    uint16_t frequency;
} PWM_Measurement;



PWM_Measurement Handle_TIM_IC_Capture();





#endif /* SRC_PWM_INPUT_H_ */
