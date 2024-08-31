/*
 * GPIO_LED.h
 *
 *  Created on: Aug 15, 2024
 *      Author: DELL
 */
#include "stm32f4xx_hal.h"

#ifndef INC_GPIO_LED_H_
#define INC_GPIO_LED_H_



uint8_t Control_LED(uint8_t led_num, uint8_t action);
GPIO_PinState Read_LED(uint8_t led_num);


#endif /* INC_GPIO_LED_H_ */
