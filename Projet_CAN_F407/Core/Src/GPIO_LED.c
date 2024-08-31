/*
 * GPIO_LED.c
 *
 *  Created on: Aug 15, 2024
 *      Author: DELL
 */
#include "stm32f4xx_hal.h"
#include"GPIO_LED.h"

#define GPIO_Input_Number 4U //4 pinsPD12 ,PD13 ,PD14 , PD15 (0,1,2,3)
#define Bad_parameter 0xFEU
uint16_t led_matrix[GPIO_Input_Number] ={GPIO_PIN_12,  GPIO_PIN_13,  GPIO_PIN_14,  GPIO_PIN_15};
typedef enum
{
	LED_RESET,   //reset=0
    LED_SET      //set=1

} LED_Action;


uint8_t Control_LED(uint8_t led_num, uint8_t action)
{
	uint8_t ReturnStatus = 0U;   //default value //no error


	if ( (action > LED_SET )||(led_num >= GPIO_Input_Number ))   //LED_SET =1  //GPIO_Input_Number 0..3
	{
		ReturnStatus = Bad_parameter ;   //Error Message
	}
	else   //ReturnStatus = 0U;
	{
        HAL_GPIO_WritePin(GPIOD, (uint16_t)led_matrix[led_num], (GPIO_PinState)action); // configure  pin number and action :set or reset
	}
	return ReturnStatus;      //last value of ReturnStatus
}


GPIO_PinState Read_LED(uint8_t led_num)
{
    GPIO_PinState bitstatus = GPIO_PIN_RESET;  // Default state is OFF
    // Check if the led_num is within valid range
    if (led_num < GPIO_Input_Number)
    {
        // Read the state of the specified LED
    	bitstatus = HAL_GPIO_ReadPin(GPIOD, led_matrix[led_num]);
    }
    else
    {
    	bitstatus = Bad_parameter ;
    }
    return bitstatus;
}


