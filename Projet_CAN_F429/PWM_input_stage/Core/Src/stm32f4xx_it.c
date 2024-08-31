/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern CAN_RxHeaderTypeDef RxHeader;
extern uint8_t Tx_Data[8];
extern uint8_t Rx_Data[8];

#define module_GPIO_led_ID 1U
#define LED_Set_Val 1U
#define LED_Get_Val 2U
#define module_PWM_control_ID 2U
#define module_ADC_DMA_control_ID 3U
#define module_PWM_Input  4U
#define PWM_Input_Get 1U

#define PWM_Set_Freq 1U
#define PWM_Get_Freq 2U
#define ADC_start 1U
#define ADC_get_values 2U
#define ADC_stop 3U
#define Module_ID_not_defined 0xFAU
#define Routine_ID_not_defined 0xFFU
#define Driver_error 0xFBU
#define Bad_parameter 0xFEU
#define BUFFER_SIZE 4U

uint16_t adc_buffer[BUFFER_SIZE];
GPIO_PinState bitstatus;
uint8_t ReturnStatus = 0U;
uint8_t status =0U;
uint16_t duty_cycle;
uint16_t frequency;


/*extern volatile uint32_t IC_Val1;
extern volatile uint32_t IC_Val2;
extern volatile uint32_t Difference;
extern volatile uint8_t Is_First_Captured;*/

/*extern volatile uint32_t Frequency;
extern volatile uint32_t DutyCycle;*/

uint32_t channel;

typedef struct {
	uint16_t dutyCycle;
	uint16_t frequency;
} PWM_Measurement;

extern PWM_Measurement pwm;

typedef enum{
    LED_RESET,
	LED_SET
} LED_Action;
extern ADC_HandleTypeDef hadc1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern uint8_t Control_LED(uint8_t led_num, uint8_t action);
extern GPIO_PinState Read_LED(uint8_t led_num);
extern void Send_CAN_Message(uint8_t *data, uint32_t length);
extern uint8_t Set_PWM_output(uint8_t channel, uint16_t duty_cycle, uint32_t frequency);
extern uint8_t Get_PWM_output(uint32_t channel, uint16_t *duty_cycle, uint16_t *frequency);
extern PWM_Measurement Handle_TIM_IC_Capture(uint32_t channel);
//extern PWM_Measurement Handle_PWM_Input();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim5;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 TX interrupts.
  */
void CAN1_TX_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_TX_IRQn 0 */

  /* USER CODE END CAN1_TX_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_TX_IRQn 1 */

  /* USER CODE END CAN1_TX_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

    // Check if a CAN message has been received successfully
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, Rx_Data) == HAL_OK)
    {
        switch (Rx_Data[0]) // Module ID = Rx_Data[0]
        {
            case module_GPIO_led_ID:  // Module ID for GPIO LED control
                switch (Rx_Data[1])  // Routine ID = Rx_Data[1]
                {
                    case LED_Set_Val: // Set LED value
                        ReturnStatus = Control_LED(Rx_Data[2], Rx_Data[3]);
                        Rx_Data[0] += 0x40U;  // Mark as response
                        Rx_Data[7] = ReturnStatus;
                        Send_CAN_Message(Rx_Data, 0x8U);  // Send response
                        break;

                    case LED_Get_Val: // Get LED value
                        bitstatus = Read_LED(Rx_Data[2]);
                        Rx_Data[0] += 0x40U;  // Mark as response
                        Rx_Data[7] = (uint8_t)bitstatus;  // Update with the state of the LED
                        Send_CAN_Message(Rx_Data, 0x8U);  // Send response
                        break;

                    default:
                        // Routine ID not defined
                        Rx_Data[0] += 0x40U;
                        Rx_Data[7] = Routine_ID_not_defined;
                        Send_CAN_Message(Rx_Data, 0x8U);
                        break;
                }
                break;

            case module_PWM_control_ID:  // Module ID for PWM control
                switch (Rx_Data[1])  // Routine ID = Rx_Data[1]
                {
                    case PWM_Set_Freq: // Set PWM frequency
                        ReturnStatus = Set_PWM_output(
                            Rx_Data[2],
                            (Rx_Data[3] << 8) | Rx_Data[4],
                            (Rx_Data[5] << 8) | Rx_Data[6]
                        );
                        Rx_Data[0] += 0x40U;  // Mark as response
                        Rx_Data[7] = ReturnStatus;
                        Send_CAN_Message(Rx_Data, 0x8U);  // Send response
                        break;

                    case PWM_Get_Freq: // Get PWM frequency and duty cycle
                        status = Get_PWM_output(Rx_Data[2], &duty_cycle, &frequency);
                        if (status == 0U)
                        {
                            // Successfully retrieved PWM configuration
                            // Populate the response data
                            Rx_Data[0] += 0x40U;  // Mark as response
                            Rx_Data[6] = (uint8_t)frequency;
                            Rx_Data[5] = (uint8_t)(frequency >> 8);
                            Rx_Data[4] = (uint8_t)duty_cycle;
                            Rx_Data[3] = (uint8_t)(duty_cycle >> 8);
                            Send_CAN_Message(Rx_Data, 0x8U);  // Send response
                        }
                        else
                        {
                            Rx_Data[0] += 0x40U;
                            Rx_Data[7] = status;  // Error code
                            Send_CAN_Message(Rx_Data, 0x8U);  // Send response with error
                        }
                        break;

                    default:
                        // Routine ID not defined
                        Rx_Data[0] += 0x40U;
                        Rx_Data[7] = Routine_ID_not_defined;
                        Send_CAN_Message(Rx_Data, 0x8U);
                        break;
                }
                break;

            case module_ADC_DMA_control_ID:  // Module ID for ADC DMA
                switch (Rx_Data[1])  // Routine ID = Rx_Data[1]
                {
                    case ADC_start:
                        Rx_Data[0] += 0x40U;  // Mark as response
                        Rx_Data[7] = 0U;
                        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, (uint32_t)BUFFER_SIZE) != HAL_OK)
                        {
                            Rx_Data[7] = Driver_error;
                        }
                        Send_CAN_Message(Rx_Data, 0x8U);  // Send response
                        break;

                    case ADC_get_values: // Get conversion values
                    {
                        uint8_t channel_index = Rx_Data[2];
                        if (channel_index >= 4 && channel_index <= 7)
                        {
                            uint16_t adc_value = adc_buffer[channel_index - 4]; // Extract value of chosen channel
                            Rx_Data[3] = (uint8_t)(adc_value & 0xFF); // Low byte
                            Rx_Data[4] = (uint8_t)((adc_value >> 8) & 0xFF); // High byte
                            Rx_Data[0] += 0x40U;
                            Rx_Data[7] = 0U; // Success
                        }
                        else
                        {
                            Rx_Data[0] += 0x40U;
                            Rx_Data[7] = Bad_parameter; // Invalid channel
                        }
                        Send_CAN_Message(Rx_Data, 0x8U); // Send response
                    }
                    break;

                    case ADC_stop:
                        Rx_Data[0] += 0x40U;  // Mark as response
                        Rx_Data[7] = 0U;
                        if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
                        {
                            Rx_Data[7] = Driver_error;
                        }
                        Send_CAN_Message(Rx_Data, 0x8U);  // Send response
                        break;

                    default:
                        // Routine ID not defined
                        Rx_Data[0] += 0x40U;
                        Rx_Data[7] = Routine_ID_not_defined;
                        Send_CAN_Message(Rx_Data, 0x8U);
                        break;
                }
                break;

            case module_PWM_Input:  // Module ID for PWM Input
                switch (Rx_Data[1])  // Routine ID = Rx_Data[1]
                {
                    case PWM_Input_Get: // Get PWM Input values
                        pwm = Handle_TIM_IC_Capture(channel);
                    	//pwm = Handle_PWM_Input();
                        if (pwm.dutyCycle > 0.0f && pwm.frequency > 0.0f)
                        {
                        	// Convert float values to integers
                            uint16_t dutyCycle_int = (uint16_t)pwm.dutyCycle;
                            uint16_t frequency_int = (uint16_t)pwm.frequency;

                            // Successfully retrieved PWM configuration
                            // Populate the response data
                            Rx_Data[0] += 0x40U;  // Mark as response
                            Rx_Data[6] = (uint8_t)(frequency_int & 0xFF); // Low byte of frequency
                            Rx_Data[5] = (uint8_t)((frequency_int >> 8) & 0xFF); // High byte of frequency
                            Rx_Data[4] = (uint8_t)(dutyCycle_int & 0xFF); // Low byte of duty cycle
                            Rx_Data[3] = (uint8_t)((dutyCycle_int >> 8) & 0xFF); // High byte of duty cycle
                            Rx_Data[7] = 0U; // No error
                        }
                        else
                        {
                            Rx_Data[0] += 0x40U;
                            Rx_Data[7] = Driver_error;  // Error code
                        }
                        Send_CAN_Message(Rx_Data, 0x8U);  // Send response
                        break;

                    default:
                        // Routine ID not defined
                        Rx_Data[0] += 0x40U;
                        Rx_Data[7] = Routine_ID_not_defined;
                        Send_CAN_Message(Rx_Data, 0x8U);
                        break;
                }
                break;

            default:
                // Module ID not defined
                Rx_Data[0] += 0x40U;
                Rx_Data[7] = Module_ID_not_defined;
                Send_CAN_Message(Rx_Data, 0x8U);
                break;
        }
    }

    // Call the default CAN interrupt handler
    HAL_CAN_IRQHandler(&hcan1);

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
	//int a;
  /* USER CODE BEGIN TIM5_IRQn 0 */
    // Vérifier si l'interruption est due à un événement de capture
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC1) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim5, TIM_IT_CC1) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
            pwm = Handle_TIM_IC_Capture( TIM_CHANNEL_1);
        }
    }
	/*GPIO_PinState x= HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
	if(x == GPIO_PIN_SET)
	{
		a++;
	}*/
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
