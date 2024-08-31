/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPIO_Input_Number 4U //4 pinsPD12 ,PD13 ,PD14 , PD15 (0,1,2,3)
#define IRQ_Enable (CAN_IT_TX_MAILBOX_EMPTY         \
                    | CAN_IT_RX_FIFO0_MSG_PENDING   \
                    | CAN_IT_RX_FIFO0_FULL          \
                    | CAN_IT_RX_FIFO0_OVERRUN       \
                    | CAN_IT_RX_FIFO1_MSG_PENDING   \
                    | CAN_IT_RX_FIFO1_FULL          \
                    | CAN_IT_RX_FIFO1_OVERRUN)

#define Driver_error 0xFBU
#define Bad_parameter 0xFEU
#define Module_ID_not_defined 0xFAU
#define Routine_ID_not_defined 0xFFU
#define pwm_channel_max 4U
#define duty_cycle_max 0x2710U   //duty cycle =100% equivalent to 10000(HEX 2710)  //precision=0.01%


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t Rx_Data[8];
uint8_t Tx_Data[8] = {0};
uint32_t TxMailbox;
GPIO_TypeDef* GPIOx;
uint16_t led_matrix[GPIO_Input_Number] ={GPIO_PIN_12,  GPIO_PIN_13,  GPIO_PIN_14,  GPIO_PIN_15};
typedef enum
{
	LED_RESET,   //reset=0
    LED_SET      //set=1

} LED_Action;


static uint32_t IC_Val1 = 0, IC_Val2 = 0, IC_Val3 = 0;
static uint8_t CaptureCount = 0;
uint8_t edgeType = 0;


typedef struct {
    uint16_t dutyCycle;
    uint16_t frequency;
} PWM_Measurement;

PWM_Measurement pwm = {0, 0};  // Initialize with default values
uint32_t HighTime = 0, Period = 0,LowTime=0;



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


void Send_CAN_Message(uint8_t *data, uint32_t length)  //variable data length
{
    TxHeader.DLC = length;
    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK)
    {
        Error_Handler();          // Handle transmission error
    }
}


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


PWM_Measurement Handle_TIM_IC_Capture(uint32_t channel)
{
    static uint8_t edgeType = 0;  // Type de front (0: indéfini, 1: montant, 2: descendant)
    static uint32_t previousIC_Val1 = 0;  // Valeur précédente pour détecter les erreurs
    static uint8_t lastState = 0;  // Stocke le dernier état du front (0: descendant, 1: montant)
    uint8_t currentState = 0;  // Stocke l'état actuel du front

    if (channel == TIM_CHANNEL_1)
    {
        switch (CaptureCount)
        {
            case 0: // Première capture (front montant ou descendant)
                IC_Val1 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

                // Si IC_Val1 est identique à la valeur précédente, il peut s'agir d'une erreur
                if (IC_Val1 == previousIC_Val1)
                {
                    // Ignore cette capture, attends le prochain front
                    break;
                }

                // Détermine le type de front actuel
                GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
                currentState = (pinState == GPIO_PIN_SET) ? 1 : 0;  // 1: front montant, 0: front descendant

                // S'assure que le premier front est montant
                if (currentState == 1)
                {
                    edgeType = 1;  // Premier front est montant, continuer normalement
                    previousIC_Val1 = IC_Val1;  // Met à jour la valeur précédente
                    CaptureCount = 1;
                    lastState = currentState;  // Stocke le dernier état
                }
                else
                {
                    // Si le premier front n'est pas montant, attends le bon front
                    break;
                }
                break;

            case 1: // Deuxième capture (front descendant)
                IC_Val2 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

                // Met à jour l'état actuel en fonction du front détecté
                pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
                currentState = (pinState == GPIO_PIN_SET) ? 1 : 0;  // 1: front montant, 0: front descendant

                // Ne continue que si la transition de front est correcte (descendant après montant)
                if (lastState == 1 && currentState == 0)
                {
                    edgeType = 2;  // Front descendant valide après montant
                    CaptureCount = 2;
                }
                else
                {
                    // Ignore les transitions incorrectes
                    break;
                }
                break;

            case 2: // Troisième capture (front montant à nouveau)
                IC_Val3 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

                // Vérifie si le edgeType correspond à la séquence correcte de transitions
              while(edgeType == 2) // Si le premier front était montant
                {
                	;
                }

                    // Calcul du HighTime (temps haut)
                    if (IC_Val2 >= IC_Val1)
                    {
                        HighTime = IC_Val2 - IC_Val1;
                    }
                    else // Gérer le débordement
                    {
                        HighTime = (0xFFFF - IC_Val1 + IC_Val2 + 1);
                    }

                    // Calcul de la période (Period)
                    if (IC_Val3 >= IC_Val1)
                    {
                        Period = IC_Val3 - IC_Val1;
                    }
                    else // Gérer le débordement
                    {
                        Period = (0xFFFF - IC_Val1 + IC_Val3 + 1);
                    }

                    // Calcul de la fréquence et du duty cycle
                    uint32_t timerClock = HAL_RCC_GetPCLK1Freq() / (htim5.Init.Prescaler + 1);
                    if (Period != 0)
                    {
                        pwm.frequency = (uint16_t)(timerClock / Period);
                        pwm.dutyCycle = (uint16_t)(((float)HighTime / (float)Period) * 100.0f);

                        if (pwm.dutyCycle > 100)
                        {
                            pwm.dutyCycle = 100;
                        }
                    }
                    else
                    {
                        pwm.frequency = Bad_parameter;
                        pwm.dutyCycle = Bad_parameter;
                    }

                // Prépare pour le prochain cycle
                IC_Val1 = IC_Val3;  // Commence le prochain cycle avec IC_Val3
                CaptureCount = 1;   // Réinitialise CaptureCount pour continuer depuis le deuxième front
                lastState = currentState;  // Met à jour le dernier état pour la prochaine capture
                break;
        }
    }
    else
    {
        // Gère le cas où le canal n'est pas TIM_CHANNEL_1
        pwm.frequency = Bad_parameter;
        pwm.dutyCycle = Bad_parameter;
    }

    return pwm;
}


/*PWM_Measurement Handle_TIM_IC_Capture(uint32_t channel)
{
    static uint8_t edgeType = 0;  // Edge type (0: undefined, 1: rising, 2: falling)
    static uint32_t previousIC_Val1 = 0;  // Store previous value to detect errors
    static uint8_t lastState = 0;  // Store the last edge state (0: falling, 1: rising)
    uint8_t currentState = 0;  // Store the current edge state

    if (channel == TIM_CHANNEL_1)
    {
        switch (CaptureCount)
        {
            case 0: // First capture (rising or falling edge)
                IC_Val1 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

                // If IC_Val1 is identical to the previous value, it might be an error
                if (IC_Val1 == previousIC_Val1)
                {
                    // Ignore this capture, wait for the next edge
                    break;
                }

                // Determine the current edge type
                GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
                if (pinState == GPIO_PIN_SET)
                {
                    currentState = 1;  // Rising edge
                }
                else if (pinState == GPIO_PIN_RESET)
                {
                    currentState = 0;  // Falling edge
                }

                // Ensure the first edge is rising
                if (currentState == 1)
                {
                    edgeType = 1;  // First edge is rising, proceed with normal flow
                    previousIC_Val1 = IC_Val1;  // Update previous value
                    CaptureCount = 1;
                    lastState = currentState;  // Store the last state
                }
                else
                {
                    // If the first edge is not rising, wait for the correct edge
                    break;
                }
                break;

            case 1: // Second capture (falling edge)
                IC_Val2 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

                // Update current state based on the edge detected
                pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
                currentState = (pinState == GPIO_PIN_SET) ? 1 : 0;

                // Only proceed if the edge transition is correct (falling after rising)
                if (lastState == 1 && currentState == 0)
                {
                    edgeType = 2;  // Valid falling edge after rising
                    CaptureCount = 2;
                }
                else
                {
                    // Ignore incorrect transitions
                    break;
                }
                break;

            case 2: // Third capture (rising edge again)
                IC_Val3 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

                if (edgeType == 1) // If the first edge was rising
                {
                    // Calculate high time (HighTime)
                    if (IC_Val2 >= IC_Val1)
                    {
                        HighTime = IC_Val2 - IC_Val1;
                    }
                    else // Handle overflow
                    {
                        HighTime = (0xFFFF - IC_Val1 + IC_Val2 + 1);
                    }

                    // Calculate period (Period)
                    if (IC_Val3 >= IC_Val1)
                    {
                        Period = IC_Val3 - IC_Val1;
                    }
                    else // Handle overflow
                    {
                        Period = (0xFFFF - IC_Val1 + IC_Val3 + 1);
                    }

                    // Calculate frequency and duty cycle
                    uint32_t timerClock = HAL_RCC_GetPCLK1Freq() / (htim5.Init.Prescaler + 1);
                    if (Period != 0)
                    {
                        pwm.frequency = (uint16_t)(timerClock / Period);
                        pwm.dutyCycle = (uint16_t)(((float)HighTime / (float)Period) * 100.0f);

                        if (pwm.dutyCycle > 100)
                        {
                            pwm.dutyCycle = 100;
                        }
                    }
                    else
                    {
                        pwm.frequency = Bad_parameter;
                        pwm.dutyCycle = Bad_parameter;
                    }
                }

                // Prepare for the next cycle
                IC_Val1 = IC_Val3;  // Start the next cycle with IC_Val3
                CaptureCount = 1;   // Reset CaptureCount to continue from the second edge
                lastState = currentState;  // Update last state for the next capture
                break;
        }
    }
    else
    {
        // Handle the case where the channel is not TIM_CHANNEL_1
        pwm.frequency = Bad_parameter;
        pwm.dutyCycle = Bad_parameter;
    }

    return pwm;
}
*/



/*PWM_Measurement Handle_TIM_IC_Capture(uint32_t channel)
{
    static uint8_t edgeType = 0;  // Edge type (0: undefined, 1: rising, 2: falling)
    static uint32_t previousIC_Val1 = 0;  // Store previous value to detect errors

    if (channel == TIM_CHANNEL_1)
    {
        switch (CaptureCount)
        {
            case 0: // First capture (rising or falling edge)
                IC_Val1 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

                // If IC_Val1 is identical to the previous value, it might be an error
                if (IC_Val1 == previousIC_Val1)
                {
                    // Ignore this capture, wait for the next edge
                    break;
                }

                // Determine edge type
                GPIO_PinState pinState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
                if (pinState == GPIO_PIN_SET)
                {
                    edgeType = 1;  // Rising edge
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
                    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);  // LED for Falling edge
                }
                else if (pinState == GPIO_PIN_RESET)
                {
                    edgeType = 2;  // Falling edge
                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
                    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);  // LED for Falling edge
                }

                previousIC_Val1 = IC_Val1;  // Update previous value
                CaptureCount = 1;
                break;

            case 1: // Second capture (opposite edge)
                IC_Val2 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);
                CaptureCount = 2;
                break;

            case 2: // Third capture (same as first edge)
                IC_Val3 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_1);

                if (edgeType == 1) // If the first edge was rising
                {
                    // Calculate high time (HighTime)
                    if (IC_Val2 >= IC_Val1)
                    {
                        HighTime = IC_Val2 - IC_Val1;
                    }
                    else // Handle overflow
                    {
                        HighTime = (0xFFFF - IC_Val1 + IC_Val2 + 1);
                    }

                    // Calculate period (Period)
                    if (IC_Val3 >= IC_Val1)
                    {
                        Period = IC_Val3 - IC_Val1;
                    }
                    else // Handle overflow
                    {
                        Period = (0xFFFF - IC_Val1 + IC_Val3 + 1);
                    }

                    // Calculate frequency and duty cycle
                    uint32_t timerClock = HAL_RCC_GetPCLK1Freq() / (htim5.Init.Prescaler + 1);
                    if (Period != 0)
                    {
                        pwm.frequency = (uint16_t)(timerClock / Period);
                        pwm.dutyCycle = (uint16_t)(((float)HighTime / (float)Period) * 100.0f);

                        if (pwm.dutyCycle > 100)
                        {
                            pwm.dutyCycle = 100;
                        }
                    }
                    else
                    {
                        pwm.frequency = Bad_parameter;
                        pwm.dutyCycle = Bad_parameter;
                    }
                }
                else if (edgeType == 2) // If the first edge was falling
                {
                    // Calculate low time (LowTime)
                    if (IC_Val2 >= IC_Val1)
                    {
                        LowTime = IC_Val2 - IC_Val1;
                    }
                    else // Handle overflow
                    {
                        LowTime = (0xFFFF - IC_Val1 + IC_Val2 + 1);
                    }

                    // Calculate period (Period)
                    if (IC_Val3 >= IC_Val1)
                    {
                        Period = IC_Val3 - IC_Val1;
                    }
                    else // Handle overflow
                    {
                        Period = (0xFFFF - IC_Val1 + IC_Val3 + 1);
                    }

                    // Calculate frequency and duty cycle
                    uint32_t timerClock = HAL_RCC_GetPCLK1Freq() / (htim5.Init.Prescaler + 1);
                    if (Period != 0)
                    {
                        pwm.frequency = (uint16_t)(timerClock / Period);
                        pwm.dutyCycle = 100 - (uint16_t)(((float)LowTime / (float)Period) * 100.0f);

                        if (pwm.dutyCycle > 100)
                        {
                            pwm.dutyCycle = 100;
                        }
                    }
                    else
                    {
                        pwm.frequency = Bad_parameter;
                        pwm.dutyCycle = Bad_parameter;
                    }
                }

                // Prepare for the next cycle
                IC_Val1 = IC_Val3;  // Start the next cycle with IC_Val3
                CaptureCount = 1;   // Reset CaptureCount to continue from the second edge
                break;
        }
    }
    else
    {
        // Handle the case where the channel is not TIM_CHANNEL_1
        pwm.frequency = Bad_parameter;
        pwm.dutyCycle = Bad_parameter;
    }

    return pwm;
}
*/


void CAN_ConfigFilter(uint32_t filterBank, uint32_t id, uint32_t idType)
{
    CAN_FilterTypeDef canFilterConfig;  // Declare a filter configuration structure
    canFilterConfig.FilterBank = filterBank;   // Set the filter bank number to configure
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // Set the filter mode to ID mask
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;// Set the filter scale to 32 bits (for extended IDs)
    canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;  // Assign the filter to FIFO0 (First In First Out)
    canFilterConfig.FilterActivation = ENABLE;   // Activate the filter
    if (idType == CAN_ID_STD)
    {
        // Configure the standard ID filter
        canFilterConfig.FilterIdHigh = id << 5; // Shift the ID left by 5 bits for the high part
        canFilterConfig.FilterIdLow = 0x0000;   // Low part is not used for standard IDs
        // Set the mask for standard IDs (allow any ID matching the higher bits)
        canFilterConfig.FilterMaskIdHigh = 0xFFFF << 5; // Mask high part
        canFilterConfig.FilterMaskIdLow = 0x0000;       // Mask low part
    }
    else if (idType == CAN_ID_EXT)
    {
        // Configure the extended ID filter
        canFilterConfig.FilterIdHigh = (id >> 13) & 0xFFFF; // Get the high part of the extended ID
        canFilterConfig.FilterIdLow = (id << 3) & 0xFFFF;   // Get the low part of the extended ID
        // Set the mask for extended IDs
        canFilterConfig.FilterMaskIdHigh = (0x1FFFFFFF >> 13) & 0xFFFF; // Max mask high part
        canFilterConfig.FilterMaskIdLow = (0x1FFFFFFF << 3) & 0xFFFF;   // Max mask low part
    }
    if (HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig) != HAL_OK)  // Configure the CAN filter with the defined settings
    {
        Error_Handler();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
  //simplify the timer configuration and interrupt handling code by using a single channel to capture both edges of the signal

  if (HAL_CAN_ActivateNotification(&hcan1,IRQ_Enable) != HAL_OK)  //activate the notification after successful transmission
  {
      Error_Handler();
  }
  HAL_TIM_Base_Start(&htim2);
  HAL_CAN_Start(&hcan1);

  // Configuration of header of message CAN
  TxHeader.DLC=0x8U; // Length data (8 bytes )
  TxHeader.IDE=CAN_ID_STD; // standard Identifier
  TxHeader.RTR=CAN_RTR_DATA; // Data frame
  TxHeader.StdId=0x400U;// standard Identifier
  TxHeader.TransmitGlobalTime=DISABLE;   //not include a global timestamp
  CAN_ConfigFilter(0, 0x466, CAN_ID_STD);   //accepted ID  =0x466

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  HAL_StatusTypeDef status;
  status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, Tx_Data, &TxMailbox);
  if (status != HAL_OK)
  {
      Error_Handler();
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 4;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 39;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 39999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 39;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
