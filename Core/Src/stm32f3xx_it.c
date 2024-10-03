/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
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
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liquidcrystal_i2c.h"
#include "alarm_system.h"
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
double raw = 0.0;
char buffer[5];
uint8_t idx = 0;
uint16_t buzzer_length_counter = 0;

uint8_t alarm_rhythm_counter = 0;
bool triggered = false;
bool setting = false;
bool door_opened = false;
bool overwritten = false;
uint8_t button_pressed = 10;

char m[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Update_Buffer(char val);
void Generate_Tone(bool enable, uint16_t tone_length);
void Generate_Silenece(int length);
static inline void Check_IR_Signal(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
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
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

	// Simple software debouncing
	for (int i = 0; i < 65535; i++);

	if (HAL_GPIO_ReadPin(LUB_GPIO_Port, LUB_Pin)) {
		// If the system cannot be locked, try to unlock the system
		Check_IR_Signal();

		user_input = buffer;
		// The door has to be closed for the system to be able to lock
		if (raw <= 1000.0 || !Lock_System())	{
			if (Unlock_System()) {
				HAL_GPIO_WritePin(SM_GPIO_Port, SM_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(RM_GPIO_Port, RM_Pin, GPIO_PIN_SET);

				setting = false;
				triggered = false;
				alarm_rhythm_counter = 0;
				user_input = NULL;
				Generate_Tone(false, 0);
			} else {
				alarm_rhythm_counter = 2;
				Generate_Tone(false, OPEN_ON_SET_SILENT_LENGTH);
			}
		} else {
			HAL_GPIO_WritePin(SM_GPIO_Port, SM_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RM_GPIO_Port, RM_Pin, GPIO_PIN_RESET);

			setting = true;
			alarm_rhythm_counter = LOCK_COUNTDOWN_COUNT;
			Generate_Tone(true, LOCK_COUNTDOWN_BEEP_LEGNTH);
		}


		// Reset input from number pad
		buffer[0] = '\0';
		idx = 0;
	}

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(LUB_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

	// Simple software debouncing
	for (int i = 0; i < 65535; i++);

	if (HAL_GPIO_ReadPin(RPB_GPIO_Port, RPB_Pin)) {
//		HD44780_Clear();
//		buffer[0] = '\0';
//		idx = 0;
	}

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(RPB_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if (!buzzer_length_counter--) {

		if (setting) {
			alarm_rhythm_counter--;

			Generate_Tone(alarm_rhythm_counter % 2 == 0 ? true : false, LOCK_COUNTDOWN_BEEP_LEGNTH);
			if (alarm_rhythm_counter == 0)		setting = false;
		} else if (triggered && !overwritten) {
			alarm_rhythm_counter++;
			alarm_rhythm_counter %= 6;

			Generate_Tone(alarm_rhythm_counter % 2 == 0 ? false : true, alarm_rhythm_counter == 2 ? OPEN_ON_SET_SILENT_LENGTH : OPEN_ON_SET_BEEP_LENGTH);
		} else {
			buzzer_length_counter = 0;

			if (overwritten) {
				overwritten = false;
				if (triggered) {
					alarm_rhythm_counter = 2;
					Generate_Tone(false, OPEN_ON_SET_SILENT_LENGTH);
				}
			}

			Generate_Tone(false, 0);
		}
	}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

	// Poll for the value of the IR sensor
	Check_IR_Signal();

	sprintf(m, "%f\r\n", raw);
	HAL_UART_Transmit(&huart2, (uint8_t*) m, 50, 100);

	// At raw < 1000.0, the door has been opened enough to trigger the alarm
	if (raw < 800.0) {
		// BEGIN TO SOUND THE ALARM
		if (__GET_SYSTEM_STATE == ready || setting) {
			if (!door_opened) {
				Generate_Tone(true, OPEN_ON_READY_BEEP_LENGTH);

				door_opened = true;
			}
		} else {
			if (!door_opened) {
				triggered = true;
				alarm_rhythm_counter = 1;
				Generate_Tone(true, OPEN_ON_SET_BEEP_LENGTH);

				door_opened = true;
			}
		}
	} else {
		door_opened = false;
	}

	// Poll for the number pad
	if        (!HAL_GPIO_ReadPin(NP0_GPIO_Port, NP0_Pin)) {
		if (button_pressed != 0) {
			button_pressed = 0;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('0');
		}
	} else if (!HAL_GPIO_ReadPin(NP1_GPIO_Port, NP1_Pin)) {
		if (button_pressed != 1) {
			button_pressed = 1;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('1');
		}
	} else if (!HAL_GPIO_ReadPin(NP2_GPIO_Port, NP2_Pin)) {
		if (button_pressed != 2) {
			button_pressed = 2;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('2');
		}
	} else if (!HAL_GPIO_ReadPin(NP3_GPIO_Port, NP3_Pin)) {
		if (button_pressed != 3) {
			button_pressed = 3;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('3');
		}
	} else if (!HAL_GPIO_ReadPin(NP4_GPIO_Port, NP4_Pin)) {
		if (button_pressed != 4) {
			button_pressed = 4;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('4');
		}
	} else if (!HAL_GPIO_ReadPin(NP5_GPIO_Port, NP5_Pin)) {
		if (button_pressed != 5) {
			button_pressed = 5;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('5');
		}
	} else if (!HAL_GPIO_ReadPin(NP6_GPIO_Port, NP6_Pin)) {
		if (button_pressed != 6) {
			button_pressed = 6;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('6');
		}
	} else if (!HAL_GPIO_ReadPin(NP7_GPIO_Port, NP7_Pin)) {
		if (button_pressed != 7) {
			button_pressed = 7;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('7');
		}
	} else if (!HAL_GPIO_ReadPin(NP8_GPIO_Port, NP8_Pin)) {
		if (button_pressed != 8) {
			button_pressed = 8;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('8');
		}
	} else if (!HAL_GPIO_ReadPin(NP9_GPIO_Port, NP9_Pin)) {
		if (button_pressed != 9) {
			button_pressed = 9;
			overwritten = true;

			Generate_Tone(true, INPUT_BEEP_LENGTH);
			Update_Buffer('9');
		}
	}





		if      (HAL_GPIO_ReadPin(NP0_GPIO_Port, NP0_Pin) && button_pressed == 0)		button_pressed = 10;
		else if (HAL_GPIO_ReadPin(NP1_GPIO_Port, NP1_Pin) && button_pressed == 1) 		button_pressed = 10;
		else if (HAL_GPIO_ReadPin(NP2_GPIO_Port, NP2_Pin) && button_pressed == 2) 		button_pressed = 10;
		else if (HAL_GPIO_ReadPin(NP3_GPIO_Port, NP3_Pin) && button_pressed == 3) 		button_pressed = 10;
		else if (HAL_GPIO_ReadPin(NP4_GPIO_Port, NP4_Pin) && button_pressed == 4) 		button_pressed = 10;
		else if (HAL_GPIO_ReadPin(NP5_GPIO_Port, NP5_Pin) && button_pressed == 5) 		button_pressed = 10;
		else if (HAL_GPIO_ReadPin(NP6_GPIO_Port, NP6_Pin) && button_pressed == 6) 		button_pressed = 10;
		else if (HAL_GPIO_ReadPin(NP7_GPIO_Port, NP7_Pin) && button_pressed == 7) 		button_pressed = 10;
		else if (HAL_GPIO_ReadPin(NP8_GPIO_Port, NP8_Pin) && button_pressed == 8)		button_pressed = 10;
		else if (HAL_GPIO_ReadPin(NP9_GPIO_Port, NP9_Pin) && button_pressed == 9)		button_pressed = 10;

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void Update_Buffer(char val) {
	HD44780_SetCursor(idx, 0);
	HD44780_PrintStr((char*)&val);

	if (idx < 4)
		buffer[idx++] = val;
}

void Generate_Tone(bool enable, uint16_t tone_length) {
	if (enable) {
		__HAL_TIM_SET_AUTORELOAD(&htim1, ENABLE_BEEP * 2);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ENABLE_BEEP);
	} else {
		__HAL_TIM_SET_AUTORELOAD(&htim1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	}

	buzzer_length_counter = tone_length;
}

void Generate_Silenece(int length) {
	buzzer_length_counter = (uint16_t) length;
}

static inline void Check_IR_Signal(void) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	raw = (double) HAL_ADC_GetValue(&hadc1);
}
/* USER CODE END 1 */
