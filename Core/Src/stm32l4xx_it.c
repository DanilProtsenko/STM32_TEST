/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"
#include "stdio.h"
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
float voltage1;
uint32_t value;
char voltage_str[50];
extern uint32_t flag1;
extern uint32_t flag2;
extern double y_value[ARRSIZE];
extern double newAvg[ARRSIZE];
uint32_t danil;

uint32_t pres_pc13; //Нажатие 
uint32_t prev_pc13 = GPIO_IDR_ID13; //Предыдущее значение кнопки
uint32_t cnt_pc13 = 0; //счетчик
uint8_t key_press_counter = 0;
uint32_t timer_intr_counter = 0;
//int tick = 0;
extern uint8_t key_press;
uint8_t* ptr_key_press = &key_press;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void Debounce(uint32_t* pres_px, uint32_t* cnt_px, uint32_t* prev_px, uint32_t GPIOx_IDR);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Debounce(uint32_t* pres_px, uint32_t* cnt_px, uint32_t* prev_px, uint32_t GPIOx_IDR)
{
	/* Read current state */
  uint32_t cur_px = GPIOx_IDR;
	/* If level has changed  */
	if (cur_px != *prev_px) 
    {
      (*cnt_px)++;
      if (*cnt_px >= 4)
      {
        *prev_px = cur_px;
        *cnt_px = 0;
        if(cur_px == 0)
        {
          *pres_px = 1;
        }
      }
    }
  else
  {
    *cnt_px = 0;
  }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

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
  * @brief This function handles Prefetch fault, memory access fault.
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
  static int tick = 0;
  DAC->DHR12R1=dhr(y_value[tick]);
  DAC->DHR12R2=dhr(newAvg[tick]);
  tick++;
  if (tick>=ARRSIZE)
    tick=0;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */
void TIM7_IRQHandler(void)
{
    TIM7 -> SR &= ~TIM_SR_UIF;
    timer_intr_counter++;
    Debounce(&pres_pc13, &cnt_pc13, &prev_pc13, GPIOC->IDR);
    if (pres_pc13)
    {
      pres_pc13 = 0;
      key_press_counter++;
      timer_intr_counter = 0;
    }
    else if(timer_intr_counter>=50 && key_press_counter != 0)
    { 
      *ptr_key_press = key_press_counter;
      key_press_counter = 0;
      timer_intr_counter = 0;
    }
}

void ADC1_2_IRQHandler(void)
{
  ADC1->ISR |= ADC_ISR_EOC;
}
void TIM1_BRK_TIM15_IRQHandler(void)
{
  TIM15 -> SR &= ~TIM_SR_UIF;
  danil++;
  if(flag1)
  {
    uint32_t odr;
    odr = GPIOA-> ODR;
    GPIOA -> BSRR = ((odr & 0x0020) << 16u) | (~odr & 0x0020);
  }
  if(flag2)
  {
    value = ADC1->DR;
    voltage1 = 3.3f * value / 4096.0f;
    sprintf(voltage_str,"Напряжение:%.3fВ\n", voltage1);
    print_to_USART(voltage_str);
  }

}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
