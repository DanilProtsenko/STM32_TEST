/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "DSP.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
states FSM_table[4][4] = {
  [initial][sign0] = initial,
  [initial][sign1] = state_led,
  [initial][sign2] = state_uart,
  [initial][sign3] = state_off,
  [state_led][sign0] = initial,
  [state_led][sign1] = initial,
  [state_led][sign2] = initial,
  [state_led][sign3] = initial,
  [state_uart][sign0] = initial,
  [state_uart][sign1] = initial,
  [state_uart][sign2] = initial,
  [state_uart][sign3] = initial,
  [state_off][sign0] = initial,
  [state_off][sign1] = initial,
  [state_off][sign2] = initial,
  [state_off][sign3] = initial
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define dhr(u) ((int32_t)((4095.0f/3.3f)*(u)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int test = 0;
uint32_t flag1 = 0;
uint32_t flag2 = 0;
uint8_t key_press = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void adc_init(void);
static void tim15_init(void);
static void tim7_init(void);
static void gp_init(void);
static void uart_init(void);
static void dac_init(void);
static void sys_clock(void);
signals getSignal(uint8_t* value);
states current_state = initial;
extern double y_value[ARRSIZE];
double newAvg[ARRSIZE] = {0};
double arrNumbers[125] = {0};
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  test = 0;
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  //SystemClock_Config();
  
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
  sys_clock();
  gp_init();
  adc_init();
  //tim15_init();
  tim7_init();
  uart_init();
  dac_init();
  NVIC_EnableIRQ(ADC1_2_IRQn);
  NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE END 2 */
  dsp();
  int pos = 0;
  double newAvg2 = 0;
  double sum = 0;
  double len = sizeof(arrNumbers) / sizeof(double);
  for(int i = 0; i < ARRSIZE; i++){
    newAvg2 = movAvg(arrNumbers, &sum, pos, len, y_value[i]);
    newAvg[i]= newAvg2;
    pos++;
    if (pos >= len){
      pos = 0;
    }
  }
   for(int i = 0; i < ARRSIZE; i++){
    newAvg2 = movAvg(arrNumbers, &sum, pos, len, y_value[i]);
    newAvg[i]= newAvg2;
    pos++;
    if (pos >= len){
      pos = 0;
    }
  }
  //int count = sizeof(y_value) / sizeof(double);
//  moving_average_t* sensor_av = allocate_moving_average(50);
//  for(int i = 0; i < ARRSIZE; i++)
//  {
//    newAvg[i] = movingAvg(sensor_av, y_value[i]);
//  }
//  for(int i = 0; i < ARRSIZE; i++)
//  {
//    newAvg[i] = movingAvg(sensor_av, y_value[i]);
//  }
//  free_moving_average(sensor_av);
//  sensor_av = NULL;
  //newAvg[ARRSIZE-1] =  newAvg[0];
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {     
//    signals current_signal = getSignal(&key_press);
//    states new_state = FSM_table[current_state][current_signal];
//    switch(new_state)
//    {
//      case initial:
//        break;
//      case state_led:
//        flag1 = flag1 ? 0:1;
//      if(GPIOA-> ODR && flag1 == 0)
//      {
//        uint32_t odr;
//        odr = GPIOA-> ODR;
//        GPIOA -> BSRR = ((odr & 0x0020) << 16u) | (~odr & 0x0020);
//      }
//        break;
//      case state_uart:
//        flag2 = flag2 ? 0:1;
//        break;
//      case state_off:
//       if(GPIOA-> ODR && flag1 == 0)
//      {
//        uint32_t odr;
//        odr = GPIOA-> ODR;
//        GPIOA -> BSRR = ((odr & 0x0020) << 16u) | (~odr & 0x0020);
//      }
//       flag1 = 0;
//       flag2 = 0;
//        break;
//      default:;
//    }
//    current_state = new_state;
//    /* USER CODE END WHILE */
//    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void sys_clock(void)
{
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
//  {
//    Error_Handler();
//  }
  FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
  RCC->CR |=   RCC_CR_MSIRANGE_6 | RCC_CR_MSION;

  RCC->PLLCFGR ^= 0x00003000;
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_MSI;
  RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
                  
  RCC->CFGR |= RCC_CFGR_SW;
  
  RCC->CR |= RCC_CR_PLLON;
}
static void gp_init(void)
{
  RCC->AHB2ENR|=RCC_AHB2ENR_GPIOAEN;
  //GPIOA->MODER &= 0xABFFF7FF;
  
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
  GPIOC->MODER &= 0xF3FFFFFF;
  GPIOC->PUPDR |= 0x04000000;

}
static void adc_init()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;
  RCC->CR |= RCC_CR_PLLSAI1ON;
  RCC->PLLSAI1CFGR |=  RCC_PLLSAI1CFGR_PLLSAI1R_1;
  RCC->PLLSAI1CFGR |=  RCC_PLLSAI1CFGR_PLLSAI1N_1;
  RCC->PLLSAI1CFGR |=  RCC_PLLSAI1CFGR_PLLSAI1REN;
  RCC->CCIPR |= RCC_CCIPR_ADCSEL_0;
  
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;  
  GPIOC->ASCR = 0x0010;
  ADC1->CR &= ~ADC_CR_DEEPPWD;
  ADC1->CR |= ADC_CR_ADVREGEN;
  ADC1->CR &= ~ADC_CR_ADEN;
  ADC1->CR &= ~ADC_CR_ADCALDIF;
  ADC1->CR |= ADC_CR_ADCAL;
  while((ADC1->CR & ADC_CR_ADCAL) != 0);
  //ADC1->CFGR |= ADC_CFGR_CONT;
  ADC1->SQR1 = 0x00000340;
  ADC1->SMPR1 |= ADC_SMPR1_SMP1_1;
  //ADC1->IER &= ~ADC_IER_OVRIE;
  //ADC1->IER |= ADC_IER_EOCIE;
  ADC1->CR |= ADC_CR_ADEN;
  while((ADC1->ISR & ADC_ISR_ADRDY) == 0);
  
  ADC1->CFGR |= ADC_CFGR_EXTEN_0;
  ADC1->CFGR |= 0x000380;
  
  ADC1->CR |= ADC_CR_ADSTART;
}
static void tim15_init(void)
{
  RCC -> APB2ENR |= RCC_APB2ENR_TIM15EN;
  TIM15 -> CR1 |= TIM_CR1_CEN;
  TIM15 -> DIER |= TIM_DIER_UIE;
  TIM15 -> PSC = 63999;
  TIM15 -> ARR = 1000;
  
  TIM15->CR2 |= TIM_CR2_MMS_1;
}
static void tim7_init(void)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
  TIM7->CR1 |= TIM_CR1_CEN;
  TIM7->DIER |= TIM_DIER_UIE;
  TIM7->PSC = 63999;
  TIM7->ARR = 10;
}
static void uart_init(void)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
  GPIOA->MODER &= ~GPIO_MODER_MODER2;
  GPIOA->MODER &= ~GPIO_MODER_MODER2;
  GPIOA->MODER |= GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1;
  GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED2 | GPIO_OSPEEDR_OSPEED3;
  GPIOA->AFR[0] = (GPIO_AFRL_AFSEL2_3 - 0x100) | (GPIO_AFRL_AFSEL3_3 - 0x1000);
  USART2->CR1 |= USART_CR1_UE;
  USART2->BRR = 0x22c;
  USART2->CR1 |= USART_CR1_TE;
  USART2->CR1 |= USART_CR1_RE;
}

static void dac_init(void)
{
  RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;
  DAC->CR|=DAC_CR_CEN1 | DAC_CR_CEN2;
  __NOP();
  DAC->CR=0;
  DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;
}
void print_to_USART(char* str)
{
  while(*str != '\0')
  {
    while((USART2->ISR & USART_ISR_TC) == 0){}
    USART2->TDR = (uint8_t)*str;
    str++;
  }
}

signals getSignal(uint8_t* value)
{
  uint8_t temp = *value;
  *value = 0;
  if (temp == 1)
  {
    test+=1;
    return sign1;
  }
  else if (temp == 2)
    return sign2;
  else if (temp == 3)
    return sign3;
  else
    return sign0; 
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
