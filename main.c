/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"


USART_TypeDef servo;

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


void ADC_init(ADC_TypeDef * adc, uint8_t resolution, uint8_t channel) ;

void AlternateFunction(GPIO_TypeDef * port, uint8_t pin, uint8_t af);

void PWM(TIM_TypeDef * timer, uint8_t canal, uint32_t HCLKFrequency, uint32_t PWMFrequency,float duty_cycle) ;


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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  AlternateFunction(GPIOB, 4, 1);
   PWM(TIM2, 2, 16000000, 5,0.25);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	 // while((ADC1->ISR& ADC_ISR_EOC)==0);

	 	 /* if(GPIOC->IDR &(1<<7)) Led_turnOn(&led);
	 	  else Led_turnOff(&led);*/


	 		  Servo_turnOn(&servo);


    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }

  LL_Init1msTick(16000000);

  LL_SetSystemCoreClock(16000000);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */


/* USER CODE BEGIN 4 */

void ADC_init(ADC_TypeDef * adc, uint8_t resolution, uint8_t channel) {

	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    //configuration de l'horloge en mode PCLK
        adc->CFGR2|=(0b11<<ADC_CFGR2_CKMODE_Pos); // 30

        //Activation de la conversion en continu
        	adc->CFGR1|=(0b1<<ADC_CFGR1_CONT_Pos);
        	ADC_CFGR1_CONT == 1;

        //choix de la résolution
		   if (resolution == 12){
				adc->CFGR1 &= ~(0b11<<3U); //
			}else if (resolution == 10){
				adc->CFGR1 &= ~(0b1<<4);
				adc->CFGR1 |= (0b1<<3);
			}else if (resolution == 8){
				adc->CFGR1 |= (0b1<<4);
				adc->CFGR1 &= ~(0b1<<3);
			}else if (resolution == 6){
				adc->CFGR1 |= (0b11<<3U);
			}

    	adc->CHSELR|=0b1<<channel; //selection de channel


        adc->CR|=0b1<<ADC_CR_ADEN_Pos; //activer le convertisseur


        adc->CR|=0b1<<ADC_CR_ADSTART_Pos; //lancer la conversion


    }


	void AlternateFunction(GPIO_TypeDef * port, uint8_t pin, uint8_t af){

			/*Activation de l'horlogr*/
			uint8_t nb_port;
			nb_port=(uint32_t)((uint32_t *)port - IOPPERIPH_BASE)/ (uint32_t)0x400;
			RCC->IOPENR|=1<<nb_port;

			/*configuration de pin PC7 en alternate*/
			GPIOC->MODER&=~(0b11<<(7*2));
			GPIOC->MODER|=(0b10<<(7*2));
			GPIOC->AFR[0]&=~(0b1111<<((pin-8)*4)); /*AFR[0] représenteAFRL et AFR[1] AFRH*/

	}

	void PWM(TIM_TypeDef * timer, uint8_t canal, uint32_t HCLKFrequency, uint32_t PWMFrequency,float duty_cycle){

		if (timer == TIM22){
				RCC->APB2ENR|= RCC_APB2ENR_TIM22EN;
			} else if(timer == TIM21){
				RCC->APB2ENR|= RCC_APB2ENR_TIM21EN;

			}else if(timer == TIM6){
				RCC->APB1ENR|= RCC_APB1ENR_TIM6EN;
			}else if(timer == TIM2){
				RCC->APB1ENR|= RCC_APB1ENR_TIM2EN;
			}

			//configuration en mode1 PWM de canal2
			timer->CCMR1&= ~TIM_CCMR1_OC2M_0;
			timer->CCMR1|= TIM_CCMR1_OC2M_1| TIM_CCMR1_OC2M_2;

			timer->CCER|= TIM_CCER_CC2E;  //activation de canal2


			timer->ARR= HCLKFrequency / (PWMFrequency *1000)-1;

			timer->CCR1=799;
			timer->CCR1=200; //25%



			timer->CR1|=1; //lancement de timer

	}



/* USER CODE END 4 */


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
