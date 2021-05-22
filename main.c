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
#include "stm32l053xx.h"
#include "led.h"
#include "segment.h"
#include "button.h"
#include "sensor.h"
#include <stdio.h>
#include "gpio.h"
#include "usart.h"
//pour les interruptions
#include "stm32l0xx_ll_exti.h"




/* Private typedef -----------------------------------------------------------*/
void Bouton_Led_Polling(uint8_t *last_button_state, SEGMENT_TypeDef *led, BUTTON_TypeDef *button);

void SystemClock_Config(void);

/* Private variables ---------------------------------------------------------*/
LED_TypeDef led;
uint8_t n_pn;
SENSOR_TypeDef sensor;
uint8_t sensorValue;
uint8_t last_button_state=1;
BUTTON_TypeDef button;
SEGMENT_TypeDef segment;



/* function prototypes -----------------------------------------------*/

void AlternateFunction(GPIO_TypeDef * port, uint8_t pin, uint8_t af){
		uint8_t nb_port;
		nb_port=(uint32_t)((uint32_t *)port - IOPPERIPH_BASE)/ (uint32_t)0x400;
		/*Activer l’horloge */
		RCC->IOPENR|=1<<nb_port;
		/*configuration de la pin en mode alternate*/
		port->MODER&=~(0b11<<(2*pin));
		port->MODER|=(0b10<<(2*pin));

		/*Activer la fonction alternative « af »*/
		if (pin<8)
		{
			port->AFR[0] &=~(0b1111<<(4*pin));
			port->AFR[0] |=(af<<(4*pin));
		}
		else if (pin<16)
		{
			port->AFR[1]&=~(0b1111<<4*(pin-8));
			port->AFR[1]|=(af<<4*(pin-8));
		}

}




void Servo_PWM(TIM_TypeDef * timer, uint8_t canal, uint32_t HCLKFrequency, uint32_t PWMFrequency ,float duty_cycle) {
	/* Activer l’horloge du timer*/

	if(timer ==TIM2){
			   RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}else if(timer ==TIM22){
			   RCC->APB2ENR |= RCC_APB2ENR_TIM22EN;
		   }else if(timer ==TIM6){
			   RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	}else if(timer ==TIM21){
			   RCC->APB2ENR |= RCC_APB2ENR_TIM21EN;
	} timer->CR1|=TIM_CR1_DIR_Msk;
		   /*Activation du PWM */
		   if( canal ==1){
					  timer->CCMR1 &= ~TIM_CCMR1_OC1M_0;
					  timer->CCMR1 |= TIM_CCMR1_OC1M_1| TIM_CCMR1_OC1M_2;
					  timer->CCER |= TIM_CCER_CC1E;/*activation du PWM du canal*/
				  }
				  else if(canal==2){
					  timer->CCMR1 &= ~TIM_CCMR1_OC2M_0;
					  timer->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
					  timer->CCER |= TIM_CCER_CC2E;
				  }int ARR = HCLKFrequency/PWMFrequency - 1;
				  if (ARR>65535) {
					  /* la valeur du prescaler*/
					  timer->PSC=999;
					  timer->ARR = HCLKFrequency/(PWMFrequency*(timer->PSC+1))-1;

				  }else {
					  timer->ARR=ARR;

				  }
				  /*configuration la valeur du registre de comparaison*/
				  if(canal==1){
					  timer->CCR1 = (timer->ARR+1)*duty_cycle;
				  }
				  else if(canal==2){
					  timer->CCR2 = (timer->ARR+1)*duty_cycle;
				  } /*activatin du timer*/
				  timer->CR1|=TIM_CR1_CEN_Msk;

}

int main(void)
{

	 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	 LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	 LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

	 //sensor
	 Sensor_init(&sensor, ADC1, 8, 8);

	//configuration de l'horloge du système en 16MHz
	  RCC->IOPENR|= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;
	  LL_Init1msTick(16000000);



	 SystemClock_Config();


	 //configuration de la led sur la pin PC
	 Led_init(&led, GPIOC,5,6,8);

	 //configuration du bouton sur la pin PA10
	 Button_init(&button,GPIOA,8,LL_GPIO_PULL_DOWN);

	 //Configuration 7seg pin PA
	 Segment_init(&segment,GPIOA,6,7,8,9);



	 RCC->IOPENR|=RCC_IOPENR_GPIOAEN; GPIOA->MODER&=~(0b11<<(2*0));
	 GPIOA->MODER |= (0b01<<(2*0));

  while (1)
  {




    /* USER CODE END WHILE */
	  Bouton_Led_Polling(&last_button_state, &led, &button);


	   //Servo

	  	  AlternateFunction(GPIOC, 7, 0);
	      Servo_PWM(TIM22, 2, 16000000, 50,0.01);

	    //sensor
	      sensorValue = Set_Value(&sensor); //sensorValue < 15





	    //si bouton appuyé allumer seg,
  		  if ((Button_State(&button) == 1) || (sensorValue < 20)) {
  			  Led_turnOn(&led, 1);
  			  Segment_turnOn(&segment,1);
  			  Segment_turnOn(&segment,2);
  			  Segment_turnOn(&segment,3);
  			  Segment_turnOn(&segment,4);
  			   Servo_PWM(TIM22, 2, 16000000, 50,0.01);

  		  }
  		//si le sensor detecte un objet allumer seg
  		  else if (sensorValue < 20){
  			  Led_turnOn(&led, 1);
  			  Segment_turnOn(&segment,1);
  			  Segment_turnOn(&segment,2);
  			  Segment_turnOn(&segment,3);
  			  Segment_turnOn(&segment,4);

  			   Servo_PWM(TIM22, 2, 16000000, 50,0.01);


  		  }
  		  //sinon éteindre la seg
  		  else {
  			  Led_turnOff(&led,1);
  			  Segment_turnOff(&segment,1);
  			  Segment_turnOff(&segment,2);
  			  Segment_turnOff(&segment,3);
  			  Segment_turnOff(&segment,4);
  		  }


  }
  }




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




void Bouton_Led_Polling(uint8_t *last_button_state, SEGMENT_TypeDef *led, BUTTON_TypeDef *button){



	   uint8_t current_button_state = Button_State(button);
	    if((*last_button_state==1) && (current_button_state!=1)){
	        Led_toggle(led, 1);
	        Led_toggle(led, 2);
	        Led_toggle(led, 3);
	        LL_mDelay(20);
	    }

	    if((*last_button_state!=1) &&  (current_button_state==1)){
	        LL_mDelay(20);
	    }

	    *last_button_state = current_button_state;

	}



/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

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
