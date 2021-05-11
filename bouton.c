

#include "bouton.h"
#include "stm32l0xx_ll_exti.h"

void Bouton_init(BOUTON_TypeDef *bouton, GPIO_TypeDef * port, uint8_t pn,
		uint8_t pl) {

	bouton->gpioPort = port;
	bouton->pin = pn;
	bouton->pull = pl;

	//Activation de l'horloge sur le port en question
	//1-déterminer le numéro du port 0--> GPIOA, 1-->GPIOB, etc.
	uint8_t nb_port;
	nb_port = ((uint32_t) port - IOPPERIPH_BASE) / 0x400;
	//2-activation de l'hologe
	RCC->IOPENR |= 1 << nb_port;

	//configuration de la pin en entrée
	bouton->gpioPort->MODER &= ~(0b11 << 2 * pn);

	//configuration du type de pull
	bouton->gpioPort->PUPDR &= ~(0b11 << 2 * pn);
	bouton->gpioPort->PUPDR |= (pl << 2 * pn);
}

uint8_t Bouton_State(BOUTON_TypeDef *bouton) {
	if(bouton->gpioPort->IDR & (1<<bouton->pin))
	    {
	        return 1;
	    }else
	        return 0;
}

void Bouton_enableIRQ(BOUTON_TypeDef *bouton, uint8_t trigger) {

	//activation de l'interruption externe
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/*chercher à quel registre EXTI de SYSCFG on doit accéder parmi les quatre disponibles*/
	//1-détermination du numéro d'EXTI
	uint8_t nb_EXTI = bouton->pin / 4;
	//2-déterminer le numéro du port 0--> GPIOA, 1-->GPIOB, etc.
	uint8_t nb_port;
	nb_port = ((uint32_t) bouton->gpioPort - IOPPERIPH_BASE) / 0x400;
	//3-configuration du registre EXTI de SYSCFG
	SYSCFG->EXTICR[nb_EXTI] &= ~(0b1111 << 4 * (bouton->pin % 4));
	SYSCFG->EXTICR[nb_EXTI] |= (nb_port << 4 * (bouton->pin % 4));


	//activation d'EXTI dans IMR pour qu'elle ne soit pas masquée
	EXTI->IMR |= (1 << bouton->pin);

        //activation du type du trigger (front montant/descendant)
	switch (trigger) {
	case LL_EXTI_TRIGGER_RISING:
	//activation de l'interruption sur front montant dans RTSR
		EXTI->RTSR|=(1 << bouton->pin);
		break;
	case LL_EXTI_TRIGGER_FALLING:
	//activation de l'interruption sur front descendant dans FTSR
		EXTI->FTSR|=(1 << bouton->pin);
		break;
	case LL_EXTI_TRIGGER_RISING_FALLING:
		//activation de l'interruption sur front montant
		EXTI->RTSR|=(1 << bouton->pin);
		//activation de l'interruption sur front descendant
		EXTI->FTSR|=(1 << bouton->pin);
		break;
	}

	/*activer un vecteur d’interruption EXTI parmi les trois vecteurs disponibles selon le numéro de pin*/
	if (bouton->pin < 2) {
	//activer les requêtes d’interruption EXTI0_1
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_SetPriority(EXTI0_1_IRQn, 0);
	}
	else if (bouton->pin < 4) {
	//activer les requêtes d’interruption EXTI2_3
			NVIC_EnableIRQ(EXTI2_3_IRQn);
			NVIC_SetPriority(EXTI2_3_IRQn, 0);
	}
	else {
	//activer les requêtes d’interruption EXTI4_15
			NVIC_EnableIRQ(EXTI4_15_IRQn);
			NVIC_SetPriority(EXTI4_15_IRQn, 0);
	}

    }










/////////////////:













///////////////////////




