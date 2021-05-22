
#include "stm32l053xx.h"
#include "sensor.h"


void Sensor_init(SENSOR_TypeDef *sensor, ADC_TypeDef* adc, uint8_t resolution, uint8_t channel) {

	sensor->resolution=resolution;
	sensor->channel=channel;

	sensor->adc=adc;

	//activation de l'horloge
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	//mode synchrone
	adc->CFGR2|=(0b11<<ADC_CFGR2_CKMODE_Pos);

	//calibration optionnelle
	if((adc->CR & ADC_CR_ADEN) != 0) {
		adc->CR &= ~(uint32_t)ADC_CR_ADEN;
	}
	adc->CR|= ADC_CR_ADCAL; // Calibration
	// AttenteADCAL = 0 (fin calibration)
	while((adc->CR & ADC_CR_ADCAL) != 0);

	//conversion en continu
	adc->CFGR1 |= (1<<ADC_CFGR1_CONT_Pos);

	//choix de la résolution
	switch (resolution) {
		case 6:
			adc->CFGR1 |= (0b11<<ADC_CFGR1_RES_Pos);
			break;
		case 8:
			adc->CFGR1 |= (0b10<<ADC_CFGR1_RES_Pos);
			break;
		case 10:
			adc->CFGR1 |= (0b01<<ADC_CFGR1_RES_Pos);
			break;
		case 12:
			adc->CFGR1 &= ~(0b11<<ADC_CFGR1_RES_Pos);
			break;
	}

	//sélection du canal
	adc->CHSELR |= (1<<channel);

	//activation puis lancement du convertiseur
	adc->CR |= (1<<ADC_CR_ADEN_Pos);
	adc->CR |= (1<<ADC_CR_ADSTART_Pos);
}


uint8_t Set_Value(SENSOR_TypeDef *sensor1){

	while((sensor1->adc->ISR& ADC_ISR_EOC)==0);
	// Afficherle résultatde la conversion
	return sensor1->adc->DR;
}

