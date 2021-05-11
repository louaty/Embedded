#include "servo.h"

void Servo_init(SERVO_TypeDef*servo, GPIO_TypeDef* port, uint8_t pn){

servo->gpioPort=port;
servo->pin=pn;
//Activation de l'horloge sur le port en question
//1-déterminer le numéro du port 0--> GPIOA, 1-->GPIOB, etc.
uint8_t nb_port;
nb_port=(uint32_t)((uint32_t*)port -IOPPERIPH_BASE)/ (uint32_t)0x400;
//2-activation de l'hologe
RCC->IOPENR|=1<<nb_port;
//configuration de la pin en sortie
servo->gpioPort->MODER&=~(0b11<<2*pn);
servo->gpioPort->MODER|=(0b01<<2*pn);
}
//

void Servo_turnOn(SERVO_TypeDef*servo){
	servo->gpioPort->ODR |= (1<<servo->pin);
}
//
void Servo_turnOff(SERVO_TypeDef *servo){

	servo->gpioPort->ODR &= ~(1<<servo->pin);

}

uint8_t Servo_isOn(SERVO_TypeDef*servo){
	if(servo->gpioPort->ODR & (1<<servo->pin))
	        return 1;
	    else return 0;

}

uint8_t Servo_isOff(SERVO_TypeDef*servo){
	  if(!(servo->gpioPort->ODR & (1<<servo->pin)))
	        return 1;
	    else return 0;

}






