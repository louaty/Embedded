#include "led.h"


void Led_init(LED_TypeDef *led, GPIO_TypeDef * port, uint8_t pn1,uint8_t pn2,uint8_t pn3){
led->gpioPort=port;
led->pin1=pn1;
led->pin2=pn2;
led->pin3=pn3;
//Activation de l'horloge sur le port en question
//1-déterminer le numéro du port 0--> GPIOA, 1-->GPIOB, etc.
uint8_t nb_port;
nb_port=(uint32_t)((uint32_t )port - IOPPERIPH_BASE)/ (uint32_t)0x400;
//2-activation de l'hologe
RCC->IOPENR|=1<<nb_port;
//configuration de la pin en sortie
led->gpioPort->MODER&=~(0b11<<2*pn1);
led->gpioPort->MODER|=(0b01<<2*pn1);

led->gpioPort->MODER&=~(0b11<<2*pn2);
led->gpioPort->MODER|=(0b01<<2*pn2);

led->gpioPort->MODER&=~(0b11<<2*pn3);
led->gpioPort->MODER|=(0b01<<2*pn3);
}


void Led_turnOn(LED_TypeDef *led, uint8_t n_pn){

    if(n_pn == 1){
    led->gpioPort->ODR |= (1<<led->pin1);}

    else if(n_pn ==2){

    led->gpioPort->ODR |= (1<<led->pin2);}

    else if(n_pn == 3){

    led->gpioPort->ODR |= (1<<led->pin3);}
}


void Led_turnOff(LED_TypeDef *led, uint8_t n_pn){

    if(n_pn == 1){

    led->gpioPort->ODR &= ~(1<<led->pin1);}

    else if(n_pn == 2){

    led->gpioPort->ODR &= ~(1<<led->pin2);}

    else if (n_pn == 3){

    led->gpioPort->ODR &= ~(1<<led->pin3);
    }
}

void Led_toggle(LED_TypeDef *led, uint8_t n_pn){

    if (n_pn == 1){
    led->gpioPort->ODR ^= (1<<led->pin1);}
    else if(n_pn ==2){

    led->gpioPort->ODR ^= (1<<led->pin2);}

    else if (n_pn ==3)
    {

    led->gpioPort->ODR ^= (1<<led->pin2);}
}


//ici, on allume les 3 couleurs en même temps mais on pourrait choisir d'allumer une seule PIN pour choisir une couleur (rouge, bleu ou vert)
uint8_t Led_isOn(LED_TypeDef *led, uint8_t n_pn){
    if((led->gpioPort->ODR & (1<<led->pin1)) && (n_pn ==1) ){
         return 1;

    }
    else if((led->gpioPort->ODR & (1<<led->pin2)) && (n_pn ==2) ){
        return 1;

   }

    else if((led->gpioPort->ODR & (1<<led->pin3)) && (n_pn ==3) ){
            return 1;

       }
    else return 0;

}

//ici, on etteint les 3 couleurs en même temps mais on pourrait choisir d'allumer une seule PIN pour choisir une couleur (rouge, bleu ou vert)
uint8_t Led_isOff(LED_TypeDef *led, uint8_t n_pn){
    if(!((led->gpioPort->ODR & (1<<led->pin1)) && (n_pn == 1) ) )
        return 1;

    else if(!((led->gpioPort->ODR & (1<<led->pin2)) && (n_pn == 2) ) )
        return 1;

    if(!((led->gpioPort->ODR & (1<<led->pin3)) && (n_pn == 3) ) )
            return 1;

    else return 0;
}
