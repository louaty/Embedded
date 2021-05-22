#include "segment.h"


void Segment_init(SEGMENT_TypeDef *seg, GPIO_TypeDef * port, uint8_t pn1,uint8_t pn2,uint8_t pn3,uint8_t pn4){
seg->gpioPort=port;
seg->pins1=pn1;
seg->pins2=pn2;
seg->pins3=pn3;
seg->pins4=pn4;
//Activation de l'horloge sur le port en question
//1-déterminer le numéro du port 0--> GPIOA, 1-->GPIOB, etc.
uint8_t nb_port;
nb_port=(uint32_t)((uint32_t )port - IOPPERIPH_BASE)/ (uint32_t)0x400;
//2-activation de l'hologe
RCC->IOPENR|=1<<nb_port;
//configuration de la pin en sortie
seg->gpioPort->MODER&=~(0b11<<2*pn1);
seg->gpioPort->MODER|=(0b01<<2*pn1);

seg->gpioPort->MODER&=~(0b11<<2*pn2);
seg->gpioPort->MODER|=(0b01<<2*pn2);

seg->gpioPort->MODER&=~(0b11<<2*pn3);
seg->gpioPort->MODER|=(0b01<<2*pn3);

seg->gpioPort->MODER&=~(0b11<<2*pn4);
seg->gpioPort->MODER|=(0b01<<2*pn4);
}


void Segment_turnOn(SEGMENT_TypeDef *seg, uint8_t n_pn){

    if(n_pn == 1){
   seg->gpioPort->ODR |= (1<<seg->pins1);}

    else if(n_pn ==2){

    seg->gpioPort->ODR |= (1<<seg->pins2);}

    else if(n_pn == 3){

    seg->gpioPort->ODR |= (1<<seg->pins3);}

    else if(n_pn == 4){

        seg->gpioPort->ODR |= (1<<seg->pins4);}
}


void Segment_turnOff(SEGMENT_TypeDef *seg, uint8_t n_pn){

    if(n_pn == 1){

    seg->gpioPort->ODR &= ~(1<<seg->pins1);}

    else if(n_pn == 2){

    seg->gpioPort->ODR &= ~(1<<seg->pins2);}

    else if (n_pn == 3){

    seg->gpioPort->ODR &= ~(1<<seg->pins3);
    }
    else if (n_pn == 4){

        seg->gpioPort->ODR &= ~(1<<seg->pins4);
        }
}
