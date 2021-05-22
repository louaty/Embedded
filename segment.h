#include "stm32l053xx.h"
typedef struct
{
GPIO_TypeDef * gpioPort;
uint8_t pins1;
uint8_t pins2;
uint8_t pins3;
uint8_t pins4;
}SEGMENT_TypeDef;


void Segment_init(SEGMENT_TypeDef *seg, GPIO_TypeDef * port, uint8_t pn1,uint8_t pn2,uint8_t pn3,uint8_t pn4);

void Segment_turnOn(SEGMENT_TypeDef *seg, uint8_t n_pn);
