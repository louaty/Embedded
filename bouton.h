

#ifndef INC_BOUTON_H_
#define INC_BOUTON_H_



#include "stm32l053xx.h"
typedef struct
{
GPIO_TypeDef * gpioPort;
uint8_t pin;
uint8_t pull;

}BOUTON_TypeDef;

void BOUTON_init(BOUTON_TypeDef *BOUTON, GPIO_TypeDef * port, uint8_t pn, uint8_t pl);

uint8_t BOUTON_State(BOUTON_TypeDef *BOUTON);

void BOUTON_enableIRQ(BOUTON_TypeDef *BOUTON, uint8_t trigger);


#endif /* INC_BOUTON_H_ */
