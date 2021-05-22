#include "stm32l053xx.h"
typedef struct
{
GPIO_TypeDef * gpioPort;
uint8_t pin1;
uint8_t pin2;
uint8_t pin3;
}LED_TypeDef;

void Led_init(LED_TypeDef *led, GPIO_TypeDef * port, uint8_t pn1, uint8_t pn2,uint8_t pn3);

uint8_t Led_isOn(LED_TypeDef *led, uint8_t n_pn);
uint8_t Led_isOff(LED_TypeDef *led, uint8_t n_pn);
void Led_turnOn(LED_TypeDef *led, uint8_t n_pn);
void Led_turnOff(LED_TypeDef *led, uint8_t n_pn);
void Led_toggle(LED_TypeDef *led, uint8_t n_pn);
