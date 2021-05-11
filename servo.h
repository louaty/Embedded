#include "stm32l053xx.h"
typedef struct
{
GPIO_TypeDef* gpioPort;
uint8_t pin;
}SERVO_TypeDef;

void Servo_init(SERVO_TypeDef *servo, GPIO_TypeDef* port, uint8_t pn);


void Servo_turnOn(SERVO_TypeDef*servo);

void Servo_turnOff(SERVO_TypeDef *servo);

uint8_t Servo_isOn(SERVO_TypeDef*servo);

uint8_t Servo_isOff(SERVO_TypeDef*servo);
