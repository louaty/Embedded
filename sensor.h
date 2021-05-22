#include "stm32l053xx.h"
typedef struct
{
uint8_t resolution;
uint8_t channel;
ADC_TypeDef* adc;


}SENSOR_TypeDef;

void Sensor_init(SENSOR_TypeDef *sensor, ADC_TypeDef* adc, uint8_t resolution, uint8_t channel);
uint8_t Set_Value(SENSOR_TypeDef *sensor1);

