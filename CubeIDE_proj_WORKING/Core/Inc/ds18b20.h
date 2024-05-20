#ifndef DS18B20_H
#define DS18B20_H

#include "stm32f1xx_hal.h"

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_2

void DS18B20_Init(void);
float DS18B20_ReadTemperature(void);

#endif // DS18B20_H
