#ifndef __SENSORS_H
#define __SENSORS_H

#include "system.h"

#define SENSORS_ADC_CHANNEL_TEMPERATURE         10

#define SENSORS_ADC_TEMPERATURE_V_T0        986 // Voltage (mV) at 0°C
#define SENSORS_ADC_TEMPERATURE_V_TC        3.55 // Temperature coefficient (mV/0°C)

void Sensors_Init();

// Опрос встроенного датчика температуры
float Sensors_IntTemperature_Read();

#endif // __SENSORS_H
