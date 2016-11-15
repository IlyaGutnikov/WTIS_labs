#include "sensors.h"

void Sensors_Init()
{
  // ADC setup:
  //   ADC clock = SMCLK/8 = 7995392 Hz/8 = 999424 Hz
  //   single-conversion mode
  //   internal reference 2.5 V
  //   sample-and-hold time = 256 clock cycles
  //   sample period = (256+13) clock cycles = 999424 hz/269 = 3715.33 Hz
  ADC12CTL0 = ADC12ON + SHT0_8 + REFON + REF2_5V;
  ADC12CTL1 = SHP + ADC12SSEL_3 + ADC12DIV_7 + CONSEQ_0;
  ADC12MCTL0 = SREF_1;  
  
  // Delay for internal reference start-up
  Sys_Delay(8 * CYCLES_PER_MSEC);
  Sys_Delay(8 * CYCLES_PER_MSEC);
  Sys_Delay(2 * CYCLES_PER_MSEC);

  ADC12CTL0 |= ENC;
}

unsigned int ADC_Convert(unsigned char Channel)
{
  if (Channel < 8)
    P6SEL |= (1 << Channel);
  ADC12CTL0 &= ~ENC;
  ADC12MCTL0 &= 0xF0;
  ADC12MCTL0 |= Channel;
  ADC12CTL0 |= ENC;
  ADC12CTL0 |= ADC12SC;
  while (!(ADC12IFG & 0x01))
    ;
  return ADC12MEM0;
}

// Опрос встроенного датчика температуры
float Sensors_IntTemperature_Read()
{
  #define TEMP_SAMPLES_COUNT 10
  unsigned long x;
  float t;
  unsigned char i;

  x = 0;
  for (i = 0; i < TEMP_SAMPLES_COUNT; i++)
    x += ADC_Convert(SENSORS_ADC_CHANNEL_TEMPERATURE);

  t = float(x)/float(TEMP_SAMPLES_COUNT);
  t = ((2500 * t / 4095) - SENSORS_ADC_TEMPERATURE_V_T0) / SENSORS_ADC_TEMPERATURE_V_TC;
  
  return t;
}

