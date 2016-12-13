#ifndef __ADC_H
#define __ADC_H

#ifdef __cplusplus
 extern "C" {
#endif

void ADC_Init(void);
unsigned int ADC_Read(void);
float ADC_Read_DegC(void);

#ifdef __cplusplus
}
#endif

#endif // __TEMP_H