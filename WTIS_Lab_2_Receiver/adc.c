#include "adc.h"
#include <msp430fg4618.h>

volatile unsigned int ADCresult;


void Temp_ADC_Init(void)
{
unsigned int i;

ADC12CTL0 = ADC12ON + REFON + REF2_5V +  SHT0_10;  // Setup ADC12, ref., sampling time

ADC12CTL1 = SHP;                                 // Use sampling timer
  
ADC12MCTL0 = INCH_10 + SREF_1;                   // Select channel A10, Vref+
  
//ADC12IE = 0x01;                                 // Enable ADC12IFG.0
  
for (i = 0; i < 0x3600; i++);                   // Delay for reference start-up
  
ADC12CTL0 |= ENC;                                // Enable conversions
  
//__enable_interrupt();                           // Enable interrupts   
}  
 
unsigned int Temp_Read(void)
{
 ADC12CTL0 |= ADC12SC;          // Start conversion
  //unsigned int i;
  //for (i=0;i<100;i++);  
//__bis_SR_register(LPM0_bits);   // Enter LPM0  
 while (!(ADC12IFG & 0x0001));           // Conversion done?
 ADCresult=ADC12MEM0;                   // Access result

  //for (i=0;i<100;i++);
 __no_operation();                       // SET BREAKPOINT HERE
 return(((((long)ADCresult-1615)*7040)/4095)); 
} 

float Temp_Read_DegC(void)
{
float DegC;

ADC12CTL0 |= ADC12SC;          // Start conversion
    
__bis_SR_register(LPM0_bits);   // Enter LPM0

//  DegC = (Vsensor - 986mV)/3.55mV
    
//  Vsensor = (Vref)(ADCresult)/4095)
    
//  DegC -> ((ADCresult - 1615)*704)/4095
DegC = ((((long)ADCresult-1615)*704)/4095);

return(DegC); 
}

//#pragma vector=ADC12_VECTOR

//__interrupt void ADC12ISR(void)
//{
  
//ADCresult = ADC12MEM0;                    // Move results, IFG is cleared
  
//__bic_SR_register_on_exit(LPM0_bits);     // Exit LPM0

//}