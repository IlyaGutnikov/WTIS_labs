#ifndef __SPI_H
#define __SPI_H

#include "system.h"

//#define SPI_0_INTERRUPT_DRIVEN
#ifndef SPI_0_INTERRUPT_DRIVEN
//#define SPI_1_INTERRUPT_DRIVEN
#endif

//#define SPI_SS_PIN 0x00

void SPI_Shutdown(unsigned char Port);
void SPI_Wakeup(unsigned char Port);
void SPI_MasterInit(unsigned char Port, unsigned char Settings, unsigned int Baudrate);

#define st(x)      do { x } while (__LINE__ == -1)

#define SPI_1_WaitCompletion()	while(!(IFG2 & URXIFG1));
#define SPI_1_SendByte(x)	st( IFG2 &= ~URXIFG1;  U1TXBUF = x; while(!(IFG2 & UTXIFG1));) //st( IFG2 &= ~URXIFG1;  U1TXBUF = x; while(!(IFG2 & UTXIFG1)); )
#define SPI_1_Wakeup(x)		st( P4SEL |= 0x38; U1CTL &= ~SWRST; )
#define SPI_1_Shutdown(x)	st( U1CTL |= SWRST; P4SEL &= ~0x38; )

unsigned char SPI_1_SendReceiveByte(unsigned char Data);
unsigned char SPI_1_ReceiveByte();




#endif // __SPI_H

