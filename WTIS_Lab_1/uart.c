#include "uart.h"
#include <msp430fg4618.h>

void UART_Init()
{
  UCA0CTL1 |= UCSWRST;//Set UCSWRST -- needed for re-configuration process
  UCA0CTL0 = 0x00;
  UCA0CTL1 = 0x81;     
  UCA0BR0 = 0x04; // 9600 from 8MHz -> SMCLK  //0x04 115200 from 8MHz -> SMCLK
  UCA0BR1 = 0x00;      
  UCA0MCTL = 0x51; //0x11~9600 0x51~115200 
  P2SEL |= 0x30;//P2.4,P2.5 = USCI_A0 TXD/RXD
  UCA0CTL1 &= ~UCSWRST;                     //Initialize USCI state machine
}

void UART_Shutdown()
{
  UCA0CTL1 |= UCSWRST; // Hold reset state
  P2SEL &= ~BIT4;
}

void UART_Wakeup()
{
  P2SEL |= BIT4;;
  UCA0CTL1 &= ~UCSWRST; // Hold reset state
}

void UART_SendByte(unsigned char Data)
{
  while (!(IFG2&UCA0TXIFG))
    ;
  UCA0TXBUF = Data; 
}

void UART_SendBytes(unsigned char *DataBuf, unsigned char Size)
{
  unsigned char *pData = DataBuf;
  while (Size--)
    UART_SendByte(*pData++);
}

unsigned char UART_ReceiveByte(void)
{
  while (!(IFG2&UCA0RXIFG));
  return UCA0RXBUF;
}

unsigned char UART_ReceiveBytes(unsigned char *DataBuf, unsigned char Size)
{
  unsigned char *pData = DataBuf;
  unsigned char Count = Size;
  while (Count--)
    *pData++ = UART_ReceiveByte();
  return Size;
}

void UART_Print(const unsigned char *Str)
{
  while ((Str) && (*Str))
    UART_SendByte(*Str++);
}

void UART_PrintLn(const unsigned char *Str)
{
  UART_Print(Str);
  UART_SendByte(13);
  UART_SendByte(10);
}

