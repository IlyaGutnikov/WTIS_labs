#ifndef __UART_H
#define __UART_H

void UART_Init();
void UART_Shutdown();
void UART_Wakeup();
void UART_SendByte(unsigned char Data);
void UART_SendBytes(unsigned char *DataBuf, unsigned char Size);
unsigned char UART_ReceiveByte(void);
unsigned char UART_ReceiveBytes(unsigned char *DataBuf, unsigned char Size);
void UART_Print(const unsigned char *Str);
void UART_PrintLn(const unsigned char *Str);

#endif // __UART_H