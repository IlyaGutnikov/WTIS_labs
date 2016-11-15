#include "system.h"
#include "uart.h"
#include <yfuns.h>

extern "C" int __low_level_init(void)
{
  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer

  return (1);
}

extern "C" size_t __write(int handle, const unsigned char * buffer, size_t size)
{
  size_t nChars = 0;

  if (buffer == 0)
  {
    return 0;
  }

  if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR)
  {
    return _LLIO_ERROR;
  }

  for (; size != 0; --size)
  {
    UART_SendByte(*buffer++);
    ++nChars;
  }

  return nChars;
}

void Sys_Init(void)
{
  volatile unsigned int i;
  //  8MHz with auto-calibration by the FLL+.
  //  ACLK = LFXT1 = 32768Hz, MCLK = SMCLK = DCO = (121+1) x 2 x ACLK = 7995392Hz
  //  //* An external watch crystal between XIN & XOUT is required for ACLK *//	
  //
  //                 MSP430xG461x
  //             -----------------
  //         /|\|              XIN|-
  //          | |                 | 32kHz
  //          --|RST          XOUT|-
  //            |                 |
  //            |             P1.1|--> MCLK = 8MHz
  //            |                 |
  //            |             P1.5|--> ACLK = 32kHz
  //            |                 |
 
  WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer
  
  // Loop until 32kHz crystal stabilizes
  do
  {
    IFG1 &= ~OFIFG; // Clear oscillator fault flag
    for (i = 50000; i; i--); // Delay
  }
  while (IFG1 & OFIFG); // Test osc fault flag

  FLL_CTL0 |= DCOPLUS + XCAP18PF; // DCO+ set, freq = xtal x D x N+1
  SCFI0 = FLLD_2 | FN_4; // x2 DCO freq, 8MHz nominal DCO
  SCFQCTL = 121; // (121+1) x 32768 x 2 = 7.99 MHz
  
  // WatchDog interval timer mode (15.625 ms period)
  WDTCTL = WDT_ADLY_16;
  IE1 |= 0x01;

  // Superviser, 2.4V
  SVSCTL=0x00; //off SVS
  SVSCTL=0x50; //on SVS   0x10~1.9V 0x20~2.1V 0x30~2.2V 0x40~2.3 0x50~2.4V 0x60~2.5V 0x70~2.65V 0x80~2.8V 0x90~2.9V 0xA0~3.05V 0xB0~3.2V 0xC0~3.35V 0xD0~3.5V 0xE0~3.7V
}

void Sys_Sleep(TTimeout Ticks)
{
  if (Ticks > 0)
  {
    OS::TBaseProcess::Sleep(Ticks);
  }
}

void Sys_SleepMsec(unsigned int MSec)
{
  Sys_Sleep(MSEC_TO_TICKS(MSec));
}

void Sys_SleepSec(unsigned int Sec)
{
  Sys_Sleep(SEC_TO_TICKS(Sec));
}

void Sys_Delay(unsigned int Cycles)
{
  Cycles >>= 3;
  while (Cycles--)
  {
    __no_operation();
    __no_operation();
    __no_operation();
  }
}

void Sys_DelayUsec(unsigned int USec)
{
  #define USEC_CORRECTION_OFFSET 0
  Sys_Delay((USec-USEC_CORRECTION_OFFSET) * CYCLES_PER_USEC);
}

void Sys_DelayMsec(unsigned int MSec)
{
  Sys_Delay(MSec * CYCLES_PER_MSEC);
}
