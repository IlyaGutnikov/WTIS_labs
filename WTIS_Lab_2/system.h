#ifndef __SYSTEM_H
#define __SYSTEM_H

#include <scmRTOS.h>
#include <assert.h>

#ifndef FALSE
	#define FALSE 0
#endif
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef NULL
	#define NULL 0
#endif

#define st(x)      do { x } while (__LINE__ == -1)

#define CPU_FREQ 7995392
#define CYCLES_PER_USEC (CPU_FREQ / 1000000)
#define CYCLES_PER_MSEC (CPU_FREQ / 1000)

#define MSEC_PER_TICK           16
#define MSEC_TO_TICKS(msec)     (msec/MSEC_PER_TICK)
#define TICKS_TO_MSEC(ticks)    (ticks*MSEC_PER_TICK)

#define TICKS_PER_SEC           64
#define SEC_TO_TICKS(sec)       ((TTimeout)sec*TICKS_PER_SEC)
#define TICKS_TO_SEC(ticks)     (ticks/TICKS_PER_SEC)

#define TICKS_PER_MIN           3840
#define MIN_TO_TICKS(min)       ((TTimeout)min*TICKS_PER_MIN)

void Sys_Init(void);         

void Sys_Sleep(TTimeout Ticks);
void Sys_SleepMsec(unsigned int MSec);
void Sys_SleepSec(unsigned int Sec);

void Sys_Delay(unsigned int Cycles);
void Sys_DelayUsec(unsigned int USec);
void Sys_DelayMsec(unsigned int MSec);

#endif // __SYSTEM_H