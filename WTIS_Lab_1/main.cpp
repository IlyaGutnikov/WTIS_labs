#include "system.h"
#include "uart.h"
#include "sensors.h"
#include <math.h>
#include <stdio.h>

// Process types
typedef OS::process<OS::pr0, 500> PROCESS_MAIN;

// Process objects
static PROCESS_MAIN Process_Main;

size_t StackSlacks[OS::PROCESS_COUNT];

#define LED_1_Init() (P2DIR |= (1 << 2)) // LED 1 = P2.2
#define LED_1_On() (P2OUT |= (1 << 2))
#define LED_1_Off() (P2OUT &= ~(1 << 2))
#define LED_1_Toggle() (P2OUT ^= (1 << 2))
#define LED_2_Init() (P2DIR |= (1 << 1)) // LED 2 = P2.1
#define LED_2_On() (P2OUT |= (1 << 1))
#define LED_2_Off() (P2OUT &= ~(1 << 1))
#define LED_2_Toggle() (P2OUT ^= (1 << 1))

#define T_Sample 64
#define T_Send 256
#define T_Treshold 0.1
// коэффициент сглаживания
#define alpha 0.7 

void main()
{
  Sys_Init(); // Инициализация системы тактирования

  UART_Init(); // Инициализация последовательного порта (115200 8N1)

  Sensors_Init(); // Инициализация датчиков
 
  LED_1_Init(); // Инициализация светодиодов
  LED_2_Init();
    
  OS::Run(); // Запуск операционной системы
}
//---------------------------------------------------------------------------

void OS::SystemTimerUserHook() 
{
  // Обработчик прерывания системного таймера
}
//---------------------------------------------------------------------------

void OS::IdleProcessUserHook()
{
  // Тело Idle-процесса
}
//---------------------------------------------------------------------------

//Симуляция воздействия ФНЧ на серию цифровых отсчетов
float Filter(float last_temp, float temp) {

  return alpha * temp + (1 - alpha) * last_temp;

}

template<> void PROCESS_MAIN::exec()
{
  float current_temp;
  float prev_temp;
  float filtered_temp;
  
  LED_1_On();
  LED_2_Off();
  for (;;)
  {
    // Тело основного процесса
    current_temp = Sensors_IntTemperature_Read();
    filtered_temp = Filter(prev_temp, current_temp);
    printf("Temperature=%2.2f degrees Celsius\n\r", filtered_temp);
    LED_1_Toggle();
    LED_2_Toggle();
    sleep(T_Send);
    
    prev_temp = filtered_temp;
  }
}
//---------------------------------------------------------------------------

