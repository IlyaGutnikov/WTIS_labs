#include "system.h"
#include "uart.h"
#include "sensors.h"
#include "cc2420.h"
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

static OS::TEventFlag RxPacketEvent;
static RADIO_PACKET_RX_INFO PacketInfo;
static unsigned char PacketData[CC2420_FRAME_PAYLOAD_MAX_SIZE];
static int PacketDataSize;

void CC2420_OnReceivedPacket(RADIO_PACKET_RX_INFO &Info, unsigned char *Data, unsigned char Size);

void main()
{
  Sys_Init(); // Инициализация системы тактирования

  UART_Init(); // Инициализация последовательного порта (115200 8N1)

  Sensors_Init(); // Инициализация датчиков
 
  LED_1_Init(); // Инициализация светодиодов
  LED_2_Init();

  CC2420_Init(&CC2420_OnReceivedPacket); // Установка адреса обработчика принятого пакета
  CC2420_Setup();
  CC2420_SetChannel(11); // Установка номера частотного канала (от 11 до 26)
  CC2420_SetAddress(1); // Установка адреса узла  
   
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

template<> void PROCESS_MAIN::exec()
{
  LED_1_On();
  LED_2_Off();
  for (;;)
  {
    // Тело основного процесса
    LED_1_Toggle();
    LED_2_Toggle();
    sleep(64);
  }
}
//---------------------------------------------------------------------------

void CC2420_OnReceivedPacket(RADIO_PACKET_RX_INFO &Info, unsigned char *Data, unsigned char Size)
{
  // Обработчик принятого пакета
}
//---------------------------------------------------------------------------