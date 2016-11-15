#include "system.h"
#include "uart.h"
#include "sensors.h"
#include "cc2420.h"
#include <math.h>
#include <stdio.h>

// Process types
typedef OS::process<OS::pr0, 500> PROCESS_MAIN;
typedef OS::process<OS::pr1, 200> PROCESS_READ;

// Process objects
static PROCESS_MAIN Process_Main;
static PROCESS_READ Process_Read;

//Передача сообщений между процессами
OS::message<float> temp_message;

//флаг событий
OS::TEventFlag flag;

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

float sended_temp;

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


//Симуляция воздействия ФНЧ на серию цифровых отсчетов
float Filter(float last_temp, float temp) {

  return alpha * temp + (1 - alpha) * last_temp;

}


template<> void PROCESS_READ::exec()
{
  float current_temp;
  float prev_temp;
  float filtered_temp;
  
  prev_temp = Sensors_IntTemperature_Read();
  
  LED_1_On();
  LED_2_Off();
  
  for (;;)
  {
    // Тело основного процесса
    current_temp = Sensors_IntTemperature_Read();
    filtered_temp = Filter(prev_temp, current_temp);
    
    temp_message = filtered_temp;
    
    //если температура по модулю больше чем в константе
    if (abs(sended_temp - filtered_temp) > T_Treshold) {
      //сигналим о изменениях
      flag.signal();
    }
    
    //отправка отфильтрованных данных
    temp_message.send();
      
    prev_temp = filtered_temp;
    
    LED_1_Toggle();
    LED_2_Toggle();
    
    sleep(T_Sample);
   
  }
}
//---------------------------------------------------------------------------

template<> void PROCESS_MAIN::exec()
{
  float temp;
  
  int packet_count = 0;
  
  for (;;)
  {
    
    RADIO_PACKET_TX_INFO tx_info = {
      
      1,
      2,
      packet_count,
      0
    };
    
    packet_count = packet_count + 1;
      
    if (flag.wait(T_Send)) {
      
      temp_message.out(temp);
      //printf("Alarm out temp=%2.2f degrees celsius\n\r", temp);
      sended_temp = temp;
      
      CC2420_SendPacket(tx_info, (unsigned char *) &sended_temp, sizeof(sended_temp));
      
    } else {
      temp_message.out(temp);
      //printf("Out temp=%2.2f degrees celsius\n\r", temp);
      sended_temp = temp;
      
      CC2420_SendPacket(tx_info, (unsigned char *) &sended_temp, sizeof(sended_temp));
    }
  }
}


void CC2420_OnReceivedPacket(RADIO_PACKET_RX_INFO &Info, unsigned char *Data, unsigned char Size)
{
  
}
//---------------------------------------------------------------------------
