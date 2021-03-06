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

//�������� ��������� ����� ����������
OS::message<float> temp_message;

//���� �������
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
// ����������� �����������
#define alpha 0.7 


static OS::TEventFlag RxPacketEvent;
static RADIO_PACKET_RX_INFO PacketInfo;
static unsigned char PacketData[CC2420_FRAME_PAYLOAD_MAX_SIZE];
static int PacketDataSize;

void CC2420_OnReceivedPacket(RADIO_PACKET_RX_INFO &Info, unsigned char *Data, unsigned char Size);

void main()
{
  Sys_Init(); // ������������� ������� ������������

  UART_Init(); // ������������� ����������������� ����� (115200 8N1)

  Sensors_Init(); // ������������� ��������
 
  LED_1_Init(); // ������������� �����������
  LED_2_Init();

  CC2420_Init(&CC2420_OnReceivedPacket); // ��������� ������ ����������� ��������� ������
  CC2420_Setup();
  CC2420_SetChannel(23); // ��������� ������ ���������� ������ (�� 11 �� 26)
  CC2420_SetAddress(1); // ��������� ������ ����  
   
  OS::Run(); // ������ ������������ �������
}
//---------------------------------------------------------------------------

void OS::SystemTimerUserHook() 
{
  // ���������� ���������� ���������� �������
}
//---------------------------------------------------------------------------

void OS::IdleProcessUserHook()
{
  // ���� Idle-��������
}
//---------------------------------------------------------------------------


//��������� ����������� ��� �� ����� �������� ��������
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
  int counter=0;
  for (;;)
  {
    // ���� ��������� ��������
    current_temp = Sensors_IntTemperature_Read();
    filtered_temp = Filter(prev_temp, current_temp);
    
    temp_message = filtered_temp;
    
    //���� ����������� �� ������ ������ ��� � ���������
    if (abs(prev_temp - filtered_temp) > T_Treshold) {
      //�������� � ����������
      flag.signal();
    }

    //�������� ��������������� ������
    if (counter==5){
    temp_message.send();
    prev_temp = filtered_temp;
    counter = 0;
    }
    counter++;
  
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
    
    unsigned char * buf;
      
    if (flag.wait(T_Send)) {
      
      temp_message.out(temp);
      //printf("Alarm out temp=%2.2f degrees celsius\n\r", temp);
      buf = (unsigned char *) &temp;
      CC2420_SendPacket(tx_info, buf, sizeof(temp));
      
    } else {
      temp_message.out(temp);
      //printf("Out temp=%2.2f degrees celsius\n\r", temp);
  
      buf = (unsigned char *) &temp;
      
      CC2420_SendPacket(tx_info, buf, sizeof(temp));
      
    }
    
    
    sleep(T_Sample);
  }
}


void CC2420_OnReceivedPacket(RADIO_PACKET_RX_INFO &Info, unsigned char *Data, unsigned char Size)
{
  
}
//---------------------------------------------------------------------------
