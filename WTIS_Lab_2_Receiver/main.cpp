#include "system.h"
#include "uart.h"
#include "sensors.h"
#include "cc2420.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

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

#define t_link 1024

static OS::TEventFlag RxPacketEvent;

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
  CC2420_SetAddress(2); // ��������� ������ ����  
   
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

int count_receive = 0;
int packet_not_receive = 0;

template<> void PROCESS_MAIN::exec()
{
  LED_1_On();
  LED_2_Off();
  CC2420_SetReceiveMode(); //��������� � ����� ������
  for (;;)
  {
    // ���� ��������� ��������
    LED_1_Toggle();
    LED_2_Toggle();
    float percent = ((float)count_receive /((float)count_receive + (float)packet_not_receive))*100;
    printf(" receive = %d; ", count_receive);
    printf(" left = %d; ", packet_not_receive);
    printf(" Percent = %d;\n\r", (int)percent);
    count_receive = 0;
    packet_not_receive = 0;
    sleep(t_link);
  }
}
//---------------------------------------------------------------------------

  int last_packets = 0;

void CC2420_OnReceivedPacket(RADIO_PACKET_RX_INFO &Info, unsigned char *Data, unsigned char Size)
{
  // ���������� ��������� ������
  union {
    float f;
    unsigned char b[4];
  } u;
  
  if(last_packets+1!=Info.Seq)
    packet_not_receive=Info.Seq-last_packets-1;
  
  last_packets = Info.Seq;
  count_receive++;
  
  
  printf("Sender address = %d;", Info.SrcID);
  printf(" Power = %d;", Info.RSSI);
  printf(" Packet count = %d;", Info.Seq);
  printf(" Quality = %d;", Info.LQI);
  printf(" Size = %d;", Size);

  for (int i = 0;i<Size;i++)
  {
    u.b[i] = Data[i];
  }

  float h = (u.f - (int)u.f)*1000;
  
  printf(" DATA = %d", (int)u.f);
  printf(".%d;\n\r", (int)h);

  CC2420_SetReceiveMode();
  
}
//---------------------------------------------------------------------------