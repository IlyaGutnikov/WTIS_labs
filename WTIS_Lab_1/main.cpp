#include "system.h"
#include "uart.h"
#include "sensors.h"
#include <math.h>
#include <stdio.h>

// Process types
typedef OS::process<OS::pr0, 500> PROCESS_MAIN;
typedef OS::process<OS::pr1, 500> PROCESS_READ;

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

float sended_temp;

void main()
{
  Sys_Init(); // ������������� ������� ������������

  UART_Init(); // ������������� ����������������� ����� (115200 8N1)

  Sensors_Init(); // ������������� ��������
 
  LED_1_Init(); // ������������� �����������
  LED_2_Init();
    
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
  
  for (;;)
  {
    // ���� ��������� ��������
    current_temp = Sensors_IntTemperature_Read();
    filtered_temp = Filter(prev_temp, current_temp);
    
    temp_message = filtered_temp;
    
    //���� ����������� �� ������ ������ ��� � ���������
    if (abs(sended_temp - filtered_temp) > T_Treshold) {
      //�������� � ����������
      flag.signal();
    }
    
    //�������� ��������������� ������
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
  
  for (;;)
  {
    if (flag.wait(T_Send)) {
      temp_message.out(temp);
      printf("Alarm out temp=%2.2f degrees celsius\n\r", temp);
      sended_temp = temp;
      
    } else {
      temp_message.out(temp);
      printf("Out temp=%2.2f degrees celsius\n\r", temp);
      sended_temp = temp;
    }
  }
}
