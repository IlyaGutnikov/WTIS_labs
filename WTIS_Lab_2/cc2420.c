#include <cc2420.h>
#include <uart.h>

static CC2420_RX_PACKET_CALLBACK CC2420_OnReceivedPacketCallback = 0;

static RADIO_STATUS RF_Status;
static unsigned int RF_PANID;

static unsigned char PayloadBuffer[CC2420_FRAME_PAYLOAD_MAX_SIZE];

void CC2420_Receive_ISR();

unsigned char CC2420_SPI_SendReceiveByte(unsigned char Data)
{
  CC2420_SPI_SendByte(Data);
  CC2420_SPI_WaitCompletion();

  return CC2420_SPI_REG_RXBUF;
}

unsigned char CC2420_SPI_ReceiveByte()
{
  return CC2420_SPI_SendReceiveByte(0);
}

#pragma vector=CC2420_PORT_INT_VECTOR
OS_INTERRUPT void CC2420_FIFOP_ISR()
{
  LPM4_EXIT;
  OS::scmRTOS_ISRW_TYPE ISRW;

  if (CC2420_FIFOP_Int_Asserted())
  {
    CC2420_FIFOP_Int_Clear();
    CC2420_Receive_ISR();
  }
}

void CC2420_Receive_ISR()
{
  unsigned char Length;
  CC2420_FRAME_HEADER MHR;
  RADIO_PACKET_RX_INFO Info;
  unsigned char Footer[2];

  // FIFO overflow
  if (CC2420_CheckRxOverflow())
    return;

  CC2420_ReadFIFO(&Length, 1);
  Length &= 0x7F;

  if (Length < CC2420_FRAME_ACK_SIZE) // Too small packet
  {
    // Dummy read data from FIFO
    CC2420_ReadFIFO(PayloadBuffer, Length);
//    CC2420_CheckRxOverflow();
    CC2420_ClearRxFIFO();
    return;
  }

  if (Length == CC2420_FRAME_ACK_SIZE) // Maybe ack frame
  {
    unsigned int FrameControl;
    CC2420_ReadFIFO((unsigned char *)&FrameControl, 2);
    MHR.FrameControl = FrameControl;
    CC2420_ReadFIFO(&(MHR.Seq), 1);
    CC2420_ReadFIFO(Footer, 2);
    if ((Footer[1] & CC2420_CRC_OK_MASK) && (RF_Status.LastTxSeq == MHR.Seq))
      RF_Status.AckReceived = TRUE;
  }
  else if (Length < CC2420_FRAME_OVERHEAD) // Too small data packet
  {
    // Dummy read data from FIFO
    CC2420_ReadFIFO(PayloadBuffer, Length);
  }
  else // Packet with valid size
  {
    Length = Length - CC2420_FRAME_OVERHEAD;
    assert(Length <= CC2420_FRAME_PAYLOAD_MAX_SIZE);
    CC2420_ReadFIFO((unsigned char *)&MHR, sizeof(CC2420_FRAME_HEADER));
    CC2420_ReadFIFO(PayloadBuffer, Length);
    CC2420_ReadFIFO(Footer, 2);
    if (Footer[1] & CC2420_CRC_OK_MASK)
    {
      Info.SrcID = MHR.SrcAddress;
      Info.DestID = MHR.DestAddress;
      Info.Seq = MHR.Seq;
      Info.RSSI = CC2420_RSSI_2_DBM(Footer[0]);
      Info.LQI = Footer[1] & ~CC2420_CRC_OK_MASK;
      if (CC2420_OnReceivedPacketCallback)
        CC2420_OnReceivedPacketCallback(Info, PayloadBuffer, Length);
    }
  }

  CC2420_ClearRxFIFO();
}

void CC2420_Init(CC2420_RX_PACKET_CALLBACK Callback)
{
  // Initialize SPI
  CC2420_SPI_PORT_SEL |= BIT5 + BIT4 + BIT3; // Enable MISO, MOSI and SCLK pins
  CC2420_SPI_CS_PORT_DIR |= CC2420_SPI_CS_PIN; // Enable CS pin
  CC2420_SPI_REG_CTL = CHAR + SYNC + MM + SWRST; // 8-bit, SPI, Master
  CC2420_SPI_REG_TCTL = CKPH + SSEL1 + STC; // 3-wire
  CC2420_SPI_REG_BR0 = 2;
  CC2420_SPI_REG_BR1 = 0;
  CC2420_SPI_REG_MCTL = 0;
  CC2420_SPI_REG_ME = USPIE1;
  CC2420_SPI_REG_CTL &= ~SWRST;
  
  CC2420_VREG_EN_PORT_OUT &= ~CC2420_VREG_EN_PIN;
  CC2420_VREG_EN_PORT_DIR |= CC2420_VREG_EN_PIN;
  CC2420_RESET_PORT_DIR |= CC2420_RESET_PIN;
  CC2420_FIFO_PORT_DIR &= ~CC2420_FIFO_PIN;
  CC2420_FIFOP_PORT_DIR &= ~CC2420_FIFOP_PIN;
  CC2420_CCA_PORT_DIR &= ~CC2420_CCA_PIN;
  CC2420_SFD_PORT_DIR &= ~CC2420_SFD_PIN;

  CC2420_FIFOP_PORT_IES &= ~CC2420_FIFOP_PIN;

  CC2420_VRegEnable();

  CC2420_Wakeup();

  CC2420_WriteRegister(CC2420_MDMCTRL0, 0x0AF2); // AutoACK enabled, CCA mode=3
//  CC2420_WriteRegister(CC2420_MDMCTRL0, 0x0AE2); // AutoACK disabled
  CC2420_WriteRegister(CC2420_MDMCTRL1, 0x0500);
  CC2420_WriteRegister(CC2420_IOCFG0, 0x007F);
  CC2420_WriteRegister(CC2420_SECCTRL0, 0x01C4);
  CC2420_WriteRegister(CC2420_TXCTRL, 0xA0FF);
  CC2420_WriteRegister(CC2420_RXCTRL1, 0x2A56);

  // Testing
//  CC2420_WriteRegister(CC2420_TOPTST, 0x0002);
//  CC2420_WriteRegister(CC2420_IOCFG1, (0 << CC2420_IOCFG1_SFDMUX) + (3 << CC2420_IOCFG1_CCAMUX));
//  CC2420_WriteRegister(CC2420_AGCTST1, 0x0F54);
//  CC2420_WriteRegister(CC2420_AGCCTRL, 0x07FC);

  CC2420_OnReceivedPacketCallback = Callback;
}

void CC2420_Reset()
{
  CC2420_RESET_PORT_OUT &= ~CC2420_RESET_PIN;
  Sys_Delay(5 * CYCLES_PER_MSEC);
  CC2420_RESET_PORT_OUT |= CC2420_RESET_PIN;
  Sys_Delay(5 * CYCLES_PER_MSEC);
}

void CC2420_Setup()
{
  CC2420_SetChannel(11);
  CC2420_SetPANID(CC2420_PAN_ID);
}

void CC2420_Shutdown()
{
  CC2420_FIFOP_Int_Disable();
  CC2420_WriteCommand(CC2420_SXOSCOFF);
//  SPI_Shutdown(CC2420_SPI_PORT);
}

void CC2420_Wakeup()
{
  unsigned char Status;

//  SPI_Wakeup(CC2420_SPI_PORT);

  CC2420_WriteCommand(CC2420_SXOSCON);

  do
  {
    Status = CC2420_ReadStatus();
  }
  while (!(Status & CC2420_XOSC16M_STABLE));
}

void CC2420_VRegEnable()
{
  CC2420_RESET_PORT_OUT &= ~CC2420_RESET_PIN; // Reset chip
  CC2420_VREG_EN_PORT_OUT |= CC2420_VREG_EN_PIN; // Enable voltage regulator
  Sys_Delay(5*CYCLES_PER_MSEC); // Wait for chip power up
  CC2420_RESET_PORT_OUT |= CC2420_RESET_PIN; // Release from reset
  Sys_Delay(5*CYCLES_PER_MSEC); // Wait for reset to settle
}

void CC2420_VRegDisable()
{
  CC2420_RESET_PORT_OUT &= ~CC2420_RESET_PIN; // Reset chip
  CC2420_VREG_EN_PORT_OUT &= ~CC2420_VREG_EN_PIN; // Disable voltage regulator
}

void CC2420_SetChannel(unsigned char Channel)
{
  CC2420_SetFrequency(5*(Channel - 11));
}

unsigned char CC2420_GetChannel()
{
  unsigned char f;

  f = CC2420_GetFrequency();

  return (f/5 + 11);
}

void CC2420_SetFrequency(unsigned char Frequency)
{
  unsigned int k;

  k = 357 + Frequency + 0x4000;
  CC2420_WriteRegister(CC2420_FSCTRL, k);
}

unsigned char CC2420_GetFrequency()
{
  unsigned int d;

  d = CC2420_ReadRegister(CC2420_FSCTRL);
  d &= 0x3FF;

  return (d - 357);
}

void CC2420_SetAddress(unsigned int Address)
{
  CC2420_WriteRAM(CC2420RAM_SHORTADDR, (unsigned char *)&Address, 2);
}

void CC2420_SetPANID(unsigned int PANID)
{
  RF_PANID = PANID;
  CC2420_WriteRAM(CC2420RAM_PANID, (unsigned char *)&RF_PANID, 2);
}

unsigned char CC2420_GetRSSILevel()
{
  unsigned int d;
  unsigned char RSSI;

  CC2420_WriteCommand(CC2420_SRXON);
  do
  {
    d = CC2420_ReadStatus();

  }
  while (!(d & CC2420_RSSI_VALID));

  RSSI = CC2420_ReadRegister(CC2420_RSSI) & 0xFF;

  CC2420_WriteCommand(CC2420_SRFOFF);

  return RSSI;
}

unsigned char CC2420_GetRSSI()
{
  signed char RSS;

  RSS = CC2420_GetRSSILevel();

  RSS = CC2420_RSSI_PERCENT(RSS);
  if (RSS > 100) RSS = 100;
  if (RSS < 0) RSS = 0;

  return RSS;
}

void CC2420_SetTransmitPower(unsigned char Level)
{
  unsigned int d;

  d = CC2420_ReadRegister(CC2420_TXCTRL);
  d &= ~0x001F;
  d |= Level;
  CC2420_WriteRegister(CC2420_TXCTRL, d);
}

void CC2420_SetCCAThreshold(unsigned char Value)
{
  CC2420_WriteRegister(CC2420_RSSI, Value << 8);
}

unsigned char CC2420_ClearChannelAssessment()
{
  unsigned char CCA;

  CC2420_WriteCommand(CC2420_SRXON);
  while (!(CC2420_ReadStatus() & CC2420_RSSI_VALID))
    ;

  CCA = (CC2420_CCA_PORT_IN & CC2420_CCA_PIN);

//  CC2420_CheckRxOverflow();
  CC2420_ClearRxFIFO();

  CC2420_WriteCommand(CC2420_SRFOFF);

  if (CCA)
    return 1; // channel is clear
  else
    return 0; // channel is busy
}

unsigned char CC2420_CheckRxOverflow()
{
  unsigned char Overflow;
  unsigned char Dummy;

  Overflow = (CC2420_FIFOP_PORT_IN & CC2420_FIFOP_PIN) && !(CC2420_FIFO_PORT_IN & CC2420_FIFO_PIN);

  if (Overflow)
  {
    CC2420_ReadFIFO(&Dummy, 1);
    CC2420_WriteCommand(CC2420_SFLUSHRX);
    CC2420_WriteCommand(CC2420_SFLUSHRX);
  }

  return Overflow;
}

void CC2420_ClearRxFIFO()
{
  while (CC2420_FIFO_PORT_IN & CC2420_FIFO_PIN)
  {
    CC2420_WriteCommand(CC2420_SFLUSHRX);
  }
}

void CC2420_WriteCommand(unsigned char Command)
{
  TCritSect cs;

  CC2420_SPI_Wakeup();
  CC2420_SPI_CS_Enable();

  CC2420_SPI_SendByte(Command);
  CC2420_SPI_WaitCompletion();

  CC2420_SPI_CS_Disable();
  CC2420_SPI_Shutdown();
}

void CC2420_WriteRegister(unsigned char Address, unsigned int Data)
{
  TCritSect cs;

  CC2420_SPI_Wakeup();
  CC2420_SPI_CS_Enable();

  CC2420_SPI_SendByte(Address);
  CC2420_SPI_SendByte(Data >> 8);
  CC2420_SPI_SendByte(Data);
  CC2420_SPI_WaitCompletion();

  CC2420_SPI_CS_Disable();
  CC2420_SPI_Shutdown();
}

unsigned int CC2420_ReadRegister(unsigned char Address)
{
  TCritSect cs;
  unsigned char DataHi, DataLo;

  CC2420_SPI_Wakeup();
  CC2420_SPI_CS_Enable();

  CC2420_SPI_SendByte(0x40 + Address);
  DataHi = CC2420_SPI_ReceiveByte();
  DataLo = CC2420_SPI_ReceiveByte();

  CC2420_SPI_CS_Disable();
  CC2420_SPI_Shutdown();

  return ((DataHi << 8) + DataLo);
}

void CC2420_WriteRAM(unsigned int Address, unsigned char *DataBuf, unsigned char Size)
{
  TCritSect cs;

  CC2420_SPI_Wakeup();
  CC2420_SPI_CS_Enable();

  CC2420_SPI_SendByte(0x80 + (Address & 0x7F));
  CC2420_SPI_SendByte((Address >> 1) & 0xC0);
  while (Size--)
    CC2420_SPI_SendByte(*DataBuf++);
  CC2420_SPI_WaitCompletion();

  CC2420_SPI_CS_Disable();
  CC2420_SPI_Shutdown();
}

void CC2420_WriteFIFO(unsigned char *DataBuf, unsigned char Size)
{
  TCritSect cs;

  CC2420_SPI_Wakeup();
  CC2420_SPI_CS_Enable();

  CC2420_SPI_SendByte(CC2420_TXFIFO);
  while (Size--)
    CC2420_SPI_SendByte(*DataBuf++);

  CC2420_SPI_WaitCompletion();

  CC2420_SPI_CS_Disable();
  CC2420_SPI_Shutdown();
}

void CC2420_ReadFIFO(unsigned char *DataBuf, unsigned char Size)
{
  TCritSect cs;

  CC2420_SPI_Wakeup();
  CC2420_SPI_CS_Enable();

  CC2420_SPI_SendByte(0x40 + CC2420_RXFIFO);
  Sys_Delay(1);

  while (Size--)
    *DataBuf++ = CC2420_SPI_ReceiveByte();

  CC2420_SPI_CS_Disable();
  CC2420_SPI_Shutdown();
}

unsigned char CC2420_ReadStatus()
{
  TCritSect cs;
  unsigned char Data;

  CC2420_SPI_Wakeup();
  CC2420_SPI_CS_Enable();

  Data = CC2420_SPI_SendReceiveByte(CC2420_SNOP);

  CC2420_SPI_CS_Disable();
  CC2420_SPI_Shutdown();

  return Data;
}

void CC2420_DumpRegisters()
{
  #define REG_COUNT 64
  unsigned int Reg[REG_COUNT];
  unsigned int i;

  for (i = 0; i < REG_COUNT; i++)
  {
    Reg[i] = CC2420_ReadRegister(i);
/*
    UART_SendByte(i);
    UART_SendByte(Reg[i] >> 8);
    UART_SendByte(Reg[i]);
*/
  }
}

void CC2420_SetReceiveMode()
{
  CC2420_FIFOP_Int_Disable();

  CC2420_ClearRxFIFO();
//  CC2420_CheckRxOverflow();

  CC2420_WriteCommand(CC2420_SRXON);

  CC2420_FIFOP_Int_Clear();
  CC2420_FIFOP_Int_Enable();
}

void CC2420_SetIdleMode()
{
  CC2420_WriteCommand(CC2420_SRFOFF);
}

unsigned char CC2420_SendPacket(RADIO_PACKET_TX_INFO &Info, unsigned char *Data, unsigned char Size)
{
  unsigned char FrameLength;
  CC2420_FRAME_HEADER MHR;

  __disable_interrupt();

  FrameLength = CC2420_FRAME_OVERHEAD + Size;

  if (Info.AckRequest)
    MHR.FrameControl = CC2420_FCF_ACK;
  else
    MHR.FrameControl = CC2420_FCF_NO_ACK;

  MHR.Seq = Info.Seq;
  MHR.DestPANID = RF_PANID;
  MHR.DestAddress = Info.DestID;
  MHR.SrcAddress = Info.SrcID;

  RF_Status.LastTxSeq = Info.Seq;
  RF_Status.AckRequest = Info.AckRequest;
  RF_Status.AckReceived = FALSE;
/*
  // Wait for radio idle state
  while ((CC2420_PORT_IN & CC2420_FIFOP) || (CC2420_PORT_IN & CC2420_SFD))
  {
    CC2420_CheckRxOverflow();
  }
*/
  CC2420_ClearRxFIFO();
//  CC2420_CheckRxOverflow();

  CC2420_FIFOP_Int_Disable();

  CC2420_WriteCommand(CC2420_SFLUSHTX); // Flush TX FIFO

  CC2420_WriteFIFO((unsigned char *)&FrameLength, 1);
  CC2420_WriteFIFO((unsigned char *)&MHR, sizeof(CC2420_FRAME_HEADER));
  CC2420_WriteFIFO(Data, Size);

//  CC2420_SFD_Int_Clear();

  CC2420_WriteCommand(CC2420_STXON);

  // Wait for transmission start (SFD goes high)
  while (!(CC2420_SFD_PORT_IN & CC2420_SFD_PIN));

  CC2420_FIFOP_Int_Clear();
  CC2420_FIFOP_Int_Enable();
  __enable_interrupt();

  // Wait for transmission end (SFD goes low)
  while (CC2420_SFD_PORT_IN & CC2420_SFD_PIN);

  Sys_DelayUsec(CC2420_ACK_TIMEOUT);

  CC2420_WriteCommand(CC2420_SRFOFF);

  if (RF_Status.AckRequest)
    return RF_Status.AckReceived;
  else
    return FALSE;
}

unsigned char CC2420_ResendPacket()
{
  __disable_interrupt();

  RF_Status.AckReceived = FALSE;
/*
  // Wait for radio idle state
  while ((CC2420_PORT_IN & CC2420_FIFOP) || (CC2420_PORT_IN & CC2420_SFD))
  {
    CC2420_CheckRxOverflow();
  }
*/
  CC2420_ClearRxFIFO();
//  CC2420_CheckRxOverflow();

  CC2420_FIFOP_Int_Disable();

//  CC2420_SFD_Int_Clear();

  CC2420_WriteCommand(CC2420_STXON);

  // Wait for transmission start (SFD goes high)
  while (!(CC2420_SFD_PORT_IN & CC2420_SFD_PIN));

  CC2420_FIFOP_Int_Clear();
  CC2420_FIFOP_Int_Enable();
  __enable_interrupt();

  // Wait for transmission end (SFD goes low)
  while (CC2420_SFD_PORT_IN & CC2420_SFD_PIN);

  Sys_DelayUsec(CC2420_ACK_TIMEOUT);

  CC2420_WriteCommand(CC2420_SRFOFF);

  if (RF_Status.AckRequest)
    return RF_Status.AckReceived;
  else
    return FALSE;
}
