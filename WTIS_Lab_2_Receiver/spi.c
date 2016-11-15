#include "spi.h"

const unsigned int SPI_PORT_SEL[2] = {P3SEL_, P5SEL_};
const unsigned int SPI_PORT_DIR[2] = {P3DIR_, P5DIR_};
//const unsigned int SPI_PORT_OUT[2] = {P3OUT_, P5OUT_};
const unsigned int SPI_UCTL[2] = {U1CTL_, U1CTL_};
const unsigned int SPI_UTCTL[2] = {U1TCTL_, U1TCTL_};
const unsigned int SPI_UBR0[2] = {U1BR0_, U1BR0_};
const unsigned int SPI_UBR1[2] = {U1BR1_, U1BR1_};
const unsigned int SPI_UMCTL[2] = {U1MCTL_, U1MCTL_};
//const unsigned int SPI_UTXBUF[2] = {U0TXBUF_, U1TXBUF_};
//const unsigned int SPI_URXBUF[2] = {U0RXBUF_, U1RXBUF_};
const unsigned int SPI_ME[2] = {ME2_, ME2_};
#if defined(SPI_0_INTERRUPT_DRIVEN) || defined(SPI_1_INTERRUPT_DRIVEN)
const unsigned int SPI_IE[2] = {IE1_, IE2_};
#endif
//const unsigned int SPI_IFG[2] = {IFG1_, IFG2_};

const unsigned char SPI_USPIE[2] = {USPIE1, USPIE1};
#if defined(SPI_0_INTERRUPT_DRIVEN) || defined(SPI_1_INTERRUPT_DRIVEN)
const unsigned char SPI_UTXRXIE[2] = {UTXIE0+URXIE0, UTXIE1+URXIE1};
#endif
//const unsigned char SPI_UTXIFG[2] = {UTXIFG0, UTXIFG1};
//const unsigned char SPI_URXIFG[2] = {URXIFG0, URXIFG1};

#define SPI_REG(REG, PORT) *(volatile unsigned char *)(REG[PORT])

void SPI_Shutdown(unsigned char Port)
{
  SPI_REG(SPI_UCTL, Port) |= SWRST; // Hold reset state
  SPI_REG(SPI_PORT_SEL, Port) &= ~0x38;
}

void SPI_Wakeup(unsigned char Port)
{
  SPI_REG(SPI_PORT_SEL, Port) |= 0x38;
  SPI_REG(SPI_UCTL, Port) &= ~SWRST; // Release reset state
}

void SPI_MasterInit(unsigned char Port, unsigned char Settings, unsigned int Baudrate)
{
  SPI_REG(SPI_PORT_SEL, Port) |= 0x38;
  SPI_REG(SPI_PORT_DIR, Port) |= BIT2;
  SPI_REG(SPI_UCTL, Port) = CHAR + SYNC + MM + SWRST; // 8-bit, SPI, Master
  SPI_REG(SPI_UTCTL, Port) = Settings + STC; // 3-wire
  SPI_REG(SPI_UBR0, Port) = Baudrate & 0xFF;
  SPI_REG(SPI_UBR1, Port) = (Baudrate >> 8) & 0xFF;
  SPI_REG(SPI_UMCTL, Port) = 0x00;
  SPI_REG(SPI_ME, Port) = SPI_USPIE[Port]; // Module enable
  SPI_REG(SPI_UCTL, Port) &= ~SWRST; // SPI initialize
#ifdef SPI_0_INTERRUPT_DRIVEN
  if (Port == 0) SPI_REG(SPI_IE, Port) = SPI_UTXRXIE[Port]; // Interrupts enable
#endif
#ifdef SPI_1_INTERRUPT_DRIVEN
  if (Port == 1) SPI_REG(SPI_IE, Port) = SPI_UTXRXIE[Port]; // Interrupts enable
#endif
}

unsigned char SPI_1_SendReceiveByte(unsigned char Data)
{
  SPI_1_SendByte(Data);
  SPI_1_WaitCompletion();

  return U1RXBUF;
}

unsigned char SPI_1_ReceiveByte()
{
  return SPI_1_SendReceiveByte(0);
}

