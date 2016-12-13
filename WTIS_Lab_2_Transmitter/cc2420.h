#ifndef __CC2420_H
#define __CC2420_H

#include <system.h>

#define CC2420_SPI_PORT_SEL     P4SEL

#define CC2420_SPI_CS_PIN	BIT2
#define CC2420_SPI_CS_PORT_DIR	P4DIR
#define CC2420_SPI_CS_PORT_OUT	P4OUT

#define CC2420_SPI_REG_CTL      U1CTL
#define CC2420_SPI_REG_TCTL     U1TCTL
#define CC2420_SPI_REG_BR0      U1BR0
#define CC2420_SPI_REG_BR1      U1BR1
#define CC2420_SPI_REG_MCTL     U1MCTL
#define CC2420_SPI_REG_ME       ME2
#define CC2420_SPI_REG_TXBUF    U1TXBUF
#define CC2420_SPI_REG_RXBUF    U1RXBUF
#define CC2420_SPI_REG_IFG      IFG2
#define CC2420_SPI_BIT_TXIFG    UTXIFG1
#define CC2420_SPI_BIT_RXIFG    URXIFG1

#define CC2420_SPI_WaitCompletion()	while(!(CC2420_SPI_REG_IFG & CC2420_SPI_BIT_RXIFG));
#define CC2420_SPI_SendByte(x)	        st( CC2420_SPI_REG_IFG &= ~CC2420_SPI_BIT_RXIFG;  CC2420_SPI_REG_TXBUF = x; while(!(CC2420_SPI_REG_IFG & CC2420_SPI_BIT_TXIFG));)
#define CC2420_SPI_Wakeup(x)		st( CC2420_SPI_PORT_SEL |= 0x38; CC2420_SPI_REG_CTL &= ~SWRST; )
#define CC2420_SPI_Shutdown(x)	        st( CC2420_SPI_REG_CTL |= SWRST; CC2420_SPI_PORT_SEL &= ~0x38; )

#define CC2420_VREG_EN_PIN	BIT5
#define CC2420_FIFO_PIN		BIT6
#define CC2420_FIFOP_PIN	BIT7
#define CC2420_CCA_PIN		BIT2
#define CC2420_SFD_PIN		BIT3
#define CC2420_RESET_PIN	BIT4

#define CC2420_RESET_PORT_DIR	P1DIR
#define CC2420_RESET_PORT_OUT	P1OUT
#define CC2420_VREG_EN_PORT_DIR	P1DIR
#define CC2420_VREG_EN_PORT_OUT	P1OUT
#define CC2420_CCA_PORT_DIR	P1DIR
#define CC2420_CCA_PORT_IN	P1IN
#define CC2420_FIFO_PORT_DIR	P1DIR
#define CC2420_FIFO_PORT_IN	P1IN
#define CC2420_FIFOP_PORT_DIR	P1DIR
#define CC2420_FIFOP_PORT_IN	P1IN
#define CC2420_FIFOP_PORT_IES	P1IES
#define CC2420_FIFOP_PORT_IE	P1IE
#define CC2420_FIFOP_PORT_IFG	P1IFG
#define CC2420_SFD_PORT_DIR	P1DIR
#define CC2420_SFD_PORT_IN	P1IN

#define CC2420_PORT_INT_VECTOR	PORT1_VECTOR

#define CC2420_SPI_CS_Enable() 		(CC2420_SPI_CS_PORT_OUT &= ~CC2420_SPI_CS_PIN)
#define CC2420_SPI_CS_Disable()		(CC2420_SPI_CS_PORT_OUT |= CC2420_SPI_CS_PIN)

#define CC2420_FIFOP_Int_Enable()	(CC2420_FIFOP_PORT_IE |= CC2420_FIFOP_PIN)
#define CC2420_FIFOP_Int_Disable()	(CC2420_FIFOP_PORT_IE &= ~CC2420_FIFOP_PIN)
#define CC2420_FIFOP_Int_Asserted()	(CC2420_FIFOP_PORT_IFG & CC2420_FIFOP_PIN)
#define CC2420_FIFOP_Int_Clear()	(CC2420_FIFOP_PORT_IFG &= ~CC2420_FIFOP_PIN)

#define CC2420_PAN_ID		0x2420

// CC2420 register constants
#define CC2420_SNOP             0x00
#define CC2420_SXOSCON          0x01
#define CC2420_STXCAL           0x02
#define CC2420_SRXON            0x03
#define CC2420_STXON            0x04
#define CC2420_STXONCCA         0x05
#define CC2420_SRFOFF           0x06
#define CC2420_SXOSCOFF         0x07
#define CC2420_SFLUSHRX         0x08
#define CC2420_SFLUSHTX         0x09
#define CC2420_SACK             0x0A
#define CC2420_SACKPEND         0x0B
#define CC2420_SRXDEC           0x0C
#define CC2420_STXENC           0x0D
#define CC2420_SAES             0x0E

#define CC2420_MAIN             0x10
#define CC2420_MDMCTRL0         0x11
#define CC2420_MDMCTRL1         0x12
#define CC2420_RSSI             0x13
#define CC2420_SYNCWORD         0x14
#define CC2420_TXCTRL           0x15
#define CC2420_RXCTRL0          0x16
#define CC2420_RXCTRL1          0x17
#define CC2420_FSCTRL           0x18
#define CC2420_SECCTRL0         0x19
#define CC2420_SECCTRL1         0x1A
#define CC2420_BATTMON          0x1B
#define CC2420_IOCFG0           0x1C
#define CC2420_IOCFG1           0x1D
#define CC2420_MANFIDL          0x1E
#define CC2420_MANFIDH          0x1F
#define CC2420_FSMTC            0x20
#define CC2420_MANAND           0x21
#define CC2420_MANOR            0x22
#define CC2420_AGCCTRL          0x23
#define CC2420_AGCTST0          0x24
#define CC2420_AGCTST1          0x25
#define CC2420_AGCTST2          0x26
#define CC2420_FSTST0           0x27
#define CC2420_FSTST1           0x28
#define CC2420_FSTST2           0x29
#define CC2420_FSTST3           0x2A
#define CC2420_RXBPFTST         0x2B
#define CC2420_FSMSTATE         0x2C
#define CC2420_ADCTST           0x2D
#define CC2420_DACTST           0x2E
#define CC2420_TOPTST           0x2F
#define CC2420_RESERVED         0x30

#define CC2420_TXFIFO           0x3E
#define CC2420_RXFIFO           0x3F
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Memory
// Sizes
#define CC2420_RAM_SIZE		368
#define CC2420_FIFO_SIZE	128

// Addresses
#define CC2420RAM_TXFIFO	0x000
#define CC2420RAM_RXFIFO	0x080
#define CC2420RAM_KEY0		0x100
#define CC2420RAM_RXNONCE	0x110
#define CC2420RAM_SABUF		0x120
#define CC2420RAM_KEY1		0x130
#define CC2420RAM_TXNONCE	0x140
#define CC2420RAM_CBCSTATE	0x150
#define CC2420RAM_IEEEADDR	0x160
#define CC2420RAM_PANID		0x168
#define CC2420RAM_SHORTADDR	0x16A
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Status byte
#define CC2420_XOSC16M_STABLE	0x40
#define CC2420_TX_UNDERFLOW	0x20
#define CC2420_ENC_BUSY		0x10
#define CC2420_TX_ACTIVE	0x08
#define CC2420_LOCK		0x04
#define CC2420_RSSI_VALID	0x02

//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// SECCTRL0
#define CC2420_SECCTRL0_NO_SECURITY         0x0000
#define CC2420_SECCTRL0_CBC_MAC             0x0001
#define CC2420_SECCTRL0_CTR                 0x0002
#define CC2420_SECCTRL0_CCM                 0x0003

#define CC2420_SECCTRL0_SEC_M_IDX           2

#define CC2420_SECCTRL0_RXKEYSEL0           0x0000
#define CC2420_SECCTRL0_RXKEYSEL1           0x0020

#define CC2420_SECCTRL0_TXKEYSEL0           0x0000
#define CC2420_SECCTRL0_TXKEYSEL1           0x0040

#define CC2420_SECCTRL0_SEC_CBC_HEAD        0x0100
#define CC2420_SECCTRL0_RXFIFO_PROTECTION   0x0200

#define CC2420_IOCFG1_CCAMUX		  	0
#define CC2420_IOCFG1_SFDMUX		  	5

//-------------------------------------------------------------------------------------------------------
// RSSI to Energy Detection conversion
// RSSI_OFFSET defines the RSSI level where the PLME.ED generates a zero-value
#define RSSI_OFFSET -38
#define RSSI_2_ED(rssi)   ((rssi) < RSSI_OFFSET ? 0 : ((rssi) - (RSSI_OFFSET)))
#define ED_2_LQI(ed) (((ed) > 63 ? 255 : ((ed) << 2)))
//-------------------------------------------------------------------------------------------------------

#pragma pack(1)

#define CC2420_BROADCAST_ADDRESS	0xFFFF

// MAC frame control field
typedef
  struct
  {
    unsigned int FrameType:3;
    unsigned int SecurityEnabled:1;
    unsigned int FramePending:1;
    unsigned int AckRequest:1;
    unsigned int IntraPAN:1;
    unsigned int Reserved:3;
    unsigned int DestAddrMode:2;
    unsigned int Reserved2:2;
    unsigned int SrcAddrMode:2;
  } CC2420_FCF;

#define CC2420_FRAME_TYPE_BEACON	0
#define CC2420_FRAME_TYPE_DATA		1
#define CC2420_FRAME_TYPE_ACK		2
#define CC2420_FRAME_TYPE_CMD		3

#define CC2420_FCF_ACK			0x8861
#define CC2420_FCF_NO_ACK		0x8841

// IEEE 802.15.4 MAC frame header
typedef
  struct
  {
//    CC2420_FCF FrameControl;
    unsigned int FrameControl;
    unsigned char Seq;
    unsigned int DestPANID;
    unsigned int DestAddress;
    unsigned int SrcAddress;
  } CC2420_FRAME_HEADER;

#define CC2420_FRAME_OVERHEAD (sizeof(CC2420_FRAME_HEADER) + 2) // Frame header+FCS
#define CC2420_FRAME_PAYLOAD_MAX_SIZE (127 - CC2420_FRAME_OVERHEAD)
#define CC2420_FRAME_ACK_SIZE 5

typedef
  struct
  {
    unsigned int SrcID;
    unsigned int DestID;
    unsigned char Seq;
    unsigned char AckRequest:1;
  } RADIO_PACKET_TX_INFO;

typedef
  struct
  {
    unsigned int SrcID;
    unsigned int DestID;
    unsigned char Seq;
    signed char RSSI;
    unsigned char LQI;
  } RADIO_PACKET_RX_INFO;

typedef
  struct
  {
    unsigned char LastTxSeq;
    unsigned char AckRequest;
    unsigned char AckReceived;
  } RADIO_STATUS;

#define CC2420_CRC_OK_MASK		0x80

#define CC2420_USEC_PER_BYTE		32
#define CC2420_USEC_PER_SYMBOL		16
#define CC2420_PACKET_DURATION(n)	(12*CC2420_USEC_PER_SYMBOL + (4+1+1+11+n)*CC2420_USEC_PER_BYTE)
#define CC2420_ACK_DURATION		(12*CC2420_USEC_PER_SYMBOL + (4+1+1+CC2420_FRAME_ACK_SIZE)*CC2420_USEC_PER_BYTE)
#define CC2420_ACK_TIMEOUT		(CC2420_ACK_DURATION+10) // = ack frame duration + software overhead

#pragma pack()

#define CC2420_RSSI_MAX			42
#define CC2420_RSSI_PERCENT(R)		(R+(100-CC2420_RSSI_MAX))
#define CC2420_RSSI_LEVEL(P)		(signed char)(P-(100-CC2420_RSSI_MAX))
#define CC2420_RSSI_2_DBM(R)		(signed char)(R-45)

typedef void (*CC2420_RX_PACKET_CALLBACK) (RADIO_PACKET_RX_INFO &Info, unsigned char *Data, unsigned char Size);

void CC2420_Init(CC2420_RX_PACKET_CALLBACK Callback);
void CC2420_Reset();
void CC2420_Setup();
void CC2420_Shutdown();
void CC2420_Wakeup();
void CC2420_VRegEnable();
void CC2420_VRegDisable();
void CC2420_SetChannel(unsigned char Channel);
unsigned char CC2420_GetChannel();
void CC2420_SetFrequency(unsigned char Frequency);
unsigned char CC2420_GetFrequency();
void CC2420_SetAddress(unsigned int Address);
void CC2420_SetPANID(unsigned int PANID);
void CC2420_SetReceiveMode();
void CC2420_SetIdleMode();
void CC2420_WriteCommand(unsigned char Command);
void CC2420_WriteRegister(unsigned char Address, unsigned int Data);
unsigned int CC2420_ReadRegister(unsigned char Address);
void CC2420_WriteRAM(unsigned int Address, unsigned char *DataBuf, unsigned char Size);
void CC2420_WriteFIFO(unsigned char *DataBuf, unsigned char Size);
void CC2420_ReadFIFO(unsigned char *DataBuf, unsigned char Size);
unsigned char CC2420_ReadStatus();
unsigned char CC2420_GetRSSI();
unsigned char CC2420_GetRSSILevel();
void CC2420_SetTransmitPower(unsigned char Level);
void CC2420_SetCCAThreshold(unsigned char Value);
unsigned char CC2420_ClearChannelAssessment();
unsigned char CC2420_CheckRxOverflow();
void CC2420_ClearRxFIFO();

void CC2420_DumpRegisters();

unsigned char CC2420_SendPacket(RADIO_PACKET_TX_INFO &Info, unsigned char *Data, unsigned char Size);
unsigned char CC2420_ResendPacket();

#endif // __CC2420_H
