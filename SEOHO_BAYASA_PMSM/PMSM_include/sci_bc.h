/*****************************************************************************
*
* File Name: SCI_BC.H
*
* Description: Header file for SCI_B, SCI_C driver
*
* Modules Included:
*
*****************************************************************************/


#ifndef __SCI_BC_H
#define __SCI_BC_H

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "main_def.h"





typedef	union
{
	unsigned Word;
	struct
	{
		unsigned b0	:8;	// CRC16 ���� ����Ʈ
		unsigned b1	:8;	// CRC16 ���� ����Ʈ
	} Byte;
} CRC_flg ;

// SCI-B, SCI-C Interrupt Service Function ����
#pragma CODE_SECTION(scib_tx_isr, "ramfuncs");
#pragma CODE_SECTION(scib_rx_isr, "ramfuncs");
#pragma CODE_SECTION(scic_tx_isr, "ramfuncs");
#pragma CODE_SECTION(scic_rx_isr, "ramfuncs");

#define	CPUCLK			150000000L							// CPU Main Clock
#define	SCIB_LSPCLK		(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-B
#define	SCIB_BAUDRATE	115200L								// SCI-B Baudrate
#define	SCIB_BRR_VAL	(SCIB_LSPCLK/(8*SCIB_BAUDRATE)-1)	// SCI-B BaudRate ���� Register ��

#define	SCIC_LSPCLK		(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-C
#define	SCIC_BAUDRATE	115200L								// SCI-C Baudrate
//#define	SCIC_BAUDRATE	9600L								// SCI-C Baudrate
#define	SCIC_BRR_VAL	(SCIC_LSPCLK/(8*SCIC_BAUDRATE)-1)	// SCI-C BaudRate ���� Register ��

/* BPS ������ *****************************************************************
*    BPS	   CPUCLK	  LSPCLK	         BRR_VAL	     BPS	error
*   4800	150000000	37500000	975.5625	976 	4797.851 	-0.045 
*   9600	150000000	37500000	487.28125	487 	9605.533 	0.058 
*  19200	150000000	37500000	243.140625	243 	19211.066 	0.058 
*  38400	150000000	37500000	121.0703125	121 	38422.131 	0.058 
*  57600	150000000	37500000	80.38020833	80 		57870.370 	0.469 
* 115200	150000000	37500000	39.69010417	40 		114329.268 	-0.756 
******************************************************************************/

#define	SCIB_BUF_SIZE	50


#define	SCIB_TX_START	{	if(ScibRegs.SCICTL2.bit.TXRDY){						\
								ScibRegs.SCICTL2.bit.TXINTENA=1;				\
								ScibRegs.SCITXBUF = scib_tx_buf[scib_tx_pos++];	\
								if(scib_tx_pos >= SCIB_BUF_SIZE) scib_tx_pos=0;	\
							}													\
							else ScibRegs.SCICTL2.bit.TXINTENA=1;				\
						}

#define	SCIB_TX_STOP	ScibRegs.SCICTL2.bit.TXINTENA=0

#define	SCIC_BUF_SIZE	50

#define	SCIC_TX_START	{	if(ScicRegs.SCICTL2.bit.TXRDY){						\
								ScicRegs.SCICTL2.bit.TXINTENA=1;				\
								ScicRegs.SCITXBUF = scic_tx_buf[scic_tx_pos++];	\
								if(scic_tx_pos >= SCIC_BUF_SIZE) scic_tx_pos=0;	\
							}													\
							else ScicRegs.SCICTL2.bit.TXINTENA=1;				\
					}

#define	SCIC_TX_STOP	ScicRegs.SCICTL2.bit.TXINTENA=0

//---------------------------------------------------------
#define	GEN_POLYNOMAL	0x8821		// CRC ����

#define QUERY		0x01
#define SEND		0x02
#define RESPONSE	0x03
#define REQUEST		0x04


#define DEV_IM		0x01
#define DEV_DCDC	0x02
#define DEV_PMSM	0x03

//��� ������ ���
#define Buf_MAX 3400		             // RYU

extern unsigned int EEPROM_WRITE_ENABLE_Rx;
extern unsigned int EEPROM_WRITE_ENABLE_Tx;

//���� ��� ����
extern Uint16 Rx_index;
extern Uint16 Tx_index;
extern Uint16 TxIntervalCnt;
extern Uint16 TxInterval_1s;
extern int Dummy_comm;
extern CRC_flg	CRC;
extern long Test_count_Rx;
extern long Test_count_Tx;
extern int Rx_counter_1s;
extern int Tx_counter_1s;
extern long CRC_check_counter;


extern char scib_tx_buf[SCIB_BUF_SIZE+1];
extern char scib_tx_pos, scib_tx_end;
extern char scib_rx_buf[SCIB_BUF_SIZE+1];
extern char scib_rxd;

extern char scic_tx_buf[SCIC_BUF_SIZE+1];
extern char scic_tx_pos, scic_tx_end;
extern char scic_rx_buf[SCIC_BUF_SIZE+1];
extern char scic_rxd;

//-- Serial Data Stack  
extern WORD Data_Registers[Buf_MAX];
extern WORD CAN_Registers[Buf_MAX];
extern WORD SCI_Registers[Buf_MAX];  


void scib_init(void);
void scic_init(void);
void scib_TxChar(char c);
void scib_TxString(char *p);
void scic_TxChar(char c);
void scic_TxString(char *p);
void scib_tx_start(void);
void scic_tx_start(void);
void scib_putc(char d);
//void scic_putc(char d);
void scib_puts(char *p);
void scic_puts(char *p);
void sci_hex2(BYTE d);
char hex_to_asc(char hex);

interrupt void scib_tx_isr(void);
interrupt void scib_rx_isr(void);
interrupt void scic_tx_isr(void);
interrupt void scic_rx_isr(void);

void scic_test(void);
void CRC_16(unsigned char input);
#endif



