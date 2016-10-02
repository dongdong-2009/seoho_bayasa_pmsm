#include "00_main_def.h"
extern int time_share;
#define	CPUCLK			150000000L							// CPU Main Clock
#define	SCIB_LSPCLK		(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-B
#define	SCIB_BAUDRATE	115200L								// SCI-B Baudrate
#define	SCIB_BRR_VAL	(SCIB_LSPCLK/(8*SCIB_BAUDRATE)-1)	// SCI-B BaudRate 설정 Register 값

#define	SCIC_LSPCLK		(CPUCLK/4)							// Peripheral Low Speed Clock for SCI-C
#define	SCIC_BAUDRATE	19200L								// SCI-C Baudrate
//#define	SCIC_BAUDRATE	9600L								// SCI-C Baudrate
#define	SCIC_BRR_VAL	(SCIC_LSPCLK/(8*SCIC_BAUDRATE)-1)	// SCI-C BaudRate 설정 Register 값

/* BPS 에러율 *****************************************************************
*    BPS	   CPUCLK	  LSPCLK	         BRR_VAL	     BPS	error
*   4800	150000000	37500000	975.5625	976 	4797.851 	-0.045
*   9600	150000000	37500000	487.28125	487 	9605.533 	0.058
*  19200	150000000	37500000	243.140625	243 	19211.066 	0.058
*  38400	150000000	37500000	121.0703125	121 	38422.131 	0.058
*  57600	150000000	37500000	80.38020833	80 		57870.370 	0.469
* 115200	150000000	37500000	39.69010417	40 		114329.268 	-0.756
******************************************************************************/



#define	SCIB_TX_START	{	if(ScibRegs.SCICTL2.bit.TXRDY){						\
								ScibRegs.SCICTL2.bit.TXINTENA=1;				\
								ScibRegs.SCITXBUF = scib_tx_buf[scib_tx_pos++];	\
								if(scib_tx_pos >= SCIB_BUF_SIZE) scib_tx_pos=0;	\
							}													\
							else ScibRegs.SCICTL2.bit.TXINTENA=1;				\
						}

#define	SCIB_TX_STOP	ScibRegs.SCICTL2.bit.TXINTENA=0

#define	SCIC_TX_START	{	if(ScicRegs.SCICTL2.bit.TXRDY){						\
								ScicRegs.SCICTL2.bit.TXINTENA=1;				\
								ScicRegs.SCITXBUF = scic_tx_buf[scic_tx_pos++];	\
								if(scic_tx_pos >= SCIC_BUF_SIZE) scic_tx_pos=0;	\
							}													\
							else ScicRegs.SCICTL2.bit.TXINTENA=1;				\
					}

#define	SCIC_TX_STOP	ScicRegs.SCICTL2.bit.TXINTENA=0

//---------------------------------------------------------
#define	GEN_POLYNOMAL	0x8821		// CRC 젯수

#define QUERY		0x01
#define SEND		0x02
#define RESPONSE	0x03
#define REQUEST		0x04


// SCI-B, SCI-C Interrupt Service Function 선언
//interrupt void scib_tx_isr(void);
//interrupt void scib_rx_isr(void);
interrupt void scic_tx_isr(void);
interrupt void scic_rx_isr(void);
//#pragma CODE_SECTION(scib_tx_isr, "ramfuncs");
//#pragma CODE_SECTION(scib_rx_isr, "ramfuncs");
#pragma CODE_SECTION(scic_tx_isr, "ramfuncs");
#pragma CODE_SECTION(scic_rx_isr, "ramfuncs");

//void CRC_16(unsigned char input);
unsigned int CRC16( unsigned char * pucFrame,unsigned int usLen );
#pragma CODE_SECTION(CRC16, "ramfuncs");
void SCIC_Process(void);
#pragma CODE_SECTION(SCIC_Process, "ramfuncs"); 

/************************************************************************/
/*      Initialize SCI                                                  */
/************************************************************************/
/*---------------------------------------------*/
/*      Initialize SCI                         */
/*---------------------------------------------*/
/*
void scib_init(void)
{
	ScibRegs.SCIFFTX.all = 0x8000;			// FIFO reset
 	ScibRegs.SCIFFCT.all = 0x4000;			// Clear ABD(Auto baud bit)
 	
 	ScibRegs.SCICCR.all = 0x0007;  			// 1 stop bit,  No loopback 
                                   			// No parity,8 char bits,
                                   			// async mode, idle-line protocol
	ScibRegs.SCICTL1.all = 0x0003; 			// enable TX, RX, internal SCICLK, 
                                   			// Disable RX ERR, SLEEP, TXWAKE

	ScibRegs.SCICTL2.bit.RXBKINTENA = 1;	// RX/BK INT ENA=1,
	ScibRegs.SCICTL2.bit.TXINTENA = 1;		// TX INT ENA=1,

  	ScibRegs.SCIHBAUD = SCIB_BRR_VAL >> 8;
  	ScibRegs.SCILBAUD = SCIB_BRR_VAL & 0xff;

	ScibRegs.SCICTL1.all = 0x0023;			// Relinquish SCI from Reset  
    
	// Initialize SCI-B RX interrupt
  	EALLOW;
	PieVectTable.SCIRXINTB = &scib_rx_isr;
  	PieVectTable.SCITXINTB = &scib_tx_isr;
   
  // Enable internal pull-up for the selected pins 
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0; // Enable pull-up for GPIO11 (SCIRXDB)
	GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;  // Enable pull-up for GPIO9  (SCITXDB)

	// Set qualification for selected pins to asynch only 
	GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO11 (SCIRXDB)

	// Configure SCI-B pins using GPIO regs
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;   // Configure GPIO11 for SCIRXDB operation
	GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3;    // Configure GPIO9 for SCITXDB operation
	EDIS;

  // Enable CPU INT9 for SCI-B
	IER |= M_INT9;
	
  // Enable SCI-B RX INT in the PIE: Group 9 interrupt 3
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;

  // Enable SCI-B TX INT in the PIE: Group 9 interrupt 4
	PieCtrlRegs.PIEIER9.bit.INTx4 = 1;
}
*/


static const char aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const char aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};


unsigned int CRC16( unsigned char * pucFrame,unsigned int usLen )
{
    char           ucCRCHi = 0xFF;
    char           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( char )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( unsigned int )( ucCRCHi << 8 | ucCRCLo );
}



void scic_init(){
	ScicRegs.SCIFFTX.all = 0x8000;			// FIFO reset
 	ScicRegs.SCIFFCT.all = 0x4000;			// Clear ABD(Auto baud bit)
 	
 	ScicRegs.SCICCR.all = 0x0007;  			// 1 stop bit,  No loopback 
                                   			// No parity,8 char bits,
                                   			// async mode, idle-line protocol
	ScicRegs.SCICTL1.all = 0x0003; 			// enable TX, RX, internal SCICLK, 
                                   			// Disable RX ERR, SLEEP, TXWAKE

	ScicRegs.SCICTL2.bit.RXBKINTENA = 1;	// RX/BK INT ENA=1,
	ScicRegs.SCICTL2.bit.TXINTENA = 1;		// TX INT ENA=1,

    ScicRegs.SCIHBAUD = SCIC_BRR_VAL >> 8;
    ScicRegs.SCILBAUD = SCIC_BRR_VAL & 0xff;

	ScicRegs.SCICTL1.all = 0x0023;			// Relinquish SCI from Reset  
    
	// Initialize SCI-C RX interrupt
    EALLOW;
	PieVectTable.SCIRXINTC = &scic_rx_isr;
    PieVectTable.SCITXINTC = &scic_tx_isr;
   
    /* Enable internal pull-up for the selected pins */
	GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0; // Enable pull-up for GPIO62 (SCIRXDC)
	GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;  // Enable pull-up for GPI63  (SCITXDC)

	/* Set qualification for selected pins to asynch only */
	GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO11 (SCIRXDC)

	/* Configure SCI-C pins using GPIO regs*/
	GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 for SCIRXDC operation
	GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;    // Configure GPI63 for SCITXDC operation
	EDIS;

    // Enable CPU INT8 for SCI-C
	IER |= M_INT8;

	// Enable SCI-C RX INT in the PIE: Group 8 interrupt 5
	PieCtrlRegs.PIEIER8.bit.INTx5 = 1;

	// Enable SCI-C TX INT in the PIE: Group 8 interrupt 6
	PieCtrlRegs.PIEIER8.bit.INTx6 = 1;
}

/************************************************************************/
/*      Transmmit data by polling                                       */
/************************************************************************/
/*---------------------------------------------*/
/*      Transmmit Character                    */
/*---------------------------------------------*/
void scib_TxChar(char c)
{
    while(!ScibRegs.SCICTL2.bit.TXRDY);
    ScibRegs.SCITXBUF=c;
}    

/*---------------------------------------------*/
/*      Transmmit String                       */
/*---------------------------------------------*/
void scib_TxString(char *p)
{
    char	rd;
    while(rd = *p++) scib_TxChar(rd);
}

void scic_TxChar(char c)
{
    while(!ScicRegs.SCICTL2.bit.TXRDY);
    ScicRegs.SCITXBUF=c;
}    

/*---------------------------------------------*/
/*      Transmmit String                       */
/*---------------------------------------------*/
void scic_TxString(char *p)
{
    char	rd;
    while(rd = *p++) scic_TxChar(rd);
}

/************************************************************************/
/*      Transmmit character by interrupt                                */
/************************************************************************/
/*---------------------------------------------*/
/*      SCI TX Start                           */
/*---------------------------------------------*/
void scib_tx_start(void)
{
	SCIB_TX_START;
}

void scic_tx_start(void)
{
	SCIC_TX_START;
}

/*---------------------------------------------*/
/*      Transmmit Character                    */
/*---------------------------------------------*/
void scib_putc(char d)
{
	scib_tx_buf[scib_tx_end++] = d;
	if(scib_tx_end >= SCIB_BUF_SIZE) scib_tx_end = 0;
}

void scic_putc(char d)
{
	scic_tx_buf[scic_tx_end++] = d;
	if(scic_tx_end >= SCIC_BUF_SIZE) scic_tx_end = 0;
}

/*---------------------------------------------*/
/*      Transmmit String                       */
/*---------------------------------------------*/
void scib_puts(char *p)
{
  char rd;

	while(rd = *p++){             
		scib_tx_buf[scib_tx_end++] = rd;
		if(scib_tx_end >= SCIB_BUF_SIZE) scib_tx_end = 0;
	}
}

void scic_puts(char *p)
{
  char rd;

	while(rd = *p++){             
		scic_tx_buf[scic_tx_end++] = rd;
		if(scic_tx_end >= SCIC_BUF_SIZE) scic_tx_end = 0;
	}
}

//-----
int tx_cnt=0, rx_cnt=0;
interrupt void scib_tx_isr(void)
{
	if(scib_tx_pos != scib_tx_end){
		ScibRegs.SCITXBUF = scib_tx_buf[scib_tx_pos++];
		if(scib_tx_pos >= SCIB_BUF_SIZE) scib_tx_pos = 0;
	}
	else{                              
		SCIB_TX_STOP;
	}

	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}

//-----
/*
interrupt void scib_rx_isr(void)
{
	//------------------------------
	// 이 사이에서 rx 처리 한다. 
	scib_rxd = ScibRegs.SCIRXBUF.all;
	
	scib_puts("SCI-B: rxd = ");
	scib_putc(scib_rxd);
	scib_puts("\r\n");
	

	// 버퍼에 저장하기만 하는 경우
//	scib_rx_buf[scib_rx_end++] = scib_rxd;
//	if (scib_rx_end >= SCIC_BUF_SIZE) scib_rx_end = 0;

	//------------------------------
	SCIB_TX_START;
	
	// Acknowledge this interrupt to recieve more interrupts from group 9
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}
*/
//-----
interrupt void scic_tx_isr(void)
{
    tx_cnt++;
//	delay_ms(1);
	if(scic_tx_pos != scic_tx_end)
	{
		//if(ScicRegs.SCICTL2.bit.TXRDY)
		{	
			ScicRegs.SCITXBUF = scic_tx_buf[scic_tx_pos++];
			if(scic_tx_pos >= SCIC_BUF_SIZE) scic_tx_pos = 0;
		}
	}
	else
	{                              
		SCIC_TX_STOP;
	}

	// Acknowledge this interrupt to recieve more interrupts from group 8
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

//-----------------------------
// 데이타 수신
//-----------------------------

//unsigned char Temp_A;
//unsigned char Temp_B;
//unsigned int Temp_C;
interrupt void scic_rx_isr(void)
{
	char c;

	scic_rxd = ScicRegs.SCIRXBUF.all;
	if(!SciC_RxFlag)
	{
		if(SciC_RxStep == 0)//sync1
		{
			if(scic_rxd == 0xAB)
			{
				
				RxBuf[0] = 0xAB;
				SciC_RxStep++;
			}
			else SciC_RxStep=0;
		}
		else if(SciC_RxStep == 1)//sync2
		{
			if(scic_rxd == 0xCD)
			{
				RxBuf[1] = 0xCD;
				SciC_RxStep++;
			}
			else SciC_RxStep=0;
		}
		else if(SciC_RxStep == 2)//type
		{
			RxBuf[2] = scic_rxd;
			SciC_RxStep++;
		}
		else if(SciC_RxStep == 3)//addr_h
		{
			RxBuf[3] = scic_rxd;
			SciC_RxStep++;
		}
		else if(SciC_RxStep == 4)//addr_l
		{
			RxBuf[4] = scic_rxd;
			SciC_RxStep++;
		}
		else if(SciC_RxStep == 5)//data_h
		{
			RxBuf[5] = scic_rxd;
			SciC_RxStep++;
		}
		else if(SciC_RxStep == 6)//data_l
		{
			RxBuf[6] = scic_rxd;
			SciC_RxStep++;
		}
		else if(SciC_RxStep == 7)//crc_H
		{
			RxBuf[7] = scic_rxd;
			SciC_RxStep++;
		}
		else//crc_L
		{
			RxBuf[8] = scic_rxd;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
			//CRC.Word = 0;
			//CRC_16(RxBuf[0]);
			//CRC_16(RxBuf[1]);
			//CRC_16(RxBuf[2]);
			//CRC_16(RxBuf[3]);
			//CRC_16(RxBuf[4]);
			//CRC_16(RxBuf[5]);
			//CRC_16(RxBuf[6]);

			CRC.Word = CRC16(RxBuf,7);
			
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
			RxType = RxBuf[2];
			RxAddr = ((unsigned int)RxBuf[3]<<8) | RxBuf[4] ;
			RxData = ((unsigned int)RxBuf[5]<<8) | RxBuf[6] ;
			RxCRC   = ((unsigned int)RxBuf[7]<<8) | RxBuf[8] ;

			if((RxBuf[7] == CRC.Byte.b1) && (RxBuf[8] == CRC.Byte.b0))
			{
				SciC_RxFlag=1;
				SCI_Registers[RxAddr] = RxData;

				if(RxType == SEND)
				{
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
					CRC.Word = 0;

					RxBuf[2] = RESPONSE;

					CRC.Word = CRC16(RxBuf,7);

					for(c=0;c<7;c++)
					{
						scic_putc(RxBuf[c]);
					
					}
					//scic_putc(RxBuf[0]);		CRC_16(RxBuf[0]);
					//scic_putc(RxBuf[1]);		CRC_16(RxBuf[1]);
					//scic_putc(RESPONSE);		CRC_16(RESPONSE);
					//scic_putc(RxBuf[3]);		CRC_16(RxBuf[3]);
					//scic_putc(RxBuf[4]);		CRC_16(RxBuf[4]);
					//scic_putc(RxBuf[5]);		CRC_16(RxBuf[5]);
					//scic_putc(RxBuf[6]);		CRC_16(RxBuf[6]);
					scic_putc(CRC.Byte.b1);
					scic_putc(CRC.Byte.b0);
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
					SciC_TxFlag = 1;
					//SCIC_TX_START;
					Data_Registers[RxAddr] = RxData;

					// (110107 by HHH)
					Rx_index= RxAddr;
					Read_Data_Registers(Rx_index);
					Flag.Monitoring.bit.EEPROM_WRITE_ENABLE_Rx= 1;
				}
				else if(RxType == REQUEST)
				{

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
					CRC.Word = 0;

					RxBuf[2] = SEND;
					RxBuf[5] = (char)(Data_Registers[RxAddr]>>8);
					RxBuf[6] = (char)Data_Registers[RxAddr];

					CRC.Word = CRC16(RxBuf,7);

					for(c=0;c<7;c++)scic_putc(RxBuf[c]);
				
					//CRC.Word = 0;
					//scic_putc(RxBuf[0]);								CRC_16(RxBuf[0]);
					//scic_putc(RxBuf[1]);								CRC_16(RxBuf[1]);
					//scic_putc(SEND);									CRC_16(SEND);
					//scic_putc(RxBuf[3]);								CRC_16(RxBuf[3]);
					//scic_putc(RxBuf[4]);								CRC_16(RxBuf[4]);
					//scic_putc((char)(Data_Registers[RxAddr]>>8)); CRC_16((char)(Data_Registers[RxAddr]>>8));
					//scic_putc((char)Data_Registers[RxAddr]);		CRC_16((char)Data_Registers[RxAddr]);

					scic_putc(CRC.Byte.b1);
					scic_putc(CRC.Byte.b0);
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
					SCI_Registers[RxAddr] = 0;

					SciC_TxFlag = 1;
					//SCIC_TX_START;

					// (110107 by HHH)
					Rx_index= RxAddr;
					Read_Data_Registers(Rx_index);
					Flag.Monitoring.bit.EEPROM_WRITE_ENABLE_Rx= 1;
				}
				else if(RxType == QUERY)
				{
					Communication_Fault_Cnt = 3;
					if(RxData==1)
					{
					
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	
						//if(share_time == 8)	Device_type= 1;
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&	
					}
					else Device_type= 0;
				}
			}
			SciC_RxStep=0;
		}
	}
	else SciC_RxStep=0;
	// Acknowledge this interrupt to recieve more interrupts from group 8
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

//-----
void scic_test(void)
{
    scic_puts("SCI-C Test\r\n");
	SCIC_TX_START;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//===============================================================================================
/*void CRC_16(unsigned char input)
{
	unsigned char 	i ;
	unsigned int 	tmp_CRC ;

	tmp_CRC=((CRC.Word >> 8) ^ input) << 8 ;
	for (i = 0 ; i < 8 ; i++)
	{
		if (tmp_CRC & 0x8000) tmp_CRC = (tmp_CRC << 1) ^ GEN_POLYNOMAL ;
		else tmp_CRC <<= 1 ;
	}
	CRC.Word = (CRC.Word << 8) ^ tmp_CRC ;
}*/

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

// WORD SCI_Registers[Buf_MAX];
WORD SCI_TxOffset=0;
void SCIC_Process(void)
{
	float b = 0.;
	unsigned char 	c ;
	static int Tx_complete= 1;
	int time_share = 0;
	 unsigned char TxBuf[9];
//Rx========================================
	if(ScicRegs.SCIRXST.bit.RXERROR) scic_init(); // Detection of RXERROR
    if(SciC_RxFlag)	SciC_RxFlag = 0;
//Tx========================================
// Tx_count 시간에 도달 하지 못한 경우 자동 대기 함
//	SCI_TxOffset= 2310;
	if(!Data_Registers[3195])
	{
		if (Device_type== 0)
		{
			if(Tx_count_25ms>= (Uint16)(25e-3*Fsw-10.)) // 25.
			{
				if (Tx_count_1s>=(Uint16)(1.0*Fsw))
				{
					CRC.Word = 0;
					b= (float)CpuTimer0Regs.TIM.all;

					b= (b-(float)CpuTimer0Regs.TIM.all)/150.;  // us Time
					if (Interrupt_time_max<b)	Interrupt_time_max= b; 

//					scic_putc(0xAB);			CRC_16(0xAB);
//					scic_putc(0xCD);			CRC_16(0xCD);
//					scic_putc(QUERY);			CRC_16(QUERY);
//					scic_putc(0);				CRC_16(0);
//					scic_putc(0);				CRC_16(0);
//					scic_putc(0);				CRC_16(0);
//					scic_putc(0);				CRC_16(0);
//					scic_putc(CRC.Byte.b1);
//					scic_putc(CRC.Byte.b0);

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
					CRC.Word = 0;
					
					TxBuf[0] = 0xAB;
					TxBuf[1] = 0xCD;
					TxBuf[2] = QUERY;
					TxBuf[3] = 0;	
					TxBuf[4]=  0;
					TxBuf[5] = 0;
					TxBuf[6] = 0;

					CRC.Word = CRC16(TxBuf,7);

					for(c=0;c<7;c++)	scic_putc(TxBuf[c]);
					
					scic_putc(CRC.Byte.b1);
					scic_putc(CRC.Byte.b0);
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

					SCIC_TX_START;

					if(!Communication_Fault_Cnt)Communication_Fault_Flag=1;
					else 
					{
						Communication_Fault_Cnt--;
						Communication_Fault_Flag=0;
					}
				
					Tx_count_1s= 0;
				}
				else if(SciC_TxFlag)
				{
					SCIC_TX_START;
					SciC_TxFlag=0;
				}
				else 
				{
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&				
					CRC.Word = 0;
					TxBuf[0] = 0xAB;
					TxBuf[1] = 0xCD;
					TxBuf[2] = SEND;
					TxBuf[3] = ((char)(SCI_TxOffset>>8));	
					TxBuf[4]=  ((char)SCI_TxOffset);
					TxBuf[5] = (char)(Data_Registers[SCI_TxOffset]>>8);
					TxBuf[6] = (char)Data_Registers[SCI_TxOffset];

					CRC.Word = CRC16(TxBuf,7);

					for(c=0;c<7;c++)	scic_putc(TxBuf[c]);
					
					scic_putc(CRC.Byte.b1);
					scic_putc(CRC.Byte.b0);
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
					SCIC_TX_START;

					Tx_index= (int)SCI_TxOffset;
					SCI_TxOffset ++;
					Tx_complete= 1;
				}
				if( time_share == 8){  time_share = 0,	Tx_count_25ms= 0;}
			}

			if ( (Data_Registers[SCI_TxOffset] == SCI_Registers[SCI_TxOffset])&&(Tx_complete== 1) )
				SCI_TxOffset ++;
			else Tx_complete= 0;
			if(Buf_MAX <= SCI_TxOffset)	SCI_TxOffset = 0;
		}
		else
		{
			if (Tx_count_1s>=(Uint16)(Fsw))
			{
				if(!Communication_Fault_Cnt) Communication_Fault_Flag=1;
				else 
				{
					Communication_Fault_Cnt--;
					Communication_Fault_Flag=0;
				}
				Tx_count_1s= 0;
			}
			else if(SciC_TxFlag)
			{
				SCIC_TX_START;
				SciC_TxFlag=0;
			}
		}
	}
}
