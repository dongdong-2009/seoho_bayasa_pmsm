	#include "DSP2833x_Device.h"   
	#include "DSP2833x_Examples.h"   
	#include "math.h"
	#include "sci_bc.h"
	#include "01_PrototypeRef.h"
	#include "02_ExternRef.h"
	#include "03_All_Struct_Variables.h"

#define Red_LED_on		(GpioDataRegs.GPASET.bit.GPIO11 	= 1)	// D01
#define Red_LED_off		(GpioDataRegs.GPACLEAR.bit.GPIO11 	= 1)	// D01
#define Red_LED_toggle	(GpioDataRegs.GPATOGGLE.bit.GPIO11 	= 1)	// D01

#define Green_LED_on		(GpioDataRegs.GPBSET.bit.GPIO37 	= 1)	// D01
#define Green_LED_off		(GpioDataRegs.GPBCLEAR.bit.GPIO37 	= 1)	// D01
#define Green_LED_toggle	(GpioDataRegs.GPBTOGGLE.bit.GPIO37 	= 1)	// D01

#define digital_out0_on()	(GpioDataRegs.GPBSET.bit.GPIO58 = 1)	// D01
#define digital_out0_off()	(GpioDataRegs.GPBCLEAR.bit.GPIO58 = 1)	// D01

#define digital_out1_on()	(GpioDataRegs.GPBSET.bit.GPIO59 = 1)	// D02
#define digital_out1_off()	(GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1)	// D02

#define digital_out2_on()	(GpioDataRegs.GPBSET.bit.GPIO60 = 1)	// D03
#define digital_out2_off()	(GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1)	// D03

#define digital_out3_on()	(GpioDataRegs.GPBSET.bit.GPIO48 = 1)	// GPIO.1
#define digital_out3_off()	(GpioDataRegs.GPBCLEAR.bit.GPIO48 = 1)	// GPIO.1

#define digital_out4_on()	(GpioDataRegs.GPBSET.bit.GPIO49 = 1)	// GPIO.2
#define digital_out4_off()	(GpioDataRegs.GPBCLEAR.bit.GPIO49 = 1)	// GPIO.2

#define digital_out5_on()	(GpioDataRegs.GPBSET.bit.GPIO50 = 1)	// GPIO.3
#define digital_out5_off()	(GpioDataRegs.GPBCLEAR.bit.GPIO50 = 1)	// GPIO.3

#define digital_out6_on()	(GpioDataRegs.GPBSET.bit.GPIO51 = 1)	// GPIO.4
#define digital_out6_off()	(GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1)	// GPIO.4

#define digital_out7_on()	(GpioDataRegs.GPBSET.bit.GPIO52 = 1)	// GPIO.5
#define digital_out7_off()	(GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1)	// GPIO.5

#define nRESET_DRIVER_SET	(GpioDataRegs.GPASET.bit.GPIO7 	= 1)  
#define nRESET_DRIVER_CLEAR	(GpioDataRegs.GPACLEAR.bit.GPIO7= 1)  

#define nDC_CONTACT_CLEAR	(GpioDataRegs.GPACLEAR.bit.GPIO10 = 1)
#define nDC_CONTACT_SET		(GpioDataRegs.GPASET.bit.GPIO10   = 1)

#define nPWM_ENABLE_CLEAR	(GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1)
#define nPWM_ENABLE_SET		(GpioDataRegs.GPBSET.bit.GPIO34   = 1)

#define nBOOT_MODE_CLEAR	(GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1)
#define nBOOT_MODE_SET		(GpioDataRegs.GPBSET.bit.GPIO35   = 1)

#define nBACKUP_ENABLE_CLEAR (GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1)
#define nBACKUP_ENABLE_SET	 (GpioDataRegs.GPBSET.bit.GPIO39   = 1)

#define nFLT2_ON			 (GpioDataRegs.GPASET.bit.GPIO13 = 1)
#define nFLT2_OFF			 (GpioDataRegs.GPACLEAR.bit.GPIO13 = 1)

#define	nWP_ENABLE			(GpioDataRegs.GPBSET.bit.GPIO39 = 1)		// WP = 1, Write protected
#define nWP_DISABLE         (GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1)		// WP = 0, Write unprotected

#define		PI				3.141592654
#define		SQRT2			1.414213562
#define		SQRT3			1.732050808
#define		INV_SQRT3		(1./SQRT3)
#define		SYS_CLK			(float)150000000
#define		Deadtime		(float)2.0e-6
#define		RUNN			1
#define		STOP			0 
#define		BOUND_PI(x)		((x) - 2.*PI*(int)((x + PI)/(2.*PI)))

// Sequence Definition
#define	SEQ_NoReady	 0	// 시스템 전원 투입후 잠시 또는 DC build up 이루어지기 전 구간
#define SEQ_Wait	 1	// 시스템 Ready 상태이며 Run Signal 들어 오기 전
#define	SEQ_Normal	 2	// Run 신호 인가 후 시스템 정상 동작
#define	SEQ_Fault	 3	// 시스템 펄트 상태
#define	SEQ_Retrial	 4	// Fault Reset 후 운전 재게 확인 모드
#define SEQ_F_R_Read 5	// Fault Record Read
#define	SEQ_SYS_INIT 6	// System Initialize Sequence 

// Status Index Definition
#define	STATE_POWER_ON			'Y'
#define	STATE_CALIBRATION		'Z'
#define	STATE_READY				'R'
#define	STATE_STOP				'S'
#define	STATE_ACCELERATING		'A'
#define	STATE_DECELERATING		'D'
#define	STATE_RUNNING			'O'
#define	STATE_TUNING			'T'
#define	STATE_FAULT				'F'

