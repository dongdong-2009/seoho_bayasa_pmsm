#include "00_main_def.h"

#define DI_NONE						0
#define DI_DRIVE_ENABLE				1 
#define DI_MULTI_STEP_BIT0			2
#define DI_MULTI_STEP_BIT1			3
#define DI_MULTI_STEP_BIT2			4
#define DI_MULTI_STEP_BIT3			5
#define DI_FAULT_RESET				6
#define DI_JOG						7
#define DI_AI_REF_ACTIVE			8
#define DI_AI_LOCAL_REMOTE			9
#define DI_EXT_FAULT_A				10
#define DI_EXT_FAULT_B				11
#define DI_MOTOR_SELECT				12
#define DI_MB_BRAKE_STATE			13
#define DI_ACCEL_DECEL_SWITCH		14
#define DI_REF_TUNING_INC			15
#define DI_REF_TUNING_DEC			16
#define DI_ACC_DEC_BYP				17
#define DI_PID_CNTL_ENABLE			18
#define DI_AUTO_PID_MODE			19
#define DI_PID_GAIN_SEL				20
#define DI_PID_I_RESET				21
#define DI_TRQ_REF_OPT_BYP			22
#define DI_TRQ_SIGN					23
#define DI_DI_TRQ_OUT_ZERO			24
#define DI_TIMER_RUN_ENABLE			25
#define DI_SLAVE_RUN_STATUS			26
#define DI_SYNC_CTRL_OPT_BPY		27
#define DI_FLYING_START				28
#define DI_DISABLE_PROFIBUS			29

#define DI_1FWD_2REV				0
#define	DI_1RUN_2DIR				1

#define DO_DISABLED_Aux_SW_Ctrl		0
#define DO_DRIVE_READY				1
#define DO_FAULT_OUT_A				2
#define DO_FAULT_OUT_B				3
#define DO_MOTOR_BRAKE				4
#define DO_RUN_STOP_STATUS			5
#define DO_WARNING_STATUS			6
#define DO_DIRECTION				7
#define DO_JOG_INPUT_STATE			8
#define DO_OV_OC_UV_LIMITING_FUNC	9
#define DO_FREE_FUNCTION			10

void set_digit_input_funtion( int input_state, int function_option, int di_port );

int input_state(int port)
{
	unsigned int xbus_in;
	int i,j;
	i = 1;
	j = port;
	while( j ){	i *= 2; j--; }
	asm ("      nop");
	xbus_in = ( ZONE0_BUF[0x0050] & i );
	if( xbus_in ) 	return 0;
	else 			return 1;
}


void set_digit_input_funtion( int input_state, int function_option, int di_port )
{
	if(di_port<=1)
	{
		if(!input_function_option[1])	// 1.FWD / 2.REV
		{
			if(di_port==0)	// DI 1
			{
				if	(!Flag.DI.bit.INVERT_RUN )	Flag.DI.bit.DRIVE_ENABLE = input_state;
				else							Flag.DI.bit.DRIVE_ENABLE = 0;
			}
			else if (di_port==1)	// DI 2
			{
				if(!Flag.DI.bit.DRIVE_ENABLE )	Flag.DI.bit.INVERT_RUN = input_state;
				else							Flag.DI.bit.INVERT_RUN = 0;
			}
		}
		else	// 1.RUN / 2.DIR
		{
			if		(di_port == 0)	Flag.DI.bit.DRIVE_ENABLE = input_state;
			else if (di_port == 1)	Flag.DI.bit.INVERT_DIR = input_state;
		}
	}
	else
	{
		switch( function_option )
		{
			case DI_NONE:	
				break;
			case DI_DRIVE_ENABLE:		
								{		
										Flag.DI.bit.DRIVE_ENABLE 		&= input_state;	
//										Run_Stop				 		&= input_state;
								}														break;
			case DI_MULTI_STEP_BIT0:	Flag.DI.bit.MULTI_STEP_BIT0	 	= input_state;	break;
			case DI_MULTI_STEP_BIT1:	Flag.DI.bit.MULTI_STEP_BIT1	 	= input_state;	break;
			case DI_MULTI_STEP_BIT2:	Flag.DI.bit.MULTI_STEP_BIT2	 	= input_state;	break;
			case DI_MULTI_STEP_BIT3:	Flag.DI.bit.MULTI_STEP_BIT3	 	= input_state;	break;
			case DI_FAULT_RESET:		Flag.DI.bit.FAULT_RESET 	 	= input_state;	break;
			case DI_JOG:				Flag.DI.bit.JOG			 	 	= input_state;	break;
			case DI_AI_REF_ACTIVE:		Flag.DI.bit.AI_REF_ACTIVE	 	= input_state;	break;
			case DI_AI_LOCAL_REMOTE:	Flag.DI.bit.AI_LOCAL_REMOTE  	= input_state;	break;
			case DI_EXT_FAULT_A:		Flag.DI.bit.EXT_FAULT_A	  	 	= input_state;	break;
			case DI_EXT_FAULT_B:		Flag.DI.bit.EXT_FAULT_B	  	 	= input_state;	break;
			case DI_MOTOR_SELECT:		Flag.DI.bit.MOTOR_SELECT	 	= input_state;	break;
			case DI_MB_BRAKE_STATE:		Flag.DI.bit.MB_BRAKE_STATE 	 	= input_state;	break;
			case DI_ACCEL_DECEL_SWITCH:	Flag.DI.bit.ACCEL_DECEL_SWITCH 	= input_state;	break;
			case DI_REF_TUNING_INC:		Flag.DI.bit.REF_TUNING_INC	 	= input_state;	break;
			case DI_REF_TUNING_DEC:		Flag.DI.bit.REF_TUNING_DEC	 	= input_state;	break;
			case DI_ACC_DEC_BYP:		Flag.DI.bit.ACC_DEC_BYP		 	= input_state;	break;
			case DI_PID_CNTL_ENABLE:	Flag.DI.bit.PID_CNTL_ENABLE	 	= input_state;	break;
			case DI_AUTO_PID_MODE:		Flag.DI.bit.AUTO_PID_MODE	 	= input_state;	break;
			case DI_PID_GAIN_SEL:		Flag.DI.bit.PID_GAIN_SEL	 	= input_state;	break;
			case DI_PID_I_RESET:		Flag.DI.bit.PID_I_RESET		 	= input_state;	break;
			case DI_TRQ_REF_OPT_BYP:	Flag.DI.bit.TRQ_REF_OPT_BYP	 	= input_state;	break;
			case DI_TRQ_SIGN:			Flag.DI.bit.TRQ_SIGN		 	= input_state;	break;
			case DI_DI_TRQ_OUT_ZERO:	Flag.DI.bit.TRQ_OUT_ZERO	 	= input_state;	break;
			case DI_TIMER_RUN_ENABLE:	Flag.DI.bit.TIMER_RUN_ENABLE 	= input_state;	break;
			case DI_SLAVE_RUN_STATUS:	Flag.DI.bit.SLAVE_RUN_STATUS 	= input_state;	break;
			case DI_SYNC_CTRL_OPT_BPY:	Flag.DI.bit.SYNC_CTRL_OPT_BPY	= input_state;	break;
			case DI_FLYING_START:		Flag.DI.bit.FLYING_START		= input_state;	break;
			case DI_DISABLE_PROFIBUS:	Flag.DI.bit.DISABLE_PROFIBUS	= input_state;	break;
		}
	}
}

void digital_input_proc()
{
	static int temp;
	static int a=0;

// 함수가 실행 될때 마다 A를 하나씩 증가 시키면서 
// A에 해당된 DI 입력을 확인 한다. A는 0~8
	if(a<9)	a++;
//	else if (( a< 0)||(a>8)) a=0;
	else a=0;

	temp = input_state(a);
	set_digit_input_funtion( temp, input_function_option[a], a);
	switch (a)
	{
		case 0:	if( input_state(a))	DI_STATUS |= 0x0001;
				else  				DI_STATUS &= 0x00FE;
				break;
		case 1:	if( input_state(a))	DI_STATUS |= 0x0002;
				else  				DI_STATUS &= 0x00FD;
				break;
		case 2:	if( input_state(a))	DI_STATUS |= 0x0004;
				else  				DI_STATUS &= 0x00FB;
				break;
		case 3:	if( input_state(a))	DI_STATUS |= 0x0008;
				else  				DI_STATUS &= 0x00F7;
				break;
		case 4:	if( input_state(a))	DI_STATUS |= 0x0010;
				else  				DI_STATUS &= 0x00EF;
				break;
		case 5:	if( input_state(a))	DI_STATUS |= 0x0020;
				else  				DI_STATUS &= 0x00DF;
				break;
		case 6:	if( input_state(a))	DI_STATUS |= 0x0040;
				else  				DI_STATUS &= 0x00BF;
				break;
		case 7:	if( input_state(a))	DI_STATUS |= 0x0080;
				else  				DI_STATUS &= 0x007F;
				break;
	}
}

int digital_port_check( int out_function )
{
	int i;
	i = 0;
	switch (out_function)
	{
		case 0:		i= 0;						break;
		case 1:		i= Flag.DO.bit.DRIVE_READY;	break;
		case 2:		i= Flag.DO.bit.FAULT_OUT_A;	break;
		case 3:		i= Flag.DO.bit.FAULT_OUT_B;	break;
		case 4:		i= Flag.DO.bit.MOTOR_BRAKE;	break;
		case 5:		i= Flag.DO.bit.RUN_STOP;	break;
		case 6:		i= Flag.DO.bit.WARNING;		break;
		case 7:		i= Flag.DO.bit.DIRECTION;	break;
		case 8:		i= Flag.DO.bit.JOG_INPUT;	break;
		case 9:		i= Flag.DO.bit.VC_LIMIT;	break;
		case 10:	i= Flag.DO.bit.FREE;		break;
		default:	i= 0;						break;
	}
	return i;
}  	


void digital_out_proc ()
{
	int i;
	// Flag.DOtion Update // 2011_04_01 dbsgln
//	if(( FaultReg1[1] )||( FaultReg2[1] ))
	if(FaultInfo)
	{
		Flag.DO.bit.FAULT_OUT_A = 1;
		Flag.DO.bit.FAULT_OUT_B = 0;
		Flag.DO.bit.DRIVE_READY = 0;
	}
	else
	{
		Flag.DO.bit.FAULT_OUT_A = 0;
		Flag.DO.bit.FAULT_OUT_B = 1;
//		if(Seq != SEQ_NoReady ) Flag.DO.bit.DRIVE_READY = 1;
	}
	Flag.DO.bit.RUN_STOP = Run_Stop_Status;
// 2011_04_19 (Singapore PSA 현장 ) dbsgln
//	if		 (Temperature >= 42. ) Fan_power_En = 1;
//	else if  (Temperature <= 38. ) Fan_power_En = 0;

//	if (Fan_power_En)	Flag.DO.bit.WARNING = 1;
//	else 				Flag.DO.bit.WARNING = 0;
///

	i = digital_port_check( digital_out_funtion[0] );
	if( i ) digital_out0_off(), 	DO_STATUS |= 0x0001;
	else 	digital_out0_on(), DO_STATUS &= 0x00FE;

	i = digital_port_check( digital_out_funtion[1] );
	if( i ) digital_out1_off(),	DO_STATUS |= 0x0002;
	else 	digital_out1_on(),	DO_STATUS &= 0x00FD;

	i = digital_port_check( digital_out_funtion[2] );
	if( i ) digital_out2_off(),	DO_STATUS |= 0x0004;
	else 	digital_out2_on(),	DO_STATUS &= 0x00FB;
}

