#include "00_main_def.h"

Uint16 EEPROM_WRITE_CHECK = 0;
#define EEPROM_WRITE_CHECK_VAL 31
float tester1 = 0.;
float tester2 = 0.;
void main(void)
{
	unsigned int i;

    InitSysCtrl();
	InitXintf();
	InitGpio();
	dev_DacSetup();
	dev_InitDACVariable(); 
	MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
	InitFlash();

	DINT;
	InitPieCtrl();
	IER = 0x0000;
	IFR = 0x0000;
	InitCpuTimers();   // For this example, only initialize the Cpu Timers
	ConfigCpuTimer(&CpuTimer0, 150, 1000);	// debug 2008.07.16
	StartCpuTimer0(); 
	InitPieVectTable();

	Relay_setup();
	pwm_buffer_setup();

// Initialize Communication Buffers
	for(i=0;i<Buf_MAX;i++)
	{
		Data_Registers[i]=0x0000;
		CAN_Registers[i]=0x0000;
		SCI_Registers[i]=0x0000;
	}
	
	// Initialize Fault Record Data buffer
/*
	for(i=0;i<Data_number;i++)
	{
		rec_data[i]=0;
	}
*/
	Init_I2C_eeprom();
	delay_ms(10);
	Data_Registers[3195] = 1;
// Initialize some parameters from eeprom
	Word_Read_data(81, &EEPROM_WRITE_CHECK);

	Word_Write_data(2306, 0);	// Run_Stop ??? ??  
	Word_Write_data(2300, 0);	// Local/Remote ??? ??: 1/0 
	Word_Write_data(2374, 0);	// Read_Fault_Record_En
	Word_Write_data(2384, 0);	// Read_Fault_Record_Num

	Word_Write_data(3184, 200);	// Read_Fault_Record_Num

// Eeprom Write & Read

	if(EEPROM_WRITE_CHECK!=EEPROM_WRITE_CHECK_VAL)
	{
		// Initialize Eeprom by default parameter value
		Parameter_Initialization();
		for(i=0; i<Buf_MAX;i++)
		{
			Write_Data_Registers_Offline(i);
			SCI_Registers[i] = Data_Registers[i];
			Word_Write_data(i, Data_Registers[i]);
			Word_Read_data (i, &Data_Registers[i]);
			SCI_Registers[i] = Data_Registers[i];
			Read_Data_Registers(i);
		}
		Data_Registers[1] = 1;
		Word_Write_data(81, EEPROM_WRITE_CHECK_VAL);
	}
	else
	{
		for(i=0; i<Buf_MAX;i++)
		{
			Word_Read_data(i, &Data_Registers[i]);
			SCI_Registers[i] = Data_Registers[i];
			Read_Data_Registers(i);
		}
	}
	
	Init_dsc();
	nRESET_DRIVER_SET;	//316J PWM on
	// Initialize SCI-A for data monitoring

	sci_debug_init();

	// Initialize SCI-C for Manager Communication
	scic_init();
	System_Config();
	Init_var();
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

	nBOOT_MODE_SET;
	nDC_CONTACT_SET;
	// Call Flash Initialization to setup flash waitstates
	// This function must reside in RAM

//	delay_long(1000000);
//	nDC_CONTACT_CLEAR;
//	delay(1000000); //delay_msecs(100);		// Delay for Setting Time

	Flag.DI.all = 0;
	T_dead_cnt_2 = Deadtime*SYS_CLK/2.0;
	Interrupt_time_max = 0.;

// WatchDog Initialize
/*	ServiceDog();
	EALLOW;
	SysCtrlRegs.WDCR = 0x0028;
	EDIS;
*/
//	Set_InitialTime();	// ?ð? ???? ?? ????
	FaultReg1[0] = FaultReg1[1] = Flag.Fault1.all = 0;
	FaultReg2[0] = FaultReg2[1] = Flag.Fault2.all = 0;
	FaultReg3[0] = FaultReg3[1] = Flag.Fault3.all = 0;
	Flag.Fault_Cntl.all = 0;

// Device & Software number
	Device_Info = 22;
	Software_Version = 'A';
	Software_Number = 300;

	Start_Flag = 1;
	Seq = SEQ_NoReady;
	Flag_First_Run = 1;

    while(1)
    {
		Is_mag = sqrt(Idss * Idss + Iqss * Iqss);
//		tester1 = P.G21.P15_Base_Speed_x1;
//		Word_Read_data(1565, &tester2);

		// Digital Input Monitoring
		digital_input_proc ();
		// Digital Output Handling
		digital_out_proc ();
		// Analog Output
		Analog_Output_Monitoring();
		EPwm5Regs.CMPA.half.CMPA = count;

//		SCIC_Process();
		SEQUENCE();

		Adc_Temperature = (int)(AdcRegs.ADCRESULT14>> 4) & 0x0FFF;
		Temperature = 1e-1 * Temperature_Calculation(Adc_Temperature);	// 온도 센싱

		if (Flag.Monitoring.bit.EEPROM_WRITE_ENABLE_Rx)
		{
			Word_Write_data(Rx_index, Data_Registers[Rx_index]);
			Flag.Monitoring.bit.EEPROM_WRITE_ENABLE_Rx= 0;
		}
		Write_Data_Registers_Online();
		if (Flag.Monitoring.bit.EEPROM_WRITE_ENABLE_Tx)
		{
	 		Word_Write_data(Tx_index, Data_Registers[Tx_index]);
			Flag.Monitoring.bit.EEPROM_WRITE_ENABLE_Tx= 0;
		}


		Read_Time(&time);	// 2011_03_27 dbsgln

		if(Data_Registers[3195])
		{
			for(i=0;i<Buf_MAX;i++)	SCI_Registers[i]=0x0000;
			Data_Registers[3195] = 0;
			SCI_Registers[3195] = 1;
		}

		Vmag_ref = Vdse_ref * Vdse_ref + Vqse_ref * Vqse_ref;
		Vmag_ffa = Vdse_ref_ffa * Vdse_ref_ffa + Vqse_ref_ffa * Vqse_ref_ffa;
		if(Vmag_ref > Vmag_ffa) flag_Vmag = 0;
		else					flag_Vmag = 1;

		Vmag_delta = Vmag_ref - Vmag_ffa;
		P_rate = 100 * P.G01.P00_Rated_power_x10_kW;


		if(flag_speed_cmd == 1)
		{
			if(Wrpm_ref_set >= 60000.)	Wrpm_ref_set = 60000.;
			if(Wrpm_ref_set <= 0. ) 	Wrpm_ref_set = 0.;
		}
   }
}

void SEQUENCE()
{
	FaultReg1[1] = Flag.Fault1.all;
	FaultReg2[1] = Flag.Fault2.all;

	if(( FaultReg1[1] )||( FaultReg2[1] ))
	{
		Fault_seq = Seq;
		Seq=SEQ_Fault;
	}

	switch(Seq)
	{
	// 시스템 전원 투입후 잠시 또는 Run신호 인가 전 시스템 비정상 일 경우
		case SEQ_NoReady:							//	" 0 "
			Drive_Off();
			//	전원 공급 시 시간 들레이 만으로 초충전 완료 신호 발생 
			Init_Charge_cnt_EN=1;
			if(Init_Charge_cnt >= (float)Init_Charge_Time * 1e-3 / Tsamp)		// 초기 충전 시간 3초
			{
				nDC_CONTACT_CLEAR;
				Flag.Fault_Cntl.bit.UV_Check_En = 1;
				if(Init_Charge_cnt >= ((float)Init_Charge_Time + 100.) * 1e-3 / Tsamp)	// 초기 충전을 포함 한 3.1초 후 시퀀스 전환 
				{
					// 차후 초기 충전 회로 고장을 대비하여 바로 다음 모드로 넘어 가지 않고
					// 충전 전압을 확인 후 넘어가도록 수정 할 것
					Seq = SEQ_Wait;
					Init_Charge_cnt_EN=0;
				}
			}
//			Seq = SEQ_Wait;
			Green_LED_off;
		break;


	// 시스템 Ready 상태이며 Run Signal 들어 오기 전
		case SEQ_Wait:						//	" 1 "
				if(( FaultReg1[1] )||( FaultReg2[1] )) Fault_seq = SEQ_Wait, Seq=SEQ_Fault;
				else
				{
					State_Index = STATE_READY;
					Run_Stop_Status = 0;
					Run_Stop_Operation();
					if(DRIVE_ENABLE==RUNN)	Seq=SEQ_Normal;
					else					Drive_Off();
				}
				Red_LED_off;
				State_Index = STATE_STOP;
		break;

	// Run 신호 인가 후 시스템 정상 동작
		case SEQ_Normal:							//	" 2 "
			if(( FaultReg1[1] )||( FaultReg2[1] )) Fault_seq = SEQ_Normal, Seq=SEQ_Fault;
			else 
			{
				State_Index = STATE_RUNNING;
				// Run_Time_Delay_time 후 Machine_state 1이 된다
/*				if (Machine_state == RUNN)
				{
//					Flag.Monitoring.bit.
				}
*/				
				Run_Stop_Operation();
				if(DRIVE_ENABLE==RUNN)	Drive_On();
				else
				{
					if((Position_Flag)||(Start_Flag))	Seq=SEQ_Wait;
					else 								Wrpm_ref_set = 0.0;
				}
			}
			Red_LED_off;
			Green_LED_on;
		break;


	// System Fault
		case SEQ_Fault:							//	" 3 "

			State_Index = STATE_FAULT;
			Run_Stop_Status = 0;
			Drive_Off(); // Converter Off
			// 정상 운전에서 펄트 모드로 전환 함을 표시
		
			if((!FaultReg1[1])&&(!FaultReg2[1])) Seq=SEQ_Retrial;

			if ((( FaultReg1[1] ) || ( FaultReg2[1] ))
				 && (( FaultReg1[1] > FaultReg1[0] ) || ( FaultReg2[1] > FaultReg2[0] )))
			{
				// 새로 기록 감시된 펄트에 대해서 관련 파라메타들을 초기화
				Fault_count++;
				Word_Write_data(2379, Fault_count);		// Fault 횟수를 EEPROM에 저장 해야 한다.
				Flag.Fault_Cntl.bit.Rec_Complete = 0;
				Flag.Fault_Cntl.bit.Rst_Complete = 0;
//				if (!FAULT_RECORD_COMPLETE)	Fault_Recording( Fault_count );
				Fault_Recording( Fault_count );
			}

			if ((!Flag.Fault_Cntl.bit.Rst_Complete)&&((Flag.DI.bit.FAULT_RESET == 1)||(Flag.Fault_Cntl.bit.Reset == 1)))
			{
				FaultReg1[0] = FaultReg1[1] = 0;
				FaultReg2[0] = FaultReg2[1] = 0;
	
				OL_TimeOver_Count = 0;
				MaxCon_Curr_Count = 0;
				OverVoltCount = 0;
				UnderVoltCount = 0;
				Flag.Fault_Cntl.bit.Reset = 0;
				Flag.Fault1.all=0;
				Flag.Fault2.all=0;
				Flag.Fault3.all=0;
				Seq = SEQ_Retrial;
				Flag.Fault_Cntl.bit.Rst_Complete = 1;
			}

			FaultReg1[0] = FaultReg1[1];
			FaultReg2[0] = FaultReg2[1];

			Red_LED_on;
			Green_LED_off;

		break;


	//  Retrial for Operation
		case SEQ_Retrial:							//	" 4 "
			{
				State_Index = STATE_STOP;
				Retry_cnt_En = 1;
				if( Retry_cnt >= Retry_Time_set)
				{
					Retry_cnt_En = 0;
					Seq = SEQ_Wait;
				}
			}
		break;
		default: Seq=SEQ_NoReady;
	}
	asm("   nop");					//END_SEQ:
}

void Drive_Off()
{
	run_Sensorless = 0;
	cnlt_flag.FOC = 0;

	nBOOT_MODE_SET;	
	nPWM_ENABLE_SET;
	Flag.Monitoring.bit.RUN_STOP_STATUS = 0;
	Init_var();
}

void Drive_On()
{
	nBOOT_MODE_CLEAR; 
	nPWM_ENABLE_CLEAR;

	Flag.Monitoring.bit.RUN_STOP_STATUS = 1;

	if(sensorless_mode == 4)	flag_speed_cmd = 1;
	else						flag_speed_cmd = 0;
	if(FaultInfo == 0)
	{
		cnlt_flag.control = 1;
		if((cnlt_flag.FOC == 1)&&(cnlt_flag.start ==1)) 
		{
			sensorless_mode = 0; 
			cnlt_flag.start = 0;
			run_Sensorless = 1;
		}
	}
}

void Run_Stop_Operation()
{
	if(OP.Run_stop.bit.Local)	// Local
	{
		Run_Stop_Src = 1;
		if	( OP.Run_stop.bit.Local_RUN )	DRIVE_ENABLE = RUNN;
		else								DRIVE_ENABLE = STOP;
	}
	else				// Remote
	{
		OP.Run_stop.bit.Local_RUN = 0;	// Local --> Remote 시 Key-Pad의 의한 Run 신호 가 clear 되어야 함
		switch(Run_Stop_meth)
		{
			case 0	:	// Terminal
				Run_Stop_Src = 0;
				if	(Flag.DI.bit.DRIVE_ENABLE == 1 )	DRIVE_ENABLE = RUNN;
				else									DRIVE_ENABLE = STOP;
//				if	(DI_Func.bit.DRIVE_ENABLE == 1 )	DRIVE_ENABLE = RUNN;
//				else if	( DI_Func.bit.DRIVE_ENABLE == 0 )	DRIVE_ENABLE = STOP;
				break;
			case 1  :	
			break;
			case 2	:		// Synchronous_Ctrl
			break;
			case 3	:		// Fieldbus ( Profibus, Modbus, CAN bus)
			break;
			case 4	:		// Free Function
			break;
		}
	}
} 

