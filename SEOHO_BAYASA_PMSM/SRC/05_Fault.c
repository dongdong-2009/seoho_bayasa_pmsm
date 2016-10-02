#include "00_main_def.h"

// 파라미터 오류 코드 	: 1 ~ 799

#define TRIP_MOTOR_RPM_UNDER				1002
#define TRIP_POWER_FACTOR_UNDER				1001
#define TRIP_POWER_FACTOR_OVER				1000

#define TRIP_OVER_LOAD_TIME					201	// 11 RMS 값으로 입력하는 것으로 수정 
#define TRIP_OVER_CURRENT					301	// 11 RMS 값으로 입력하는 것으로 수정 

#define CODE_OVER_VOLTAGE_LEVEL				302	// 12 DC link Voltage OV1은 자동으로 계산 할 것   
#define TRIP_UNDER_VOLT						303	// 13 수정 
#define CODE_OVER_SPEED_RATIO				304	// 14 수정 
#define CODE_ETHERMAL_LEVEL					305	// 15 New 수정 
#define CODE_PRE_CHARGE_TIME				306	// 초기충전 시간 
#define CODE_I_MAX_COEFF					307	// 
#define CODE_OVER_CURRENT_TIME				308	// 
#define CODE_SPEED_DETECTION_ERROR			309 // Encoder Pulse Error

#define ERR_IGBT_UH							811
#define ERR_IGBT_UL							812
#define ERR_IGBT_VH							813
#define ERR_IGBT_VL							814
#define ERR_IGBT_WH							815
#define ERR_IGBT_WL							816
#define ERR_IGBT_DB							817
#define ERR_IGBT_RESET						818

#define FAULT_DRIVER						819
#define FAULT_DB							820
#define FAULT_ZC							821

#define	ERR_OC0								821
#define	ERR_OC								822
#define	ERR_UC0								823
#define	ERR_OV0								824
#define	ERR_OV								825
#define	ERR_UV0								826
#define	ERR_UV								827
#define	ERR_OS0								828
#define	ERR_OS								829

#define	ERR_SUV								831	
#define	ERR_SOV								832
#define	ERR_SOC								833
#define ERR_OVER_CURRENT_U_PHASE			834 
#define ERR_OVER_CURRENT_V_PHASE			835 
#define ERR_OVER_CURRENT_W_PHASE			836

#define	ERR_NAME_PLATE_PAR_MISMATCH			841
#define	ERR_NAME_PLATE_PAR_MISMATCH0		842
#define	ERR_NAME_PLATE_PAR_MISMATCH1		843

#define	ERR_PM_Is							844
#define	ERR_GM_Is							845
#define	ERR_K_Damp_Is						846


#define	ERR_Tr_Over							851
#define	ERR_Tr_Under						852
#define	ERR_sigma_Under						853
#define	ERR_sigma_Over						854
#define	ERR_Ki_Is_Under						856
#define	ERR_Kp_Is_Over						857
#define	ERR_Kp_Is_Under						858

#define	ERR_EXCITATION_TIME					859

// SCI 
#define ERR_SCI_CMD_ADDR_GROUP_UNDER		861
#define ERR_SCI_CMD_ADDR_GROUP_OVER			862
#define ERR_SCI_CMD_ADDR_1ST_UNDER			863
#define ERR_SCI_CMD_ADDR_1ST_OVER			864
#define ERR_SCI_CMD_ADDR_2ND_UNDER			865
#define ERR_SCI_CMD_ADDR_2ND_OVER			867

// 오토튜닝 오류
#define	ERR_Req_Under						920
#define	ERR_Req_Over						921
#define	ERR_Leq_Under0						923
#define	ERR_Leq_Over0						924
#define	ERR_Rs_Under						925
#define	ERR_Rs_Over							926
#define	ERR_Ls_Over0						927
#define	ERR_Ls_Under0						928	
#define	ERR_Ls_Under1						929
#define	ERR_Ls_Over1						930
#define	ERR_Leq_Over1						931
#define	ERR_Rr_Over							932
#define	ERR_Rr_Under						933
#define	ERR_Jm_Under						934
#define	ERR_Jm_Over							945
#define	ERR_WRONG_INTERRUPT_CMD				950

// 하드웨어 프로텍션 (TMS320F240)
#define	ERR_PRECHARGING_FAIL				960		// 초기 충전 실패 
#define	ERR_PWM								961
#define	ERR_HOC								962
#define	ERR_HOV								963
#define	ERR_HUV								964
#define	ERR_OVER_HEAT						965

#define	ERR_EXT_TRIP						966

#define	ERR_PWM_CNT							971		// 스위칭 주기의 카운터 값 오류 

#define	ERR_INV_DISABLE						972		// 운전중 인버터 디스에이블 

// Fault Record Definition
#define	Data_number							20
#define Rec_Length							50
#define Fault_Record_Start_Addr				3500

//2011_04_02 dbsgln
// _CONTROL_FAULT_MSG_TABLE:
#define No_Any_Fault		0	// fault 없을때 
#define F1_Over_Load_d    	1	// 소프트웨어 과부하 (시간 누적)
#define F2_Over_Current_d 	2	// 소프트웨어 과전류 (순시 비교)
#define F3_Over_Curr_H		3	// 하드웨어 과전류
#define F4_Zero_Seq_I   	4	// 영상전류 -> 소프트웨어 검출
#define F5_Zero_Seq_I_H		5	// 영상전류 -> 하드웨어 검출
#define F6_Under_Current_d	6	// 저전류 : 결선 이상
#define F7_Over_Bus_Volt_d	7	// 소프트웨어 과전압 (순시 비교)
#define F8_Over_Volt_H		8	// 하드웨어 과전압
#define F9_Low_Bus_Volt_d 	9	// 소프트웨어 저전압 (순시 비교)
#define F10_Over_Speed_d  	10	// 모터 과속 	
#define F11_Out_of_Ctrl_d 	11	// 제어 불능

#define F12_U_Cap_OV    	12	// 부하 과전압 : 소프트웨어 보호
#define F13_U_Cap_V_Sens	13	// 전압센서
#define F14_U_Cap_OV_H    	14	// 부하 과전압 : 하드웨어 보호
#define F15_U_Cap_OT    	15	// 부하 과열 : 하드웨어 보호

#define F16_OC_A_d    		16	// A상 과전류 : 소프트웨어 검출 
#define F17_OC_B_d			17	// B상 과전류 : 소프트웨어 검출
#define F18_OC_C_d			18	// C상 과전류 : 소프트웨어 검출
#define F19_Over_Volt_IN	19	// 입력 과전압 : 소프트웨어 검출
#define F20_Low_Volt_IN		20	// 입력 저전압 : 소프트웨어 검출

// _DRIVE_OPERATION_FAULT_MSG_TABLE:
#define F21_Over_Temp_d   	21	// 방열판 과열
#define F22_Device_Short_d	22	// 게이트 드라이브 오류
#define F23_Charging_Err_d	23	// 초기 충전 실패
#define F24_GateDrv_Pwr 	24	// 게이트 구동전원 이상
#define F25_Ext_Fault_d   	25	// 외부 결함
#define F26_No_Current  	26	// 무전류 : 무결선 또는 게이트 드라이브 전원 이상
#define F27_Open_Phase  	27	// 결상
#define F28_Motor_Lock  	28	// 모터 구속
#define F29_Keypad_Error	29	// 키패드 통신 오류 
#define F30_SyncComm_Err	30	// 동기 통신 오류
#define F31_Line_UV     	31	// 입력 라인 저전압
#define F32_Line_OPEN   	32	// 라인 결상
#define F33_Line_OV     	33	// 라인 과전압
#define F34_Line_Seq_Err	34	// 라인 시퀀스 오류
#define F35_Line_Unbal  	35	// 라인 불평형
#define F36_Profibus_Err	36	// 프로피버스 통신 오류
#define F37_F_Logic_1   	37	// 폴트로 정의되는 로직
#define F38_F_Logic_2   	30	// 폴트로 정의되는 로직
#define F39_Master_Fault	39	// 마스터 폴트
#define F40_Iinit_Charge	40	// 마스터 폴트


// _AUTO_TUNING_FAULT_MSG_TABLE:
#define F41_Wrong_Conn  	41	// 모터 결선 오류
#define F42_High_Freq_R 	42	// 고주파 등가 저항 오류
#define F43_High_Freq_L 	43	// 고주파 등가 인덕턴스 오류
#define F44_Stator_R_Rs		44	// 고정자 저항 오류
#define F45_Rotor_R_Rr		45	// 회전자 저항 오류
#define F46-Stator_L_Ls		46	// 고정자 인덕턴스 오류
#define F47_Rotor_L_Lr		47	// 회전자 인덕턴스 오류
#define F48_Inertia_Jm		48	// 회전관성 추정 오류
#define F49_Motor_Stall 	49	// 모터가 구속 상태 -> 브레이크 개방
#define F50_Tn_Time_Over	50	// 튜닝 시간 초과 -> 파라미터가 수렴 안됨


int n = 0; // recording number
int b = 0; // reading number
int TripCode;
int first = 0;
extern float tester1, tester2;	
void fault_chk( )
{
	TripCode = 0;

	if ( ( TripCode = CheckOverCurrent()	) != 0) TripProc();
	if ( ( TripCode = CheckFaultDriver()	) != 0) TripProc();
	if ( ( TripCode = CheckOverVolt() 		) != 0) TripProc();
	if ( ( TripCode = CheckOverHeat() 		) != 0) TripProc();
	if ( ( TripCode = CheckSpeedDetection()	) != 0) TripProc();
	if ( ( TripCode = ExternalFault() 		) != 0) TripProc();
	if(Flag.Fault_Cntl.bit.UV_Check_En)
	{
		if ( ( TripCode = CheckUnderVolt() ) !=0) TripProc();
	}

//		if ( ( TripCode = CheckFaultIGBT()		) != 0) return TripCode	;
//		if ( ( TripCode = CheckFaultZC()		) != 0) return TripCode ;
}

int CheckOverCurrent( )
{
// Current Limit
// Irate는 입력 전격 전류이며 (Prate/Vrate) Vrate rated input voltage이다 
// 각 상의 입력 전격 전류는 Irate/N_SW 이다.
	if(first > 20)
	{
		first = 21;
//		tester1 = (float) P.G01.P02_Rated_current_x10;
//		tester2 = 1e-3*(float)(P.G05.P11_Over_current_trip_x10);
	// 각 상의 OC_Trip 확인
		
		if ((Ias > OC_TRIP_LEVEL )||(Ias < (-1.)* OC_TRIP_LEVEL))
		{
			Flag.Fault1.bit.F16_OC_A = 1;
			return F16_OC_A_d;
		}
		if ((Ibs > OC_TRIP_LEVEL )||(Ibs < (-1.)* OC_TRIP_LEVEL))
		{
			Flag.Fault2.bit.F17_OC_B = 1;
			return F17_OC_B_d;
		}
		if ((Ics > OC_TRIP_LEVEL )||(Ics < (-1.)* OC_TRIP_LEVEL))
		{
			Flag.Fault2.bit.F18_OC_C = 1;
			return F18_OC_C_d;
		}

		if (Is_max > OC_TRIP_LEVEL )
		{
			Flag.Fault1.bit.F2_Over_Current = 1;			// OC_MAG
			return F2_Over_Current_d;
		}

	// Over_Load 확인
		if (Is_mag > OL_TRIP_LEVEL )  //135%
		{
			if (OL_TimeOver_Count > OL_TRIP_TIME )	// Over Load time	// 135%이상 부하가 10초 연속을 경우 발생
			{
				OL_TimeOver_Count = OL_TRIP_TIME+1.;
				Flag.Fault1.bit.F1_Over_Load = 1;
				return F1_Over_Load_d;
			} 
			else OL_TimeOver_Count += Tsamp;
		}
		else
		{
			if(OL_TimeOver_Count>0) OL_TimeOver_Count-=Tsamp;
			else 					OL_TimeOver_Count = 0;
		}

	// 연속 부하 확인
		if (Is_mag > MAX_CON_CUR_LEVEL )		// 95% Load
		{
			if (MaxCon_Curr_Count > MAX_CON_CUR_TIME )	// 60 seconds
			{
				MaxCon_Curr_Count = MAX_CON_CUR_TIME + 1.;
				Flag.Fault1.bit.F1_Over_Load = 1;
				return F1_Over_Load_d;
			} 
			else MaxCon_Curr_Count+=Tsamp; //95%			(float)(Max_Con_cur/100)
		}
		else 
		{
			if(MaxCon_Curr_Count>0) MaxCon_Curr_Count-=Tsamp;
		}
	}
	else first ++; 
	return No_Any_Fault;
}



int CheckOverVolt()
{
	static int OverVoltCount = 0;

	if(Vdc_measured > OV_TRIP_LEVEL ) OverVoltCount++;//||(GpioDataRegs.GPADAT.bit.GPIO15==0)) OverVoltCount++;
	else if( OverVoltCount > 0) OverVoltCount --;
	if (OverVoltCount > 5)
	{
		OverVoltCount = 6;
		if( !Flag.Fault_Neglect1.bit.F7_Overbus_Volt )
		{
			Flag.Fault1.bit.F7_Overbus_Volt;
			FaultInfo = CODE_OVER_VOLTAGE_LEVEL;
			return CODE_OVER_VOLTAGE_LEVEL;
		}
	}
	return 0;
}

int CheckUnderVolt( )
{
	if (Vdc_measured < UV_TRIP_LEVEL ) UnderVoltCount++;
	else if( UnderVoltCount > 0)	UnderVoltCount--;

	if (UnderVoltCount > 20 )
	{
		UnderVoltCount = 21;
		if( !Flag.Fault_Neglect1.bit.F9_Lowbus_Volt )
		{
			FaultInfo = TRIP_UNDER_VOLT;
			Flag.Fault1.bit.F9_Lowbus_Volt = 1;
			return TRIP_UNDER_VOLT;
		}
	}
	return 0;
}

int CheckOverHeat( )
{
	static int OverHeatCount = 0;

	if (Temperature > OT_TRIP_LEVEL ) OverHeatCount++;
	else if( OverHeatCount > 0)	OverHeatCount--;

	if( OverHeatCount > 10 )
	{
		OverHeatCount = 11;
		if( !Flag.Fault_Neglect2.bit.F21_Over_Temp )
		{
			FaultInfo = ERR_OVER_HEAT;
			Flag.Fault2.bit.F21_Over_Temp = 1;
			return ERR_OVER_HEAT;
		}
	}
	return 0 ;
}

int CheckFaultDriver( )
{
	if (GpioDataRegs.GPADAT.bit.GPIO12==0)
	{
		FaultInfo = FAULT_DRIVER;
		Flag.Fault2.bit.F22_Device_Short = 1;
		return FAULT_DRIVER;
	}
	return 0;
}

int ExternalFault( )
{
	if (( Flag.DI.bit.EXT_FAULT_A)||(Flag.DI.bit.EXT_FAULT_B))
	{
		if( ! Flag.Fault_Neglect2.bit.F25_Ext_Fault )
		{
			Flag.Fault2.bit.F25_Ext_Fault=1;
			return F25_Ext_Fault_d;
		}
	}
	else Flag.Fault2.bit.F25_Ext_Fault=0;
	return No_Any_Fault;
}

int CheckSpeedDetection( )
{
	static int OverSpeedCount=0;//, Speed_detec_fault_count=0;

	if ( Wrpm_hat > OS_TRIP_LEVEL ) OverSpeedCount++;
	else if( OverSpeedCount > 0) OverSpeedCount --;

	if (OverSpeedCount > 5 )
	{
		OverSpeedCount = 6;
		if( !Flag.Fault_Neglect1.bit.F10_Over_Speed )
		{
			FaultInfo = CODE_OVER_SPEED_RATIO;
			Flag.Fault1.bit.F10_Over_Speed = 1;
			return CODE_OVER_SPEED_RATIO;
		}
	}
	return 0;
/*
//	if (Speed_detect_fault)	Speed_detec_fault_count++;
//	else if (Speed_detec_fault_count>0)	Speed_detec_fault_count--;

	if (Speed_detec_fault_count > 5)
	{
		Speed_detec_fault_count = 6;
		FaultInfo = CODE_SPEED_DETECTION_ERROR;
//		trip_recording( CODE_SPEED_DETECTION_ERROR,Wrpm_MT,"Trip Speed Detect");

		return CODE_SPEED_DETECTION_ERROR;
	}
*/
}

int CheckFaultZC( )
{
	static int FaultZCCount = 0;

	if (GpioDataRegs.GPADAT.bit.GPIO16==0 )	FaultZCCount++;
	else if( FaultZCCount > 0) 	FaultZCCount--;

	if (FaultZCCount > 3 )
	{
		FaultZCCount = 4;
		FaultInfo = FAULT_ZC;
		return FAULT_ZC;
	}
	return 0;
}

void TripProc( )
{

 //	gMachineState = STATE_TRIP;
	if(FaultInfo != 0) {
	cnlt_flag.FOC = 0;
	nFLT2_ON;
	cnlt_flag.control = 0;
	}
}

void Fault_Recording( int Fault_cnt )
{
	int Record_Addr;
	Read_Time(&time);	// 2011_04_03 dbsgln
	if(Fault_count<=Rec_Length)
	{
		Record_Addr = Fault_cnt;
		Total_Fault_Record = Fault_cnt;
	}
	else 
	{
		Record_Addr = Fault_count - 50 * (int)(Fault_count/50);
		Total_Fault_Record = 50;
	}
	// 기존에 저장된 값보다 추가 변경 되어야 만 새로 저장
	if(FaultReg1[1] > FaultReg1[0])
	{
		if		((FaultReg1[1] & 0x8000) - (FaultReg1[0] & 0x8000)) rec_data[3] = F1_Over_Load_d;		// bit 16
		else if ((FaultReg1[1] & 0x4000) - (FaultReg1[0] & 0x4000)) rec_data[3] = F2_Over_Current_d;	// bit 15
		else if ((FaultReg1[1] & 0x0200) - (FaultReg1[0] & 0x0200)) rec_data[3] = F7_Over_Bus_Volt_d;	// bit 10

		else if ((FaultReg1[1] & 0x0080) - (FaultReg1[0] & 0x0080)) rec_data[3] = F9_Low_Bus_Volt_d;	// bit 8
		else if ((FaultReg1[1] & 0x0001) - (FaultReg1[0] & 0x0001)) rec_data[3] = F16_OC_A_d;			// bit 1
	}

	else if(FaultReg2[1] > FaultReg2[0])
	{
		if		((FaultReg2[1] & 0x8000) - (FaultReg2[0] & 0x8000)) rec_data[3] = F17_OC_B_d;			// bit 16
		else if ((FaultReg2[1] & 0x4000) - (FaultReg2[0] & 0x4000)) rec_data[3] = F18_OC_C_d;			// bit 15
		else if ((FaultReg2[1] & 0x2000) - (FaultReg2[0] & 0x4000)) rec_data[3] = F19_Over_Volt_IN;		// bit 14
		else if ((FaultReg2[1] & 0x1000) - (FaultReg2[0] & 0x4000)) rec_data[3] = F20_Low_Volt_IN;		// bit 13

		else if ((FaultReg2[1] & 0x0800) - (FaultReg2[0] & 0x0800)) rec_data[3] = F21_Over_Temp_d;		// bit 12
		else if ((FaultReg2[1] & 0x0400) - (FaultReg2[0] & 0x0400)) rec_data[3] = F22_Device_Short_d;		// bit 11
		else if ((FaultReg2[1] & 0x0080) - (FaultReg2[0] & 0x0080)) rec_data[3] = F25_Ext_Fault_d;		// bit 8
		else												rec_data[3] = No_Any_Fault;
	}
	rec_data[0] = ( time.year 	<< 8 )|time.month;		// Year/Month
	rec_data[1] = ( time.date	<< 8 )|time.hour;		// Day/Hour
	rec_data[2] = ( time.minute	<< 8 )|time.second;		// Minute/Second
	rec_data[4] = Is_mag * 10.;							// Input current	x 10
	rec_data[5] = Vdc_measured;							// Input Voltage	x 1
	rec_data[6] = 0.;								// Output Voltage 	x 1
	rec_data[7] = Temperature * 10.;					//		x 10
	rec_data[8] = 0.;								// Input Power		x 10
	rec_data[9] = Fault_seq;							// Fault 발생 시 시스템 상태 (Fault 상태 아니면 동작 상태 등 ...)

	Word_Write_data(2381, Total_Fault_Record);

	for(n=-1;n<Data_number;n++)
	{
		Word_Write_data((Fault_Record_Start_Addr+(Record_Addr-1)*Data_number + n), rec_data[n]);
	}
	Flag.Fault_Cntl.bit.Rec_Complete = 1;
}
