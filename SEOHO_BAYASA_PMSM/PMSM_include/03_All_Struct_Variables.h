#ifndef		__STRUCT_VARIABLES_
#define		__STRUCT_VARIABLES_



//--------------------------------------//
/*
F1 Over_Load
F2 Device_short
F3 Over_Current
F4 Over_Voltage
F5 Under_Voltage
F6 Over_Speed
F7 Over_Temp
F8 DB_Fault
F9 Zero_Sequence_Current
F10 Ext_Fault_A
F11 Ext_Fault_B
F12 Hardware_Fault
F13 Over_Current_A
F14 Over_Current_B
F15 Over_Current_C
F16 Init_Charge
F17 Speed_Detection
*/
// Fault 는 상위 bit->하위 bit 순으로 배치함 
// 외부인력 요청 (박병훈)
// Fault 1 Group
struct FAULT1_BITS	// bits   description 
{          
	Uint16	F16_OC_A:				1;		// "F16 OC_A    "			; A상 과전류 : 소프트웨어 검출
	Uint16	F15_U_Cap_OT:			1;		// "F15 U-Cap OT    "		; 부하 과열 : 하드웨어 보호
	Uint16	F14_U_Cap_OV_H:			1;		// "F14 U-Cap OV    "		; 부하 과전압 : 하드웨어 보호
	Uint16	F13_U_Cap_V_Sens:		1;		// "F13 U-Cap V_Sens"		; 전압센서
	Uint16	F12_U_Cap_OV:			1;		// "F12 U-Cap OV    "		; 부하 과전압 : 소프트웨어 보호
	Uint16	F11_Out_of_Ctrl:		1;		// "F11 Out_of_Ctrl "		; 제어 불능
	Uint16	F10_Over_Speed:			1;		// "F10 Over_Speed  "		; 모터 과속 	
	Uint16	F9_Lowbus_Volt:			1;		// "F9 Lowus_Volt "		; 소프트웨어 저전압 (순시 비교)
	Uint16	F8_Over_Volt_H:			1;		// "F8 Over_Volt [H]"		; 하드웨어 과전압
	Uint16	F7_Overbus_Volt:		1;		// "F7 Over_Bus_Volt"		; 소프트웨어 과전압 (순시 비교)
	Uint16	F6_Under_Current:		1;		// "F6 Under_Current"		; 저전류 : 결선 이상
	Uint16	F5_Zero_Seq_I_H:		1;		// "F5 Zero_Seq_I[H]"		; 영상전류 -> 하드웨어 검출
	Uint16	F4_Zero_Seq_I:			1;		// "F4 Zero_Seq_I   "		; 영상전류 -> 소프트웨어 검출
	Uint16	F3_Over_Curr_H:			1;		// "F3 Over_Curr [H]"		; 하드웨어 과전류
	Uint16	F2_Over_Current:		1;		// "F2 Over_Current "		; 소프트웨어 과전류 (순시 비교)
	Uint16	F1_Over_Load:			1;		// "F1 Over_Load    "		; 소프트웨어 과부하 (시간 누적)
};

union FAULT1_REG 
{
   Uint16					all;
   struct	FAULT1_BITS		bit;
}; 

// Fault 2 Group
struct FAULT2_BITS	// bits   description 
{          
	Uint16	F32_Line_OPEN:			1;		// "F32 Line_OPEN   "		; 라인 결상
	Uint16	F31_Line_UV:			1;		// "F31 Line_UV     "		; 입력 라인 저전압
	Uint16	F30_SyncComm_Err:		1;		// "F30 SyncComm_Err"	; 동기 통신 오류
	Uint16	F29_Keypad_Error:		1;		// "F29 Keypad_Error"	; 키패드 통신 오류 
	Uint16	F28_Motor_Lock:			1;		// "F28 Motor Lock  "	; 모터 구속
	Uint16	F27_Open_Phase:			1;		// "F27 Open Phase  "	; 결상
	Uint16	F26_No_Current:			1;		// "F26 No_Current  "	; 무전류 : 무결선 또는 게이트 드라이브 전원 이상
	Uint16	F25_Ext_Fault:			1;		// "F25 Ext_Fault   "	; 외부 결함
	Uint16	F24_GateDrv_Pwr:		1;		// "F24 GateDrv_Pwr "	; 게이트 구동전원 이상
	Uint16	F23_Charging_Err:		1;		// "F23 Charging_Err"	; 초기 충전 실패
	Uint16	F22_Device_Short:		1;		// "F22 Device_Short"	; 게이트 드라이브 오류
	Uint16	F21_Over_Temp:			1;		// "F21 Over_Temp   "	; 방열판 과열
	Uint16	F20_UV_IN:				1;		// "F20 UV_IN    "		; 입력 저전압 : 소프트웨어 검출
	Uint16	F19_OV_IN:				1;		// "F19 OV_IN    "		; 입력 과전압 : 소프트웨어 검출
	Uint16	F18_OC_C:				1;		// "F18 OC_C "			; C상 과전류 : 소프트웨어 검출
	Uint16	F17_OC_B:				1;		// "F17 OC_B    "		; A상 과전류 : 소프트웨어 검출
};

union FAULT2_REG 
{
   Uint16					all;
   struct	FAULT2_BITS		bit;
};

// Fault 3 Group
struct FAULT3_BITS	// bits   description 
{          
	Uint16	F48_Inertia_Jm:			1;		// 회전관성 추정 오류
	Uint16	F47_Rotor_L_Lr:			1;		// 회전자 인덕턴스 오류
	Uint16	F46_Stator_L_Ls:		1;		// 고정자 인덕턴스 오류
	Uint16	F45_Rotor_R_Rr:			1;		// 회전자 저항 오류
	Uint16	F44_Stator_R_Rs:		1;		// 고정자 저항 오류
	Uint16	F43_High_Freq_L:		1;		// 고주파 등가 인덕턴스 오류
	Uint16	F42_High_Freq_R:		1;		// 고주파 등가 저항 오류
	Uint16	F41_Wrong_Conn:			1;		// 모터 결선 오류 
	Uint16	F40_Iinit_Charge:		1;		// 마스터 폴트
	Uint16	F39_Master_Fault:		1;		// "F39 Master_Fault"		; 마스터 폴트
	Uint16	F38_F_Logic2:			1;		// "F38 F_Logic 2   "		; 폴트로 정의되는 로직
	Uint16	F37_F_Logic1:			1;		// "F37 F_Logic 1   "		; 폴트로 정의되는 로직
	Uint16	F36_Profibus_Err:		1;		// "F36 Profibus_Err"		; 프로피버스 통신 오류
	Uint16	F35_Line_Unbal:			1;		// "F35 Line_Unbal  "		; 라인 불평형
	Uint16	F34_Line_Seq_Err:		1;		// "F34 Line_Seq_Err"		; 라인 시퀀스 오류
	Uint16	F33_Line_OV:			1;		// "F33 Line_OV     "		; 라인 과전압
};

union FAULT3_REG 
{
   Uint16					all;
   struct	FAULT3_BITS		bit;
};

// Fault_Neglect 1 Group

union FAULT_NEGLECT1_REG 
{
   Uint16							all;
   struct	FAULT1_BITS				bit;
};  

union FAULT_NEGLECT2_REG 
{
   Uint32							all;
   struct	FAULT2_BITS				bit;
};

union FAULT_NEGLECT3_REG 
{
   Uint32							all;
   struct	FAULT3_BITS				bit;
};

struct FAULT_CNTL_BITS
{
   Uint16	Rd_Rec_En:			1;		// bit . 1
   Uint16	Reset:				1;		// bit . 2
   Uint16	Rst_Complete:		1;		// bit . 3
   Uint16	Rec_Complete:		1;		// bit . 4
   Uint16	Rec_Rd_Complete:	1;		// bit . 5
   Uint16	UV_Check_En:		1;		// bit . 6
   Uint16	rsvd1:				10;		// bit . 7~16
};

union FAULT_CNTL_REG
{
	Uint16							all;
	struct 	FAULT_CNTL_BITS			bit;
};

struct DIGITAL_IN_BITS
{      
	Uint16 DRIVE_ENABLE:1;      // 
	Uint16 MULTI_STEP_BIT0:1;	// 
	Uint16 MULTI_STEP_BIT1:1;	// 
	Uint16 MULTI_STEP_BIT2:1;	// 
	Uint16 MULTI_STEP_BIT3:1;	// 
	Uint16 FAULT_RESET:1;		// 
	Uint16 JOG:1;				//
	Uint16 AI_REF_ACTIVE:1;		//
	Uint16 AI_LOCAL_REMOTE:1;	//
	Uint16 EXT_FAULT_A:1;		//
	Uint16 EXT_FAULT_B:1;		//
	Uint16 MOTOR_SELECT:1;		//
	Uint16 MB_BRAKE_STATE:1;	//
	Uint16 ACCEL_DECEL_SWITCH:1;	//
	Uint16 REF_TUNING_INC:1;	//
	Uint16 REF_TUNING_DEC:1;	//
	Uint16 ACC_DEC_BYP:1;		//
	Uint16 PID_CNTL_ENABLE:1;	//
	Uint16 AUTO_PID_MODE:1;		//
	Uint16 PID_GAIN_SEL:1;		//
	Uint16 PID_I_RESET:1;		//
	Uint16 TRQ_REF_OPT_BYP:1;	//
	Uint16 TRQ_SIGN:1;			//
	Uint16 TRQ_OUT_ZERO:1;		//
	Uint16 TIMER_RUN_ENABLE:1;	//
	Uint16 SLAVE_RUN_STATUS:1;	//
	Uint16 SYNC_CTRL_OPT_BPY:1;	//
	Uint16 FLYING_START:1;		//
	Uint16 DISABLE_PROFIBUS:1;	//
	Uint16 INVERT_DIR:1;		//
	Uint16 INVERT_RUN:1;		//
};

struct DIGITAL_OUT_BITS
{
	Uint16 DRIVE_READY:1;	// driver ready
	Uint16 FAULT_OUT_A:1;
	Uint16 FAULT_OUT_B:1;
	Uint16 MOTOR_BRAKE:1;
	Uint16 RUN_STOP:1;
	Uint16 WARNING:1;
	Uint16 DIRECTION:1;
	Uint16 JOG_INPUT:1;
	Uint16 VC_LIMIT:1;
	Uint16 FREE:1;
};

union DIGITAL_OUT_REG
{
	Uint16					all;
	struct DIGITAL_OUT_BITS bit;
};

union DIGITAL_IN_REG
{
   Uint32					all;
   struct DIGITAL_IN_BITS	bit;
} ; 

struct MONITORING_BITS
{
	Uint16 DRIVE_CAL:1;
	Uint16 FAULT_RESET_COMPLETE:1;
	Uint16 AUTO_TUNING_COMPLETE:1;
	Uint16 ACC:1;
	Uint16 DEC:1;
	Uint16 PWM_ON:1;
	Uint16 FW_MODE:2;
	Uint16 EEPROM_WRITE_ENABLE_Rx:1;
	Uint16 EEPROM_WRITE_ENABLE_Tx:1;
	Uint16 RUN_STOP_STATUS:1;
	Uint16 DIR_STATUS:1;
};

union MONITORING_REG
{
	Uint16			all;
	struct MONITORING_BITS	bit;
};



typedef volatile struct FLAG_DEFINE 
{
	union	FAULT1_REG				Fault1;
	union	FAULT2_REG				Fault2;
	union	FAULT3_REG				Fault3;
	union	FAULT_NEGLECT1_REG		Fault_Neglect1;
	union	FAULT_NEGLECT2_REG		Fault_Neglect2;
	union	FAULT_NEGLECT3_REG		Fault_Neglect3;
	union	FAULT_CNTL_REG			Fault_Cntl;
	union	DIGITAL_IN_REG			DI;
	union	DIGITAL_OUT_REG			DO;
	union	MONITORING_REG			Monitoring;
	//	union	STATE_REG	State;
}FLAG;

/*
typedef		char			U8;
typedef		signed char		S8;
typedef		unsigned short	U16;
typedef		short			S16;


typedef		unsigned long	U32;
typedef		long			S32;
typedef		double			U64;
typedef		double			S64;

typedef 	float			F32;
typedef 	double			D64;
*/

typedef struct
{
	int PWM;
	int OVM;
	int speed_control;
}MODE;

typedef struct
{
	int FOC;
	int PWM_CTR;
	int start;
	int control;
} CNTL_FLAG; 


typedef volatile struct
{
    char  ucCh;
    char  ucType;
	float fValue;
	float fScale;
	float *piChData;
} DAC;

typedef struct
{
	float offset[16];
	float tmp[16];
	int conv_val[16];
}	ADC;

struct PROGRAM_CONTROL
{
	int	Password;
	int IGBT_current;
	int Drive_power_x10_kW;					
};
struct BASIC_CONTROL_SETUP1
{
	int		P00_Rated_power_x10_kW;	
	int		P01_Rated_voltage;
	int		P02_Rated_current_x10;
	int		P03_Rated_frequency_x10;
	int		P04_Number_of_poles;
	unsigned int P05_Rated_speed;
	int		P06_Control_method;
	int		P07_PWM_frequency_x10_kHz;
	int		P08_Supply_voltage;	 
};

struct RAMP_CONTROL_CONFIGURATION1
{
	int		P00_Run_stop_source;
	int		P01_Reference_source;
	int		P03_Stop_mode;
	int		P04_Stop_hold_time;
	int		P05_Output_off_hold_time;
	int		P09_Accel_switching_ref1_x1000;
	int		P10_Accel_switching_ref2_x1000;
	int		P16_Accel_time1_x10;
	int		P17_Accel_time2_x10;
	int		P26_Decel_switching_ref1_x1000;
	int		P27_Decel_switching_ref2_x1000;
	int		P33_Decel_time1_x10;	 
	int		P34_Decel_time2_x10;
	int 	P42_Conunter_deceleration_ramp;
	int 	P43_Conunter_deceleration_time_x10;
	int		P49_Emergency_stop_mode;
	int		P50_Emergency_stop_decel_time_x10;
};

struct PROTECTION_SETUP
{
	int		P00_Current_limit_x10;
	int		P07_Max_continuous_current_x10;
	int		P08_Overload_current_x10;
	int		P09_Over_load_time_x100;
	int		P10_Over_load_fault_action;
	int		P11_Over_current_trip_x10;
	int		P12_Zero_sequence_current_trip_x10;
	int		P13_Over_voltage_limiting_function;
	int		P14_Over_voltage_limit;
	int		P15_Over_voltage_trip;
	int		P16_Under_voltage_compensation;
	int		P17_UV_compensation_voltage;
	int		P18_Under_voltage_trip; 
	int		P19_Open_phase_protection;
	int		P20_Supply_frequency;
	int		P21_Built_in_dynamic_brake;
	int		P22_DB_switching_frequency;
	int		P23_DB_start_voltage;
	int		P24_DB_full_voltage;
	int		P25_Over_temperature_trip_action;
	int		P30_Auto_restart_count;
	int		P31_Retry_delay_time_x10;
	int		P32_OC_auto_reset;
	int		P33_OV_auto_reset;
	int		P34_UV_auto_reset;
	int		P37_Out_of_control_auto_reset;
	int		P38_Out_of_control_time_x10;
	int		P39_Out_of_control_current_x10;
	int		P40_Over_temperature_trip_x10;
	int		P42_Line_under_voltage_trip_x10;
	int		P43_Line_under_voltage_auto_reset;
	int		P44_Line_unbalance_auto_reset;
};

struct AI_SETUP
{
	int		P01_Function;
	int		P02_Type;
	int		P03_Filter_time_constant_x10_mA;
	int		P04_Offset_x10;
	int		P05_Minimum_x10;
	int		P06_Maximum_x10;
	int		P07_Inversion;
	int		P08_Discretness;
	int		P09_Unit_selection;
};

union ANALOG_INPUT
{
	int		AI0_Analog_reference_source;
	struct	AI_SETUP					AI1;
	struct	AI_SETUP					AI2;
};

struct	FREE_FUNCTION
{
	int		P00_Control_mode;
};
						
struct	DIGITAL_INPUT						
{
	int		P00_Run_stop_control;
	int		P01_DI03_function;		int		P02_DI04_function;
	int		P03_DI05_function;		int		P04_DI06_function;
	int		P05_DI07_function;		int		P06_DI08_function;
	int		P07_DI09_function;		int		P08_DI10_function;
	int		P09_DI11_function;		int		P10_DI12_function;
	int		P11_Run_delay_time;
};

struct	MULTI_STEP_SET_POINT1				
{
	int		P00_JOG_set_x1000;
	int		P01_Step01_set_x10;		int		P02_Step02_set_x10;
	int		P03_Step03_set_x10;		int		P04_Step04_set_x10;
	int		P05_Step05_set_x10;		int		P06_Step06_set_x10;
	int		P07_Step07_set_x10;		int		P08_Step08_set_x10;
	int		P09_Step09_set_x10;		int		P10_Step10_set_x10;
	int		P11_Step11_set_x10;		int		P12_Step12_set_x10;
	int		P13_Step13_set_x10;		int		P14_Step14_set_x10;
	int		P15_Step15_set_x10;	
	int		P16_Unit_selection;
};

struct	AO_SETUP						
{
	int		P00_Output_selection;
	int		P01_Type;
	int		P02_Adjustment_0mA;
	int		P03_Adjustment_4mA;
	int		P04_Adjustment_20mA;
	int		P05_Max_output_x1000;
	int		P06_Inversion;	
};

struct	ANALOG_OUTPUT						
{
	struct	AO_SETUP					AO1;
	struct	AO_SETUP					AO2;
};

struct	DIGITAL_OUTPUT						
{
	int		P00_DO1_function;
	int		P00_DO2_function;
	int		P00_DO3_function;
};

struct	MOTOR_BRAKE_CONTROL					
{
	int		P00_M1_locked_state_up_speed_up_x1000;
};

struct	AUTO_TUNING
{
	int		P09_Dead_time_compansation_x100_us;
};

struct	VECTOR_CONTROL
{
	int		P00_Number_of_encoder_pulses;
	int		P01_Inversion_of_PG_direction;
	int		P03_Minimum_speed;
	int		P04_Maximum_speed_x1000;
	int		P05_Over_speed_limit_x1000;
};

struct	MOTOR_CONSTANT1						
{
	int		P00_Stator_resistance_x1000;
	int		P01_Rotor_resistance_x1000;
	int		P02_Stator_inductance_x1000;
	int		P03_Rotor_inductance_x1000;
	int		P04_Stator_transient_inductance_x1000;
	int		P05_Inertia_x1000;
	int		P09_Stator_Rx1000;
	int		P10_D_axis_Inductance_x_e6;
	int		P11_Q_axis_Inductance_x_e6;
	int		P12_Back_EMF_x_e5;
	int		P15_Base_Speed_x1;	
};


typedef volatile struct PARAMETER_DEFINE 
{
	struct	PROGRAM_CONTROL						G00;
	struct	BASIC_CONTROL_SETUP1				G01;
	struct	RAMP_CONTROL_CONFIGURATION1			G03;
	struct	PROTECTION_SETUP					G05;
	union	ANALOG_INPUT						G06;
	struct	FREE_FUNCTION						G07;
	struct	DIGITAL_INPUT						G08;
	struct	MULTI_STEP_SET_POINT1				G09;
	struct	ANALOG_OUTPUT						G11;
	struct	DIGITAL_OUTPUT						G12;
	struct	MOTOR_BRAKE_CONTROL					G13;
	struct	AUTO_TUNING							G14;
	struct	VECTOR_CONTROL						G19;
	struct	MOTOR_CONSTANT1						G21;
}PARAMETER;

struct RUN_STOP_CONTROL_BITS
{
	Uint16 Local_RUN:1;
	Uint16 Local_DIR:1;
	Uint16 Fieldbus_RUN:1;
	Uint16 Fieldbus_DIR:1;
	Uint16 Local:1;
	Uint16 Emergency_STOP:1;	// All mode
	Uint16 AUTO_TUNING:1;	// Only Local Mode
	Uint16 FAULT_RESET:1;
	Uint16 rsvd:8;
};

union RUN_STOP_CONTROL
{
	Uint16			all;
	struct RUN_STOP_CONTROL_BITS	bit;
};

struct REFERENCE
{
	int Speed;
	int Frequency_x10;
	int Torque_x1000;
	int PID_x1000;
};


union REFERENCE_CONTROL
{
	struct REFERENCE		Local;
	struct REFERENCE		Fieldbus;
};

typedef volatile struct OPERATION_DEFINE 
{
	union RUN_STOP_CONTROL		Run_stop;
	union REFERENCE_CONTROL		Reference;

}OPERATION;
//--------------------------------------//

// ePWM Configuration Variables
// File Name : "09_PWM.c"
typedef struct
{
	int op_state;
	int sampling_mode;
	int phase_duty_max_scaled;
	int phase_duty_half_scaled;
	int phase_duty_min_scaled;
	int phase_a_duty_scaled;
	int phase_b_duty_scaled;
	int phase_c_duty_scaled;
}	PWM;

// RTC Variables
// File Name : "10_I2C_RTC"
typedef struct
{
    Uint16 year;
    Uint16 month;
    Uint16 date;
	Uint16 week;
    Uint16 hour;
    Uint16 minute;
    Uint16 second;
} TIME;

// SCI Communication Variables
// File Name : 12_SCI_BC.c
typedef	union
{
	unsigned Word;
	struct
	{
		unsigned b0	:8;	// CRC16 하위 바이트
		unsigned b1	:8;	// CRC16 상위 바이트
	} Byte;
} CRC_flg ;


//--------------------------------------//
#endif

