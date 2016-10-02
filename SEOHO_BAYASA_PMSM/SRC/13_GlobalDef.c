#ifndef __Global_Variable_Definitions_
#define __Global_Variable_Definitions_
#include "00_main_def.h"

Uint16 P_rate;

// Main Sequence Variables
// File Name : "00_Main.c"
	int Seq = 0;
	int Fault_seq = 0;
	int Init_Charge_cnt_EN = 0;
	int Retry_cnt_En = 0;
	long Retry_cnt = 0;
	int Retry_Time_set = 0;
	int DRIVE_ENABLE = 0;
	long Init_Charge_cnt = 0.;
	unsigned int Init_Charge_Time = 3000;	// 3초
	unsigned int Drive_Status = 0;
	unsigned int State_Index = 0;

// Analog Input Parameter
// File Name : 01_analog_inout.c
	int Count20mA = 0;
	int Count4mA = 0;
	int Count0mA = 0;
	int		Anaolg_Output_mA_x1 = 0;
	unsigned int count=0;
	unsigned int count_20mA=0;
	unsigned int count_4mA =0;
	unsigned int count_0mA =0;
//	ao1 AO1;
//	ao2 AO2;

// Digital In/Out Parameter
// File Name : 02_digital_inout.c
	int input_function_option[12];
	int digital_out_funtion[8];
	Uint16 DI_STATUS;
	Uint16 DO_STATUS;
	#pragma DATA_SECTION(ZONE0_BUF,"ZONE0DATA");
	volatile unsigned int ZONE0_BUF[2048];

// DAC Variables
// File Name : "04_DAC.c"
	DAC	deg_Dac [4];
	int AC_DC_TYPE[4];
	unsigned short deg_sDacTmp;

// Fault Variables
// File Name : "05_Fault.c"
	Uint16 FaultReg1[2];
	Uint16 FaultReg2[2];
	Uint16 FaultReg3[2];
	int Fault_count = 0;
	int Total_Fault_Record = 0;
	int rec_data[Data_number]; 
	int FAULT_RECORD_COMPLETE;
	float OC_TRIP_LEVEL = 0.;
	float OL_TRIP_LEVEL = 0.;
	float OL_TRIP_TIME = 0.;
	float MAX_CON_CUR_LEVEL = 0.;
	float MAX_CON_CUR_TIME = 0.;
	float OV_TRIP_LEVEL = 0.;
	float UV_TRIP_LEVEL = 0.;
	float OT_TRIP_LEVEL = 0.;
	float OS_TRIP_LEVEL = 0.;

	int OL_TimeOver_Count = 0;
	int MaxCon_Curr_Count = 0;
	int OverVoltCount = 0;
	int UnderVoltCount = 0;
	int OverVoltinCount = 0;
	int UnderVoltinCount = 0;
	int DC_Build_Up_delay_cnt = 0;
	int UV_Check_Enable = 0;
	int Speed_detect_fault;
	int FaultInfo;

// Speed Control Variables 
// File Name : 06_speed_controller.c
	int   op_cnt;
	float Kp_sc;
	float Ki_scT;
	float Wrpm_ref;
	float ref_slew_up;
	float ref_slew_dn;
	float Iqse_limit;
	float Wrm_err;
	float Wrm_err_integ;
	float Wrpm_ref_set;
	float Wrpm_ref_cmd;
	float Wrpm_base_speed;
	float Wrm_ref;
	float Wr_ref;
	float thetar_openloop;
	float thetar_hat1;
	float Wrm_hat;
	float Wr_hat;
	float Wrpm_hat;
	float Wrpm_min;
	float cos_theta;
	float sin_theta;
	float cos_theta1;
	float sin_theta1;
	float thetar_hat_cm;
	float thetar_hat_cm1;
	float cos_theta_cm;
	float sin_theta_cm;
	float Wr_hat_cm;

// Current Control Variables
// File Name : 07_current_controller.c
	float Fsw;
	float PWM_TMR_PERIOD;
	float Tsamp;
	float Wc_cc;
	float fc_cc;
	float Kp_cc;
	float Ki_ccT;
	float Ka_cc;
	float alpha_cc;
	float Vdse_ref_ff;
	float Vqse_ref_ff;
	float Vdse_anti;
	float Vqse_anti;
	float Vdse_ref_integ;
	float Vqse_ref_integ;
	float Vdse_ref_fb;
	float Vqse_ref_fb;
	float Vdse_ref_ffa;
	float Vqse_ref_ffa;
	float Vmag_ref;
	float Vmag_ffa;
	float Vmag_delta;
	int	  flag_Vmag;
	float Idss;
	float Iqss;
	float Is_mag;
	float Idse;
	float Iqse;
	float Idse_ref;
	float Iqse_ref;
	float Err_Idse;
	float Err_Iqse;
	float INV_Vdc_2;	// 2xVdc
	float Ka;
	float K_Id;
	float K_Is;
	float K_Is_2;
	float K_estimation;
	float Is_max;
	float Idse_ref_max;
	float Idse_ref_min;
	float Idse_ref_sensorless;
	float Id_cm;
	float Iq_cm;
	float Idse1;
	float Iqse1;
	float Vdse_ref1;
	float Vqse_ref1;
	float La;
	float Lb;
	float Wr_Fc;

// WatchDog Variables  
	int watchdog_cnt;
	float dog_time;

// Sensed Variables
	float Vdc_measured;
	float Vdc_eff;
	float AICMD;
	float AINI1;
	float AINV1;
	float AINI2;
	float AINV2;
	float Ias;
	float Ibs;
	float Ics;
	float Izs;
	float I_dead;
	float Temperature;
	int	  Adc_Temperature;

// Template Variables
	float tmp_float;
	float tmp_float1;
	long int tmp_cnt;
	long int tmp_cnt1;

// Motor Parameters
	float Rs;
	float Ls;
	float I_integ;
	float LAMpm;
	float Current_Slop_Build_Up;
	float Current_Slop_ACC;
// SVPWM Variables
	float Vdss_ref;
	float Vqss_ref;
	float Vdse_ref;
	float Vqse_ref;
	float Vas_ref;
	float Vbs_ref;
	float Vcs_ref;
	float Van_ref;
	float Vbn_ref;
	float Vcn_ref;
	float integ;
	float Vsn;
	float Vsn_max;
	float Vsn_min;
	float Vmax;
	float Vmin;
	float Ta_cnt;
	float Tb_cnt;
	float Tc_cnt;

	unsigned int Run_Stop_Status;

	int cc_cnt;
	int sc_cnt;
	float T_dead_cnt_2;	// 0.5 * Deadtime


// ADC Scale
	float adc_scale_0;
	float adc_scale_1;
	float adc_scale_3;
	float Ias_offset;
	float Ibs_offset;


// Sensorless Sequence Variables
	int   run_Sensorless;
	int   sensorless_mode;
	int   flag_speed_cmd;
	int   Flag_STOP;
	int   Flag_First_Run;
	int   Position_Flag;
	int   Start_Flag;

	float V_rate;
	float Drive_Voltage;
	float Interrupt_time_max;

// Sysetm Variables
// File Name : 08_system.c 
	unsigned int Device_Info = 0;
	unsigned int Software_Version = 0;
	unsigned int Software_Number = 0;
	unsigned int Run_Stop_meth = 0;
	unsigned int Run_Stop_Src = 0;
	unsigned int Ref_method = 0;
	float 		 I_scale;

//	ePWM Configuration Variables
// File Name : "09_PWM.c"
	int  carrier_mode = 0;
	PWM	 pwm_g1;

// SCI Communication Variables
// File Name : 12_SCI_BC.c
	unsigned int SciC_RxStep=0;
	unsigned int SciC_RxFlag=0;
	unsigned int SciC_TxFlag=0;
	unsigned int Communication_Fault_Cnt = 3;
	unsigned int Communication_Fault_Flag = 0;
	unsigned int Device_type=0;
	unsigned int RxType=0;
	unsigned int RxAddr=0;
	unsigned int RxData=0;
	unsigned int RxCRC=0;
	unsigned char RxBuf[9];
	//변수 통신 관련
	Uint16 Rx_index= 0;
	Uint16 Tx_index= 0;
	Uint16 Tx_count_25ms= 0;
	Uint16 Tx_count_10ms= 0;
	Uint16 Tx_count_1s= 0;

	// Variables for Serial Communication
	char scib_tx_buf[SCIB_BUF_SIZE+1];
	char scib_tx_pos=0, scib_tx_end=0;
	char scib_rx_buf[SCIB_BUF_SIZE+1];
	char scib_rxd=' ';
	char scic_tx_buf[SCIC_BUF_SIZE+1];
	char scic_tx_pos=0, scic_tx_end=0;
	char scic_rx_buf[SCIC_BUF_SIZE+1];
	char scic_rxd=' '; 
	//-- Serial Data Stack  
	WORD Data_Registers[Buf_MAX];
	WORD CAN_Registers[Buf_MAX];
	WORD SCI_Registers[Buf_MAX];



	MODE	mode;
	FLAG	Flag;
	CNTL_FLAG cnlt_flag;
	PARAMETER P;
	OPERATION OP;
	ADC		adc;
	CRC_flg	CRC;
	TIME	time;
#endif // __Global_Variable_Definitions_
