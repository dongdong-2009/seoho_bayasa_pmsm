#ifndef __EXTERN_VARIABLE_DECLARATION_
#define __EXTERN_VARIABLE_DECLARATION_ 

#include "DSK2833x_Define.h"

#define Buf_MAX 3400
#define	SCIB_BUF_SIZE	50
#define	SCIC_BUF_SIZE	50
#define Data_number		20
#include "03_All_Struct_Variables.h"

extern Uint16 P_rate;

// Main Sequence Variables
// File Name : "00_Main.c"
	extern int Seq;
	extern int Fault_seq;
	extern int Init_Charge_cnt_EN;
	extern long Init_Charge_cnt;
	extern int Retry_cnt_En;
	extern long Retry_cnt;
	extern int Retry_Time_set;
	extern int DRIVE_ENABLE;
	extern unsigned int Init_Charge_Time;
	extern unsigned int Drive_Status;
	extern unsigned int State_Index;

// Analog Input Parameter
// File Name : 01_analog_in_out.c
	extern int Count20mA;
	extern int Count4mA;
	extern int Count0mA;
	extern int		Anaolg_Output_mA_x1;
	extern unsigned int count;
	extern unsigned int count_20mA;
	extern unsigned int count_4mA;
	extern unsigned int count_0mA;
//	extern ao1 AO1;
//	extern ao2 AO2;

// Digital In/Out Parameter
// File Name : 02_digital_inout.c
	extern int input_function_option[12];
	extern int digital_out_funtion[8];
	extern Uint16 DI_STATUS, DO_STATUS;
	extern volatile unsigned int ZONE0_BUF[2048];

// DAC Variables
// File Name : "04_DAC.c"
	extern DAC	deg_Dac [4];
	extern int AC_DC_TYPE[4];
	extern unsigned short deg_sDacTmp;

// Fault Variables
// File Name : "05_Fault.c"
	extern Uint16 FaultReg1[2];
	extern Uint16 FaultReg2[2];
	extern Uint16 FaultReg3[2];
	extern int Fault_count;
	extern int Total_Fault_Record;
	extern int rec_data[Data_number];
	extern int FAULT_RECORD_COMPLETE;
	extern float OC_TRIP_LEVEL;
	extern float OL_TRIP_LEVEL;
	extern float OL_TRIP_TIME;
	extern float MAX_CON_CUR_LEVEL;
	extern float MAX_CON_CUR_TIME;
	extern float OV_TRIP_LEVEL;
	extern float UV_TRIP_LEVEL;
	extern float OT_TRIP_LEVEL;
	extern float OS_TRIP_LEVEL;
	extern int OL_TimeOver_Count;
	extern int MaxCon_Curr_Count;
	extern int OverVoltCount;
	extern int UnderVoltCount;
	extern int OverVoltinCount;
	extern int UnderVoltinCount;
	extern int DC_Build_Up_delay_cnt;
	extern int UV_Check_Enable;
	extern int Speed_detect_fault;
	extern int FaultInfo;

// Speed Control Variables  
// File Name : 06_speed_controller.c 
	extern int   op_cnt;
	extern float Kp_sc;
	extern float Ki_scT;
	extern float Wrpm_ref;
	extern float ref_slew_up;
	extern float ref_slew_dn;
	extern float Iqse_limit;
	extern float Wrm_err;
	extern float Wrm_err_integ;
	extern float Wrpm_ref_set;
	extern float Wrpm_ref_cmd;
	extern float Wrpm_base_speed;
	extern float Wrm_ref;
	extern float Wr_ref; 
	extern float thetar_openloop;
	extern float thetar_hat1;
	extern float Wrm_hat;
	extern float Wr_hat;
	extern float Wrpm_hat;
	extern float Wrpm_min;
	extern float cos_theta;
	extern float sin_theta;
	extern float cos_theta1;
	extern float sin_theta1;
	extern float thetar_hat_cm;
	extern float thetar_hat_cm1;
	extern float cos_theta_cm;
	extern float sin_theta_cm;
	extern float Wr_hat_cm;

// Current Control Variables
// File Name : 07_current_controller.c
	extern float Fsw;
	extern float PWM_TMR_PERIOD;
	extern float Tsamp;
	extern float Wc_cc;
	extern float fc_cc;
	extern float Kp_cc;
	extern float Ki_ccT;
	extern float Ka_cc;
	extern float alpha_cc;
	extern float Vdse_ref_ff;
	extern float Vqse_ref_ff;
	extern float Vdse_anti;
	extern float Vqse_anti;
	extern float Vdse_ref_integ;
	extern float Vqse_ref_integ;
	extern float Vdse_ref_fb;
	extern float Vqse_ref_fb;
	extern float Vdse_ref_ffa;
	extern float Vqse_ref_ffa;
	extern float Vmag_ref;
	extern float Vmag_ffa;
	extern float Vmag_delta;
	extern int	 flag_Vmag;
	extern float Idss;
	extern float Iqss;
	extern float Is_mag;
	extern float Idse;
	extern float Iqse;
	extern float Idse_ref;
	extern float Iqse_ref;
	extern float Err_Idse;
	extern float Err_Iqse;
	extern float INV_Vdc_2;	// 2xVdc
	extern float Ka;
	extern float K_Id;
	extern float K_Is;
	extern float K_Is_2;
	extern float K_estimation;
	extern float Is_max;
	extern float Idse_ref_max;
	extern float Idse_ref_min;
	extern float Idse_ref_sensorless;
	extern float Id_cm;
	extern float Iq_cm;
	extern float Idse1;
	extern float Iqse1;
	extern float Vdse_ref1;
	extern float Vqse_ref1;
	extern float La;
	extern float Lb;
	extern float Wr_Fc;

// WatchDog Variables  
	extern int watchdog_cnt;
	extern float dog_time;

// Sensed Variables
	extern float Vdc_measured;
	extern float Vdc_eff;
	extern float AICMD;
	extern float AINI1;
	extern float AINV1;
	extern float AINI2;
	extern float AINV2;
	extern float Ias;
	extern float Ibs;
	extern float Ics;
	extern float Izs;
	extern float I_dead;
	extern float Temperature;
	extern int	  Adc_Temperature;

// Template Variables
	extern float tmp_float;
	extern float tmp_float1;
	extern long int tmp_cnt;
	extern long int tmp_cnt1;

// Motor Parameters
	extern float Rs;
	extern float Ls;
	extern float I_integ;
	extern float LAMpm;
	extern float Current_Slop_Build_Up;
	extern float Current_Slop_ACC;
// SVPWM Variables
	extern float Vdss_ref;
	extern float Vqss_ref;
	extern float Vdse_ref;
	extern float Vqse_ref;
	extern float Vas_ref;
	extern float Vbs_ref;
	extern float Vcs_ref;
	extern float Van_ref;
	extern float Vbn_ref;
	extern float Vcn_ref;
	extern float integ;
	extern float Vsn;
	extern float Vsn_max;
	extern float Vsn_min;
	extern float Vmax;
	extern float Vmin;
	extern float Ta_cnt;
	extern float Tb_cnt;
	extern float Tc_cnt;

	extern unsigned int Run_Stop_Status;

	extern int cc_cnt;
	extern int sc_cnt;
	extern float T_dead_cnt_2;	// 0.5 * Deadtime


// ADC Scale
	extern float adc_scale_0;
	extern float adc_scale_1;
	extern float adc_scale_3;
	extern float Ias_offset;
	extern float Ibs_offset;

// Sensorless Sequence Variables
	extern int   run_Sensorless;
	extern int   sensorless_mode;
	extern int   flag_speed_cmd;
	extern int   Flag_STOP;
	extern int   Flag_First_Run;
	extern int   Position_Flag;
	extern int   Start_Flag;

	extern float V_rate;
	extern float Drive_Voltage;
	extern float Interrupt_time_max;

// Sysetm Variables
// File Name : 08_system.c 
	extern ADC adc;
	extern unsigned int Device_Info;
	extern unsigned int Software_Version;
	extern unsigned int Software_Number;
	extern unsigned int Run_Stop_meth;
	extern unsigned int Run_Stop_Src;
	extern unsigned int Ref_method;
	extern unsigned int Drive_Status;
	extern unsigned int State_Index;
	extern float 		I_scale;
//	ePWM Configuration Variables
// File Name : "09_PWM.c"
	extern int carrier_mode;
	extern PWM pwm_g1;

// SCI Communication Variables
// File Name : 12_SCI_BC.c
/*
	extern unsigned int SciC_RxStep;
	extern unsigned int SciC_RxFlag;
	extern unsigned int SciC_TxFlag;
	extern unsigned int Communication_Fault_Cnt;
	extern unsigned int Communication_Fault_Flag;
	extern unsigned int Device_type;
	extern unsigned int RxType;
	extern unsigned int RxAddr;
	extern unsigned int RxData;
	extern unsigned int RxCRC;
	extern unsigned char RxBuf[9];
	//변수 통신 관련
	extern Uint16 Rx_index;
	extern Uint16 Tx_index;
	extern Uint16 Tx_count_25ms;
	extern Uint16 Tx_count_10ms;
	extern Uint16 Tx_count_1s;
	extern WORD Data_Registers[Buf_MAX];
	extern WORD CAN_Registers[Buf_MAX];
	extern WORD SCI_Registers[Buf_MAX];
	extern char scib_tx_buf[SCIB_BUF_SIZE+1];
	extern char scib_tx_pos, scib_tx_end;
	extern char scib_rx_buf[SCIB_BUF_SIZE+1];
	extern char scib_rxd;
	extern char scic_tx_buf[SCIC_BUF_SIZE+1];
	extern char scic_tx_pos, scic_tx_end;
	extern char scic_rx_buf[SCIC_BUF_SIZE+1];
	extern char scic_rxd;

*/

	extern MODE		mode;
	extern FLAG 	Flag;
	extern CNTL_FLAG cnlt_flag;
	extern PARAMETER P;
	extern OPERATION OP;
//	extern CRC_flg	CRC;
	extern TIME 	time;
	extern void Write_Time(TIME time);
	extern void Read_Time(TIME *time);

#endif //__EXTERN_VARIABLE_DECLARATION_ 
