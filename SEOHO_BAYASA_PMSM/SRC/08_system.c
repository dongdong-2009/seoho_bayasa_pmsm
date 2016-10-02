#include "00_main_def.h"

void Relay_setup()
{
    EALLOW;
	GpioCtrlRegs.GPBMUX1.bit.GPIO38 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO38 = 1;
	EDIS;
}

void Init_GPIO(void)
{
	EALLOW;
	GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;
	GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;
	GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;
	EDIS;
}

void delay(int delay_cnt)
{
	while(delay_cnt--);
}

void delay_long(long int delay_cnt)
{
	while(delay_cnt--);
}


void Init_AD_converter()
{
	AdcRegs.ADCTRL3.bit.ADCBGRFDN = 0x3;
	delay(10000);
	AdcRegs.ADCTRL3.bit.ADCPWDN = 1;
	delay(10000);
	AdcRegs.ADCTRL1.bit.ACQ_PS = 8;
	AdcRegs.ADCTRL1.bit.CPS = 1;
	AdcRegs.ADCTRL3.bit.ADCCLKPS = 0;
	AdcRegs.ADCMAXCONV.all = 0x000F;
	AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;
	AdcRegs.ADCTRL1.bit.CONT_RUN = 0;
	AdcRegs.ADCCHSELSEQ1.all = 0x3210;
	AdcRegs.ADCCHSELSEQ2.all = 0x7654;
	AdcRegs.ADCCHSELSEQ3.all = 0xBA98;
	AdcRegs.ADCCHSELSEQ4.all = 0xFEDC;
}

int system_first_rst = 0;

void Init_var()
{
	int i = 0;
	float a = 0.0;	
 
	if(!system_first_rst)
	{
		for(i=0;i<16;i++)
		{
//			adc.offset[i] = 0.0;
			adc.conv_val[i] = 0;
			adc.tmp[i] = 0.0;
		}
	system_first_rst = 1;
	}

	// Fault 관련 변수 초기화 
	Retry_Time_set = 5; // [초]
	Retry_cnt_En = 0;
	Retry_cnt = 0;

	// current controller variables 
	cnlt_flag.FOC = 0;
	cnlt_flag.PWM_CTR = 0;
	cnlt_flag.control = 0;
	cnlt_flag.start = 1;

// Initialize Protection Variables
	FaultInfo = 0;


// Initinilize control Variables
// Speed Controller
	op_cnt = 0;
	Kp_sc = 0.84; //0.7;
	Ki_scT = 6e-4;//5e-005;
	Wrpm_ref = 0.0;
	ref_slew_up = 1000. * Tsamp;	// 속도 증가 기울기 = 1000 * Tsamp * Fsw = 1000rpm/sec
	ref_slew_dn = 1000. * Tsamp;
	Iqse_limit = 0.0;
	Wrm_err = 0.0;
	Wrm_err_integ = 0.0;
	Wrpm_ref_set = 0.0;
	Wrpm_ref_cmd = 0.0;
	Wrpm_base_speed = 24000.;
	Wrm_ref = 0.;
	Wr_ref = 0.;
	thetar_openloop = 0.;
	thetar_hat1 = 0.0;
	Wrm_hat = 0.;
	Wr_hat = 0.;
	Wrpm_hat = 0.;
	Wrpm_min = 300.0;
	cos_theta = 1.0;
	sin_theta = 0.0;
	cos_theta1 = 1.0;
	sin_theta1 = 0.0;
	thetar_hat_cm = 0.0; 
	thetar_hat_cm1 = 0.0;
	cos_theta_cm = 1.0;
	sin_theta_cm = 0.0;
	Wr_hat_cm = 0.0;
// Current Controller
	fc_cc = 500.;
	Wc_cc = 6.283184 * fc_cc;		// CC BW : 2*PI*fc = 200[Hz]
	Kp_cc = Wc_cc * Ls;
	Ki_ccT = Wc_cc * Rs * Tsamp;
	Ka_cc = 0.0;
	alpha_cc = 1.0;
	Vdse_ref_ff = 0.0;
	Vqse_ref_ff = 0.0;
	Vdse_anti = 0.0;
	Vqse_anti = 0.0;
	Vdse_ref_integ = 0.0;
	Vqse_ref_integ = 0.0;
	Vdse_ref_fb = 0.0;
	Vqse_ref_fb = 0.0;
	Vdse_ref_ffa = 0.0;
	Vqse_ref_ffa = 0.0;
	Vmag_ref = 0.0;
	Vmag_ffa = 0.0;
	Vmag_delta = 0.0;
	flag_Vmag = 0;
	Idse_ref = 0.0;
	Iqse_ref = 0.0;
	Is_max = 67.0;
	Idse_ref_max = 67.0;
	Idse_ref_min = 7.0;
	Idse_ref_sensorless = 8.;
	Id_cm = 0.0;
	Iq_cm = 0.0;
	Idse1 = 0.0;
	Iqse1 = 0.0;
	Vdss_ref = 0.0;
	Vqss_ref = 0.0;
	Vdse_ref = 0.0;
	Vqse_ref = 0.0;
	Vdse_ref1 = 0.0;
	Vqse_ref1 = 0.0;

	Wr_Fc = 100.0;
	a = 2.0*PI*Wr_Fc;
    La = (2.0-a*Tsamp)/(2.0+a*Tsamp);
    Lb = (a*Tsamp)/(2.0+a*Tsamp);


// WatchDog Variables  
	dog_time = 0.0042;

// Template Variables
	tmp_float = 0.;
	tmp_float1 = 0.;
	tmp_cnt = 0;
	tmp_cnt1 = 0;

// Parameters of Antle Motor with Inductor
	Rs = 0.073;
	Ls = 0.000617;	 
	LAMpm = 0.03353;
	I_integ = Tsamp/Ls;
	Ka = 0.0004;
	K_Id = 1.5;
	K_Is = 1.0;
	K_Is_2 = K_Is * K_Is;
	K_estimation = Ka*Fsw*Ls/LAMpm;
// Current Build Up은 재 기동시 DC 전류 인가 하는 기울기다
//	Current_Build_Up_Time 에 Idse_ref_max으로 도달 
// Current_Build_Up_Time = 1.0;
//	Current_Slop_Build_Up = Idse_ref_max * Tsamp/Current_Build_Up_Time;
	Current_Slop_Build_Up = Idse_ref_max * Tsamp/1.0;
//	I/F 가속 기울 기 
//	Current_Accel_Time 으로 설정
//	Current_Accel_Time 에 Idse_ref_max으로 도달 
//	Current_Slop_ACC = Idse_ref_max * Tsamp/Current_Slop_Build_Up;
	Current_Slop_ACC = Idse_ref_max * Tsamp/1.0;


	Run_Stop_Status = 0;
//	Is_max = 63.0;

// 모터 파라메타 초기화 	
//	Rs = 0.0645;		// 0.024;
//	Ls = 0.000785;		// 0.000135;  0.000602
//	LAMpm = 0.03353;  	// 0.0139  0.01358 0.03394/0.03353 0.0425
// Parameters of Antle Motor 
//	Rs = 0.044;
//	Ls = 0.000135;		// 0.000135;  0.000602
//	LAMpm = 0.03353;  	// 0.0139  0.01358 0.03394/0.03353 0.0425

// Sensorless Sequence Variables Initialize
	sensorless_mode = 99;		
	flag_speed_cmd = 0;
	Flag_STOP = 0;
	Position_Flag = 0;
	Start_Flag = 0;

	mode.OVM = 0;
	mode.PWM = 0;
	mode.speed_control = 0;


// fault process variables 초기화
	// Drive_Voltage의 값 설정
	// Key_pad 설정 값에 의해서 Drive_Voltage의 값을 설정 하도록 했다. 
	// 2011_06_29, dbsgln
	OL_TRIP_LEVEL = 1e-1 * (float)( P.G01.P02_Rated_current_x10 ) * 1e-3 * (float)( P.G05.P08_Overload_current_x10 );
	OL_TRIP_TIME =  1e-2 * (float)( P.G05.P09_Over_load_time_x100);
	OC_TRIP_LEVEL = 1e-1 * (float)( P.G01.P02_Rated_current_x10 ) * 1e-3 * (float)( P.G05.P11_Over_current_trip_x10 );
	MAX_CON_CUR_LEVEL = 1e-1 * (float)( P.G01.P02_Rated_current_x10 ) * 1e-3 * (float)( P.G05.P07_Max_continuous_current_x10 );
	MAX_CON_CUR_TIME = 1e-2 * (float)( P.G05.P09_Over_load_time_x100 );

	OV_TRIP_LEVEL = (float)P.G05.P15_Over_voltage_trip;
	UV_TRIP_LEVEL = (float)P.G05.P18_Under_voltage_trip;
	OT_TRIP_LEVEL = 1e-1 * (float)P.G05.P40_Over_temperature_trip_x10;
	OS_TRIP_LEVEL = 1.05 * (float)(P.G01.P05_Rated_speed);
	if 		( V_rate <= 250.) 					   Drive_Voltage=200;
	else if (( V_rate >250.) && ( V_rate <= 460 )) Drive_Voltage=400; 
	else if (( V_rate >460.) && ( V_rate <= 600 )) Drive_Voltage=500;  

	if(Drive_Voltage==200)			// 200V급 : ~250V
	{
//		fault.OV_set = 410.0;
//		fault.UV_set = 180.0;
	}
	else if (Drive_Voltage==400)	// 400V급 : 330 ~ 460V
	{
//		fault.OV_set = 815.0;
//		fault.UV_set = 350.0;
	}
	else if (Drive_Voltage==500)	// 500V급 : 460 ~ 600V
	{
//		fault.OV_set = 900.0;
//		fault.UV_set = 450.0;
	}	// fault process variables

// ADC Scale initilize
//	adc.scale[0] = 0.105;	// HTB100-P
//	adc.scale[1] = 0.105;
//	adc.scale[0] = 0.07583;	// HTB100-P
//	adc.scale[1] = 0.07583;

//	adc.scale[0] = 0.1961368;	// HTB100-P 2011.5.18
//	adc.scale[1] = 0.1961368;	// HTB100-P 2011.5.18
//	adc.scale[1] = 0.03913;
//	adc.scale[0] = 0.03902;			 // for LAH50-P 500A CT
//	adc.scale[1] = 0.03902;
	
//	adc.scale[0] = 0.0775376;			 // for HAS200-S (ct card bypass)
//	adc.scale[1] = 0.0775376; // 0.07326  0.0775376 0.07401316

	adc_scale_0 = 0.0775376;
	adc_scale_1 = 0.0775376;
	//	adc.scale[0] = 0.05;			 // for LTS25-NP
	//	adc.scale[1] = 0.05; 
//	adc.scale[3] = 0.225672529;
	adc_scale_3 = 0.225672529;

//	adc.scale[4] = 1.0;
//	adc.scale[5] = 29.5;
//	adc.scale[6] = 1.0;
//	adc.scale[7] = 29.5;
	nFLT2_OFF;

	sc_cnt = 0;
	cc_cnt = 0;
}

void Init_dsc()
{
	Init_AD_converter();
	Init_var();
	calculateOffset();
	// Switching frequency limit
//	Fsw = 20000.;
	Fsw = 1.0e+2 * (float)P.G01.P07_PWM_frequency_x10_kHz;
//	if		(Fsw >= 20000.) Fsw = 20000.;
//	else if (Fsw <= 10000.) Fsw = 10000.;
	Tsamp = 1.0/Fsw;
	PWM_TMR_PERIOD = Tsamp * (SYS_CLK/2.); 
	pwm_g1_setup(SYS_CLK, Fsw, Deadtime);
}

void calculateOffset(void)
{
	Uint16 i;
	Uint32 	Ias_sum, Ibs_sum;

	Ias_sum = 0, Ibs_sum = 0;

	for(i=0; i<4096; i++){

	AdcRegs.ADCTRL2.bit.SOC_SEQ1 = 1;	// Start of conversion trigger for SEQ1
	AdcRegs.ADCTRL2.bit.SOC_SEQ2 = 1;	// Start of conversion trigger for SEQ2
	DELAY_US(10);

	// Ias Current Sensing
	Ias_sum += ((long)AdcRegs.ADCRESULT0);

	// Ibs Current Sensing
	Ibs_sum += ((long)AdcRegs.ADCRESULT1);
	
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;	// 14    Restart sequencer 1
	AdcRegs.ADCTRL2.bit.RST_SEQ2 = 1;	// 14    Restart sequencer 2
	DELAY_US(1);
	}
	Ias_offset = (float)(((unsigned int)(Ias_sum >> 12) >> 4)+10.);// adjust offset
	Ibs_offset = (float)(((unsigned int)(Ibs_sum >> 12) >> 4)+10.);// adjust offset


	AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1;
	EPwm1Regs.ETSEL.bit.SOCAEN = 1;
	EPwm1Regs.ETSEL.bit.SOCASEL = 1;
	EPwm1Regs.ETPS.bit.SOCAPRD = 1;
}

void System_Config()
{
	// Fault Neglect Setting
	Flag.Fault1.all = 0x0000;
	Flag.Fault2.all = 0x0000;
	Flag.Fault3.all = 0x0000;
	Flag.Fault_Neglect2.all = 0x0000;
	Flag.Fault_Neglect1.all = 0x0000;
	Flag.Fault_Neglect2.all = 0x0000;

//	point.Fault_Neglect2.bit.UV_IN = 1;	// 입력 전압 UV 무시
//	point.Fault_Neglect1.bit.Low_Bus_Volt = 1;	// 출력 전압 UV 무시
//	point.Fault_Neglect1.bit.Over_Bus_Volt= 1;	// 출력 전압 OV 무시
//	point.Fault_Neglect2.bit.Over_Temp = 1;		// OT ソ�
	Flag.DI.all = 0;
}
void Parameter_Initialization()
{
	int Drive_power= 0;

	if (GpioDataRegs.GPCDAT.bit.GPIO68) Drive_power+= 1;
	if (GpioDataRegs.GPCDAT.bit.GPIO67) Drive_power+= 2;
	if (GpioDataRegs.GPCDAT.bit.GPIO66) Drive_power+= 4;
	if (GpioDataRegs.GPCDAT.bit.GPIO65) Drive_power+= 8;
	if (GpioDataRegs.GPCDAT.bit.GPIO64) Drive_power+= 16;	
	
	switch ( Drive_power )
	{
		case 0:
			P.G00.Drive_power_x10_kW = 55;
			I_scale= 100.0/1.8;
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 55;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 1:
			P.G00.Drive_power_x10_kW = 75;
			I_scale= 100.0/1.43;
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 75;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break; 	
		case 2:
		 	P.G00.Drive_power_x10_kW = 110; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 110;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 3: 
			P.G00.Drive_power_x10_kW = 150; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 150;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 4: 
			P.G00.Drive_power_x10_kW = 185; 
			I_scale= 200.0/0.85;
			P.G00.IGBT_current= 63;
			P.G01.P00_Rated_power_x10_kW= 185;
			P.G01.P01_Rated_voltage= 415;
			P.G01.P02_Rated_current_x10= 350;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1465;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 380;
			break;
		case 5: // (110104 by HHH) 현재 실험중
			P.G00.Drive_power_x10_kW = 220;
			I_scale= 100.0/0.85;
			P.G00.IGBT_current= 63;
			P.G01.P00_Rated_power_x10_kW= 110;
			P.G01.P01_Rated_voltage= 380;
			P.G01.P02_Rated_current_x10= 232;
			P.G01.P03_Rated_frequency_x10= 60;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1770;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 380;
			break;
		case 6: 
			P.G00.Drive_power_x10_kW = 300;
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 300;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 7: 
			P.G00.Drive_power_x10_kW = 370; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 370;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 720;
			P.G01.P03_Rated_frequency_x10= 10000;
			P.G01.P04_Number_of_poles= 2;
			P.G01.P05_Rated_speed= 60000;
			P.G01.P06_Control_method= 2;
			P.G01.P07_PWM_frequency_x10_kHz= 200;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 8: 
			P.G00.Drive_power_x10_kW = 450;
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 450;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 9: 
			P.G00.Drive_power_x10_kW = 550;
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 550;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 10: 
			P.G00.Drive_power_x10_kW = 750;
			P.G00.IGBT_current= 35; 	
			P.G01.P00_Rated_power_x10_kW= 750;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 11: 
			P.G00.Drive_power_x10_kW = 900; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 900;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 12: 
			P.G00.Drive_power_x10_kW = 1100;
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 1100;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 13: 
			P.G00.Drive_power_x10_kW = 1320; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 1320;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 14: 
			P.G00.Drive_power_x10_kW = 1600; 
			P.G00.IGBT_current= 35;	
			P.G01.P00_Rated_power_x10_kW= 1600;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 15: 
			P.G00.Drive_power_x10_kW = 2000;
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 2000;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 16: 
			P.G00.Drive_power_x10_kW = 2500;
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 2500;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 17: 
			P.G00.Drive_power_x10_kW = 3150;
			P.G00.IGBT_current= 35; 
			P.G01.P00_Rated_power_x10_kW= 3150;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 18: 
			P.G00.Drive_power_x10_kW = 4000;
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 4000;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 19:
		 	P.G00.Drive_power_x10_kW = 5000; 
		 	P.G00.IGBT_current= 35;	
			P.G01.P00_Rated_power_x10_kW= 5000;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 20: 
			P.G00.Drive_power_x10_kW = 5600; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 5600;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 21: 
			P.G00.Drive_power_x10_kW = 6300; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 6300;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 22: 
			P.G00.Drive_power_x10_kW = 7150; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 7150;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 23: 
			P.G00.Drive_power_x10_kW = 7800; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 7800;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 30: 
			P.G00.Drive_power_x10_kW = 22; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 22;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
		case 31: 
			P.G00.Drive_power_x10_kW = 37; 
			P.G00.IGBT_current= 35;
			P.G01.P00_Rated_power_x10_kW= 37;
			P.G01.P01_Rated_voltage= 230;
			P.G01.P02_Rated_current_x10= 55;
			P.G01.P03_Rated_frequency_x10= 50;
			P.G01.P04_Number_of_poles= 4;
			P.G01.P05_Rated_speed= 1435;
			P.G01.P06_Control_method= 3;
			P.G01.P07_PWM_frequency_x10_kHz= 50;
			P.G01.P08_Supply_voltage= 230;
			break;
	} 

	// Protection Parameter 초기화
	P.G05.P00_Current_limit_x10 = 1450;
	P.G05.P07_Max_continuous_current_x10 = 950;
	P.G05.P08_Overload_current_x10 = 1350;
	P.G05.P09_Over_load_time_x100 = 6000;
	P.G05.P10_Over_load_fault_action = 0;
	P.G05.P11_Over_current_trip_x10 = 2200;
	P.G05.P12_Zero_sequence_current_trip_x10=150;
	P.G05.P13_Over_voltage_limiting_function=0;
	P.G05.P14_Over_voltage_limit=670;
	P.G05.P15_Over_voltage_trip=780;
	P.G05.P16_Under_voltage_compensation=0;
	P.G05.P17_UV_compensation_voltage=450;
	P.G05.P18_Under_voltage_trip=360;
	P.G05.P19_Open_phase_protection=0;
	P.G05.P20_Supply_frequency=60;
	P.G05.P21_Built_in_dynamic_brake = 1;
	P.G05.P22_DB_switching_frequency=1000;
	P.G05.P23_DB_start_voltage=690;
	P.G05.P24_DB_full_voltage=710;
	P.G05.P25_Over_temperature_trip_action=2;
	P.G05.P30_Auto_restart_count=0;
	P.G05.P31_Retry_delay_time_x10=15;
	P.G05.P32_OC_auto_reset=0;
	P.G05.P33_OV_auto_reset=0;
	P.G05.P34_UV_auto_reset=0;
	P.G05.P37_Out_of_control_auto_reset=0;
	P.G05.P38_Out_of_control_time_x10=50;
	P.G05.P39_Out_of_control_current_x10=900;
	P.G05.P40_Over_temperature_trip_x10=750;

	// Analog Output 관련 파라메타 초기화
	P.G11.AO1.P00_Output_selection = 2;
	P.G11.AO1.P01_Type = 0;
	P.G11.AO1.P02_Adjustment_0mA = 450;
	P.G11.AO1.P03_Adjustment_4mA = 2143;
	P.G11.AO1.P04_Adjustment_20mA = 880;
	P.G11.AO1.P05_Max_output_x1000 = 100;
	P.G11.AO1.P06_Inversion = 0;

	P.G11.AO2.P00_Output_selection=0;			
	P.G11.AO2.P01_Type=1;
	P.G11.AO2.P02_Adjustment_0mA=0;
	P.G11.AO2.P03_Adjustment_4mA=0;
	P.G11.AO2.P04_Adjustment_20mA=0;
	P.G11.AO2.P05_Max_output_x1000=100;
	P.G11.AO2.P06_Inversion=0;

	// Motor Parameter Intitialization
	P.G21.P00_Stator_resistance_x1000 = 0;
	P.G21.P01_Rotor_resistance_x1000 = 0;
	P.G21.P02_Stator_inductance_x1000 = 0;
	P.G21.P03_Rotor_inductance_x1000 = 0;
	P.G21.P04_Stator_transient_inductance_x1000 = 0;
	P.G21.P05_Inertia_x1000 = 0;
	P.G21.P09_Stator_Rx1000 = 73;
	P.G21.P10_D_axis_Inductance_x_e6 = 617;
	P.G21.P11_Q_axis_Inductance_x_e6 = 617;
	P.G21.P12_Back_EMF_x_e5 = 3353;
	P.G21.P15_Base_Speed_x1 = 10000;
}

