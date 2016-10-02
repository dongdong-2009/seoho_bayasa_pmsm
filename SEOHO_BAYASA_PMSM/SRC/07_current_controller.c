#include "00_main_def.h"

void current_controller()
{
	float a, b;

//	cc_smp = !cc_smp;		// cc flag
//  WatchDog Reset
//	if(watchdog_cnt >=dog_time/Tsamp) ServiceDog(), watchdog_cnt = 0;

	adc.conv_val[0] = (int)(AdcRegs.ADCRESULT0 >> 4) & 0x0FFF;	// Ias
	adc.conv_val[1] = (int)(AdcRegs.ADCRESULT1 >> 4) & 0x0FFF;	// Ibs
	adc.conv_val[3] = (int)(AdcRegs.ADCRESULT3 >> 4) & 0x0FFF;	// Vdc

//	adc.conv_val[4] = (int)(AdcRegs.ADCRESULT4 >> 4); // Analog Input I1
//	adc.conv_val[5] = (int)(AdcRegs.ADCRESULT5 >> 4); // Analog Input V1
//	adc.conv_val[6] = (int)(AdcRegs.ADCRESULT6 >> 4); // Analog Input I2
//	adc.conv_val[7] = (int)(AdcRegs.ADCRESULT7 >> 4); // Analog Input V2

	adc.tmp[0] = (float)adc.conv_val[0] - Ias_offset;
	adc.tmp[1] = (float)adc.conv_val[1] - Ibs_offset;

	adc.tmp[3] = (float)adc.conv_val[3];						// Vdc
//	adc.tmp[4] = (float)adc.conv_val[4];						// AIN.I1
//	adc.tmp[5] = (float)adc.conv_val[5] - 2262.;				// AIN.V1
//	adc.tmp[6] = (float)adc.conv_val[6];						// AIN.I2
//	adc.tmp[7] = (float)adc.conv_val[7] - 2262.;				// AIN.V2 

//	AINI1 = adc.scale[4] * adc.tmp[4];
//	AICMD = adc.scale[5] * adc.tmp[5];
//	AINI2 = adc.scale[6] * adc.tmp[6];
//	AINV2 = adc.scale[7] * adc.tmp[7];
//	AICMD = AINV1;		// bayasaa 2010.05.19

	if(sc_cnt == 10)
	{
		sc_cnt = 0;
		speed_controller();
	}
	else
	{
//		a= (float)CpuTimer0Regs.TIM.all;
		// 18.7 uS
		SCIC_Process();
//		a= (a-(float)CpuTimer0Regs.TIM.all)/150.;  // us Time
//		if (Interrupt_time_max<a)	Interrupt_time_max= a; 
	}
	sc_cnt++;

	Ias = (adc_scale_0 * adc.tmp[0]);
	Ibs = (adc_scale_1 * adc.tmp[1]);
	Ics = -(Ias + Ibs);

	Vdc_measured = (adc_scale_3 * adc.tmp[3]) - 1.5; // offset 

	Idss = Ias;
	Iqss = (Ibs - Ics)*INV_SQRT3;

	// 3.5uS
	fault_chk();

	if (cnlt_flag.control == 1) 
	{
		cc_cnt++;
		if(cc_cnt == 1000)
		{
			cc_cnt = 0;
			cnlt_flag.FOC = 1;
		}
	}

	if (run_Sensorless==1)
	{
		Wrm_ref = Wrpm_ref*0.104719755; // PI/30 = 0.10471975511965977461542144610932
		Wr_ref = Wrm_ref; // * PP;  PP = 1;

		switch(sensorless_mode)
		{
			case 0 :
				// Current_Slop_Build_Up = a = Idse_ref_max * Tsamp;	// 1sec
//				if ( Idse_ref < (Idse_ref_max - Current_Slop_Build_Up) )  	Idse_ref += Current_Slop_Build_Up;
				if(Flag_First_Run == 1)
				{
					if ( Idse_ref < Idse_ref_max )	Idse_ref += Current_Slop_Build_Up;
					else 							Idse_ref  = Idse_ref_max;
					if(thetar_openloop > (2. * PI))	    // 2sec , Wrpm_ref = 30.;
					{
	                    sensorless_mode = 1;            // mode_change 1 ( 0--> 1)
						Wrpm_ref = 0.0;
					}
					else
					{
						thetar_openloop = thetar_openloop + Wr_ref*Tsamp;
					}
				}
				else
				{
					a = Idse_ref_max * 0.5 * Tsamp;			// 2sec
					if(Idse_ref < 30.)	Idse_ref +=a;
					else				sensorless_mode = 1;            // mode_change 1 ( 0--> 1);

				}

				if ( Flag_First_Run == 1 ) 	a = 1.0;
				else							a = 0.0;
				cos_theta = cos(thetar_openloop * a);
				sin_theta = sin(thetar_openloop * a);
				thetar_hat1 = BOUND_PI(thetar_openloop + 0.5 * Wr_ref*Tsamp);
				cos_theta1 = cos(thetar_hat1 * a);
				sin_theta1 = sin(thetar_hat1 * a);
				break;

			case 1 :
				Flag_First_Run = 2;
				thetar_openloop = 0.0;
				cos_theta = 1.0;
				sin_theta = 0.0;
				thetar_hat1 = 0.0;
				cos_theta1 = 1.0;
				sin_theta1 = 0.0;

				tmp_cnt++;
				a = Idse_ref_max/(40000.0);			// 2sec
				if((tmp_cnt < 5000) && ( Flag_First_Run == 1 ))	Idse_ref = Idse_ref_max;
				else
				{
					Idse_ref -= a;					// decrease Idse_ref for open loop control
					if(Idse_ref <= (20.0 * K_Id))
					{
                        sensorless_mode = 2;        // mode_change 2 ( 1 --> 2)
					 	tmp_cnt = 0;
						Iqse_ref = sqrt(Is_max * Is_max * K_Is_2 - Idse_ref*Idse_ref);
							                        // Initial value for open loop control
					}
				}
				break;

			case 2 :
				a = Idse_ref_max/(20000.0);			// 2sec : 40000, 1sec : 20000
				if ( Idse_ref > Idse_ref_min )  	Idse_ref -= a;
				else 								Idse_ref  = Idse_ref_min;
// ********************************************************************************************
				Id_cm = (I_integ)*(Vdse_ref1 - Rs*Idse1 + Wr_hat_cm*Ls*Iqse1) + Idse1;
				Iq_cm = (I_integ)*(Vqse_ref1 - Rs*Iqse1 - Wr_hat_cm*(Ls*Idse1 + LAMpm)) + Iqse1;

				cos_theta_cm = cos(thetar_hat_cm);
				sin_theta_cm = sin(thetar_hat_cm);
				Idse1 =  cos_theta_cm*Idss + sin_theta_cm*Iqss;
				Iqse1 = -sin_theta_cm*Idss + cos_theta_cm*Iqss;

				Vdse_ref1 =  Vdse_ref; // close loop estimation
				Vqse_ref1 =  Vqse_ref; // close loop estimation

				// K_estimation = Ka*Fsw*Ls/LAMpm;
				Wr_hat_cm = Wr_hat_cm + K_estimation*( (Idse1 - Id_cm) - (Iqse1 - Iq_cm) );
				thetar_hat_cm1 = BOUND_PI(thetar_hat_cm + Wr_hat_cm*Tsamp*1.5);
				thetar_hat_cm  = BOUND_PI(thetar_hat_cm + Wr_hat_cm*Tsamp);

				Wr_hat = Wr_hat_cm;
				cos_theta = cos_theta_cm;
				sin_theta = sin_theta_cm;
				cos_theta1 = cos(thetar_hat_cm1);
				sin_theta1 = sin(thetar_hat_cm1);
// ********************************************************************************************
				Wrm_hat = Wr_hat;
				Wrpm_hat = Wrm_hat * 9.5492965855;          // 30/PI = 9.5492965855137201461330258023509
				tmp_cnt = 0;
				break;

			case 4:
// ********************************************************************************************
				Iqse_limit = sqrt(Is_max * Is_max - Idse_ref * Idse_ref);

//			if( (Wrpm_hat > Wrpm_min) && (Flag_STOP == 0) ) // Running Mode

				if(Wrpm_hat > Wrpm_min)
				{
					// I_integ = Tsamp/Ls
					Id_cm = (I_integ)*(Vdse_ref1 - Rs*Idse1 + Wr_hat_cm*Ls*Iqse1) + Idse1;
					Iq_cm = (I_integ)*(Vqse_ref1 - Rs*Iqse1 - Wr_hat_cm*(Ls*Idse1 + LAMpm)) + Iqse1;

					cos_theta_cm = cos(thetar_hat_cm);
					sin_theta_cm = sin(thetar_hat_cm);
					Idse1 =  cos_theta_cm*Idss + sin_theta_cm*Iqss;
					Iqse1 = -sin_theta_cm*Idss + cos_theta_cm*Iqss;

					Vdse_ref1 =  Vdse_ref; // close loop estimation
					Vqse_ref1 =  Vqse_ref; // close loop estimation

					Wr_hat_cm = Wr_hat_cm + K_estimation * ( (Idse1 - Id_cm) - (Iqse1 - Iq_cm) );
					thetar_hat_cm1 = BOUND_PI(thetar_hat_cm + Wr_hat_cm*Tsamp*1.5); 
					thetar_hat_cm  = BOUND_PI(thetar_hat_cm + Wr_hat_cm*Tsamp);

					Wr_hat = Wr_hat_cm;	
			
					cos_theta = cos_theta_cm;
					sin_theta = sin_theta_cm;	
					cos_theta1 = cos(thetar_hat_cm1);
					sin_theta1 = sin(thetar_hat_cm1);
					thetar_openloop = thetar_hat_cm;   			// Set Initial value for STOP Operation

	/*				if(Wrpm_hat > (Wrpm_base_speed + 4000.))
					{
						a = Idse_ref_max/(40000.0);				// 2sec 
						if (Is_mag > 35.0)                      // Reduce Idse_ref to Zero
						{                                           
							if ( Idse_ref > a )  				Idse_ref -= a;  
							else 								Idse_ref  = 0.;
						}
						else if (Is_mag < 20.0)                  // Increase Idse_ref to Idse_ref_min
						{
							if ( Idse_ref < Idse_ref_min )  	Idse_ref += a;  
							else 								Idse_ref  = Idse_ref_min;
						}
					}		*/
				}
				else									
				{
	                Flag_STOP = 1;                              // Stopping Mode
					Wr_hat = Wr_ref;
					Iqse_ref = 0.0;
					if ( Wrpm_ref > 0.0 )
					{
						thetar_openloop = BOUND_PI(thetar_openloop + Wr_ref*Tsamp);		
						cos_theta = cos(thetar_openloop);
						sin_theta = sin(thetar_openloop);
						thetar_hat1 = BOUND_PI(thetar_openloop + 0.5 * Wr_ref*Tsamp);
						cos_theta1 = cos(thetar_hat1);
						sin_theta1 = sin(thetar_hat1);
					}
					else
					{
						Iqse_ref = 0.0;
						a = 0.1*PI/180.0;
						if (fabs(thetar_openloop) > a ) thetar_openloop = BOUND_PI(thetar_openloop + 0.5 * a);
						else                             thetar_openloop  = 0.0;	
						cos_theta = cos(thetar_openloop);
						sin_theta = sin(thetar_openloop);
						thetar_hat1 = thetar_openloop;
						cos_theta1 = cos(thetar_hat1);
						sin_theta1 = sin(thetar_hat1);
						if (thetar_openloop == 0.0)
						{
							a = Idse_ref_max * 2 * Tsamp;			// 2sec : 40000, 1sec : 20000
							if ( Idse_ref > 10. )  	Idse_ref -= a;
							else
							{
//								Idse_ref  = 10.;
//								tmp_cnt1++;
//								if(tmp_cnt1 > 1500) 
//								{
						//			Flag_STOP = 0;
		                            sensorless_mode = 99;           // mode_change 4 ( 4 --> 99)
									tmp_cnt1 =0;
									Position_Flag = 1;
//								}
							}
						}
					}
				}
// ********************************************************************************************	
				Wrm_hat  = Wr_hat;
				Wrpm_hat = Wrm_hat * 9.549296;          // 30/PI = 9.5492965855137201461330258023509

				tmp_cnt++;
				if(tmp_cnt > 15000)
				{
					if(flag_speed_cmd == 1) Wrpm_ref_cmd = Wrpm_ref_set;

					if		(Wrpm_ref_cmd > (Wrpm_ref + ref_slew_up)) {	Wrpm_ref = Wrpm_ref + ref_slew_up; }
					else if	(Wrpm_ref_cmd < (Wrpm_ref - ref_slew_dn)) {	Wrpm_ref = Wrpm_ref - ref_slew_dn; }
					else 	 Wrpm_ref = Wrpm_ref_cmd;

					tmp_cnt = 15000;
					if(Wrpm_hat < (Wrpm_base_speed-8000.))
					{
						ref_slew_dn = 10000. * Tsamp;
						a = Idse_ref_max/(50000.0);			// 2sec
						if ( Idse_ref < (Idse_ref_max - a) )  	Idse_ref += a;
						else 									Idse_ref  = Idse_ref_max;
					}
				}
				break;

			case 99 :
						asm("   nop");
				break;							
			default : 	asm("   nop");
				break;														
			}
	}	// if (run_Sensorless==1)
	else
	{
		Idss = 0.0, Iqss = 0.0;
	}
	/* Idqss to Idqse Conversion */
	Idse =  cos_theta*Idss + sin_theta*Iqss;
	Iqse = -sin_theta*Idss + cos_theta*Iqss;

	if(cnlt_flag.FOC)
	{
	/* Synchronous PI Controller with feedforward compensation */
		a = Vdc_measured*0.5;

		Err_Idse = Idse_ref - Idse;
		Vdse_anti = Vdse_ref_fb + Vdse_ref_ff - Vdse_ref;
		Vdse_ref_integ += Ki_ccT*(Err_Idse - Ka_cc * Vdse_anti);
		Vdse_ref_fb = Vdse_ref_integ + alpha_cc*Kp_cc*Err_Idse - (1 - alpha_cc)*Kp_cc*Idse;

		Err_Iqse = Iqse_ref - Iqse;
		Vqse_anti = Vqse_ref_fb + Vqse_ref_ff - Vqse_ref;
		Vqse_ref_integ += Ki_ccT*(Err_Iqse - Ka_cc * Vqse_anti);
		Vqse_ref_fb = Vqse_ref_integ + alpha_cc*Kp_cc*Err_Iqse - (1 - alpha_cc)*Kp_cc*Iqse; 

	/* Feedforward */
		Vdse_ref_ff = 0.0;
		Vqse_ref_ff = LAMpm * Wr_hat + Wr_hat * Ls * Idse;

		Vdse_ref_ffa = - Wr_hat * Ls * Iqse; // 5.22
		Vqse_ref_ffa = LAMpm * Wr_hat + Wr_hat * Ls * Idse;
	}
	else
	{
		Vdse_ref_fb = 0.0;		Vdse_ref_ff = 0.0;
		Vqse_ref_fb = 0.0;		Vqse_ref_ff = 0.0;
	}

	// Voltage Reference 
	Vdse_ref = Vdse_ref_fb + Vdse_ref_ff;
	Vqse_ref = Vqse_ref_fb + Vqse_ref_ff;

	// Vdqse to Vdqss Conversion 
	Vdss_ref = cos_theta1*Vdse_ref - sin_theta1*Vqse_ref;
	Vqss_ref = sin_theta1*Vdse_ref + cos_theta1*Vqse_ref;

	// Vdqss to Vabcs Conversion 
	Vas_ref = Vdss_ref;
	Vbs_ref = -0.5*(Vdss_ref - SQRT3*Vqss_ref);
	Vcs_ref = -(Vas_ref + Vbs_ref);

// Space Vector PWM based on offset voltage method
	if(Vas_ref > Vbs_ref)
	{
		Vmax = Vas_ref;
		Vmin = Vbs_ref;
	}
	else
	{
		Vmax = Vbs_ref;
		Vmin = Vas_ref;
	}
	if(Vcs_ref > Vmax)	Vmax = Vcs_ref;	
	if(Vcs_ref < Vmin)	Vmin = Vcs_ref;

	Vsn = - (Vmax + Vmin)*0.5;

	/* Pole Voltage Reference */
	Van_ref = Vas_ref + Vsn;
	Vbn_ref = Vbs_ref + Vsn;
	Vcn_ref = Vcs_ref + Vsn;

	Vdc_eff = Vdc_measured*(1.0 - 2.0*Deadtime/Tsamp);
	INV_Vdc_2 = 2.0/Vdc_measured;
	// Over-modulation
 	if((Vmax - Vmin) > Vdc_eff)
 	{          
		a = Vdc_eff/(Vmax - Vmin);
		Vas_ref *= a;
	   	Van_ref *= a;  Vbn_ref *= a;  	Vcn_ref *= a;
		Vdse_ref *= a; Vqse_ref *= a;	Vdss_ref *= a; Vqss_ref *= a;
	}

	switch(cnlt_flag.FOC)
	{
		case 1 :	// PWM control by controller
			a = (float)pwm_g1.phase_duty_half_scaled;

			Ta_cnt = a*Van_ref*INV_Vdc_2; // INV_Vdc_2 = 2.0/Vdc_measured;
			Tb_cnt = a*Vbn_ref*INV_Vdc_2;
			Tc_cnt = a*Vcn_ref*INV_Vdc_2;

			// Dead time Compensation
			I_dead = 3.0;
			b = I_dead;
			if(fabs(Ias) > b)
			{	 if(Ias  > 0.)	Ta_cnt += T_dead_cnt_2; 	else Ta_cnt -= T_dead_cnt_2;	}
			else 				Ta_cnt += (T_dead_cnt_2*Ias*fabs(Ias)/(b*b));

			if(fabs(Ibs) > b)
			{	 if(Ibs  > 0.)	Tb_cnt += T_dead_cnt_2; 	else Tb_cnt -= T_dead_cnt_2;	}
			else 				Tb_cnt += (T_dead_cnt_2*Ibs*fabs(Ibs)/(b*b));

			if(fabs(Ics) > b)
			{	 if(Ics  > 0.)	Tc_cnt += T_dead_cnt_2; 	else Tc_cnt -= T_dead_cnt_2;	}
			else 				Tc_cnt += (T_dead_cnt_2*Ics*fabs(Ics)/(b*b));

			a = a - 1.0; // for safety

			Ta_cnt = (Ta_cnt > a) ? a : (Ta_cnt < -a) ? -a : Ta_cnt;
			pwm_g1.phase_a_duty_scaled = (int)Ta_cnt;

			Tb_cnt = (Tb_cnt > a) ? a : (Tb_cnt < -a) ? -a : Tb_cnt;
			pwm_g1.phase_b_duty_scaled = (int)Tb_cnt;
	
			Tc_cnt = (Tc_cnt > a) ? a : (Tc_cnt < -a) ? -a : Tc_cnt;
			pwm_g1.phase_c_duty_scaled = (int)Tc_cnt;
			break;

		case 0 :	default :
			pwm_g1.phase_a_duty_scaled = 0;
			pwm_g1.phase_b_duty_scaled = 0;
			pwm_g1.phase_c_duty_scaled = 0;
			break;
	} 

	EPwm1Regs.CMPA.half.CMPA = pwm_g1.phase_duty_half_scaled + pwm_g1.phase_a_duty_scaled;
	EPwm2Regs.CMPA.half.CMPA = pwm_g1.phase_duty_half_scaled + pwm_g1.phase_b_duty_scaled;
	EPwm3Regs.CMPA.half.CMPA = pwm_g1.phase_duty_half_scaled + pwm_g1.phase_c_duty_scaled;

//	dev_BackgroundDAC();
}


int Temperature_Calculation(int  Adc_Temperature)
{

	float C1, C2, R_NTC, B, Temp;
	// [5.5kW ~ 22kW]
	// Vi = 3.3V/3.7k = 0.891892V
	// Rm = 470 Ohm
	// Iref = 1.8976mA
	// R1=100k, R2=30k, R3=0.47k

	// T_25=298K, R_25=5.0 kOhm
	// T_100=373K, R_100=0.4933kOhm
	// B = (T_25 x T_100) ln (R_100/R_25) / (T_25 - T_100) = 3432.5477

	// �µ� ���� ȸ���� ���� �����⿡ �Ʒ��� ���� ����� ������
	// ȸ�ΰ� ����Ǿ� ����.
	// R_NTC^ = (R1+R2+R3)Vo/((R2/R1)(R1+R2)I-Vo)
	// Vo = (u/1024)3.28Vref
	// R_NTC = 400k R_NTC^/(400k-R_NTC^)		: 2 x 200kOhm �� ����

	// R_NTC^ = (100+30+0.47) x (u/1024) x 3.28 / {(30/100)(100+30)1.8976-(u/1024)3.28}
	//        = 0.417912 u / (74.0064 - 3.203e-3 u) x 1.0e3 [Ohm]
	//        = C1 x u /(C2 - 3.203e-3 x u) x 1.0e3	
	// C1 = 0.417912		-> tmp_float1
	// C2 = 74.0064	-> tmp_float2
	// R_NTC = 400k R_NTC^ / (400k - R_NTC^)	

	C1=0.417912;
	C2=74.0064;
	R_NTC = 5.0;				// 5.0kOhm



//	B = 3433.;
	B = 3337.;
	// ADC valid bit: 10.9 bit -> 9 bit�� �����, ���ش� 8/4096  
	Temp= (float)(Adc_Temperature>>3);
	Temp = (C1*2*Temp)/(C2-6.406e-3*Temp);		// 2 x 3.203e-3 -> 6.406e-3
	Temp = (400*Temp) / (400-Temp);				// R_NTC = 400 x R_NTC^ / (400 - R_NTC^)
	
	// tmp_int -> B + T1 ln(R2/R1)
	Temp = B + (298.* log(Temp/R_NTC));
	if (Temp<1)	Temp=1;
		
	// (B x T1)_x10 / [B + T1 ln(R2/R1)] -> (T_x10 - 2730) -> �µ� [degC]		
	Temp = (B * 2980/Temp)-2730; 			// x10 : (25 + 273)k x 10 -> 2980
	
	// -25�� ~ 150��
	if 		(Temp > 1500.)	Temp = 1500.;
	else if (Temp < -250.)	Temp = -250.;
	return Temp;
}

