#include "00_main_def.h"

/* 1msec routine */
void speed_controller()
{
//	float a;

	if(run_Sensorless)
	{
		switch(sensorless_mode)
		{
			case 0 :
				Wrpm_ref = 30.;
				break;
			case 1 :
				Wrpm_ref = 0;
				break;
			case 2 : 
				if(Wrpm_hat > Wrpm_base_speed)  
				{	
					Wrpm_ref = Wrpm_base_speed;	 
					Wrpm_ref_cmd = Wrpm_base_speed;
					Wrpm_ref_set = Wrpm_base_speed;
					sensorless_mode = 4;			// 	mode_change 3 ( 2 --> 4)
					Wrm_err_integ = Iqse_ref;  		// Initialization of integrator for S_L_Control
				}										
				break;
			case 4 :
				Wrm_err = (Wrpm_ref * 0.104719755) - Wrm_hat;// PI/30 = 0.10471975511965977461542144610932
				Wrm_err_integ += Wrm_err * Ki_scT;
//				Wrm_err_integ = (Wrm_err_integ > Is_max) ? Is_max :
//								(Wrm_err_integ < -Is_max) ? -Is_max : Wrm_err_integ;

			//	Iqse_limit = a;
//				Iqse_limit = sqrt(Is_max * Is_max - Idse_ref * Idse_ref);

				Wrm_err_integ = (Wrm_err_integ > Iqse_limit) ? Iqse_limit :
								(Wrm_err_integ < -Iqse_limit) ? -Iqse_limit : Wrm_err_integ;
				Iqse_ref = Kp_sc*Wrm_err + Wrm_err_integ;	
				
//				Iqse_ref = (Iqse_ref > Is_max) ? Is_max :
//								(Iqse_ref < -Is_max) ? -Is_max : Iqse_ref;
				Iqse_ref = (Iqse_ref > Iqse_limit) ? Iqse_limit :
								(Iqse_ref < -Iqse_limit) ? -Iqse_limit : Iqse_ref;										
//				Iqse_ref = (Iqse_ref > a) ? a :
//								(Iqse_ref < -a) ? -a : Iqse_ref;										

				break;					
			case 99 :
			default :
				break;																
		}
	}		
	else
	{
		Idse_ref = 0.0;
		Iqse_ref = 0.0;
	}
}


