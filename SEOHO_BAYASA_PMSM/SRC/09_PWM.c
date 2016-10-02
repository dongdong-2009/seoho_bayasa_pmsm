#include "00_main_def.h"

#define PWM_ACTIVE_HIGH		1
#define PWM_ACTIVE_LOW		0
#define PWM_ACTIVE_LOW_SIMULTANEOUS	5
#define SAMPLING_TOP		0x01
#define SAMPLING_BOTTOM		0x02
#define SAMPLING_BOTH		0x03

//void GetAnaMonitCount(unsigned int * piChA, unsigned * piChB);

interrupt void epwm1_timer_isr(void);
#pragma CODE_SECTION(epwm1_timer_isr, "ramfuncs");

void pwm_buffer_setup()
{
    EALLOW;
	nBOOT_MODE_SET; 
	nPWM_ENABLE_SET;
	EDIS;
}

void pwm_g1_setup(double sys_clk, double pwm_freq, double dead_time)
{
	EALLOW;

	GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Enable pull-down on GPIO0 (EPWM1A)
	GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Enable pull-down on GPIO1 (EPWM1B)   
	GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Enable pull-down on GPIO2 (EPWM2A)
	GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Enable pull-down on GPIO3 (EPWM2B)   
	GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Enable pull-down on GPIO4 (EPWM3A)
	GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Enable pull-down on GPIO5 (EPWM3B)   
	
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM2B

	EDIS;

    DINT;

    InitPieCtrl();
    InitPieVectTable();

    EALLOW;
    PieVectTable.EPWM1_INT = &epwm1_timer_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;      // Stop all the TB clocks
    EDIS;

	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.TBPRD = (unsigned int)((1/pwm_freq)/(2*(1/sys_clk)));
	EPwm1Regs.TBPHS.half.TBPHS = 0;
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;		// Load registers every ZERO
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;   
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;

    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;			
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;            
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET; 

    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBRED = (unsigned int)(dead_time*sys_clk);
    EPwm1Regs.DBFED = (unsigned int)(dead_time*sys_clk);

//    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;
    EPwm1Regs.ETSEL.bit.INTEN = 1;  // Enable INT
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 1st event
    // Setup compare 
    EPwm1Regs.CMPA.half.CMPA = (((unsigned int)((1/pwm_freq)/(2*(1/sys_clk))))>>1);

	//EPWM2 configuration
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.TBPRD = (unsigned int)((1/pwm_freq)/(2*(1/sys_clk)));
	EPwm2Regs.TBPHS.half.TBPHS = 0;
	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;
	EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;

    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // Pass through 
	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;    // Load registers every ZERO
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;   

    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;			
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;            
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET; 

    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = (unsigned int)(dead_time*sys_clk);
    EPwm2Regs.DBFED = (unsigned int)(dead_time*sys_clk);

    EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
    EPwm2Regs.ETSEL.bit.INTEN = 0;  // Enable INT
    // Setup compare 
    EPwm2Regs.CMPA.half.CMPA = (((unsigned int)((1/pwm_freq)/(2*(1/sys_clk))))>>1);   

	//EPWM3 configuration
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.TBPRD = (unsigned int)((1/pwm_freq)/(2*(1/sys_clk)));
	EPwm3Regs.TBPHS.half.TBPHS = 0;
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;  // Pass through 

    EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_PRD;
    EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_PRD;   

    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_LOC;			
	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;            
	EPwm3Regs.AQCTLA.bit.CAD = AQ_SET; 
 
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED = (unsigned int)(dead_time*sys_clk);
    EPwm3Regs.DBFED = (unsigned int)(dead_time*sys_clk);
    EPwm3Regs.ETSEL.bit.INTEN = 0;  // Enable INT
    // Setup compare 
    EPwm3Regs.CMPA.half.CMPA = (((unsigned int)((1/pwm_freq)/(2*(1/sys_clk))))>>1);   


	//------------------------------
	//InitEPwm5
	//------------------------------

   EPwm5Regs.TBPRD 				= (unsigned int)((1/pwm_freq)/(2*(1/sys_clk)));   // Set timer period
   EPwm5Regs.TBCTL.bit.CTRMODE 	= TB_COUNT_UPDOWN; 		// Count up
   EPwm5Regs.TBCTL.bit.PHSEN 	= TB_DISABLE;        // 

   EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;      // 
   EPwm5Regs.TBCTL.bit.CLKDIV 	= TB_DIV1;         // Slow so we can observe on the scope

   // Setup compare 
   EPwm5Regs.CMPA.half.CMPA 	= (unsigned int)((1/pwm_freq)/(2*(1/sys_clk))); 
   EPwm5Regs.CMPB				= (unsigned int)((1/pwm_freq)/(2*(1/sys_clk))); 

   // Set actions
   EPwm5Regs.AQCTLA.bit.CAU 		= AQ_SET;            
   EPwm5Regs.AQCTLA.bit.CAD 		= AQ_CLEAR;

   EPwm5Regs.AQCTLB.bit.CBU 		= AQ_SET;            
   EPwm5Regs.AQCTLB.bit.CBD 		= AQ_CLEAR;
   
   EPwm5Regs.DBCTL.bit.OUT_MODE 	= DB_DISABLE;
   EPwm5Regs.ETSEL.bit.INTEN 		= 0;                  // Enable INT


	// calculation pwm half duty
	pwm_g1.phase_duty_half_scaled = (((unsigned int)((1/pwm_freq)/(2*(1/sys_clk))))>>1);
	pwm_g1.phase_duty_max_scaled = pwm_g1.phase_duty_half_scaled*2;
	pwm_g1.phase_duty_min_scaled = 0;
	pwm_g1.phase_a_duty_scaled = 0;
	pwm_g1.phase_b_duty_scaled = 0;
	pwm_g1.phase_c_duty_scaled = 0;

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;         // Start all the timers synced
    EDIS;
    IER |= M_INT3;

    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
}


interrupt void epwm1_timer_isr(void)
{
	float Temp;
	TxIntervalCnt++;		// RYU
	TxInterval_1s++;		// RYU 
	watchdog_cnt ++;
//	Temp= (float)CpuTimer0Regs.TIM.all;
	// 초기 충전 체크 
	if(Init_Charge_cnt_EN) 	Init_Charge_cnt++;
	else 					Init_Charge_cnt=0;
	// 펄트 발생 후 재시작 카운터
	if (Retry_cnt_En)	Retry_cnt += Tsamp;
	else				Retry_cnt = 0;
	current_controller();

//	Temp= (Temp-(float)CpuTimer0Regs.TIM.all)/150.;  // us Time
//	if (Interrupt_time_max<Temp)	Interrupt_time_max= Temp; 

	EPwm1Regs.ETCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


