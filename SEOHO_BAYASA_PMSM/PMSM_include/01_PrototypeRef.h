#ifndef __FUNCTION_DEFINITION_
#define __FUNCTION_DEFINITION_

#include "DSK2833x_Define.h"

#pragma CODE_SECTION(current_controller, "ramfuncs");
#pragma CODE_SECTION(speed_controller, "ramfuncs"); 

// Main_Loop Functions
// File Name : "00_Main.c"
extern void SEQUENCE();
extern void Drive_Off();
extern void Drive_On();
extern void Run_Stop_Operation();

// Analog Input Functions
// File Name : "01_analog_in_out.c"
extern void Analog_Output_Monitoring();

// Digital In/Out Functions
// File Name : "02_digital_inout.c"
extern void digital_input_proc ();
extern void digital_out_proc ();

// Communication Functions
// File Name : "03_Communication.c"
extern void Read_Data_Registers(int Par_index);
extern void Write_Data_Registers_Online();
extern void Write_Data_Registers_Offline(int Par_index);

// DAC Parameter
// File Name : 04_DAC.c 
extern void dev_DacSetup();
extern void dev_InitDACVariable();
extern void dev_BackgroundDAC(); 

// Fault Related Functions
// File Name : "05_Fault.c"
#pragma CODE_SECTION(fault_chk, "ramfuncs");
#pragma CODE_SECTION(CheckOverCurrent, "ramfuncs");
#pragma CODE_SECTION(CheckFaultDriver, "ramfuncs");
#pragma CODE_SECTION(CheckOverVolt, "ramfuncs");
#pragma CODE_SECTION(CheckOverHeat, "ramfuncs");
#pragma CODE_SECTION(CheckSpeedDetection, "ramfuncs");
#pragma CODE_SECTION(ExternalFault, "ramfuncs");
#pragma CODE_SECTION(CheckUnderVolt, "ramfuncs");
#pragma CODE_SECTION(CheckFaultZC, "ramfuncs");

extern void	fault_chk();
extern int	CheckOverCurrent( );
extern int	CheckFaultDriver( );
extern int	CheckOverVolt( );
extern int	CheckOverHeat( );
extern int	CheckSpeedDetection();
extern int	ExternalFault( );
extern int	CheckUnderVolt( );
extern int	CheckFaultZC();

extern void	Fault_Recording(int Fault_cnt);
extern void	Read_Fault_Record(int Record_number);
extern void	TripProc();

// Current Controller Functions
// File : " current_control.c"
extern void Init_dsc();
extern void init_control_value();
extern void rst_chk();
extern void Init_var();
extern void Write_Data_Registers_Offline();
extern void System_Config();
extern int  Temperature_Calculation(int  Adc_Temperature);
extern void current_controller();
extern void calculateOffset();



// System_Configuration Functions
// File Name : "08_system.c"
extern void Relay_setup();
extern void	Parameter_Initialization();

// File : " speed_control.c"
extern void speed_controller();

// File : "09_PWM.c"
extern void pwm_buffer_setup();
extern void pwm_g1_setup(double sys_clk, double pwm_freq, double dead_time);

// File : "10_I2C_RTC.c"
extern void Change_I2C_RTC(void);
extern void Recover_I2C_EEPROM(void);
extern void Write_RTC_Data(Uint16 uAddr, Uint16 uData);
extern void Read_RTC_Data(Uint16 uAddr, Uint16 *pData);
extern void Set_InitialTime(void);

// EepRom Control Functions
// File : "11_I2C_eeprom.c"
extern void Init_I2C_eeprom();
extern void Write_EEPROM_Data(Uint16 uCSBit, Uint16 uAddr, Uint16 uData);
extern void Read_EEPROM_Data(Uint16 uCSBit, Uint16 uAddr, Uint16 *pData);
extern void Word_Write_data(Uint16 uAddr, Uint16 uData);
extern void Word_Read_data(Uint16 uAddr, Uint16 *pData);

// File : "12_SCI_BC.c"
extern void scib_init();
extern void scic_init();
extern void scib_TxChar(char c);
extern void scib_TxString(char *p);
extern void scic_TxChar(char c);
extern void scic_TxString(char *p);
extern void scib_tx_start(void);
extern void scic_tx_start(void);
extern void scib_putc(char d);
//extern void scic_putc(char d);
extern void scib_puts(char *p);
extern void scic_puts(char *p);
extern void scic_test(void);
#endif	//	__FUNCTION_DEFINITION_

