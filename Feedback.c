#include "PUMA_Robot_Arm.h"

int ADC_value_0 = 2048;
int ADC_value_1 = 2048;

void potentiometer_compare_speed(){
	//Set CMPA and CMPB values for ePWM1 signals
   	  ADC_value_0 = ((AdcRegs.ADCRESULT0)>>4);
   	  ADC_value_1 = ((AdcRegs.ADCRESULT1)>>4);
   	  
   	  //Sets both ePWM signals to zero for deadzone
   	  if ((ADC_value_0 > (2048-DEADZONE)) && (ADC_value_0 < (2048+DEADZONE)))
   	  {
   	  	EPwm1Regs.CMPA.half.CMPA = 0;		//Set ePWM1A to zero
   	  	EPwm1Regs.CMPB = 0;					//Set EPWM1B to zero
//   	  	GpioDataRegs.GPASET.bit.GPIO30 = 1;	//Set GPIO30 pin high to indicate being in the deadzone
   	  }
   	  
   	  //Sets ePWM1A to control motor when ADC is above the deadzone
   	  else if (ADC_value_0 >= (2048+DEADZONE))
   	  {
//   	  	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	//Set GPIO30 pin low to indicate not being in the deadzone
   	  	EPwm1Regs.CMPB = 0;						//Set ePWM1B to zero
   	  	EPwm1Regs.CMPA.half.CMPA = ((ADC_value_0-((float)2048+DEADZONE))/((float)4096-(2048+DEADZONE))*(float)4096);	//Set ePWM1A value when ADC is above deadzone
   	  }
   	  
   	  //Sets ePWM1B to control motor when ADC is below the deadzone
   	  else if (ADC_value_0 <= (2048-DEADZONE))
   	  {
//   	  	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	//Set GPIO30 pin low to indicate not being in the deadzone
   	  	EPwm1Regs.CMPA.half.CMPA = 0;			//Set ePWM1A to zero
   	  	EPwm1Regs.CMPB = ((((float)2048-DEADZONE)-ADC_value_0)/((float)2048+DEADZONE))*(float)4096;	//Set ePWM1B value when ADC is below the deadzones
   	  }
//================================================================================
if ((ADC_value_1 > (2048-DEADZONE)) && (ADC_value_1 < (2048+DEADZONE)))
   	  {
   	  	EPwm2Regs.CMPA.half.CMPA = 0;		//Set ePWM1A to zero
   	  	EPwm2Regs.CMPB = 0;					//Set EPWM1B to zero
//   	  	GpioDataRegs.GPASET.bit.GPIO30 = 1;	//Set GPIO30 pin high to indicate being in the deadzone
   	  }
   	  //Sets ePWM1A to control motor when ADC is above the deadzone
   	  if (ADC_value_1 >= (2048+DEADZONE))
   	  {
//   	  	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	//Set GPIO30 pin low to indicate not being in the deadzone
   	  	EPwm2Regs.CMPB = 0;						//Set ePWM1B to zero
   	  	EPwm2Regs.CMPA.half.CMPA = ((ADC_value_1-((float)2048+DEADZONE))/((float)4096-(2048+DEADZONE))*(float)4096);	//Set ePWM1A value when ADC is above deadzone
   	  }
   	  //Sets ePWM1B to control motor when ADC is below the deadzone
   	  if (ADC_value_1 <= (2048-DEADZONE))
   	  {
//   	  	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;	//Set GPIO30 pin low to indicate not being in the deadzone
   	  	EPwm2Regs.CMPA.half.CMPA = 0;			//Set ePWM1A to zero
   	  	EPwm2Regs.CMPB = ((((float)2048-DEADZONE)-ADC_value_1)/((float)2048+DEADZONE))*(float)4096;	//Set ePWM1B value when ADC is below the deadzones
   	  }
   	  DELAY_US(50);
}

void potentiometer_compare_position(){
	//Set CMPA and CMPB values for ePWM1 signals
   	  ADC_value_0 = ((AdcRegs.ADCRESULT0)>>4);
   	  ADC_value_1 = ((AdcRegs.ADCRESULT1)>>4);
   	//Set the encoder compare value based on potentiometer input  
   	 EQep1Regs.QPOSCMP = (ADC_value_0/(float)4095)*(float)EQep_Max;
   	 EQep2Regs.QPOSCMP = (ADC_value_1/(float)4095)*(float)EQep_Max;
   	 
   	 DELAY_US(5);
}
