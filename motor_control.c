/*--------------------------------------------------------------*/
//Revised February 16, 2013
//Description:
//This outputs ePWM1 and ePWM2 on GPIO00/GPIO01 and GPIO03/GPIO04 respectvely. 
//ePWMxA is used to control the motor in one direction, and ePWMxB is used for the opposite direction. 
//The position of the joint is determined by the poisition of potentiometers from ADCA0/ADCA1.
//An encoder is used to determine the position of the joint.
//When an LED is attached to the ePWM outputs, the result is controlling the intensity of the LED.
//This is setup for controlling two joints.
//The code is uploaded to the flash memory, so the GPIO84-87 jumpers need to be set high.
/*--------------------------------------------------------------*/ 

#include "PUMA_Robot_Arm.h"

typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;
}EPWM_INFO;

//void initialize(void);
void update_compare(EPWM_INFO *epwm_info);

// Global variables used in this example
extern EPWM_INFO epwm1_info;


// Functions that will be run from RAM need to be assigned to
// a different section.  This section will then be mapped using
// the linker cmd file.
//#pragma CODE_SECTION(PWM_isr, "ramfuncs");

// These are defined by the linker (see F28335.cmd)
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

// Global variables for eQEP
Uint16 Interrupt_Count = 0;
	
POSSPEED qep_posspeed=POSSPEED_DEFAULTS;

void main(void)
{
	Uint16 ReceivedChar;
//	Uint16 rdata_point;
//	Uint16 rdata;
//	Uint16 sdata[8];
	char *msg;
	InitSysCtrl();	
	
	// Specific clock setting for the ADC
   EALLOW;
   SysCtrlRegs.HISPCP.all = ADC_MODCLK;	// HSPCLK = SYSCLKOUT/ADC_MODCLK
   EDIS;
	
	// For this case just init GPIO pins for ePWM1, ePWM2, eQEP1 and eQEP2
   // These functions are in the DSP2833x_EPwm.c file
   InitEPwm1Gpio();
   InitEPwm2Gpio();
   InitEQep1Gpio();
   InitEQep2Gpio();
   InitSciaGpio();
	// Clear all interrupts and initialize PIE vector table:
	//Disable the interrupts
	DINT;
	//Initiate PIE control registers to default settings
	InitPieCtrl();
	//Reset CPU interrupt enable and flag
	IER = 0x0000;
	IFR = 0x0000;
	//Initialize PIE vector table
	InitPieVectTable();
	//Set pointers to ISR function
	EALLOW;
	PieVectTable.EPWM1_INT=&PWM1_isr;
	PieVectTable.EPWM2_INT=&PWM2_isr;
	PieVectTable.EQEP1_INT=&Phase_error;
	PieVectTable.EQEP2_INT=&eQEP2_isr;
	EDIS;
	
	// only initialize the ePWM
	EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;
//------------------------------------------------------------------------------------------
//	ePWM
    PWMinitialize();
   
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
//------------------------------------------------------------------------------------------
//	ADC
    ADCinitialize();
    
    // Step 5. User specific code, enable interrupts:
// Enable CPU INT3 which is connected to EPWM1-3 INT:
   IER |= M_INT3;
   IER |= M_INT5;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;
   PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
   PieCtrlRegs.PIEIER5.bit.INTx1 = 1;
   PieCtrlRegs.PIEIER5.bit.INTx2 = 1;

   //Sets register values for output pins
   LEDinitialize();

   // Copy time critical code and Flash setup code to RAM
// This includes the following ISR functions: epwm1_timer_isr(), epwm2_timer_isr()
// epwm3_timer_isr and and InitFlash();
// The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
// symbols are created by the linker. Refer to the F28335.cmd file.
   MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);

// Call Flash Initialization to setup flash waitstates
// This function must reside in RAM
   InitFlash();

// Initialize Sci registers
   scia_init();
// Initialize the SCI FIFO
   scia_fifo_init();
  
// Enable global Interrupts and higher priority real-time debug events:
   EINT;   // Enable Global interrupt INTM
   ERTM;   // Enable Global realtime interrupt DBGM

//	qep_posspeed.init(&qep_posspeed);
	POSSPEED_Init();

// Start SEQ1
   AdcRegs.ADCTRL2.all = 0x2000;

   // Initalize the send data buffer
//      for(i=0; i<8; i++)
//      {
//         sdata[i] = i;
//      }
//      rdata_point = 0;
//===============================================================================
//Infinite for loop that sets the PWM outputs by comparing the encoder counts and 
//potentiometer position
   int xmit;
	for(;;){
		// Wait for inc character
		while(SciaRegs.SCIFFRX.bit.RXFFST !=1) { } // wait for XRDY =1 for empty state

		// Get character
		ReceivedChar = SciaRegs.SCIRXBUF.all;

		EQep1Regs.QPOSCMP = ((float)ReceivedChar/(float)255)*EQep_Max;

		xmit = ReceivedChar;
//		msg = "\r\n\0";
//		scia_msg(msg);

		scia_xmit(xmit);

//		msg = "\n\0";
//		scia_msg(msg);
   }
}

//Not used in this versions
void update_compare(EPWM_INFO *epwm_info)
{
   // Every 10'th interrupt, change the CMPA/CMPB values
   if(epwm_info->EPwmTimerIntCount == 1000)
   {
       epwm_info->EPwmTimerIntCount = 0;
       GpioDataRegs.GPATOGGLE.bit.GPIO30 = 1;

       // If we were increasing CMPA, check to see if
       // we reached the max value.  If not, increase CMPA
       // else, change directions and decrease CMPA
	   if(epwm_info->EPwm_CMPA_Direction == EPWM_CMP_UP)
	   {
	       if(epwm_info->EPwmRegHandle->CMPA.half.CMPA < epwm_info->EPwmMaxCMPA)
	       {
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
	       }
	       else
	       {
	          epwm_info->EPwm_CMPA_Direction = EPWM_CMP_DOWN;
              epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
	       }
	   }

	   // If we were decreasing CMPA, check to see if
       // we reached the min value.  If not, decrease CMPA
       // else, change directions and increase CMPA
	   else
	   {
	       if(epwm_info->EPwmRegHandle->CMPA.half.CMPA == epwm_info->EPwmMinCMPA)
	       {
	          epwm_info->EPwm_CMPA_Direction = EPWM_CMP_UP;
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA++;
	       }
	       else
	       {
	          epwm_info->EPwmRegHandle->CMPA.half.CMPA--;
	       }
	   }
   }
   else
   {
      epwm_info->EPwmTimerIntCount++;
   }

   return;
}

//Limits input to a certain value
Uint16 ceil(Uint16 value, Uint16 max){
	if(value>=max)
		return max;
	else
		return value;
}

void Speed_control(volatile Uint16 *pwm)
{
//	while(EQep1Regs.QPOSCNT > (EQep1Regs.QPOSCMP+DEADZONE) || EQep1Regs.QPOSCNT < (EQep1Regs.QPOSCMP-DEADZONE))
//		{
		// Position and Speed measurement
//		   qep_posspeed.calc(&qep_posspeed);
		// Regulating RPM speed of joint
			int abs_rpm = qep_posspeed.SpeedRpm_fr;
//			int direction = qep_posspeed.DirectionQep;
//			if((abs(EQep1Regs.QPOSCNT-(EQep1Regs.QPOSCMP+DEADZONE))> 1000))
//			{
				if(abs_rpm > Joint_Speed)
					{
						(*pwm)-=(abs_rpm-Joint_Speed)*rpm_pwm_cal;
					}
				if(abs_rpm < Joint_Speed)
					{
						(*pwm)+=(-abs_rpm+Joint_Speed)*rpm_pwm_cal;
						(*pwm)=ceil((*pwm),4096);
//						if (direction == 0)
//							EPwm1Regs.CMPA.half.CMPA += 20;
//						else if (direction == 1)
//							EPwm1Regs.CMPB += 20;
					}
//			}
			if((abs(EQep1Regs.QPOSCNT-(EQep1Regs.QPOSCMP+DEADZONE))< 1000))
			{
				(*pwm)-=500;
			}
//		}
}
