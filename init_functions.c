#include "PUMA_Robot_Arm.h"

void ADCinitialize(void)
{
	// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals(); // Not required for this example
   InitAdc();         // For this example, init the ADC

// Specific ADC setup for this example:
   AdcRegs.ADCTRL1.bit.ACQ_PS = ADC_SHCLK;  // Sequential mode: Sample rate   = 1/[(2+ACQ_PS)*ADC clock in ns]
                        //                     = 1/(3*40ns) =8.3MHz (for 150 MHz SYSCLKOUT)
					    //                     = 1/(3*80ns) =4.17MHz (for 100 MHz SYSCLKOUT)
					    // If Simultaneous mode enabled: Sample rate = 1/[(3+ACQ_PS)*ADC clock in ns]
   AdcRegs.ADCTRL3.bit.ADCCLKPS = ADC_CKPS;
   AdcRegs.ADCTRL1.bit.SEQ_CASC = 1;        // 1  Cascaded mode
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 = 0x0;
   AdcRegs.ADCTRL1.bit.CONT_RUN = 1;       // Setup continuous run

   AdcRegs.ADCTRL1.bit.SEQ_OVRD = 1;       // Enable Sequencer override feature
   AdcRegs.ADCCHSELSEQ1.all = 0x0;
   AdcRegs.ADCCHSELSEQ1.bit.CONV00 |= 0x0;
   AdcRegs.ADCCHSELSEQ1.bit.CONV01 |= 0x4;
   AdcRegs.ADCCHSELSEQ2.all = 0x0;
   AdcRegs.ADCCHSELSEQ3.all = 0x0;
   AdcRegs.ADCCHSELSEQ4.all = 0x0;
   AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0x7;  // convert and store in 8 results registers
    
}

void PWMinitialize(){
//==========================================================================
//Set up ePWM1	
	// Setup TBCLK
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm1Regs.TBPRD = EPWM1A_TIMER_TBPRD;       // Set timer period
   EPwm1Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   // Setup shadow register load on ZERO
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set Compare values
   EPwm1Regs.CMPA.half.CMPA = EPWM1A_MIN_CMPA;    // Set compare A value
   EPwm1Regs.CMPB = EPWM1B_MIN_CMPB;    // Set compare B value
  
   // Set actions
   EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A high on Zero
   EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count
   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
   EPwm1Regs.AQCTLA.bit.PRD = 1	;			     // Set PWM1A high when CTR=PRD
   EPwm1Regs.AQCTLB.bit.PRD = 1	;

   // Interrupt where we will change the Compare Values
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
   
//=============================================================================
//Set up ePWM2
// Setup TBCLK
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
   EPwm2Regs.TBPRD = EPWM1A_TIMER_TBPRD;       // Set timer period
   EPwm2Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;       // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV2;   // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV2;

   // Setup shadow register load on ZERO
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

   // Set Compare values
   EPwm2Regs.CMPA.half.CMPA = EPWM1A_MIN_CMPA;    // Set compare A value
   EPwm2Regs.CMPB = EPWM1B_MIN_CMPB;    // Set compare B value
  
   // Set actions
   EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;            // Set PWM1A high on Zero
   EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;
   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;          // Clear PWM1A on event A, up count
   EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
   EPwm2Regs.AQCTLA.bit.PRD = 1	;			     // Set PWM1A high when CTR=PRD
   EPwm2Regs.AQCTLB.bit.PRD = 1	;
   
   // Interrupt where we will change the Compare Values
   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
   EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
   EPwm2Regs.ETPS.bit.INTPRD = ET_3RD;           // Generate INT on 3rd event
   
//   // Information this example uses to keep track
//   // of the direction the CMPA/CMPB values are
//   // moving, the min and max allowed values and
//   // a pointer to the correct ePWM registers
//   epwm1_info.EPwm_CMPA_Direction = 1; // Start by increasing CMPA & CMPB
//   epwm1_info.EPwmTimerIntCount = 0;             // Zero the interrupt counter
//   epwm1_info.EPwmRegHandle = &EPwm1Regs;        // Set the pointer to the ePWM module
//   epwm1_info.EPwmMaxCMPA = EPWM1A_MAX_CMPA;      // Setup min/max CMPA/CMPB values
//   epwm1_info.EPwmMinCMPA = EPWM1A_MIN_CMPA;
	
}

void LEDinitialize(void){
	// Configure GPIO30 as a GPIO output pin
   EALLOW;
   GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
   GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;
   EDIS;
   GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
   
//   /* Enable internal pull-up for the selected pins */
//// Pull-ups can be enabled or disabled by the user. 
//// This will enable the pullups for the specified pins.
//// Comment out other unwanted lines.
//
//    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
//    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
//    
//    /* Configure ePWM-1 pins using GPIO regs*/
//// This specifies which of the possible GPIO pins will be ePWM1 functional pins.
//// Comment out other unwanted lines.
//
//    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;
}

void  POSSPEED_Init(void)
{

    #if (CPU_FRQ_150MHZ)
      EQep1Regs.QUPRD=15000000;
	  EQep2Regs.QUPRD=15000000;			// Unit Timer for 100Hz at 150 MHz SYSCLKOUT
	#endif
    #if (CPU_FRQ_100MHZ)
	  EQep2Regs.QUPRD=1000000;
	  EQep2Regs.QUPRD=1000000;			// Unit Timer for 100Hz at 100 MHz SYSCLKOUT
	#endif	
	
	EQep1Regs.QDECCTL.bit.QSRC=00;		// QEP quadrature count mode
		
	EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
	EQep1Regs.QEPCTL.bit.PCRM=00;		// PCRM=00 mode - QPOSCNT reset on index event
	EQep1Regs.QEPCTL.bit.UTE=1; 		// Unit Timeout Enable 
	EQep1Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
	EQep1Regs.QPOSMAX=0xffffffff;
	EQep1Regs.QEPCTL.bit.QPEN=1; 		// QEP enable
		
	EQep1Regs.QCAPCTL.bit.UPPS=6;   	// 1/32 for unit position
	EQep1Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock
	EQep1Regs.QCAPCTL.bit.CEN=1; 		// QEP Capture Enable
	
	EQep1Regs.QPOSCTL.bit.PCE = 1;
	EQep1Regs.QEINT.bit.PCO = 1;
	EQep1Regs.QEINT.bit.PCU = 1;
	EQep1Regs.QPOSCMP = EQep_Init;
	EQep1Regs.QPOSMAX = EQep_Max;	
	EQep1Regs.QPOSINIT = EQep_Init;
	EQep1Regs.QEPCTL.bit.SWI = 1;

	EQep1Regs.QFLG.bit.PHE = 1;
//=====================================================================================
	EQep2Regs.QDECCTL.bit.QSRC=00;		// QEP quadrature count mode
		
	EQep2Regs.QEPCTL.bit.FREE_SOFT=2;
	EQep2Regs.QEPCTL.bit.PCRM=00;		// PCRM=00 mode - QPOSCNT reset on index event
	EQep2Regs.QEPCTL.bit.UTE=1; 		// Unit Timeout Enable 
	EQep2Regs.QEPCTL.bit.QCLM=1; 		// Latch on unit time out
	EQep2Regs.QPOSMAX=0xffffffff;
	EQep2Regs.QEPCTL.bit.QPEN=1; 		// QEP enable
		
	EQep2Regs.QCAPCTL.bit.UPPS=5;   	// 1/32 for unit position
	EQep2Regs.QCAPCTL.bit.CCPS=7;		// 1/128 for CAP clock
	EQep2Regs.QCAPCTL.bit.CEN=1; 		// QEP Capture Enable
	
	EQep2Regs.QPOSCTL.bit.PCE = 1;
	EQep2Regs.QEINT.bit.PCO = 1;
	EQep2Regs.QEINT.bit.PCU = 1;
	EQep2Regs.QPOSCMP = EQep_Init;
	EQep2Regs.QPOSMAX = EQep_Max;	
	EQep2Regs.QPOSINIT = EQep_Init;
	EQep2Regs.QEPCTL.bit.SWI = 1;
}

void scia_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

 	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.all =0x0003;
	SciaRegs.SCICTL2.bit.TXINTENA =1;
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;
	#if (CPU_FRQ_150MHZ)
	      SciaRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 37.5MHz.
	      SciaRegs.SCILBAUD    =0x00E7;
	#endif
	#if (CPU_FRQ_100MHZ)
      SciaRegs.SCIHBAUD    =0x0001;  // 9600 baud @LSPCLK = 20MHz.
      SciaRegs.SCILBAUD    =0x0044;
	#endif
	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

