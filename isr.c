#include "PUMA_Robot_Arm.h"

//Not used in this version
interrupt void PWM1_isr(void){
volatile Uint16 *pwm_register;
//	potentiometer_compare_position();
	
	// Position and Speed measurement
   qep_posspeed.calc(&qep_posspeed);

	// Clear INT flag for the timer
    EPwm1Regs.ETCLR.bit.INT = 1;
    
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    
			if(EQep1Regs.QPOSCNT > (EQep1Regs.QPOSCMP+DEADZONE)){
				pwm_register = &EPwm1Regs.CMPA.half.CMPA;//= 3036;//ceil((base_speed+(EQep1Regs.QPOSCNT-EQep1Regs.QPOSCMP+DEADZONE)*speed_multiplier), 1024);
				EPwm1Regs.CMPB = 0;
				Speed_control(pwm_register);

			}
			else if(EQep1Regs.QPOSCNT < (EQep1Regs.QPOSCMP-DEADZONE)){
				EPwm1Regs.CMPA.half.CMPA = 0;
				pwm_register = &EPwm1Regs.CMPB;// = 3036;//ceil((base_speed+(EQep1Regs.QPOSCMP-DEADZONE-EQep1Regs.QPOSCNT)*speed_multiplier), 1024);
				Speed_control(pwm_register);
			}
			else{
				EPwm1Regs.CMPA.half.CMPA = 0;
				EPwm1Regs.CMPB = 0;
			}

}

interrupt void PWM2_isr(void){
//	potentiometer_compare_position();
	
	// Position and Speed measurement
//   qep_posspeed.calc(&qep_posspeed);

	// Clear INT flag for the timer
    EPwm2Regs.ETCLR.bit.INT = 1;
    
    // Acknowledge this interrupt to receive more interrupts from group 3
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    
			if(EQep2Regs.QPOSCNT > (EQep2Regs.QPOSCMP+DEADZONE)){
				EPwm2Regs.CMPA.half.CMPA = ceil((base_speed+(EQep2Regs.QPOSCNT-EQep2Regs.QPOSCMP+DEADZONE)*speed_multiplier),3072);
				EPwm2Regs.CMPB = 0;
			}
			else if(EQep2Regs.QPOSCNT < (EQep2Regs.QPOSCMP-DEADZONE)){
				EPwm2Regs.CMPA.half.CMPA = 0;
				EPwm2Regs.CMPB = ceil((base_speed+(EQep2Regs.QPOSCMP-DEADZONE-EQep2Regs.QPOSCNT)*speed_multiplier), 3072);
			}
			else{
				EPwm2Regs.CMPA.half.CMPA = 0;
				EPwm2Regs.CMPB = 0;
			}
}

//ISR when under/overflow
interrupt void eQEP1_isr(){
	//Clear eQEP gloabal interrupt and position-compare module interrupts
	EQep1Regs.QCLR.bit.INT = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
	//Turn GPIO30 on then off when the encoder count equals the potentiometer value
	GpioDataRegs.GPASET.bit.GPIO30 = 1;
	DELAY_US(500);
	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;

	if(EQep1Regs.QFLG.bit.PCO == 1){
		EQep1Regs.QCLR.bit.PCO = 1;
		EPwm1Regs.CMPA.half.CMPA = 1500;
		EPwm1Regs.CMPB = 0;
	}
	if(EQep1Regs.QFLG.bit.PCU == 1){
		EQep1Regs.QCLR.bit.PCU = 1;
		EPwm1Regs.CMPA.half.CMPA = 0;
		EPwm1Regs.CMPB = 1500;
	}

	DELAY_US(5000);
	// Position and Speed measurement
   qep_posspeed.calc(&qep_posspeed);
}

//ISR when under/overflow
interrupt void eQEP2_isr(){
	//Clear eQEP gloabal interrupt and position-compare module interrupts
	EQep2Regs.QCLR.bit.INT = 1;
	if(EQep2Regs.QFLG.bit.PCO == 1)
		EQep2Regs.QCLR.bit.PCO = 1;
	if(EQep2Regs.QFLG.bit.PCU == 1)
		EQep2Regs.QCLR.bit.PCU = 1;
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP5;
	//Turn GPIO30 on then off when the encoder count equals the potentiometer value
	GpioDataRegs.GPASET.bit.GPIO30 = 1;
	DELAY_US(50000);
	GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
	EPwm2Regs.CMPA.half.CMPA = 0;
	EPwm2Regs.CMPB = 0;
	// Position and Speed measurement
   qep_posspeed.calc(&qep_posspeed);
}

//interrupt void spiTxFifoIsr(void)
//{
// 	Uint16 i;
//    for(i=0;i<8;i++)
//    {
// 	   SpiaRegs.SPITXBUF=sdata[i];      // Send data
//    }
//
//    for(i=0;i<8;i++)                    // Increment data for next cycle
//    {
// 	   sdata[i]++;
//    }
//
//
//    SpiaRegs.SPIFFTX.bit.TXFFINTCLR=1;  // Clear Interrupt flag
//	PieCtrlRegs.PIEACK.all|=0x20;  		// Issue PIE ACK
//}
//
//interrupt void spiRxFifoIsr(void)
//{
//    Uint16 i;
//    for(i=0;i<8;i++)
//    {
//	    rdata[i]=SpiaRegs.SPIRXBUF;		// Read data
//	}
////	for(i=0;i<8;i++)                    // Check received data
////	{
////	    if(rdata[i] != rdata_point+i) error();
////	}
//	rdata_point++;
//	SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
//	SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1; 	// Clear Interrupt flag
//	PieCtrlRegs.PIEACK.all|=0x20;       // Issue PIE ack
//}

interrupt void Phase_error(void){
	DELAY_US(50);
}
