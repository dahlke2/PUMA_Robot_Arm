#ifndef _init_fxns_
#define _init_fxns_

#include "DSP28x_Project.h"

#define EPWM_CMP_DOWN 0
#define EPWM_CMP_UP 1
// Configure the period for each timer
#define EPWM1A_TIMER_TBPRD  4096  // Period register
#define EPWM1A_MAX_CMPA     4095
#define EPWM1A_MIN_CMPA       0
#define EPWM1B_TIMER_TBPRD  4096  // Period register
#define EPWM1B_MAX_CMPB     4095
#define EPWM1B_MIN_CMPB       0

// ADC start parameters
#define ADC_MODCLK 0x3 // HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
#define ADC_CKPS   0x0   // ADC module clock = HSPCLK/1      = 25.5MHz/(1)   = 25.0 MHz
#define ADC_SHCLK  0x1   // S/H width in ADC module periods                  = 2 ADC cycle
//#define AVG        1000  // Average sample limit
//#define ZOFFSET    0x00  // Average Zero offset
//#define BUF_SIZE   1024  // Sample buffer size

void PWMinitialize(void);
void LEDinitialize(void); 
void ADCinitialize(void);
void POSSPEED_Init(void);
void scia_init(void);

#define DEADZONE 5

#define EQep_Max 75000
#define EQep_Init 37500;

#endif /* _init_fxns_ */
