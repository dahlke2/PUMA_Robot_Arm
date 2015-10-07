#ifndef ISR_H_
#define ISR_H_

interrupt void PWM1_isr(void);
interrupt void PWM2_isr(void);
interrupt void eQEP1_isr(void);
interrupt void eQEP2_isr(void);
interrupt void spiTxFifoIsr(void);
interrupt void spiRxFifoIsr(void);
interrupt void Phase_error(void);

#define base_speed 2500
#define speed_multiplier .1

extern Uint16 sdata[8];     // Send data buffer
extern Uint16 rdata[8];     // Receive data buffer
extern Uint16 rdata_point;  // Keep track of where we are
                     // in the data stream to check received data
#endif /*ISR_H_*/
