/*
 * Serial_Comm.h
 *
 *  Created on: Feb 25, 2013
 *      Author: Ethan Dahlke
 */

#ifndef SERIAL_COMM_H_
#define SERIAL_COMM_H_

void scia_fifo_init(void);
void scia_xmit(int a);
void scia_msg(char *msg);

#endif /* SERIAL_COMM_H_ */
