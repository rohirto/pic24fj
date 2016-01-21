/* 
 * File:   interrupts.h
 * Author: admin
 *
 * Created on 20 October, 2015, 1:43 PM
 */

#ifndef INTERRUPTS_H
#define	INTERRUPTS_H

void IntInit(void)
{
   INTCON2 = 0x0000;   /*Setup INT0, INT1, INT2, interupt on falling edge*/
   IFS1bits.INT1IF = 0;    /*Reset INT1 interrupt flag */
   IEC1bits.INT1IE = 1;    /*Enable INT1 Interrupt Service Routine */
   IPC5bits.INT1IP = 1;	/*set low priority*/
}
void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt(void)
{
    unsigned int i;
     for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
  LATDbits.LATD0=1;    //toggle through
  LATCbits.LATC13=1;
  for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
                LATDbits.LATD0=0;
                for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
  LATDbits.LATD0=1;
  LATCbits.LATC13=0;
   IFS1bits.INT1IF = 0;    //Clear the INT1 interrupt flag or else
   //the CPU will keep vectoring back to the ISR
}

//_INT1Interrupt() is the INT1 interrupt service routine (ISR).

#endif	/* INTERRUPTS_H */

