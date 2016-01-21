/*
 * File:   main.c
 * Author: admin
 *
 * Created on 27 May, 2015, 12:40 PM
 */

#include"dec.h"
#include"interrupts.h"
#include"uart.h"
int main(void)
{
    unsigned int i,c;
    unsigned char e;
    init();
    IntInit();
    init_uart2();
   init_uart1();
    //clockswitch();
    IEC0=0x0001;
    write_uart2('r');
  write_uart1('r');
    while(1)
    {
        for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
                LATDbits.LATD8 = 1;
		LATDbits.LATD10 = 1;
		for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
		LATDbits.LATD8 = 0;
		LATDbits.LATD10 = 0;
       c = read_uart2();      
         write_uart2(c);
         //delay();
        // e = read_uart1();      
         write_uart1('m');
	}
}





void init(void)
{
    unsigned int i;
     // OSCCON = 0x2200;           /*clock swutching*/
    INTCON1 = 0x8000;
    INTCON2 = 0x0000;
   // IEC0 = 0x0001;

    IPC0= 0x0007;
    AD1PCFG	=	0xFFFF;	//set to all digital I/O
	TRISDbits.TRISD8 = 0;
	TRISDbits.TRISD10 = 0;
        TRISDbits.TRISD0=0;
        TRISCbits.TRISC13=0;
        TRISEbits.TRISE0 = 0;
        TRISEbits.TRISE1 = 0;
        TRISFbits.TRISF6 = 1;
        TRISGbits.TRISG7=0;
        TRISBbits.TRISB14=0;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        LATDbits.LATD0=1;
        LATCbits.LATC13=1;
        LATBbits.LATB14=1;
        
        LATGbits.LATG7=1;
        for( i = 0; i < 50000; i++);
		for( i = 0; i < 50000; i++);
         LATDbits.LATD0=0;
         LATCbits.LATC13=0;
         LATGbits.LATG7=0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        RPINR0	=	0x2D00;	//set RPI45 to external interrupt 1
}

void clockswitch(void)
{
    int pllCounter;
    //Write new "value" to OSCCONH to
// set the new oscillator selection
__builtin_write_OSCCONH(0x78);  // unlock sequence
__builtin_write_OSCCONH(0x9a);
OSCCONbits.NOSC=3;  //primary oscillator with pll
__builtin_write_OSCCONL(0x46);
__builtin_write_OSCCONL(0x57);
OSCCONbits.OSWEN=1; // initiate clock switch

for (pllCounter = 0; pllCounter < 600; pllCounter++);  // waiting for pll to stabilize

while(OSCCONbits.OSWEN!=0);         //wait for the switch over
}
void init_uart2(void){
    //pin mapping
    UART_2_TX_PIN_SELECT;
    UART_2_RX_PIN_SELECT;
    
    //pin directions
    UART_2_TX_PIN_DIR;
    UART_2_RX_PIN_DIR;
    DIRECTION_FOR_485;
    
    U2MODE = (UART_EN) |							// Enable UART Module
			 (UART_IDLE_CON)|						// Work in IDLE mode 
			 (UART_IrDA_DISABLE)|
              (UART_EVEN_PAR_8BIT);	

 
	U2STA =  (UART_INT_TX_EACH_CHAR)|				// Interrupt on transfer of every character to TSR
			 (UART_INT_RX_CHAR)|					// Interrupt on every char receive
			 (UART_TX_ENABLE);						// Transmit enable


	U2BRG = ((FCY/UART_2_BAUDRATE)/16) - 1;				// Baudrate = 9600

	EnableIntU2RX;									// UART 2 Receive Interrupt Enable
	EnableIntU2TX;									// UART 2 Transmit Interrupt Enable
	SetPriorityIntU2RX(3);					// Set priority of Receive interrupt
	SetPriorityIntU2TX(3);					// Set priority of Transmit interrupt

	U2RX_Clear_Intr_Status_Bit;						// Cleat status bits
	U2TX_Clear_Intr_Status_Bit;

    
}
void init_uart1(void){
    //pin mapping
    UART_1_TX_PIN_SELECT;
    UART_1_RX_PIN_SELECT;
    
    //pin directions
    UART_1_TX_PIN_DIR;
    UART_1_RX_PIN_DIR;
    //DIRECTION_FOR_485;
    
    U1MODE = (UART_EN) |							// Enable UART Module
			 (UART_IDLE_CON)|						// Work in IDLE mode 
			 (UART_IrDA_DISABLE)|
              (UART_EVEN_PAR_8BIT);	

 
	U1STA =  (UART_INT_TX_EACH_CHAR)|				// Interrupt on transfer of every character to TSR
			 (UART_INT_RX_CHAR)|					// Interrupt on every char receive
			 (UART_TX_ENABLE);						// Transmit enable


	U1BRG = ((FCY/UART_1_BAUDRATE)/16) - 1;				// Baudrate = 9600

	EnableIntU1RX;									// UART 2 Receive Interrupt Enable
	EnableIntU1TX;									// UART 2 Transmit Interrupt Enable
	SetPriorityIntU1RX(3);					// Set priority of Receive interrupt
	SetPriorityIntU1TX(3);					// Set priority of Transmit interrupt

	U1RX_Clear_Intr_Status_Bit;						// Cleat status bits
	U1TX_Clear_Intr_Status_Bit;

    
}

void write_uart2(int c){
    while(U2STAbits.UTXBF);
    START_SENDING_ON_485;
    U2TXREG  = c;
    delay();
    
        
}
void write_uart1(int c){
    while(U1STAbits.UTXBF);
    //START_SENDING_ON_485;
    U1TXREG  = c;
    delay();
    
        
}

int read_uart2(void)
{
    STOP_SENDING_ON_485;
    while(!U2STAbits.URXDA);
    return U2RXREG;
}
char read_uart1(void)
{
   // STOP_SENDING_ON_485;
    while(!U1STAbits.URXDA);
    return U1RXREG;
    delay();
}
void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void)
{
    U2RX_Clear_Intr_Status_Bit;
}

void __attribute__((interrupt,no_auto_psv)) _U2TXInterrupt(void)
{
    U2TX_Clear_Intr_Status_Bit; 	// Clear Status bit.
}
void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
    U1RX_Clear_Intr_Status_Bit;
}

void __attribute__((interrupt,no_auto_psv)) _U1TXInterrupt(void)
{
    U1TX_Clear_Intr_Status_Bit; 	// Clear Status bit.
}
void delay(void)
{
    int h;
    for(h=0;h<50;h++);
    for(h=0;h<50;h++);
    for(h=0;h<50;h++);
}
