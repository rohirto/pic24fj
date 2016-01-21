/* 
 * File:   dec.h
 * Author: admin
 *
 * Created on 20 October, 2015, 1:44 PM
 */

#ifndef DEC_H
#define	DEC_H
#include <stdio.h>
#include<stdlib.h>
#include <p24FJ128GA106.h>
#include <xc.h>


/*
  PIC24FJ128GA106 Configuration Bit Settings

// 'C' source line config statements



// CONFIG3
#pragma config WPFP = WPFP511           // Write Protection Flash Page Segment Boundary (Highest Page (same as page 85))
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable bit (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Configuration Word Code Page Protection Select bit (Last page(at the top of program memory) and Flash configuration words are not protected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select bit (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = XT             // Primary Oscillator Select (XT oscillator mode selected)
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable bit (Write RP Registers Once)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSCO functions as CLKO (FOSC/2))
#pragma config FCKSM = CSECME           // Clock Switching and Monitor (Both Clock switching and Fail-safe Clock Monitor are enabled)
#pragma config FNOSC = FRCPLL           // Oscillator Select (Fast RC oscillator with Postscaler and PLL module (FRCPLL))
#pragma config IESO = ON                // Internal External Switch Over Mode (IESO mode (Two-speed start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer is enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx2               // Comm Channel Select (Emulator functions are shared with PGEC2/PGED2)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF              // JTAG Port Enable (JTAG port is enabled)
*/

_CONFIG1(0x1E7F);
_CONFIG2(0xF3FE);
_CONFIG3(0xFFFF);


#define CRYSTAL_FREQ                    (8000000)				 // < 11.0592 Mhz External Crystal. > 
#define SYSTEM_CLOCK                    ( CRYSTAL_FREQ * 4 )// < 4x PLL Enabled. >															
#define FCY                             ( SYSTEM_CLOCK / 2 )



void IntInit(void);
void __attribute__((__interrupt__)) _INT1Interrupt(void);
void init(void);
void clockswitch(void);
void delay(void);

#endif	/* DEC_H */

