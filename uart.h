/* 
 * File:   uart.h
 * Author: admin
 *
 * Created on 22 January, 2016, 12:52 PM
 */

#ifndef UART_H
#define	UART_H

/*******************************************************************************************************
	define section
********************************************************************************************************/
#define RP21			21
#define U3TX			28

#define RP23			23
#define U2TX			 5

#define RPI32			32
#define U1TX			 3

#define RP27			27
#define U4TX			30

#define RPI10			10
#define RPI16			16

#define UART_1_TX_PIN_SELECT		(RPOR15bits.RP30R = U1TX)
#define UART_1_RX_PIN_SELECT		(RPINR18bits.U1RXR = RPI16)
#define UART_1_TX_PIN_DIR			(TRISFbits.TRISF2 = 0)
#define UART_1_RX_PIN_DIR			(TRISFbits.TRISF3 = 1 )

#define UART_2_TX_PIN_SELECT		(RPOR14bits.RP29R = U2TX)
#define UART_2_RX_PIN_SELECT		(RPINR19bits.U2RXR = RPI10)
#define UART_2_TX_PIN_DIR			(TRISBbits.TRISB15 = 0)
#define UART_2_RX_PIN_DIR			(TRISFbits.TRISF4 = 1 )

/************************************  UART     ****************************************************/
#define UART_1_BAUDRATE 				19200
#define UART_2_BAUDRATE 				9600

// **************** This part of code is copied from Microchip's header file ****************************/

/* defines for UxMODE register   (ORing based) */
#define UART_EN                 0x8000 							/* Module enable */
#define UART_DIS                0x0000 							/* Module disable */
#define UART_EN_DIS_MASK        (~UART_EN)

#define UART_IDLE_STOP          0x2000							/* Stop all functions in IDLE mode*/
#define UART_IDLE_CON           0x0000 							/* Work in IDLE mode */
#define UART_IDLE_MASK          (~UART_IDLE_STOP)

#define UART_IrDA_ENABLE        0x1000 							/* IrDA encoder and decoder enabled*/
#define UART_IrDA_DISABLE       0x0000							/* IrDA encoder and decoder disabled */
#define UART_IrDA_MASK          (~UART_IrDA_ENABLE)

#define UART_MODE_SIMPLEX       0x0800 							/* UxRTS pin in Simplex mode */
#define UART_MODE_FLOW          0x0000 							/* UxRTS pin in Flow Control mode*/
#define UART_MODE_MASK          (~UART_MODE_SIMPLEX)

#define UART_UEN_11             0x0300 							/*UxTX,UxRX and BCLK pins are enabled and used; UxCTS pin controlled by port latches*/
#define UART_UEN_10             0x0200 							/*UxTX,UxRX, UxCTS and UxRTS pins are enabled and used*/
#define UART_UEN_01             0x0100 							/*UxTX,UxRX and UxRTS pins are enabled and used; UxCTS pin controlled by port latches*/
#define UART_UEN_00             0x0000 							/*UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLK pins controlled by port latches*/
#define UART_UEN_MASK           (~UART_UEN_11)

#define UART_EN_WAKE            0x0080 							/*Enable Wake-up on START bit Detect during SLEEP Mode bit*/
#define UART_DIS_WAKE           0x0000							/*Disable Wake-up on START bit Detect during SLEEP Mode bit*/
#define UART_WAKE_MASK          (~UART_EN_WAKE)

#define UART_EN_LOOPBACK        0x0040 							/*Loop back enabled*/
#define UART_DIS_LOOPBACK       0x0000 							/*Loop back disabled*/
#define UART_LOOPBACK_MASK      (~UART_DIS_LOOPBACK)

#define UART_EN_ABAUD           0x0020 							/*Enable baud rate measurement on the next character*/
#define UART_DIS_ABAUD          0x0000 							/*Baud rate measurement disabled or completed*/
#define UART_ABAUD_MASK         (~UART_EN_ABAUD)

#define UART_UXRX_IDLE_ZERO     0x0010 							/* UxRX Idle state is zero */
#define UART_UXRX_IDLE_ONE      0x0000 							/* UxRx Idle state is one */
#define UART_UXRX_IDLE_MASK     (~UART_UXRX_IDLE_ZERO)

#define UART_BRGH_FOUR          0x0008							/* BRG generates 4 clocks per bit period */
#define UART_BRGH_SIXTEEN       0x0000							/* BRG generates 16 clocks per bit period */
#define UART_BRGH_MASK          (~UART_BRGH_FOUR)

#define UART_NO_PAR_9BIT        0x0006							/*No parity 9 bit*/
#define UART_ODD_PAR_8BIT       0x0004 							/*odd parity 8 bit*/
#define UART_EVEN_PAR_8BIT      0x0002							/*even parity 8 bit*/
#define UART_NO_PAR_8BIT        0x0000 							/*no parity 8 bit*/
#define UART_PARITY_MASK        (~UART_NO_PAR_9BIT)

#define UART_2STOPBITS          0x0001							/*2 stop bits*/
#define UART_1STOPBIT           0x0000 							/*1 stop bit*/
#define UART_STOPBIT_MASK       (~UART_2STOPBITS)

/* defines for UART Status register */
#define UART_INT_TX_BUF_EMPTY   0x8000							/* Interrupt on TXBUF becoming empty */
#define UART_INT_TX_LAST_CH     0x2000							/* Interrupt when last character shifted out*/
#define UART_INT_TX_EACH_CHAR   0x0000							/* Interrupt on transfer of every character to TSR */
#define UART_INT_TX_MASK        (~(UART_INT_TX_BUF_EMPTY | UART_INT_TX_LAST_CH))

#define UART_IrDA_POL_INV_ONE   0x4000							/*IrDA encoded, UxTX Idle state is '1' */
#define UART_IrDA_POL_INV_ZERO  0x0000							/* IrDA encoded, UxTX Idel state is '0' */
#define UART_IrDA_POL_INV_MASK  (~UART_IrDA_POL_INV_ONE)

#define UART_SYNC_BREAK_ENABLED   0x0800  						/* Send sync break on next transmission */
#define UART_SYNC_BREAK_DISABLED  0x0000  						/* Sync break transmission disabled or completed */
#define UART_SYNC_BREAK_MASK      (~UART_SYNC_BREAK_ENABLED)

#define UART_TX_ENABLE           0x0400  						/* Transmit enable */
#define UART_TX_DISABLE          0x0000  						/* Transmit disable */
#define UART_TX_MASK             (~UART_TX_ENABLE)

#define UART_INT_RX_BUF_FUL     0x00C0 							/* Interrupt on RXBUF full */
#define UART_INT_RX_3_4_FUL     0x0080 							/* Interrupt on RXBUF 3/4 full */
#define UART_INT_RX_CHAR        0x0000 							/* Interrupt on every char received */
#define UART_INT_RX_MASK        (~UART_INT_RX_BUF_FUL)

#define UART_ADR_DETECT_EN      0x0020							/* address detect enable */
#define UART_ADR_DETECT_DIS     0x0000							/* address detect disable */
#define UART_ADR_DETECT_MASK    (~UART_ADR_DETECT_EN)

#define UART_RX_OVERRUN_CLEAR    0x0000 						/* Rx buffer Over run status bit clear */

/* defines for UART Interrupt configuartion */
#define UART_RX_INT_EN          0x0008							/*Receive interrupt enabled*/
#define UART_RX_INT_DIS         0x0000							/*Receive interrupt disabled*/
#define UART_RX_INT_MASK        (~UART_RX_INT_EN)

#define UART_RX_INT_PR0         0x0000 						/*Priority RX interrupt 0*/
#define UART_RX_INT_PR1         0x0001 						/*Priority RX interrupt 1*/
#define UART_RX_INT_PR2         0x0002 						/*Priority RX interrupt 2*/
#define UART_RX_INT_PR3         0x0003 						/*Priority RX interrupt 3*/
#define UART_RX_INT_PR4         0x0004 						/*Priority RX interrupt 4*/
#define UART_RX_INT_PR5         0x0005 						/*Priority RX interrupt 5*/
#define UART_RX_INT_PR6         0x0006 						/*Priority RX interrupt 6*/
#define UART_RX_INT_PR7         0x0007 						/*Priority RX interrupt 7*/
#define UART_RX_INT_PR_MASK     (~UART_RX_INT_PR7)

#define UART_TX_INT_EN          0x0080 						/*transmit interrupt enabled*/
#define UART_TX_INT_DIS         0x0000						/*transmit interrupt disabled*/
#define UART_TX_INT_MASK        (~UART_TX_INT_EN)

#define UART_TX_INT_PR0         0x0000 						/*Priority TX interrupt 0*/
#define UART_TX_INT_PR1         0x0010						/*Priority TX interrupt 1*/
#define UART_TX_INT_PR2         0x0020 						/*Priority TX interrupt 2*/
#define UART_TX_INT_PR3         0x0030						/*Priority TX interrupt 3*/
#define UART_TX_INT_PR4         0x0040						/*Priority TX interrupt 4*/
#define UART_TX_INT_PR5         0x0050 						/*Priority TX interrupt 5*/
#define UART_TX_INT_PR6         0x0060						/*Priority TX interrupt 6*/
#define UART_TX_INT_PR7         0x0070 						/*Priority TX interrupt 7*/
#define UART_TX_INT_PR_MASK     (~UART_TX_INT_PR7)


/********************************************************************************
Macro       : EnableIntU1RX

Include     : uart.h

Description : This macro sets UART Receive Interrupt

Arguments   : None

Remarks     : This macro sets UART Receive Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define EnableIntU1RX                    asm("BSET IEC0,#11")

/********************************************************************************
Macro       : EnableIntU1TX

Include     : uart.h

Description : This macro sets UART Transmit Interrupt

Arguments   : None

Remarks     : This macro sets UART Transmit Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define EnableIntU1TX                    asm("BSET IEC0,#12")

/********************************************************************************
Macro       : DisableIntU1RX

Include     : uart.h

Description : This macro disables the UART Receive Interrupt

Arguments   : None

Remarks     : This macro clears UART Receive Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define DisableIntU1RX                   asm("BCLR IEC0,#11")

/********************************************************************************
Macro       : DisableIntU1TX

Include     : uart.h

Description : This macro disables the UART Transmit Interrupt

Arguments   : None

Remarks     : This macro clears UART Transmit Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define DisableIntU1TX                   asm("BCLR IEC0,#12")

/********************************************************************************
Macro       : SetPriorityIntU1RX

Include     : uart.h

Description : This macro sets priority for UART receive interrupt.

Arguments   : priority - This input parameter is the level of interrupt priority

Remarks     : This macro sets UART Receive Interrupt Priority bits of Interrupt
              Priority Control register.
*********************************************************************************/
#define SetPriorityIntU1RX(priority)     (IPC2bits.U1RXIP = priority)

/********************************************************************************
Macro       : SetPriorityIntU1TX

Include     : uart.h

Description : This macro sets priority for UART transmit interrupt.

Arguments   : priority - This input parameter is the level of interrupt priority

Remarks     : This macro sets UART Transmit Interrupt Priority bits of Interrupt
              Priority Control register.
*********************************************************************************/
#define SetPriorityIntU1TX(priority)     (IPC3bits.U1TXIP = priority)

/*******************************************************************
Macro       : U1RX_Clear_Intr_Status_Bit
Include     : uart.h
Description : Macro to Clear external Interrupt Status bit
Arguments   : None
Remarks     : None
*******************************************************************/
#define U1RX_Clear_Intr_Status_Bit     (IFS0bits.U1RXIF = 0)

/*******************************************************************
Macro       : U1TX_Clear_Intr_Status_Bit
Include     : uart.h
Description : Macro to Clear external Interrupt Status bit
Arguments   : None
Remarks     : None
*******************************************************************/
#define U1TX_Clear_Intr_Status_Bit     (IFS0bits.U1TXIF = 0)

/********************************************************************************/

// UART 2

/********************************************************************************
Macro       : EnableIntU2RX

Include     : uart.h

Description : This macro sets UART Receive Interrupt

Arguments   : None

Remarks     : This macro sets UART Receive Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define EnableIntU2RX                    asm("BSET IEC1,#14")

/********************************************************************************
Macro       : EnableIntU2TX

Include     : uart.h

Description : This macro sets UART Transmit Interrupt

Arguments   : None

Remarks     : This macro sets UART Transmit Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define EnableIntU2TX                    asm("BSET IEC1,#15")

/********************************************************************************
Macro       : DisableIntU2RX

Include     : uart.h

Description : This macro disables the UART Receive Interrupt

Arguments   : None

Remarks     : This macro clears UART Receive Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define DisableIntU2RX                   asm("BCLR IEC1,#14")

/********************************************************************************
Macro       : DisableIntU2TX

Include     : uart.h

Description : This macro disables the UART Transmit Interrupt

Arguments   : None

Remarks     : This macro clears UART Transmit Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define DisableIntU2TX                   asm("BCLR IEC1,#15")

/********************************************************************************
Macro       : SetPriorityIntU2RX

Include     : uart.h

Description : This macro sets priority for UART receive interrupt.

Arguments   : priority - This input parameter is the level of interrupt priority

Remarks     : This macro sets UART Receive Interrupt Priority bits of Interrupt
              Priority Control register.
*********************************************************************************/
#define SetPriorityIntU2RX(priority)     (IPC7bits.U2RXIP = priority)

/********************************************************************************
Macro       : SetPriorityIntU2TX

Include     : uart.h

Description : This macro sets priority for UART transmit interrupt.

Arguments   : priority - This input parameter is the level of interrupt priority

Remarks     : This macro sets UART Transmit Interrupt Priority bits of Interrupt
              Priority Control register.
*********************************************************************************/
#define SetPriorityIntU2TX(priority)     (IPC7bits.U2TXIP = priority)

/*******************************************************************
Macro       : U2RX_Clear_Intr_Status_Bit
Include     : uart.h
Description : Macro to Clear external Interrupt Status bit
Arguments   : None
Remarks     : None
*******************************************************************/
#define U2RX_Clear_Intr_Status_Bit     (IFS1bits.U2RXIF = 0)

/*******************************************************************
Macro       : U2TX_Clear_Intr_Status_Bit
Include     : uart.h
Description : Macro to Clear external Interrupt Status bit
Arguments   : None
Remarks     : None
*******************************************************************/
#define U2TX_Clear_Intr_Status_Bit     (IFS1bits.U2TXIF = 0)
/*******************************************************************/



// UART 3

/********************************************************************************
Macro       : EnableIntU3RX

Include     : uart.h

Description : This macro sets UART Receive Interrupt

Arguments   : None

Remarks     : This macro sets UART Receive Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define EnableIntU3RX                    asm("BSET IEC5,#2")

/********************************************************************************
Macro       : EnableIntU3TX

Include     : uart.h

Description : This macro sets UART Transmit Interrupt

Arguments   : None

Remarks     : This macro sets UART Transmit Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define EnableIntU3TX                    asm("BSET IEC5,#3")

/********************************************************************************
Macro       : DisableIntU3RX

Include     : uart.h

Description : This macro disables the UART Receive Interrupt

Arguments   : None

Remarks     : This macro clears UART Receive Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define DisableIntU3RX                   asm("BCLR IEC5,#2")

/********************************************************************************
Macro       : DisableIntU3TX

Include     : uart.h

Description : This macro disables the UART Transmit Interrupt

Arguments   : None

Remarks     : This macro clears UART Transmit Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define DisableIntU3TX                   asm("BCLR IEC5,#3")

/********************************************************************************
Macro       : SetPriorityIntU3RX

Include     : uart.h

Description : This macro sets priority for UART receive interrupt.

Arguments   : priority - This input parameter is the level of interrupt priority

Remarks     : This macro sets UART Receive Interrupt Priority bits of Interrupt
              Priority Control register.
*********************************************************************************/
#define SetPriorityIntU3RX(priority)     (IPC20bits.U3RXIP = priority)

/********************************************************************************
Macro       : SetPriorityIntU3TX

Include     : uart.h

Description : This macro sets priority for UART transmit interrupt.

Arguments   : priority - This input parameter is the level of interrupt priority

Remarks     : This macro sets UART Transmit Interrupt Priority bits of Interrupt
              Priority Control register.
*********************************************************************************/
#define SetPriorityIntU3TX(priority)     (IPC20bits.U3TXIP = priority)

/*******************************************************************
Macro       : U3RX_Clear_Intr_Status_Bit
Include     : uart.h
Description : Macro to Clear external Interrupt Status bit
Arguments   : None
Remarks     : None
*******************************************************************/
#define U3RX_Clear_Intr_Status_Bit     (IFS5bits.U3RXIF = 0)

/*******************************************************************
Macro       : U3TX_Clear_Intr_Status_Bit
Include     : uart.h
Description : Macro to Clear external Interrupt Status bit
Arguments   : None
Remarks     : None
*******************************************************************/
#define U3TX_Clear_Intr_Status_Bit     (IFS5bits.U3TXIF = 0)

// UART 4
/********************************************************************************
Macro       : EnableIntU4RX

Include     : uart.h

Description : This macro sets UART Receive Interrupt

Arguments   : None

Remarks     : This macro sets UART Receive Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define EnableIntU4RX                    asm("BSET IEC5,#8")

/********************************************************************************
Macro       : EnableIntU4TX

Include     : uart.h

Description : This macro sets UART Transmit Interrupt

Arguments   : None

Remarks     : This macro sets UART Transmit Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define EnableIntU4TX                    asm("BSET IEC5,#9")

/********************************************************************************
Macro       : DisableIntU4RX

Include     : uart.h

Description : This macro disables the UART Receive Interrupt

Arguments   : None

Remarks     : This macro clears UART Receive Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define DisableIntU4RX                   asm("BCLR IEC5,#8")
/********************************************************************************
Macro       : DisableIntU4TX

Include     : uart.h

Description : This macro disables the UART Transmit Interrupt

Arguments   : None

Remarks     : This macro clears UART Transmit Interrupt Enable bit of Interrupt
              Enable Control register
*********************************************************************************/
#define DisableIntU4TX                   asm("BCLR IEC5,#9")
/********************************************************************************
Macro       : SetPriorityIntU4RX

Include     : uart.h

Description : This macro sets priority for UART receive interrupt.

Arguments   : priority - This input parameter is the level of interrupt priority

Remarks     : This macro sets UART Receive Interrupt Priority bits of Interrupt
              Priority Control register.
*********************************************************************************/
#define SetPriorityIntU4RX(priority)     (IPC22bits.U4RXIP = priority)
/********************************************************************************
Macro       : SetPriorityIntU4TX

Include     : uart.h

Description : This macro sets priority for UART transmit interrupt.

Arguments   : priority - This input parameter is the level of interrupt priority

Remarks     : This macro sets UART Transmit Interrupt Priority bits of Interrupt
              Priority Control register.
*********************************************************************************/
#define SetPriorityIntU4TX(priority)     (IPC22bits.U4TXIP = priority)
/*******************************************************************
Macro       : U4RX_Clear_Intr_Status_Bit
Include     : uart.h
Description : Macro to Clear external Interrupt Status bit
Arguments   : None
Remarks     : None
*******************************************************************/
#define U4RX_Clear_Intr_Status_Bit     (IFS5bits.U4RXIF = 0)

/*******************************************************************
Macro       : U4TX_Clear_Intr_Status_Bit
Include     : uart.h
Description : Macro to Clear external Interrupt Status bit
Arguments   : None
Remarks     : None
*******************************************************************/
#define U4TX_Clear_Intr_Status_Bit     (IFS5bits.U4TXIF = 0)

/******************************************************************************************************
 variable declaration section
********************************************************************************************************/

// ODU slave , IDU
#define STRING_LENGTH_FOR_DATA				71
#define STRING_LENGTH_FOR_NACK				24
#define STRING_LENGTH_FOR_AUTO_ADDR_DATA 	17
#define FRAME_COUNT_FOR_IDU					21
#define FRAME_COUNT_FOR_AUTO_ADDR			12
#define FRAME_COUNT_FOR_Commintioning 		27
#define FRAME_COUNT_FOR_Setting				63

#define DIRECTION_FOR_485		 (TRISFbits.TRISF5 = 0)
#define START_SENDING_ON_485	 (LATFbits.LATF5 = 1)
#define STOP_SENDING_ON_485		 (LATFbits.LATF5 = 0)

#define VERSION_0			'0'
#define VERSION_1			'B'
//for testing auto addr
#define	D0						LATDbits.LATD7
#define	D1						LATDbits.LATD6
#define	D2						LATDbits.LATD5
#define	D3						LATDbits.LATD4
#define	D4						LATDbits.LATD3

// Uart Device Status
#define TRANSMIT                            (1)
#define RECEIVE                             (2)

#define STRING_LENGTH_FOR_REMOTE_NACK		(9)
#define END_OF_DATA                         '#'
#define STAR                                '*'
#define LOOP_DATA                           (80)
#define WIRED_REMOTE_DATA_SIZE              (50)



/********************************   UART  *************************************************************/
typedef struct
{
	unsigned char Tx_bus_free;
	unsigned char Rx_bus_free;
	unsigned char device_name;
	unsigned char device_status;
	unsigned char data_frame_received;
	unsigned char Setting_Frame_Received;
	unsigned char start_comm;
	unsigned char Configuration_Frame_Recived;
    unsigned char timeout_occured;
    unsigned int  Tx_byte;
	unsigned int  Rx_byte;
    unsigned int TxBuffSize;				//total number of bytes to be transmitted on uart
	unsigned int RxBuffSize;				// Total number of bytes to br received on Uart.
    int 		 timeout_count;

    unsigned char *RxBufAdd; 				//receive data buffer address pointer
	unsigned char *TxBufAdd;				//transmit data buffer address pointer
	unsigned char *RXBufAdd_Copy;

}BSLUART;

/********************************************************************************************************
 EXTERNS
********************************************************************************************************/


/********************************************************************************************************
 Function Prototype Section
********************************************************************************************************/
void init_uart2(void);
void write_uart2(char );
void wites_uart2(char *);
int read_uart2(void);
void init_uart1(void);
void write_uart1(int );
char read_uart1(void);
/********************************************************************************************************/







#endif	/* BSL_UART_H */
