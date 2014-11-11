/*
 * File:   newmainXC16.c
 * Author: sfoix
 *
 * Created on November 4, 2014, 5:37 PM
 */


#define FCY 40000000UL
#define FP 40000000
#define BAUDRATE 9600
#define BRGVAL ((FP/BAUDRATE)/16)-1

#include "xc.h"
#include "uart.h"
#include "p33FJ128MC802.h"
#include <libpic30.h>


_FOSCSEL(FNOSC_FRC & IESO_OFF);  // Internal FRC oscillator without PLL
//_FOSCSEL(FNOSC_FRCPLL & IESO_OFF); // Internal FRC oscillator with PLL
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_OFF & POSCMD_NONE);
_FWDT(FWDTEN_OFF)
_FPOR(FPWRT_PWR128)
_FICD(JTAGEN_OFF)

void init_clock(void) {

    // Configure Oscillator to operate the device at 40 MHz
    // Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
    // Fosc = 7.37M * 65/(3 * 2) = 79.8416 MHz for 7.37MHz input clock
    // Fcy = Fosc/2 -> 40 MHz

    PLLFBD = 63; // M = PLLDIV + 2
    CLKDIVbits.PLLPRE = 1; // N1 = PLLPRE + 2
    CLKDIVbits.PLLPOST = 0; // N2 = 2 * (PLLPOST + 1)
    OSCTUN = 0; // Tune FRC oscillator, if FRC is used 0-> (7.37 MHz)
    RCONbits.SWDTEN = 0; // Disable Watch Dog Timer

    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to FRC with PLL (NOSC=0b001)
    __builtin_write_OSCCONL(OSCCON | 0x01); // Start clock switching
    while (OSCCONbits.COSC != 0b001) { }; // Wait for Clock switch to occur
    while (OSCCONbits.LOCK != 1) { }; // Wait for PLL to lock

}

void init_uart1(void) {

    AD1PCFGLbits.PCFG0 = 1; // AN0 digital
    AD1PCFGLbits.PCFG4 = 1; // AN4/RP2 -> digital
    AD1PCFGLbits.PCFG5 = 1; // AN5/RP3 -> digital

    TRISAbits.TRISA4 = 1;

    RPINR18bits.U1RXR = 0b00010; // U1RX -> RP2
    RPOR1bits.RP3R    = 0b00011; // U1TX -> RP3

    U1MODEbits.UARTEN = 0; // UART Disabled -> Enabled at the end
    U1MODEbits.USIDL  = 0; // Continue in Idle
    U1MODEbits.IREN   = 0;
    U1MODEbits.RTSMD  = 0;
    U1MODEbits.UEN    = 0b00;
    U1MODEbits.WAKE   = 0;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD  = 0; // Auto-Baud Disabled
    U1MODEbits.URXINV = 0; // 0 -> Pull-up / 1 -> Pull-down
    U1MODEbits.BRGH   = 0; // Low Speed mode
    U1MODEbits.PDSEL  = 0b00; // No Parity, 8 data bits
    U1MODEbits.STSEL  = 0; // 1 Stop bit

    U1BRG = BRGVAL; // BAUD Rate Setting for 9600

    U1STAbits.UTXISEL1 = 0; // Interrupt after one TX Character is transmitted
    U1STAbits.UTXINV   = 0;
    U1STAbits.UTXISEL0 = 0; // Interrupt after one TX Character is transmitted
    U1STAbits.URXISEL  = 0; // Interrupt with any character that is received

    // Transmission
    //IEC0bits.U1TXIE = 1; // Enable UART1 TX Interrupt
//    IPC3bits.U1TXIP = 0b100; // UART1 Tranmitter Interrupt Priority 4
    IEC0bits.U1TXIE = 0; // Disable UART1 TX Interrupt

    //Reception
    IFS0bits.U1RXIF = 0; // Clear Interrupt Flag
    IPC2bits.U1RXIP = 0b111; // UART1 Receiverr Interrupt Priority 4
    IEC0bits.U1RXIE = 1; // Enable UART1 RX Interrupt
    //IEC0bits.U1RXIE = 0; // Disable UART1 RX Interrupt


    U1MODEbits.UARTEN = 1; // Enable UART
    U1STAbits.UTXEN = 1; // Enable UART TX

}

unsigned int i;
char ReceivedChar = 'A';

int main(void) {
    // Configure clock
    init_clock();

    // Configure I/O PORTA
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA0 = 0;

    init_uart1();
    __delay_ms(5);
    
  
    while (1) {

//        // Receiving by Polling
//        /* Check for receive errors */
//        if(U1STAbits.FERR == 1) {
//            continue;
//        }
//        /* Must clear the overrun error to keep UART receiving */
//        if(U1STAbits.OERR == 1) {
//            U1STAbits.OERR = 0;
//            continue;
//        }
//        /* Get the data */
//        if(U1STAbits.URXDA == 1) {
//            ReceivedChar = U1RXREG;
//        }

        //U1TXREG = '0'; // Transmit one character
        U1TXREG = ReceivedChar; // Transmit one character
        //LATAbits.LATA0 = ~LATAbits.LATA0;
        __delay_ms(100);
    }


    while (1) { }
}

void __attribute__((__interrupt__)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0; // clear TX interrupt flag
    U1TXREG = 'A'; // Transmit one character
}

void __attribute__ ((__interrupt__)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0; // Clear Rx Interrupt Flag
    if (U1STAbits.URXDA) {
        ReceivedChar = U1RXREG;
        LATAbits.LATA0 = ~LATAbits.LATA0;
    }
}