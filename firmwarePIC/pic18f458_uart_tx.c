/* 
 * File:   pic18f458_uart_tx.c
 * Author: sfoix
 *
 * Created on October 29, 2014, 7:09 PM
 */

#include <stdio.h>
#include <stdlib.h>
#define USE_OR_MASKS

#include <pic18f458.h>
#include <delays.h>
#include <xc.h>


//-------------------------------Configuration setting ----------------------------------------------
/**
	* Oscillator is configured as HS
	* Fail safe monitor is enabled
	* watch dog timer is disabled
	* Extended instruction mode is disabled
	* oscillator switch over is enabled
*/
#pragma config OSC = HS  // Internal oscillator HS/HSPLL
#pragma config WDT = OFF // Disable watchdog timer
#pragma config LVP = OFF

unsigned char USART_Send[20] = "MICROCHIP:USART_TXD ";
unsigned char USART_Recv[20];

void usart_setup(){
    TRISC7 = 1;
    TRISC6 = 0;
    SPBRG = 0x40; // 64 -> 9600 bauds
    TXSTAbits.BRGH  = 1;
    TXSTAbits.SYNC  = 0;
    RCSTAbits.SPEN  = 1;
    RCSTAbits.ADDEN = 0; // Address detect enable bit

    PIE1bits.TXIE   = 1; // USART Transmit Interrupt Enable bit
    TXSTAbits.TX9   = 0;
    TXSTAbits.TXEN  = 1;

    PIE1bits.RCIE   = 1; // USART Recieve Interrupt Enable bit
    RCSTAbits.RX9   = 0;
    RCSTAbits.CREN  = 1; // enables continuous receive
}

/*
 * MAIN
 */
int main(int argc, char** argv) {

    // RA1 as DO
    LATA = 0;
    ADCON1 = 0x06;
    TRISA1 = 0;
    LATA1 = 1;

    usart_setup();

    unsigned char w;
    for(w=0;w<19;w++)
        USART_Recv[w]=0;

    TXREG = 0x00;
    unsigned char length=0;
    do {
        LATA1 = ~LATA1;
        TXREG = USART_Send[length++];
//        while(!TRMT);
//        while(!RCIF);
        Delay10KTCYx(100);
    } while(length!=20);

    LATA1 = 1; // Turn off LED

    while(1){};

    return (EXIT_SUCCESS);
}

