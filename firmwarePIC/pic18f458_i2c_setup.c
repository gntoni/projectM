/* 
 * File:   newmain.c
 * Author: sfoix
 *
 * Created on October 29, 2014, 7:09 PM
 */

#include <stdio.h>
#include <stdlib.h>
#define USE_OR_MASKS

#include <pic18f458.h>
#include <i2c.h>
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

unsigned char I2C_Send[21] = "MICROCHIP:I2C_SLAVE" ;
unsigned char I2C_Recv[21];

    void i2c_setup(){
    TRISC = 0b00011000; // make TRISC<3:4> = 1 -> SCL and SDA as Inputs
    LATC  = 0b00011000;
    //PORTC = 0;
    //PORTC = 0b11100111;

    // SSPSTAT
    SSPSTATbits.SMP = 1; // slew rate is disabled
    SSPSTATbits.CKE = 1; // enable SM

    // SSPCON1
    //SSPCON1 = 0b00001110;// I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled(2)
    SSPCON1 = 0b00000110;  // I2C Slave mode, 7-bit address
    SSPCON1bits.CKP   = 1; // Release clock
    SSPCON1bits.SSPEN = 1; // Enables the serial port and configures SDA SCL

    // SSPCON2
    SSPCON2bits.GCEN = 1; // 1 -> General call enable interrupt Enabled
    SSPCON2bits.SEN  = 0; // 0 -> Clock stretching is Disabled
    SSPADD = 0xA0; //initialze slave address 50
    PIR1bits.SSPIF  = 0; // SSPIF: MSSP Interrupt Flag bit, 0-> clearing it
    PIE1bits.SSPIE  = 1; //SSPIE: MSSP Interrupt Enable bit
    INTCONbits.PEIE = 1; //Enable all peripheral interrupts
    INTCONbits.GIE  = 1; //Enable global interrupts
}

void check_interruptions() {
    if (PIR1bits.SSPIF == 1) {
        PIR1bits.SSPIF    = 0; // MSSP Interrupt Flag bit, 0-> clearing it
        SSPCON1bits.CKP   = 1; // Release clock
        SSPCON1bits.SSPEN = 1; // Enables the serial port and configures SDA SCL
        INTCONbits.GIE    = 1; // Enable global interrupts
    }
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

    // I2C configuration
    i2c_setup();

    unsigned char w;
    for(w=0;w<20;w++)
        I2C_Recv[w]=0;

    unsigned int temps = 25;
    unsigned char temp, length=0;
    while(1){
        LATA1 = ~LATA1;
        Delay10KTCYx(temps);
        check_interruptions();
        if (DataRdyI2C()==1) {
            temps += 25;
            temp = ReadI2C();
            //********************* Data reception from master by slave *********************
            do {
                while(DataRdyI2C()==0);        // WAIT UNTILL THE DATA IS TRANSMITTED FROM master
                I2C_Recv[length++]=getcI2C();  // save byte received
            }
            while(length!=20);
        }
    }

    //******************** write sequence from slave *******************************
    while(SSPSTATbits.S!=1); //wait untill STOP CONDITION

    //********************* Read the address sent by master from buffer **************
    while(DataRdyI2C()==0); //WAIT UNTILL THE DATA IS TRANSMITTED FROM master
    temp = ReadI2C();

    //********************* Slave transmission ************************************
    if(SSPSTAT & 0x04) //check if master is ready for reception
        while(putsI2C(I2C_Send));			// send the data to master

    //-------------TERMINATE COMMUNICATION FROM MASTER SIDE---------------
    CloseI2C(); //close I2C module
    
    while(1) {
        LATA1 = ~LATA1;
        Delay10KTCYx(100);
    }; //End of program


    return (EXIT_SUCCESS);
}

