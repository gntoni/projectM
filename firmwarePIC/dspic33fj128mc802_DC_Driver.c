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
#include "stdio.h"
#include "stdlib.h"
#include "p33FJ128MC802.h"
#include <libpic30.h>


_FOSCSEL(FNOSC_FRC & IESO_OFF);  // Internal FRC oscillator without PLL
//_FOSCSEL(FNOSC_FRCPLL & IESO_OFF); // Internal FRC oscillator with PLL
_FOSC(FCKSM_CSECMD & IOL1WAY_OFF & OSCIOFNC_OFF & POSCMD_NONE);
_FWDT(FWDTEN_OFF)
_FPOR(FPWRT_PWR128 & PWMPIN_OFF & HPOL_ON & LPOL_ON)
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

    TRISAbits.TRISA4 = 1; // 1 -> Input / 0 -> Output

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

void init_pwm1h(void){

    P1TCONbits.PTMOD  = 0b00;
    P1TCONbits.PTCKPS = 0b00;
    P1TCONbits.PTOPS  = 0b0000;
    P1TCONbits.PTSIDL = 0b0;

    P1TPER = 3999; // 10KHz

    PWM1CON1bits.PMOD1 = 0;
    PWM1CON1bits.PMOD2 = 0;
    PWM1CON1bits.PMOD3 = 0;
    PWM1CON1bits.PEN1H = 1;
    PWM1CON1bits.PEN2H = 1;
    PWM1CON1bits.PEN3H = 1;
    PWM1CON1bits.PEN1L = 1;
    PWM1CON1bits.PEN2L = 1;
    PWM1CON1bits.PEN3L = 1;

    PWM1CON2bits.IUE = 1; // Updates to the active PxDC registers are immideate

    P1DTCON1bits.DTAPS = 0b00;
    P1DTCON1bits.DTBPS = 0b00;

    P1DTCON1bits.DTA = 10;
    P1DTCON1bits.DTB = 20;

    P1DTCON2bits.DTS3A = 0;
    P1DTCON2bits.DTS2A = 0;
    P1DTCON2bits.DTS1A = 0;
    P1DTCON2bits.DTS3I = 1;
    P1DTCON2bits.DTS2I = 1;
    P1DTCON2bits.DTS1I = 1;

    P1OVDCONbits.POVD3H = 0;
    P1OVDCONbits.POVD2H = 1;
    P1OVDCONbits.POVD1H = 1;
    P1OVDCONbits.POVD3L = 0;
    P1OVDCONbits.POVD2L = 1;
    P1OVDCONbits.POVD1L = 1;
    P1OVDCONbits.POUT3H = 0;
    P1OVDCONbits.POUT2H = 1;
    P1OVDCONbits.POUT1H = 1;
    P1OVDCONbits.POUT3L = 0;
    P1OVDCONbits.POUT2L = 1;
    P1OVDCONbits.POUT1L = 1;

    P1DC1 = 4000; // 4000 -> Duty cycle is half period
    P1DC2 = 4000;
    P1DC3 = 200;

    P1TCONbits.PTEN = 1;
    P2TCONbits.PTEN = 0;
}

void init_qei(void){

    // Remember to configure AD1PCFGLbits correctly
    // if QEI is defined in ANx Port

//    TRISBbits.TRISB8 = 1;       // 1 -> Input / 0 -> Output
//    TRISBbits.TRISB7 = 1;       // 1 -> Input / 0 -> Output
//    TRISBbits.TRISB6 = 1;       // 1 -> Input / 0 -> Output
//    RPINR14bits.QEA1R  = 0b1000; // QEA1 -> RP8
//    RPINR14bits.QEB1R  = 0b0111; // QEB1 -> RP7
//    RPINR15bits.INDX1R = 0b0110; // INDX1 -> RP6
    RPINR14bits.QEA1R  = 0b0111; // QEA1 -> RP8
    RPINR14bits.QEB1R  = 0b0110; // QEB1 -> RP7
    //RPINR15bits.INDX1R = 0b0110; // INDX1 -> RP6

    QEICONbits.QEIM    = 0b000; // Disable QEI Module
    QEICONbits.CNTERR  = 0;     // Clear any count errors
    QEICONbits.QEISIDL = 0;     // Continue operation during sleep
    QEICONbits.SWPAB   = 0;     // QEA and QEB not swapped
    QEICONbits.PCDOUT  = 0;     // Normal I/O pin operation
    QEICONbits.POSRES  = 0;     // Index pulse resets position counter 1


    DFLTCONbits.CEID   = 1;     // Count error interrupts disabled
    DFLTCONbits.QEOUT  = 1;     // Digital filters output enabled for QEn pins
    DFLTCONbits.QECK   = 0b111; // 1:256 clock divide for digital filter for QEn

    POSCNT = 0;                 // Reset position counter
    //MAXCNT = 1408; // 16*22*4 -> Micro Mo Electroics Motor
    MAXCNT = 2400; // 12*50*4 -> Pololu Micro Motor
    //QEICONbits.QEIM    = 0b110; // X4 mode with position counter reset by Index
    QEICONbits.QEIM    = 0b111; // X4 mode with position counter reset by match (MAXxCNT)
    //QEICONbits.QEIM    = 0b101; // X2 mode with position counter reset by match (MAXxCNT)
    return;
}

int AngPos[2] = {0,0}; // Two variables are used for Speed Calculation
int POSCNTcopy = 0;
void PositionCalculation(void)
{
    POSCNTcopy = (int)POSCNT;
    if (POSCNTcopy < 0)
        POSCNTcopy = -POSCNTcopy;
    AngPos[1] = AngPos[0];
    AngPos[0] = (unsigned int)(((unsigned long)POSCNTcopy * 2048)/150); // 0 <= POSCNT <= 2399 to 0 <= AngPos <= 32754
    return;
}

void InitTMR1(void)
{
    TMR1 = 0;               // Reset timer counter
    T1CONbits.TON = 0;      // Turn off timer 1
    T1CONbits.TSIDL = 0;    // Continue operation during sleep
    T1CONbits.TGATE = 0;    // Gated timer accumulation disabled
    T1CONbits.TCS = 0;      // use Tcy as source clock
    T1CONbits.TCKPS = 3;    // Tcy / 64 as input clock
    PR1 = 5000;             // Interrupt period = 0.0075 sec with a 64 prescaler
    IFS0bits.T1IF = 0;      // Clear timer 1 interrupt flag
    IEC0bits.T1IE = 1;      // Enable timer 1 interrupts
    T1CONbits.TON = 1;      // Turn on timer 1
    return;
}

float prev_error_speed=0.0, error_speed=0.0;
float derivative_speed=0.0, integral_speed=0.0;
float current_speed=0.0;

float target_speed = 0;
float control_period = 100;
float kp = 0.6;
float ki = 0.2;
float kd = 0.0;

// This is called every 7.5 ms
#define MAX_CNT_PER_REV (12*50*4 - 1)
#define MAXSPEED (unsigned int)(((unsigned long)MAX_CNT_PER_REV*2048)/150)
#define HALFMAXSPEED (MAXSPEED>>1)
int Speed;
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt (void)
{
    IFS0bits.T1IF = 0; // Clear timer 1 interrupt flag
    PositionCalculation();
    Speed = AngPos[0] - AngPos[1];
    if (Speed >= 0)
    {
        if (Speed >= HALFMAXSPEED)
            Speed = Speed - MAXSPEED;
    } else {
        if (Speed < -HALFMAXSPEED)
            Speed = Speed + MAXSPEED;
    }
    Speed *= 2;

    current_speed = Speed;
    error_speed = target_speed-current_speed;
    // compute PID
    integral_speed   += error_speed*control_period/1000;
    derivative_speed = ((error_speed - prev_error_speed)*1000)/control_period;
    current_speed    += kp*error_speed + ki*integral_speed + kd*derivative_speed;
    prev_error_speed = error_speed;
    P1DC1 = current_speed*4000/3100 + 4000; // 0 <= P1DC1 <= 8000
    P1DC2 = current_speed; // 0 <= P1DC1 <= 8000
    //P1DC1 = 4000; // 0 <= P1DC1 <= 8000

    return;
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

    init_pwm1h();

    init_qei();
    InitTMR1();

    
    
    while (1) {

        
//        if (current_speed < 57*5){

//        }else{
//            P1DC1 = 2000;
//        }
//        if (Speed < 57)
//            P1DC1 = 1900; // 1 volta per segon
//        else
//            P1DC1 = 3800; // 2 voltes per segon

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
        U1TXREG = 0x0D; // Transmit one character
        //U1TXREG = 0x0A; // Transmit one character

        char aa[6];
        unsigned int ii;
        for (ii=0; ii<sizeof(aa)/sizeof(char); ii++){
            aa[ii]=' ';
        }
        itoa(aa, kp*100, 10);
        for (ii=0; ii<sizeof(aa)/sizeof(char); ii++){
            __delay_ms(2);
            U1TXREG = aa[ii];
        }
        __delay_ms(2);
        U1TXREG = ' ';
        __delay_ms(2);
        U1TXREG = ' ';
        itoa(aa, ki*100, 10);
        for (ii=0; ii<sizeof(aa)/sizeof(char); ii++){
            __delay_ms(2);
            U1TXREG = aa[ii];
        }
        __delay_ms(2);
        U1TXREG = ' ';
        __delay_ms(2);
        U1TXREG = ' ';
        itoa(aa, kd*100, 10);
        for (ii=0; ii<sizeof(aa)/sizeof(char); ii++){
            __delay_ms(2);
            U1TXREG = aa[ii];
        }
        __delay_ms(2);
        U1TXREG = ' ';
        __delay_ms(2);
        U1TXREG = ' ';
        itoa(aa, target_speed, 10);
        for (ii=0; ii<sizeof(aa)/sizeof(char); ii++){
            __delay_ms(2);
            U1TXREG = aa[ii];
        }
        __delay_ms(2);
        U1TXREG = ' ';
        __delay_ms(2);
        U1TXREG = ' ';
        itoa(aa, Speed, 10);
        for (ii=0; ii<sizeof(aa)/sizeof(char); ii++){
            __delay_ms(2);
            U1TXREG = aa[ii];
        }
        __delay_ms(2);
        U1TXREG = ' ';
        __delay_ms(2);
        U1TXREG = ' ';
        itoa(aa, (int)error_speed, 10);
        for (ii=0; ii<sizeof(aa)/sizeof(char); ii++){
            __delay_ms(2);
            U1TXREG = aa[ii];
        }
        __delay_ms(2);
        U1TXREG = ' ';
        __delay_ms(2);
        U1TXREG = ' ';
        itoa(aa, (int)P1DC1, 10);
        for (ii=0; ii<sizeof(aa)/sizeof(char); ii++){
            __delay_ms(2);
            U1TXREG = aa[ii];
        }

        char rx = ReceivedChar;
        switch (rx){
            case 'i':
                target_speed += 1000;
                ReceivedChar = 'n';
                break;
            case 'k':
                target_speed -= 1000;
                ReceivedChar = 'n';
                break;
            case 'e':
                kp += 0.1;
                ReceivedChar = 'n';
                break;
            case 'd':
                kp -= 0.1;
                ReceivedChar = 'n';
                break;
            case 'r':
                ki += 0.1;
                ReceivedChar = 'n';
                break;
            case 'f':
                ki -= 0.1;
                ReceivedChar = 'n';
                break;
            case 't':
                kd += 0.1;
                ReceivedChar = 'n';
                break;
            case 'g':
                kd -= 0.1;
                ReceivedChar = 'n';
                break;
            default:
                break;
        }
        //        itoa(aa, error_speed, 10);
//        for (ii=0; ii<sizeof(aa)/sizeof(char); ii++){
//            WriteUART1(aa[ii]);
//        }
        //U1TXREG = ReceivedChar; // Transmit one character
//        LATAbits.LATA0 = ~LATAbits.LATA0;
//        if (P1DC1 < 7000){
//            P1DC1 += 200;
//        }else{
//            P1DC1 = 200;
//        }
//        P1DC1 = POSCNT*10;
        __delay_ms(100);

    }

}

void __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0; // clear TX interrupt flag
    U1TXREG = 'A'; // Transmit one character
}

void __attribute__ ((__interrupt__, auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0; // Clear Rx Interrupt Flag
    if (U1STAbits.URXDA) {
        ReceivedChar = U1RXREG;
        LATAbits.LATA0 = ~LATAbits.LATA0;
    }
}