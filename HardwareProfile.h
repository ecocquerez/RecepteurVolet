/*
   File:   HarwareProfile.h
   Author: Eric

   Created on 15 octobre 2011, 19:04
 */

#ifndef HarwareProfile_h
#define HarwareProfile_h
#define CLOCK_FREQ 24000000

#define RFIF            INTCON3bits.INT2IF
#define RFIE            INTCON3bits.INT2IE
#define PHY_CS          LATBbits.LATB4
#define PHY_CS_TRIS     TRISBbits.TRISB4
#define RF_INT_PIN      PORTBbits.RB2
#define RF_INT_TRIS     TRISBbits.TRISB2
#define PHY_WAKE        LATCbits.LATC0
#define PHY_WAKE_TRIS   TRISCbits.TRISC0
//#define RF_EEnCS_TRIS   TRISCbits.TRISC2
//#define RF_EEnCS        LATCbits.LATC2
#define PHY_RESETn      LATBbits.LATB5
#define PHY_RESETn_TRIS TRISBbits.TRISB5

//Gestion de la commande des volets
#define CMD_UP          LATCbits.LATC0
#define CMD_UP_TRIS     TRISCbits.TRISC0
#define CMD_DOWN        LATCbits.LATC1
#define CMD_DOWN_TRIS   TRISCbits.TRISC1

#define TMRL            TMR0L
#define GetInstructionClock()	(CLOCK_FREQ/4)
#endif // HarwareProfile_h
