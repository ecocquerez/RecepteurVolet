/*
   File:   HarwareProfile.h
   Author: Eric

   Created on 15 octobre 2011, 19:04
 */

#ifndef HarwareProfile_h
#define HarwareProfile_h
//Not use of PLL, speed not required
#define CLOCK_FREQ 8000000

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

#define TMRL            TMR0L
#define GetInstructionClock()	(CLOCK_FREQ/4)

#define TRIS_CMD_VOLET1_UP      TRISCbits.TRISC1
#define CMD_VOLET1_UP           LATCbits.LATC1
#define TRIS_CMD_VOLET1_DOWN    TRISCbits.TRISC2
#define CMD_VOLET1_DOWN         LATCbits.LATC2

#endif // HarwareProfile_h
