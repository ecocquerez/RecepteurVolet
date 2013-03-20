/*
 * File:   FirstMiwi.c
 * Author: Eric
 *
 * Created on 15 octobre 2011, 18:00
 */
#include <p18cxxx.h>
#include <p18f25k50.h>
#include <delays.h>
#include <spi.h>
#include <EEP.h>
#include <timers.h>
#include "SystemProfile.h"
#include "WirelessProtocols/MCHP_API.h"
#include "domoproto.h"

// CONFIG1L
#pragma config PLLSEL = PLL4X   // PLL Selection (4x clock multiplier)
#pragma config CFGPLLEN = OFF   // PLL Enable Configuration bit (PLL Disabled)
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Postscaler (CPU uses system clock (no divide))
#pragma config LS48MHZ = SYS24X4// Low Speed USB mode with 48 MHz system clock (System clock at 24 MHz, USB clock divider is set to 4)

// CONFIG1H
#pragma config FOSC = INTOSCIO  // Oscillator Selection (Internal oscillator)
#pragma config PCLKEN = ON      // Primary Oscillator Shutdown (Primary oscillator enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config nPWRTEN = ON     // Power-up Timer Enable (Power up timer enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable (BOR enabled in hardware (SBOREN is ignored))
#pragma config BORV = 190       // Brown-out Reset Voltage (BOR set to 1.9V nominal)
#pragma config nLPBOR = OFF     // Low-Power Brown-out Reset (Low-Power Brown-out Reset disabled)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (WDT disabled in hardware (SWDTEN ignored))
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler (1:32768)

// CONFIG3H
#pragma config CCP2MX = RC1     // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<5:0> pins are configured as digital I/O on Reset)
#pragma config T3CMX = RC0      // Timer3 Clock Input MUX bit (T3CKI function is on RC0)
#pragma config SDOMX = RB3      // SDO Output MUX bit (SDO function is on RB3)
#pragma config MCLRE = ON       // Master Clear Reset Pin Enable (MCLR pin enabled; RE3 input disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Block 0 Code Protect (Block 0 is not code-protected)
#pragma config CP1 = OFF        // Block 1 Code Protect (Block 1 is not code-protected)
#pragma config CP2 = OFF        // Block 2 Code Protect (Block 2 is not code-protected)
#pragma config CP3 = OFF        // Block 3 Code Protect (Block 3 is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protect (Boot block is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protect (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Block 0 Write Protect (Block 0 (0800-1FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Block 1 Write Protect (Block 1 (2000-3FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Block 2 Write Protect (Block 2 (04000-5FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Block 3 Write Protect (Block 3 (06000-7FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Registers Write Protect (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protect (Boot block (0000-7FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protect (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Block 0 Table Read Protect (Block 0 is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Block 1 Table Read Protect (Block 1 is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Block 2 Table Read Protect (Block 2 is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Block 3 Table Read Protect (Block 3 is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protect (Boot block is not protected from table reads executed in other blocks)

typedef struct defInternal
{
    BYTE SendAlive;
    BYTE SendAliveTimeOut;
    BYTE MyGroup;
    BYTE IndexInGroup;
    BYTE IndexOnBoard;
    BYTE MaxTimeOut;
}Internal, * pInternal;

typedef struct defShutterState
{
    BYTE stateShutter1;
    BYTE stateShutter2;
}ShutterState;

#if ADDITIONAL_NODE_ID_SIZE > 0
BYTE    AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {0x00};
#endif

#pragma romdata myaddress
rom unsigned char addr1 = 0x06;
rom unsigned char addr2 = 0x08;
rom unsigned char addr3 = 0x26;
rom unsigned char addr4 = 0x83;
rom unsigned char addr5 = 0x82;
rom unsigned char addr6 = 0x00;
rom unsigned char addr7 = 0x00;
rom unsigned char addr8 = 0x02;
rom unsigned char addr9 = 0x14;
rom unsigned char addr10 = 0x01;
rom unsigned char addr11 = 0x01;    //Send Alive
rom unsigned char addr12 = 0x3C;    //Periode
rom unsigned char addr13 = 0x01;    //Group
rom unsigned char addr14 = 0x01;    //Index In Group
rom unsigned char addr15 = 0x01;    //Index On Board
rom unsigned char addr16 = 0x50;    //TimeOut
#pragma romdata


extern ACTIVE_SCAN_RESULT ActiveScanResults[ACTIVE_SCAN_RESULT_SIZE];
extern CONNECTION_ENTRY ConnectionTable[CONNECTION_SIZE];
extern BYTE myLongAddress[MY_ADDRESS_LENGTH];
extern BYTE TxBuffer[TX_BUFFER_SIZE];
extern WORD myPrivatePanId;
// Initialisation des param�tres hardware de la carte

volatile unsigned char HaveToSendHearthBeat;
volatile BYTE ledValue = 0;

Entete Emission;
Internal travail;

void BoardInit(void);
void LitMyMiwiAddress(void);
void LitMyPrivatePanID(void);
void EcritMyPrivatePanID(void);
void EcritMyMiwiAddress(void);
unsigned char DoConnection(void);
void LitInternalParameters(pInternal pLecture);
void WriteEntete(void);

void main(void)
{
    Entete * pReception;
    ShutterState EtatCourrant;
    char value = 0;
    unsigned char IgnoreMessage;

    //Settings oscillator
    // primary internal oscillator
    //OSCCON = 0x7B;
    OSCCON = 0b01101000;
    while(OSCCONbits.HFIOFS == 0)
    {
        //Waiting for stable frequency
        Nop();
    }

    EtatCourrant.stateShutter1 = Iddle;
    EtatCourrant.stateShutter2 = Iddle;

    HaveToSendHearthBeat = 0;
    CCP1CON = 0x00;
    TRISA = TRISA & 0xB8;
    TRISB = 0x00;
    //Shutter output
    TRIS_CMD_VOLET1_DOWN = 0;
    TRIS_CMD_VOLET1_UP = 0;
    CMD_VOLET1_DOWN = 0;
    CMD_VOLET1_UP = 0;
    INTCON2bits.RBPU = 0;
    INTCON2bits.INTEDG2 = 0;
    //Timer 1
    T1CON = 0b01110011; //1000000 inc per second
    TMR1H = 0xC3;
    TMR1L = 0x50; //We have to count 20 to have 1 second
    PIE1bits.TMR1IE = 1;
    T1CONbits.TMR1ON = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    INTCONbits.GIEH = 1;


    // SPI configuration
    //Reset du miwi
    DisableIntSPI1;

    BoardInit();
    CloseSPI1();
    OpenSPI1(SPI_FOSC_64,MODE_00,SMPMID);
    //Read necessary Miwi Information in eeprom
    LitMyMiwiAddress();
    LitMyPrivatePanID();
    Emission.DeviceType = Shutter;
    Emission.Group = travail.MyGroup;
    Emission.IndexInGroup = travail.IndexInGroup;
    Emission.IndexOnBoard = travail.IndexOnBoard;
    Emission.lenValue = 0;
    //Read internal parameters
    LitInternalParameters(&travail);
    MiApp_ProtocolInit(FALSE);
    do
    {
        value = DoConnection();
    }while(value == 0xff);

    //On allume la led de controle
    LATCbits.LATC6  = 1;
    while(1)
    {
        if(MiApp_MessageAvailable())
        {
            IgnoreMessage = FALSE;
            pReception = (Entete *)rxMessage.Payload;
            if(rxMessage.flags.bits.broadcast == 1)
            {
                //if message is broadcast, check if message is for my group or group is broadcast
                if(pReception->Group != travail.MyGroup && pReception->Group != 0xFF)
                {
                    IgnoreMessage = TRUE;
                }
            }
            if(IgnoreMessage == FALSE)
            {
                //Message is not for shutter, ignore it
                if(pReception->DeviceType != Shutter)
                {
                    IgnoreMessage = TRUE;
                }
                //Message is not for all shutter on board and not for my index
                if(pReception->IndexOnBoard != 0xFF && pReception->IndexOnBoard != travail.IndexOnBoard)
                {
                    IgnoreMessage = TRUE;
                }
            }
            if(IgnoreMessage == FALSE)
            {
                if(pReception->IndexOnBoard == 1 || pReception->IndexOnBoard == 0xFF)
                {
                    //If we receive command while moving shutter, we have to stop them
                    if(EtatCourrant.stateShutter1 != Iddle && pReception->Command != ShutterStop)
                    {
                        pReception->Command = ShutterStop;
                    }
                    switch(pReception->Command)
                    {
                        case ShutterDownShort:
                        case ShutterDownLong:
                            //Down with timer
                            CMD_VOLET1_UP = 0;
                            CMD_VOLET1_DOWN = 1;
                            EtatCourrant.stateShutter1 = Descending;
                            break;
                        case ShutterUpShort:
                        case ShutterUpLong:
                            CMD_VOLET1_DOWN = 0;
                            CMD_VOLET1_UP = 1;
                            EtatCourrant.stateShutter1 = Rising;
                            break;
                        case ShutterStop:
                            CMD_VOLET1_DOWN = 0;
                            CMD_VOLET1_UP = 0;
                            EtatCourrant.stateShutter1 = Iddle;
                            break;
                    }
                }
            }
            MiApp_DiscardMessage();
        }
        else
        {
            if(HaveToSendHearthBeat)
            {
                //If we have to send Hearth beat message, send it ant reset the flag
                HaveToSendHearthBeat = 0;
                Emission.Command = HearthBeat;
                MiApp_BroadcastPacket(FALSE );
            }
        }
    }
    return;
}

void WriteEntete(void)
{
    MiApp_FlushTx();
    MiApp_WriteData(Emission.DeviceType);
    MiApp_WriteData(Emission.Group);
    MiApp_WriteData(Emission.IndexInGroup);
    MiApp_WriteData(Emission.IndexOnBoard);
    MiApp_WriteData(Emission.Command);
    MiApp_WriteData(Emission.lenValue);


}
void UserInterruptHandler(void)
{
    static unsigned char compteurSeconde = 0x00;
    static unsigned char ellapsed = 0x00;
    if(PIR1bits.TMR1IF == 1)
    {
        //Reset of timer
        TMR1H = 0xC3;
        TMR1L = 0x50; //We have to count 20 to have 1 second
        compteurSeconde++;
        if(compteurSeconde == 20)
        {
            //Just visualize timer
            LATCbits.LATC6 = ledValue;
            ledValue = !ledValue;
            //One second elapsed
            //Check if we have to send HearthBeat
            if(travail.SendAlive == 1)
            {
                ellapsed++;
                if(ellapsed == travail.SendAliveTimeOut)
                {
                    ellapsed = 0;
                    HaveToSendHearthBeat = 1;
                }
            }
            compteurSeconde = 0;
        }
        PIR1bits.TMR1IF = 0;
    }
}
//Init de la connexion ou de la reconnexion
unsigned char DoConnection(void)
{
    unsigned char numberNetwork = 0x00;
    unsigned char i = 0;
    unsigned char value = 0xFF;
    numberNetwork = MiApp_SearchConnection(5,0x07FFF800);
    if(numberNetwork > 0)
    {
        for(i = 0; i < ACTIVE_SCAN_RESULT_SIZE ; i++)
        {
            if(ActiveScanResults[i].PANID.Val == myPANID.Val)
            {
                value = MiApp_EstablishConnection(i,CONN_MODE_INDIRECT);
                break;
            }
        }
    }
    return value;
}

void BoardInit(void)
{
    WDTCONbits.SWDTEN = 0;
    INTCONbits.GIEH = 1;
    RFIF = 0;
    RFIE = 1;
    //Led de control
    TRISCbits.TRISC6 = 0;
    LATCbits.LATC6  = 0;
    //On met la commande des volets en sortie et on les passe � 0
    CMD_UP_TRIS = 0;
    CMD_UP = 0;
    CMD_DOWN_TRIS = 0;
    CMD_DOWN = 0;
}

//Lecture de mon adresse dans la eeprom interne du pic
//Les 8 premi�res bytes sont l'address
void LitMyMiwiAddress(void)
{
    unsigned char compteur = 0;
    while(compteur <8)
    {
        myLongAddress[compteur] = Read_b_eep(compteur);
        compteur++;
    }
}
//Ecriture de mon adresse dans la EEProm
void EcritMyMiwiAddress(void)
{
    unsigned char compteur = 0;
    while(compteur <8)
    {
        Write_b_eep(compteur,myLongAddress[compteur]);
        Busy_eep();
        compteur++;
    }
}
//Lecture du PanId
void LitMyPrivatePanID(void)
{
    myPrivatePanId = 0;
    myPrivatePanId = Read_b_eep(8);
    myPrivatePanId = myPrivatePanId << 8;
    myPrivatePanId |= Read_b_eep(9);
}
//Ecriture du PanId
void EcritMyPrivatePanID(void)
{
    unsigned char travail  = 0;
    travail = myPrivatePanId >> 8;
    Write_b_eep(8,travail);
    Busy_eep();
    travail = myPrivatePanId & 0xFF;
    Write_b_eep(9,travail);
    Busy_eep();
}

void LitInternalParameters(pInternal pLecture)
{
    pLecture->SendAlive = Read_b_eep(10);
    pLecture->SendAliveTimeOut = Read_b_eep(11);
    pLecture->MyGroup = Read_b_eep(12);
    pLecture->IndexInGroup = Read_b_eep(13);
    pLecture->IndexOnBoard = Read_b_eep(14);
    pLecture->MaxTimeOut = Read_b_eep(15);
}