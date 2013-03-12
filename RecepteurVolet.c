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
#include "SystemProfile.h"
#include "WirelessProtocols/MCHP_API.h"

// CONFIG1L
#pragma config PLLSEL = PLL4X   // PLL Selection (4x clock multiplier)
#pragma config CFGPLLEN = ON    // PLL Enable Configuration bit (PLL Enabled)
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
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
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



#define LED1 LATAbits.LATA0
#define LED2 LATAbits.LATA2
#define LED3 LATAbits.LATA3
#define LED4 LATAbits.LATA4
#define LED5 LATAbits.LATA5

#define INPUT1 PORTAbits.RA6
#define INPUT2 PORTAbits.RA7
#define INPUT3 PORTAbits.RA5


#if ADDITIONAL_NODE_ID_SIZE > 0
BYTE    AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {0x00};
#endif

#pragma romdata myaddress
rom unsigned char addr1 = 0x34;
rom unsigned char addr2 = 0x67;
rom unsigned char addr3 = 0x01;
rom unsigned char addr4 = 0x02;
rom unsigned char addr5 = 0x03;
rom unsigned char addr6 = 0x04;
rom unsigned char addr7 = 0x05;
rom unsigned char addr8 = 0x06;
rom unsigned char addr9 = 0x14;
rom unsigned char addr10 = 0x15;
#pragma romdata


extern ACTIVE_SCAN_RESULT ActiveScanResults[ACTIVE_SCAN_RESULT_SIZE];
extern CONNECTION_ENTRY ConnectionTable[CONNECTION_SIZE];
extern BYTE myLongAddress[MY_ADDRESS_LENGTH];
extern BYTE TxBuffer[TX_BUFFER_SIZE];
extern WORD myPrivatePanId;
// Initialisation des paramètres hardware de la carte
void BoardInit(void)
{
    // primary internal oscillator
    //OSCCON = 0x7B;
    WDTCONbits.SWDTEN = 0;
    INTCONbits.GIEH = 1;
    RFIF = 0;
    RFIE = 1;
    //Led de control
    TRISCbits.TRISC6 = 0;
    LATCbits.LATC6  = 0;
    //On met la commande des volets en sortie et on les passe à 0
    CMD_UP_TRIS = 0;
    CMD_UP = 0;
    CMD_DOWN_TRIS = 0;
    CMD_DOWN = 0;
}

//Lecture de mon adresse dans la eeprom interne du pic
//Les 8 premières bytes sont l'address
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


void main(void)
{
    char value = 0;
    BYTE i;
    BYTE noise = 0xFF;
    DWORD channelMap = 0x00000001;
    BYTE newChannel = 0x00;
    unsigned char lastLed[3];
    BYTE destinataire[MY_ADDRESS_LENGTH] = {0x31,0x32,0x87,0x67,0x04,0x33,0x22,0x11};
    CCP1CON = 0x00;
    TRISA = TRISA & 0xB8;
    TRISB = 0x00;
    //TRISD = 0xFC;
//    PORTEbits.RDPU = 1;
    LED1 = 1;
    LED2 = 1;
    INTCON2bits.RBPU = 0;
    INTCON2bits.INTEDG2 = 0;
    INTCONbits.GIEH = 1;

    LED3 = 1;
    LED4 = 1;
    LED5 = 1;

    //Configuration du SPI
    //Reset du miwi
    DisableIntSPI1;

    BoardInit();
    for(i = 0; i < 3; i++)
    {
        lastLed[i] = 0x32;
    }
    CloseSPI1();
    OpenSPI1(SPI_FOSC_64,MODE_00,SMPMID);
    //Lecture des informations Miwi
    LitMyMiwiAddress();
    LitMyPrivatePanID();

    MiApp_ProtocolInit(FALSE);
    MiApp_SetChannel(16);
    value  = MiApp_NoiseDetection(0x07FFF800,5,NOISE_DETECT_ENERGY,&noise);
    newChannel = value;
    channelMap = channelMap << value;
    value = MiApp_StartConnection(START_CONN_ENERGY_SCN,10,channelMap);
    MiApp_ConnectionMode(ENABLE_ALL_CONN);
    LATCbits.LATC6  = 1;
    while(1)
    {
        if(MiApp_MessageAvailable())
        {
            //Traitement du message
            if(rxMessage.PayloadSize == 3)
            {
                if(rxMessage.Payload[0] == 0x12)
                {
                    switch(rxMessage.Payload[1])
                    {
                        case 0x01:
                            break;
                        case 0x02:
                            break;
                        case 0x03:
                            break;
                    }
                }
            }
            for(i = 0; i < 8; i++)
            {
                value = rxMessage.SourceAddress[i];
            }
            MiApp_DiscardMessage();
        }
        else
        {
//            if(INPUT1 != lastLed[0])
//            {
//
//                MiApp_FlushTx();
//                MiApp_WriteData(0x12); //Inversion de la led 1
//                MiApp_WriteData(0x01); //Inversion de la led 1
//                MiApp_WriteData(INPUT1); //Inversion de la led 1
//                MiApp_UnicastConnection(0,FALSE);
//                lastLed[0] = INPUT1;
//            }
//            if(INPUT2 != lastLed[1])
//            {
//                MiApp_FlushTx();
//                MiApp_WriteData(0x12); //Inversion de la led 1
//                MiApp_WriteData(0x02); //Inversion de la led 1
//                MiApp_WriteData(INPUT2); //Inversion de la led 1
//                value = MiApp_UnicastAddress(destinataire,TRUE,FALSE);
//                lastLed[1] = INPUT2;
//            }
//            if(INPUT3 != lastLed[2])
//            {
//                MiApp_FlushTx();
//                MiApp_WriteData(0x12); //Inversion de la led 1
//                MiApp_WriteData(0x03); //Inversion de la led 1
//                MiApp_WriteData(INPUT3); //Inversion de la led 1
//  //              MiApp_UnicastAddress(destinataire, TRUE,FALSE);
//                MiApp_BroadcastPacket(FALSE);
//                lastLed[2] = INPUT3;
//            }
        }
    }
    return;
}
void UserInterruptHandler(void)
{
    unsigned char toto;
    toto = 0x00;
    toto = toto + 1;

}