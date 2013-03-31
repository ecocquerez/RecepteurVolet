/* 
 * File:   domoproto.h
 * Author: Eric
 *
 * Created on 12 mars 2013, 21:06
 */

#ifndef DOMOPROTO_H
#define	DOMOPROTO_H

typedef struct defEntete
{
    BYTE DeviceType; //Type de device, cf enumeration
    BYTE Group; // Numéro de groupe
    BYTE IndexInGroup; //Numéro de l'index dans le groupe
    BYTE IndexOnBoard; //Numéro de la fonctionnalité sur la carte, sur une carte 8 relais, on peut sélectionner le relais 3
    BYTE Command; //Commande à envoyer
    BYTE lenValue; //Longueur des données à suivre
}Entete;

//Enumération des différents devices
enum EnumDeviceType
{
    Shutter =1, //Commande de volet
    Relay,//Commande à relais, n'est commandé que le temps de commande
    Contactor,//Télérupteur. 1 pulse -> On, 1 pulse Off
    Display1, //Afficheur ligne 1
    Display2, //Afficheur ligne 1
    Display3, //Afficheur ligne 1
    Display4, //Afficheur ligne 1
    Display5, //Afficheur ligne 1
    Display6, //Afficheur ligne 1
};

#define TypeBroadcast 0xFF
#define GroupBroadcast 0xFF
#define IndexGroupBroadcast 0xFF
#define IndexBoardBroadcast 0xFF

#define TRUE 0x01
#define FALSE 0x00

enum EnumCommand
{
    ShutterDown = 1,
    ShutterUp,
    ShutterStop,
    RelayOn,
    RelayOff,
    Pulse,
    AfficheString,
    AfficheProgress,
    NewAddress,
    NewPanID,
    NewGroup,
    HearthBeat
};

enum EnumState
{
    Iddle,
    Descending,
    Rising
};
#endif	/* DOMOPROTO_H */

