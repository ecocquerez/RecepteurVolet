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
    BYTE Group; // Num�ro de groupe
    BYTE IndexInGroup; //Num�ro de l'index dans le groupe
    BYTE IndexInBoard; //Num�ro de la fonctionnalit� sur la carte, sur une carte 8 relais, on peut s�lectionner le relais 3
    BYTE Command; //Commande � envoyer
    BYTE lenValue; //Longueur des donn�es � suivre
}Entete;

//Enum�ration des diff�rents devices
enum EnumDeviceType
{
    Shutter =1, //Commande de volet
    Relay,//Commande � relais, n'est command� que le temps de commande
    Contactor,//T�l�rupteur. 1 pulse -> On, 1 pulse Off
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

enum EnumCommand
{
    ShutterDownShort = 1,
    ShutterUpShort,
    ShutterDownLong,
    ShutterUpLong,
    RelayOn,
    RelayOff,
    Pulse,
    AfficheString,
    AfficheProgress,
};


#endif	/* DOMOPROTO_H */
