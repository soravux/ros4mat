/* DESCRIPTION DES MESSAGES 
 * Les types de messages sont definis comme suit :
 * -> 0x0* : Message de commande au node ROS (connexion, subscribe, etc.), envoyes par
 *          Matlab.
 * -> 0x1* : Ack ou reponse des commandes 0x0* (envoyes par le node ROS)
 * -> 0x2* et 0x3* : Message d'un capteur, du robot vers Matlab
 * -> 0x8* et 0x9* : Message a envoyer vers un capteur/driver (Matlab envoie une commande)
 * 
 * Comme on peut le remarquer, ces messages sont a la fois ceux allant de ROS vers Matlab
 * et ceux allant de Matlab vers ROS.
 * 
 * La connexion s'etablit comme ceci :
 * 1) Matlab envoie une struct msgConnect au robot sur le port 1500
 * 2) Le node ROS repond par un message contenant une struct msgConnectAck. Cette struct
 *      contient entre autres un ID de client QUE MATLAB DOIT CONSERVER. C'est avec cet
 *      ID que ROS peut servir plusieurs clients Matlab (par exemple un sur le robot et
 *      un autre en remote). L'adresse IP n'est AMHA pas assez fiable.
 * 3) Matlab peut alors envoyer des struct msgSubscribe pour recevoir les infos souhaitees
 * ...
 * 4) Matlab envoie une struct msgQuit pour cesser les envois et vider les buffers. Son id
 *      est alors invalide (il ne peut plus etre utilise sans faire un nouveau connect).
 */

#if defined (__GNUC__)
#include <stdint.h>
    #define PACKEDSTRUCT __attribute__((packed))
#elif defined (__WIN32__) || (_WIN32) || (__CYGWIN32__) || (__WIN64__) || (_WIN64)
    #define PACKEDSTRUCT
    #pragma pack(1)
#endif

/*******************************************************************************
 *                                   Enums
 ******************************************************************************/

enum compression_type {
    R4M_COMP_NONE,
    R4M_COMP_JPEG,
    R4M_COMP_ZLIB
};

/*******************************************************************************
 *                              Packet Headers
 ******************************************************************************/

#define MSGID_HEADER_NOCOMPRESSION 0x00
#define MSGID_HEADER_ZLIBCOMPRESSION 0x01
 typedef struct msgHeader msgHeader;
 struct msgHeader{
    char type;                      /* Type du message (MSGID_*) */
    char info;                      /* Information supplementaire (vide) */
    uint32_t size;                  /* Nombre de structures du type 'type' envoyees */
    uint32_t uncompressSize;        /* Size (en octets) du RESTE du message non compresse */
    uint32_t compressSize;          /* Size (en octets) du RESTE du message qui est compresse */
    char compressionType;           /* Type de compression utilise dans le paquet */
    double packetTimestamp;         /* Timestamp de l'envoi du paquet (pour les mesures de perf) */
 } PACKEDSTRUCT;

#define MSGID_CONNECT 0x01
 typedef struct msgConnect msgConnect;
 struct msgConnect{
    char protocolVersion;           /* Toujours 1 */
    char compression;
} PACKEDSTRUCT;

#define MSGID_QUIT 0x02
/* Aucun message de QUIT necessaire */

#define MSGID_SUBSCRIBE 0x03
typedef struct msgSubscribe msgSubscribe;
struct msgSubscribe{
    unsigned char typeCapteur;      /* Type de capteur voulu (MSGID_*) */
    unsigned char paramSupp;        /* Parametres supplementaires : */
                                    /* - Pour l'ADC : le canal (chaque bit active un canal) */
                                    /* - ... */
    unsigned char paramSupp2;       /* Parametre supplementaire 2 : ID de la camera */
    unsigned short freqAcquisition; /* Frequence d'acquisition en Hz */
    unsigned short freqSend;        /* Frequence d'envoi en Hz */
    uint32_t bufferSize;            /* Taille du buffer circulaire utilise */
} PACKEDSTRUCT;

#define MSGID_UNSUBSCRIBE 0x04
typedef struct msgUnsubscribe msgUnsubscribe;
struct msgUnsubscribe{
    unsigned char typeCapteur;      /* Type de capteur voulu (MSGID_*) */
} PACKEDSTRUCT;

#define MSGID_SERIAL_CMD 0x05
typedef struct msgSerialCmd msgSerialCmd;
struct msgSerialCmd{
    char port[100];
    uint32_t speed;
    bool parity;
    unsigned char stopBits;
    char data[1024];
    short sendLength;
    short readLength;
    short readTimeoutSec;
    uint32_t readTimeoutMicro;
    bool closeAfterComm;
} PACKEDSTRUCT;

#define MSGID_DOUT_CTRL 0x06
typedef struct msgDigitalOut msgDigitalOut;
struct msgDigitalOut{
    char pinD0;
    char pinD1;
    char pinD2;
    char pinD3;
} PACKEDSTRUCT;

#define MSGID_CONNECT_ACK 0x10
/* Aucun message de CONNECT_ACK necessaire */

#define MSGID_ADC 0x20
typedef struct msgAdc msgAdc;
struct msgAdc{
    float canal1;                   /* Valeur lue (en V) */
    float canal2;                   /* Valeur lue (en V) */
    float canal3;                   /* Valeur lue (en V) */
    float canal4;                   /* Valeur lue (en V) */
    float canal5;                   /* Valeur lue (en V) */
    float canal6;                   /* Valeur lue (en V) */
    float canal7;                   /* Valeur lue (en V) */
    float canal8;                   /* Valeur lue (en V) */
    double timestamp;               /* Timestamp de la MESURE */
} PACKEDSTRUCT;

#define MSGID_IMU 0x21
typedef struct msgImu msgImu;
struct msgImu{
    float acceleration[3];          /* Valeur de l'acceleration en g (x,y,z) */
    float gyro[3];                  /* Valeur de la rotation en deg/s (x,y,z) */
    float posAngulaire[3];          /* Valeur du magnetometre en uT (x,y,z) */
    double timestamp;               /* Timestamp de la MESURE */
} PACKEDSTRUCT;

#define MSGID_BATTERY 0x22
typedef struct msgBattery msgBattery;
struct msgBattery{
    unsigned char state;            /* Etat de la batterie (0x00 = inactive, 0x01 = charge, etc.) */
    float cellsVoltage[6];          /* Tension des 6 cellules */
    float currentDrained;           /* Courant tire de la batterie */
    double timestamp;               /* Timestamp de la MESURE */
} PACKEDSTRUCT;

#define MSGID_GPS 0x23
#define MSGID_GPS_STATE_NOFIX 0x00
#define MSGID_GPS_STATE_FIX2D 0x10
#define MSGID_GPS_STATE_FIX3D 0x20
typedef struct msgGps msgGps;
struct msgGps{
    char state;                     /* Etat du GPS (0x00 = No fix, 0x10 = fix 2d, 0x20 = fix 3d) */
    float latitude;                 /* Latitude en degres */
    float longitude;                /* Longitude en degres */
    float altitude;                 /* Altitude en m */
    float speedModule;              /* Module de la vitesse horizontale */
    float speedAngle;               /* Angle de la vitesse horizontale */
    float verticalSpeed;            /* Vitesse verticale */
    double timestamp;               /* Timestamp de la MESURE */
} PACKEDSTRUCT;

#define MSGID_WEBCAM 0x24
/* Cette structure est speciale : elle contient la largeur, la hauteur et le nombre de canaux
 * de l'image. Elle contient aussi un sizeData, qui indique combien d'octets sont a recevoir
 * pour transmettre l'image (normalement width*height*channels).
 * Le paquet ressemble donc a :
 * | msgHeader | msgCam | data_raw ... ... ... |
 * sizeData contient le nombre d'octets de data_raw
 */
 typedef struct msgCam msgCam;
 struct msgCam{
    uint32_t width;
    uint32_t height;
    unsigned char channels;
    uint32_t sizeData;
    uint8_t compressionType;
    double timestamp;           /* Timestamp de l'acquisition de l'image */
} PACKEDSTRUCT;

#define MSGID_SERIAL_ANS 0x25
#define MSGID_SERIAL_ANS_OK 0x00
#define MSGID_SERIAL_ANS_NO_OPEN 0x10
#define MSGID_SERIAL_ANS_TIMEOUT 0x20
#define MSGID_SERIAL_ANS_BAD_SELECT 0x30
typedef struct msgSerialAns msgSerialAns;
struct msgSerialAns{
    char status;
    uint32_t bufferLength;
    char data[1024];
} PACKEDSTRUCT;

#define MSGID_COMPUTER 0x26
#define MSGID_COMPUTER_STATE_OK 0x00
#define MSGID_COMPUTER_STATE_ERROR 0x10
typedef struct msgComputer msgComputer;
struct msgComputer{
    char state;                     /* Etat de l'ordinateur */
    float cpuTemperature;           /* Temperature en degres Celcius du CPU */
    float unixLoad[3];              /* Load UNIX de l'ordinateur */
    float cpuLoad[7];               /* User, Sys, Nice, Idle, IoWait, Hardware IRQ, Software Interrupts */
    uint32_t memUsed[3];            /* Memoire utilisee [used, free, buffers]*/
    double timestamp;
} PACKEDSTRUCT;

#define MSGID_HOKUYO 0x27
typedef struct msgHokuyo msgHokuyo;
struct msgHokuyo{
    float angleMin;
    float angleMax;
    float angleIncrement;
    float timeIncrement;
    float rangeMin;
    float rangeMax;
    double timestamp;
    uint32_t sizeData;
} PACKEDSTRUCT;

#define MSGID_KINECT_DEPTH 0x28
#define MSGID_KINECT 0x29           /* Represents [RGB + Depth] in Matlab and RGB only in agent buffer */

#define MSGID_WEBCAM_STEREO 0x2A
