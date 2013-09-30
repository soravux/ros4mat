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

/* ********************
 * 0x0X => Commands
 * ******************** */

#define MSGID_PROTOCOL_VERSION 0x03

#define MSGID_HEADER_NOCOMPRESSION 0x00
#define MSGID_HEADER_ZLIBCOMPRESSION 0x01
 typedef struct msgHeader msgHeader;
 struct msgHeader{
    char type;                      /* Message type (MSGID_*) */
    char error;                     /* 0 if no error happened, otherwise contains the errur number */
                                    /* If an error happens, no data is sent and an ASCII error message is included after the header.
                                       its size is given by uncompressSize and compressSize. */
    uint32_t clientId;              /* Allows previously registered clients to reconnect to the same buffers in the case of connection lost.
                                       Set to 0 for the first connection. */
    uint32_t size;                  /* Number of data structures of type 'type' sent after this header */
    uint32_t uncompressSize;        /* Size (in bytes) of the uncompressed payload (after this header) */
    uint32_t compressSize;          /* Size (in bytes) of the compressed payload (after this header)*/
    char compressionType;           /* Compression type used by the payload of this packet. */
    double packetTimestamp;         /* This packet was sent at this timestamp (for performance ) */
 } PACKEDSTRUCT;

#define MSGID_CONNECT 0x01
 typedef struct msgConnect msgConnect;
 struct msgConnect{
    char protocolVersion;
    char compression;               /* ZLIB compression requested? */
} PACKEDSTRUCT;

#define MSGID_QUIT 0x02
/* No QUIT message required */

#define MSGID_SUBSCRIBE 0x03
typedef struct msgSubscribe msgSubscribe;
struct msgSubscribe{
    unsigned char typeCapteur;      /* Type de capteur voulu (MSGID_*) */
    char silentSubscribe;           /* Indique si on doit modifier la configuration ou non */
    uint32_t bufferSize;            /* Taille du buffer circulaire utilise en nombre de samples (sensor-independent) */
    uint32_t paramsSize;            /* Nombre d'octets de parametres supplementaires */
} PACKEDSTRUCT;

typedef struct paramsAdc paramsAdc;
struct paramsAdc{
    unsigned short freqAcquisition; /* Frequence d'acquisition en Hz */
    unsigned short freqSend;        /* Frequence d'envoi en Hz */
    unsigned char channels;         /* Canaux actives; chaque bit active un canal */
} PACKEDSTRUCT;

typedef struct paramsImu paramsImu;
struct paramsImu{
    unsigned short freqAcquisition; /* Frequence d'acquisition en Hz */
    unsigned short freqSend;        /* Frequence d'envoi en Hz */
} PACKEDSTRUCT;

typedef struct paramsGps paramsGps;
struct paramsGps{
    unsigned short freqAcquisition; /* Frequence d'acquisition en Hz */
    unsigned short freqSend;        /* Frequence d'envoi en Hz */
} PACKEDSTRUCT;

typedef struct paramsCamera paramsCamera;
struct paramsCamera{
    unsigned short fps;             /* [Don't move] Nombre d'images par seconde */
    unsigned short width;           /* [Don't move] Largeur de l'image acquise en pixels */
    unsigned short height;          /* [Don't move] Hauteur de l'image acquise en pixels */
    unsigned char id;               /* Id de la camera (p. ex. 0 pour /dev/video0) */
    unsigned char compression;      /* Compression de l'image (0 = pas de compression, 0<n<100 = JPEG) */
    short exposure;                 /* Manual camera exposure */
    unsigned char useROI;           /* Definit si on doit seulement envoyer une partie de l'image */
    unsigned short roiTopLeft;      /* Coordonnees de la sous-image a envoyer */
    unsigned short roiTopRight;
    unsigned short roiBottomLeft;
    unsigned short roiBottomRight;
} PACKEDSTRUCT;

typedef struct paramsStereoCam paramsStereoCam;
struct paramsStereoCam{
    unsigned short fps;             /* [Don't move] Nombre d'images par seconde */
    unsigned short width;           /* [Don't move] Largeur de l'image acquise en pixels */
    unsigned short height;          /* [Don't move] Hauteur de l'image acquise en pixels */
    unsigned char idLeft;           /* Id de la camera de GAUCHE (p. ex. 0 pour /dev/video0) */
    unsigned char idRight;          /* Id de la camera de DROITE */
    unsigned char compression;      /* Compression de l'image (0 = pas de compression, 0<n<100 = JPEG) */
    short exposureLeft;             /* Manual camera left exposure */
    short exposureRight;            /* Manual camera right exposure */
    unsigned char useROI;           /* Definit si on doit seulement envoyer une partie de l'image */
    unsigned short leftRoiTopLeft;  /* Coordonnees de la sous-image a envoyer pour la camera de GAUCHE */
    unsigned short leftRoiTopRight;     
    unsigned short leftRoiBottomLeft;
    unsigned short leftRoiBottomRight;
    unsigned short rightRoiTopLeft; /* Coordonnees de la sous-image a envoyer pour la camera de DROITE */
    unsigned short rightRoiTopRight;     
    unsigned short rightRoiBottomLeft;
    unsigned short rightRoiBottomRight;
} PACKEDSTRUCT;

typedef struct paramsKinect paramsKinect;
struct paramsKinect{
    unsigned short fpsRGB;          /* [Don't move] Nombre d'images par seconde */
    unsigned short widthRGB;        /* [Don't move] Largeur de l'image acquise en pixels */
    unsigned short heightRGB;       /* [Don't move] Hauteur de l'image acquise en pixels */
    unsigned char sendRGB;          /* Acquisitionner ou non l'image RGB */
    unsigned char id;               /* Id de la Kinect */
    unsigned char sendDepth;        /* Acquisitionner ou non l'image de profondeur */
    unsigned short fpsDepth;        /* Nombre d'images par seconde */
    unsigned char compressionRGB;   /* Compression de l'image RGB (0 = pas de compression, 0<n<100 = JPEG) */
} PACKEDSTRUCT;

typedef struct paramsHokuyo paramsHokuyo;
struct paramsHokuyo{
    unsigned short freqAcquisition; /* Frequence d'acquisition en Hz */
} PACKEDSTRUCT;

typedef struct paramsComputer paramsComputer;
struct paramsComputer{
    unsigned short freqAcquisition; /* Frequence d'acquisition en Hz */
} PACKEDSTRUCT;


#define MSGID_UNSUBSCRIBE 0x04
typedef struct msgUnsubscribe msgUnsubscribe;
struct msgUnsubscribe{
    unsigned char typeCapteur;      /* Type de capteur voulu (MSGID_*) */
} PACKEDSTRUCT;

/* A serial_cmd message is build like this:
 *   [Header | port | send data]   */
#define MSGID_SERIAL_CMD 0x05
typedef struct msgSerialCmd msgSerialCmd;
struct msgSerialCmd{
    uint32_t portBufferLength;
    uint32_t speed;
    bool parity;
    unsigned char stopBits;
    uint32_t sendLength;
    uint32_t readLength;
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

/* ********************
 * 0x1X => Acknowledges
 * ******************** */

#define MSGID_CONNECT_ACK 0x11
#define MSGID_SUBSCRIBE_ACK 0x13

/* ********************
 * 0x2X => Sensors
 * ******************** */

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
#define MSGID_WEBCAM_NOCOMPRESSION 0x00
/* Cette structure est speciale : elle contient la largeur, la hauteur et le nombre de canaux
 * de l'image. Elle contient aussi un sizeData, qui indique combien d'octets sont a recevoir
 * pour transmettre l'image (normalement width*height*channels).
 * Le paquet ressemble donc a :
 * | msgHeader | msgCam | data_raw | msgCam | data_raw | ... ... ... |
 * sizeData contient le nombre d'octets de data_raw
 */
 typedef struct msgCam msgCam;
 struct msgCam{
    uint32_t width;
    uint32_t height;
    unsigned char channels;
    uint32_t sizeData;
    char compressionType;
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
    uint32_t dataLength;
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
/* Aggregation of two msgCam structs. The msgKinect elements are all packed up right after the msgHeader
 * and then interleaved data (RGB + Depth) is added as a payload. Packets looks like this:
 * | msgHeader | msgKinect | msgKinect | ... | dataRGB | dataDepth | dataRGB | dataKinect | ... | ... |
 * If one of the two images aren't requested or available, its size is set to 0 in its related msgCam (inside the msgKinect)
 * The receiver can assume that both images described in msgKinect are synchronized.
 */
typedef struct msgKinect msgKinect;
struct msgKinect{
    msgCam infoRGB;
    msgCam infoDepth;
} PACKEDSTRUCT;

#define MSGID_WEBCAM_STEREO 0x2A
