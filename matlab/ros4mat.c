/*
 *    This file is part of ros4mat.
 *
 *    ros4mat is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of
 *    the License, or (at your option) any later version.
 *
 *    ros4mat is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ros4mat. If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * To compile on Windows, on a MATLAB shell:
 * mex ros4mat.c ../thirdparty/easyzlib.c ../thirdparty/picojpeg.c wsock32.lib
 *
 * To compile on Linux, on a MATLAB shell:
 * mex ros4mat.c ../thirdparty/easyzlib.c ../thirdparty/picojpeg.c
*/
#include <stdlib.h>
#include <sys/types.h>
#include <ctype.h>
#include <limits.h>
#if defined(__WIN32__) || (_WIN32) || (__CYGWIN32__) || (__WIN64__) || (_WIN64)

/* Because there is no inttypes.h for portable variables (you need C-99), here I define this abomination */
#if UCHAR_MAX == 4294967295
#define uint32_t    unsigned char
#elif USHRT_MAX == 4294967295
#define uint32_t    unsigned short
#elif UINT_MAX == 4294967295
#define uint32_t    unsigned int
#elif ULONG_MAX == 4294967295
#define uint32_t    unsigned long
#else
#define uint32_t    CANT_FIND_32BIT_INT /* Will generate a compile error downstream */
#endif
#if UCHAR_MAX == 65535
#define uint16_t    unsigned char
#elif USHRT_MAX == 65535
#define uint16_t    unsigned short
#elif UINT_MAX == 65535
#define uint16_t    unsigned int
#elif ULONG_MAX == 65535
#define uint16_t    unsigned long
#else
#define uint16_t    CANT_FIND_8BIT_INT    /* Will generate a compile error downstream */
#endif

/* Networking on Windows */
#define USE_WINSOCK
#include <winsock2.h>

#define socklen_t    int
#define close        closesocket            /* close() doesn't exist on windows, it's closesocket for winsock2 */
#define ioctl        ioctlsocket
#pragma pack(1)
#else

/* Networking on Linux */
#define USE_BSD
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>

#define SOCKET_ERROR    - 1
#include <inttypes.h>
#include <errno.h>
#include <unistd.h>
#endif
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <memory.h>
#include "mex.h"

#include "../exchangeStructs.h"
#include "../thirdparty/easyzlib.h"
#include "../thirdparty/picojpeg.h"

#define REMOTE_SERVER_PORT    1500

/* Global variables definition */
static int                    initialized = 0;
static struct sockaddr_in     *sock_in = NULL;
static int                    *main_socket = NULL;


/*******************************************************************************
 *
 *                        Initialization and cleanup
 *
 ******************************************************************************/


#if defined USE_WINSOCK

/* Winsock 2.0 initialisation (Windows) */
int init_winsock()
{
    WSADATA wsaData;
    WORD    version;
    int     ws_error;

    version = MAKEWORD(2, 0);

    ws_error = WSAStartup(version, &wsaData);

    /* check for error */
    if(ws_error != 0)
    {
        /* error occured */
        return FALSE;
    }

    /* check for correct version */
    if(LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 0)
    {
        /* incorrect WinSock version */
        WSACleanup();
        return FALSE;
    }

    return 0;
}
#endif /* #if defined USE_WINSOCK */

/* */

void logico_close()
{
    char            *msg;
    msgHeader       lHeader;
    unsigned int    i = 0;

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    msg = (char *) mxCalloc(1, sizeof(msgHeader));
    lHeader.type = MSGID_QUIT;
    lHeader.info = 0x00;
    lHeader.size = 0;
    lHeader.packetTimestamp = 0.0;

    memcpy(msg, &lHeader, sizeof(msgHeader));

    send(*main_socket, msg, sizeof(msgHeader), 0);

    close(*main_socket);

#if defined USE_WINSOCK
    WSACleanup();
#endif
    mxFree(sock_in);
    mxFree(main_socket);
    initialized = 0;
}

/* */
void cleanup()
{
    /* To be done on the parent MATLAB onClose software */
    if(initialized != 0)
    {
        logico_close();
    }
}

/* */
void logico_start(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    unsigned int        i = 0;
    int                 h = 0;
    char                *msg = NULL;
    msgConnect          lConnectMessage;
    msgHeader           lHeader;
    struct sockaddr_in  cliAddr;

    struct timeval      tv;
    int                 nagle_tempo = 1;

    int                 flags = 0;
    char                flag = 1;
    int                 result;
    int                 sockaddr_in_length = sizeof(struct sockaddr_in);
    char                compressionFlag = MSGID_HEADER_NOCOMPRESSION;

    if(initialized != 0)
    {
        mexWarnMsgTxt("Connection already established.");
        return;
    }

    msg = (char *) mxCalloc(1, sizeof(msgHeader) + sizeof(msgConnect));
    lHeader.type = MSGID_CONNECT;
    lHeader.info = 0x00;
    lHeader.size = 1;
    lHeader.packetTimestamp = 0.0;
    lConnectMessage.protocolVersion = 1;

    lConnectMessage.compression =
        (
            nrhs > 1
        &&    (uint16_t) mxGetScalar(prhs[1]) == 1
        ) ? MSGID_HEADER_ZLIBCOMPRESSION : MSGID_HEADER_NOCOMPRESSION;

#if defined USE_WINSOCK
    if(init_winsock() != 0) mexErrMsgTxt("Winsock initialisation error.");
#endif
    main_socket = (int *) mxCalloc(1, sizeof(int));
    sock_in = (struct sockaddr_in *) mxCalloc(1, sizeof(struct sockaddr_in));
    mexMakeMemoryPersistent(sock_in);
    mexMakeMemoryPersistent(main_socket);

    /* mexMakeMemoryPersistent(&initialized); */
    mexAtExit(cleanup);

    result = setsockopt(*main_socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
    (*main_socket) = (int) socket(AF_INET, SOCK_STREAM, 0);

    tv.tv_sec = 10;
    tv.tv_usec = 0;

    /* Gestion du endpoint local */
    cliAddr.sin_family = AF_INET;
    cliAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    cliAddr.sin_port = htons(0);

    /* Gestion du endpoint remote (Robot) */
    sock_in->sin_family = AF_INET;
    sock_in->sin_port = htons(REMOTE_SERVER_PORT);
    if(nrhs < 1)
    {
        mexWarnMsgTxt("Defaut host used (127.0.0.1).\n");
        sock_in->sin_addr.s_addr = inet_addr("127.0.0.1");
    }
    else
    {
        char    ip[16] = { 0 };
        if(mxGetString(prhs[0], ip, sizeof(ip) - 1)) mexErrMsgTxt("IP extraction impossible");
        sock_in->sin_addr.s_addr = inet_addr(ip);
    }

    if(bind(*main_socket, (struct sockaddr *) &cliAddr, sizeof(struct sockaddr_in)) == SOCKET_ERROR)
    {
#if defined USE_WINSOCK
        mexPrintf("Winsock error: %i\n", WSAGetLastError());
#endif
        mexErrMsgTxt("Error binding TCP port.");
    }

    h = connect(*main_socket, (struct sockaddr *) sock_in, sizeof(struct sockaddr_in));
    if(h < 0)
    {
#if defined USE_WINSOCK
        mexPrintf("Winsock error: %i\n", WSAGetLastError());
#endif
        mexErrMsgTxt("Error connecting to ros4mat.");
    }

    memcpy(msg, &lHeader, sizeof(msgHeader));
    memcpy(msg + sizeof(msgHeader), &lConnectMessage, sizeof(msgConnect));

    send(*main_socket, msg, sizeof(msgHeader) + sizeof(msgConnect), 0);

    /* Verification du message recu */
    recv(*main_socket, msg, sizeof(msgHeader), 0);
    memcpy(&lHeader, msg, sizeof(msgHeader));
    if(lHeader.type != MSGID_CONNECT_ACK) mexErrMsgTxt("Incompatible ros4mat answer.");

    initialized++;
}

struct types_capteurs_textuel
{
    char    *nom;
    char    typeCapteur;
}
tctext[] =
{
    { "adc", MSGID_ADC },
    { "gps", MSGID_GPS },
    { "imu", MSGID_IMU },
    { "batterie", MSGID_BATTERY },
    { "camera", MSGID_WEBCAM },
    { "camera_stereo", MSGID_WEBCAM_STEREO },
    { "hokuyo", MSGID_HOKUYO },
    { "kinect", MSGID_KINECT },
    { "kinect_ir", MSGID_KINECT_DEPTH },
    { "ordinateur", MSGID_COMPUTER }
};


/*******************************************************************************
 *
 *                           Image helpers (JPEG)
 *
 ******************************************************************************/

/* This is the most NON-thread-safe code you have ever seen */

char* jpeg_datastream;
int jpeg_size = 0;
int jpeg_pos = 0;

int min ( int a, int b ) { return a < b ? a : b; }

unsigned char pjpeg_need_bytes_callback(unsigned char* pBuf, unsigned char buf_size, unsigned char *pBytes_actually_read, void *pCallback_data)
{
    uint32_t n = min((uint32_t)jpeg_size - jpeg_pos, (uint32_t)buf_size);
    memcpy(pBuf, jpeg_datastream + jpeg_pos, n);
    jpeg_pos += n;
    return 0;
}


void decodeJPEG(char *inStream, uint32_t dataSize, uint32_t *outImage)
{
    pjpeg_image_info_t imageInfo;
    jpeg_datastream = inStream;
    jpeg_size = dataSize;
    int status = pjpeg_decode_init(&imageInfo, pjpeg_need_bytes_callback, NULL, 0);
    /* const unsigned int row_pitch = imageInfo.m_width * imageInfo.m_comps; */
    int x, y, bx, by;
    int mcu_x = 0;
    int mcu_y = 0;

    for ( ; ; ) {
        status = pjpeg_decode_mcu();

        /* Handle case when its done. */
        if (status) {
            if (status != PJPG_NO_MORE_BLOCKS) { return; }
            break;
        }
        if (mcu_y >= imageInfo.m_MCUSPerCol) { return; }

        /* Copy MCU's pixel blocks into the destination bitmap. */
        for (y = 0; y < imageInfo.m_MCUHeight; y += 8) {
            const int by_limit = min(8, imageInfo.m_height - (mcu_y * imageInfo.m_MCUHeight + y));
            for (x = 0; x < imageInfo.m_MCUWidth; x += 8) {

                unsigned int src_ofs = (x * 8U) + (y * 16U);
                const unsigned char *pSrcR = imageInfo.m_pMCUBufR + src_ofs;
                const unsigned char *pSrcG = imageInfo.m_pMCUBufG + src_ofs;
                const unsigned char *pSrcB = imageInfo.m_pMCUBufB + src_ofs;

                const int bx_limit = min(8, imageInfo.m_width - (mcu_x * imageInfo.m_MCUWidth + x));

                if (imageInfo.m_scanType == PJPG_GRAYSCALE) {
                    for (by = 0; by < by_limit; by++) {
                        for (bx = 0; bx < bx_limit; bx++) {
                            uint32_t color = ((*pSrcR++) << 16);
                            outImage[mcu_x*imageInfo.m_MCUWidth + x + bx
                                     + mcu_y*imageInfo.m_MCUHeight + y + by] = color;
                        }
                        pSrcR += (8 - bx_limit);
                    }
                } else {
                    for (by = 0; by < by_limit; by++) {
                        for (bx = 0; bx < bx_limit; bx++) {
                            uint32_t color = ((*pSrcR++) << 16) | ((*pSrcG++) << 8) | (*pSrcB++);
                            outImage[mcu_x*imageInfo.m_MCUWidth + x + bx
                                     + mcu_y*imageInfo.m_MCUHeight + y + by] = color;
                        }

                        pSrcR += (8 - bx_limit);
                        pSrcG += (8 - bx_limit);
                        pSrcB += (8 - bx_limit);
                    }
                }
            }
        }

        mcu_x++;
        if (mcu_x == imageInfo.m_MCUSPerRow) {
            mcu_x = 0;
            mcu_y++;
        }
    }
}

/*******************************************************************************
 *
 *                               ROS4MAT API
 *
 ******************************************************************************/

/* */

void logico_subscribe(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg;
    msgHeader       lHeader;
    msgSubscribe    lSubscribeMessage;
    unsigned int    h;
    char            StrBuffer[65];

    /* Initialize struct variables to zero. */
    memset(&lSubscribeMessage, 0, sizeof(msgSubscribe));

    if(nrhs < 1) mexErrMsgTxt("Please specify a valid node.");
    if(nrhs > 1)
    {
        lSubscribeMessage.freqAcquisition = (uint16_t) mxGetScalar(prhs[1]);
        lSubscribeMessage.freqSend = (uint16_t) ((float) lSubscribeMessage.freqAcquisition / 4.0 + 10.0);
        lSubscribeMessage.bufferSize = lSubscribeMessage.freqAcquisition;
    }

    if(nrhs > 6) mexWarnMsgTxt("Cannot understand request: Too much arguments.");
    if(nrhs > 4) lSubscribeMessage.freqSend = (uint16_t) mxGetScalar(prhs[4]);
    if(nrhs > 3) lSubscribeMessage.bufferSize = (uint32_t) mxGetScalar(prhs[3]);
    if(nrhs > 2)
    {
        if(!mxIsChar(prhs[2]))
        {
            lSubscribeMessage.paramSupp = (signed char) mxGetScalar(prhs[2]);
        }
        else
        {
            /* Gestion de la camera */
            if(mxGetString(prhs[2], StrBuffer, sizeof(StrBuffer) - 1)) mexErrMsgTxt("Error: Could not interpretate.");

            lSubscribeMessage.paramSupp = (int) atoi(StrBuffer) / 20;

            if(lSubscribeMessage.paramSupp < 8 || lSubscribeMessage.paramSupp > 80)
            {
                mexWarnMsgTxt("Unknown resolution. Reverting to default resolution (320x240).");
                lSubscribeMessage.paramSupp = 16;
            }

            if(nrhs > 5) lSubscribeMessage.paramSupp2 = (unsigned char) mxGetScalar(prhs[5]);
        }
    }

    if(lSubscribeMessage.freqSend > lSubscribeMessage.freqAcquisition)
    {
        lSubscribeMessage.freqSend = lSubscribeMessage.freqAcquisition;
    }

    if(initialized == 0) mexErrMsgTxt("No connection established.");
    if(!mxIsChar(prhs[0])) mexErrMsgTxt("'subscribe' must be followed with a node name.");

    if(mxGetString(prhs[0], StrBuffer, sizeof(StrBuffer) - 1))
        mexWarnMsgTxt("Cannot understand request: String conversion failed.");

    for(h = 0; h < sizeof(tctext) / sizeof(tctext[0]); h++)
    {
        if(!strcmp(tctext[h].nom, StrBuffer))
        {
            lSubscribeMessage.typeCapteur = tctext[h].typeCapteur;
            if(h == 4 && (lSubscribeMessage.paramSupp < 8 || lSubscribeMessage.paramSupp > 80))
            {
                mexWarnMsgTxt("Using default resolution (320x240).");
                lSubscribeMessage.paramSupp = 16;
            }

            h = 0;
            break;
        }
    }

    if(h != 0) mexErrMsgTxt("Unsupported node.");

    msg = (char *) mxCalloc(1, sizeof(msgHeader) + sizeof(msgSubscribe));
    lHeader.type = MSGID_SUBSCRIBE;
    lHeader.info = 0x00;
    lHeader.size = 1;
    lHeader.packetTimestamp = 0.0;

    memcpy(msg, &lHeader, sizeof(msgHeader));
    memcpy(msg + sizeof(msgHeader), &lSubscribeMessage, sizeof(msgSubscribe));

    send(*main_socket, msg, sizeof(msgHeader) + sizeof(msgSubscribe), 0);
}

/* */
void logico_unsubscribe(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg;
    msgHeader       lHeader;
    msgUnsubscribe  lUnsubscribeMessage;
    char            StrBuffer[65];
    unsigned int    h = 0;

    if(nrhs > 2) mexWarnMsgTxt("Cannot understand request: Too much arguments.");
    if(initialized == 0) mexErrMsgTxt("No connection established.");
    if(!mxIsChar(prhs[0])) mexErrMsgTxt("'unsubscribe' must be followed with a node name.");

    if(mxGetString(prhs[0], StrBuffer, sizeof(StrBuffer) - 1))
        mexWarnMsgTxt("Cannot understand request: String conversion failed.");

    for(h = 0; h < sizeof(tctext) / sizeof(tctext[0]); h++)
    {
        if(!strcmp(tctext[h].nom, StrBuffer))
        {
            lUnsubscribeMessage.typeCapteur = tctext[h].typeCapteur;
            h = 0;
            break;
        }
    }

    if(h != 0) mexErrMsgTxt("Unsupported node.");

    msg = (char *) mxCalloc(1, sizeof(msgHeader) + sizeof(msgUnsubscribe));
    lHeader.type = MSGID_UNSUBSCRIBE;
    lHeader.info = 0x00;
    lHeader.size = 1;
    lHeader.packetTimestamp = 0.0;

    memcpy(msg, &lHeader, sizeof(msgHeader));
    memcpy(msg + sizeof(msgHeader), &lUnsubscribeMessage, sizeof(msgUnsubscribe));

    send(*main_socket, msg, sizeof(msgHeader) + sizeof(msgUnsubscribe), 0);
}

/* */
msgHeader logico_send_data_request(char inType, char **msg, unsigned int inStructSize)
{
    /* Le double pointeur sur msg est obligatoire pour le C */
    msgHeader       lHeader;
    char            *msgCompress;
    char            *msgEmpty;
    int             i = 0;
    unsigned int    recvBytes = 0;
    fd_set          read_fds;
    unsigned int    expectedRecvSize = 0;
    long            uncompressSize = 0;
    struct timeval  timeout;

    /* Flusher le buffer de reception */
    ioctl(*main_socket, FIONREAD, &recvBytes);
    if(recvBytes > 0)
    {
        msgEmpty = (char *) mxMalloc(recvBytes);
        recv(*main_socket, msgEmpty, recvBytes, 0);
        mxFree(msgEmpty);
    }

    recvBytes = 0;

    /* Envoyer un paquet vide demandant les donnees d'un capteur */
    *msg = (char *) mxCalloc(1, sizeof(msgHeader));
    lHeader.type = inType;
    lHeader.info = 0x00;
    lHeader.size = 0;
    lHeader.packetTimestamp = 0.0;

    memcpy(*msg, &lHeader, sizeof(msgHeader));
    send(*main_socket, *msg, sizeof(msgHeader), 0);

    FD_ZERO(&read_fds);
    FD_SET(*main_socket, &read_fds);
    while(recvBytes < sizeof(msgHeader))
    {
        timeout.tv_sec = 30;
        timeout.tv_usec = 0;
        i = select(*main_socket + 1, &read_fds, NULL, NULL, &timeout);
        if(i < 0)
        {
            mexErrMsgTxt("Network error.");
        }
        else if(i == 0)
        {
            mexErrMsgTxt("Network timeout error.");
        }

        /* Reception des donnes du capteur */
        recvBytes += recv(*main_socket, *msg + recvBytes, sizeof(msgHeader), 0);
    }

    memcpy(&lHeader, *msg, sizeof(msgHeader));
    if(lHeader.type != inType)
    {
        mexErrMsgTxt("Incompatible ros4mat answer.");
    }

    if(lHeader.size == 0) return lHeader;

    /* On recoit le buffer des donnees, champ compressSize du header */
    expectedRecvSize = lHeader.compressSize;
    msgCompress = (char *) mxCalloc(1, expectedRecvSize);
    recvBytes = 0;
    while(recvBytes < expectedRecvSize)
    {
        timeout.tv_sec = 30;
        timeout.tv_usec = 0;
        i = select(*main_socket + 1, &read_fds, NULL, NULL, &timeout);
        if(i < 0)
        {
            mexErrMsgTxt("Network error.");
        }
        else if(i == 0)
        {
            mexErrMsgTxt("Network timeout error.");
        }

        recvBytes += recv(*main_socket, msgCompress + recvBytes, expectedRecvSize - recvBytes, 0);
    }

    *msg = (char *) mxCalloc(1, sizeof(msgHeader) + lHeader.uncompressSize);
    memcpy(*msg, &lHeader, sizeof(msgHeader));

    /* Decompression */
    if(lHeader.compressionType == MSGID_HEADER_NOCOMPRESSION)
    {
        memcpy(*msg + sizeof(msgHeader), msgCompress, expectedRecvSize);
    }
    else if(lHeader.compressionType == MSGID_HEADER_ZLIBCOMPRESSION)
    {
        uncompressSize = lHeader.uncompressSize;
        i = ezuncompress
            (
                *msg + sizeof(msgHeader),
                &uncompressSize,
                (unsigned char *) msgCompress,
                lHeader.compressSize
            );
        if(i < 0)
        {
            mexErrMsgTxt("Corruption error while uncompressing data!");
        }

        if((int) uncompressSize != lHeader.uncompressSize)
        {
            mexErrMsgTxt("Uncompressing data size mismatch.");
        }
    }

    mxFree(msgCompress);

    return lHeader;
}

/* */
void logico_battery(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    msgHeader       lHeader;
    unsigned int    h, i;
    msgBattery      lBatteryMessage;
    double          lPercentBattery = 0;

    double          *out_data, *out_data_ts;

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_BATTERY, &msg, sizeof(msgBattery));

    /* Formattage pour Matlab */
    plhs[0] = mxCreateDoubleMatrix(9, lHeader.size, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    out_data = (double *) mxGetPr(plhs[0]);
    out_data_ts = (double *) mxGetPr(plhs[1]);

    /* Approximation de la charge de la batterie */
    for(h = 0; h < lHeader.size; h++)
    {
        memcpy(&lBatteryMessage, msg + sizeof(msgHeader) + h * sizeof(msgBattery), sizeof(msgBattery));
        lPercentBattery = (double)
            (
                (double) lBatteryMessage.cellsVoltage[0] +
                (double) lBatteryMessage.cellsVoltage[1] +
                (double) lBatteryMessage.cellsVoltage[2] +
                (double) lBatteryMessage.cellsVoltage[3] +
                (double) lBatteryMessage.cellsVoltage[4] +
                (double) lBatteryMessage.cellsVoltage[5]
            ) * 0.3 - 6.41;
        if(lPercentBattery > 1)
        {
            lPercentBattery = 1;
        }
        else if(lPercentBattery < 0.05)
        {
            lPercentBattery = 0.05;
        }

        for(i = 0; i < 6; i++)
        {
            if(lBatteryMessage.cellsVoltage[i] < 3.0)
            {
                lBatteryMessage.state |= 0x100;
            }

            if(lBatteryMessage.cellsVoltage[i] < 3.6)
            {
                lBatteryMessage.state |= 0x200;
            }
        }

        out_data[9 * h + 0] = (double) lBatteryMessage.state;
        out_data[9 * h + 1] = (double) lPercentBattery;
        out_data[9 * h + 2] = (double) lBatteryMessage.cellsVoltage[0];
        out_data[9 * h + 3] = (double) lBatteryMessage.cellsVoltage[1];
        out_data[9 * h + 4] = (double) lBatteryMessage.cellsVoltage[2];
        out_data[9 * h + 5] = (double) lBatteryMessage.cellsVoltage[3];
        out_data[9 * h + 6] = (double) lBatteryMessage.cellsVoltage[4];
        out_data[9 * h + 7] = (double) lBatteryMessage.cellsVoltage[5];
        out_data[9 * h + 8] = (double) lBatteryMessage.currentDrained;
        out_data_ts[h] = (double) lBatteryMessage.timestamp;
    }

    mxFree(msg);
}

/* */
void logico_imu(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    msgImu          lImu;
    msgHeader       lHeader;
    unsigned int    h;
    double          *out_data, *out_data_ts;
    char            *msg = NULL;

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_IMU, &msg, sizeof(msgImu));

    /* Formattage pour Matlab */
    plhs[0] = mxCreateDoubleMatrix(9, lHeader.size, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    out_data = (double *) mxGetPr(plhs[0]);
    out_data_ts = (double *) mxGetPr(plhs[1]);

    for(h = 0; h < lHeader.size; h++)
    {
        memcpy(&lImu, msg + sizeof(msgHeader) + h * sizeof(msgImu), sizeof(msgImu));
        out_data[9 * h + 0] = (double) lImu.acceleration[0];
        out_data[9 * h + 1] = (double) lImu.acceleration[1];
        out_data[9 * h + 2] = (double) lImu.acceleration[2];
        out_data[9 * h + 3] = (double) lImu.gyro[0];
        out_data[9 * h + 4] = (double) lImu.gyro[1];
        out_data[9 * h + 5] = (double) lImu.gyro[2];
        out_data[9 * h + 6] = (double) lImu.posAngulaire[0];
        out_data[9 * h + 7] = (double) lImu.posAngulaire[1];
        out_data[9 * h + 8] = (double) lImu.posAngulaire[2];
        out_data_ts[h] = (double) lImu.timestamp;
    }

    mxFree(msg);
}

/* */
void logico_adc(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    msgHeader       lHeader;
    msgAdc          lAdc;
    unsigned int    h;
    double          *out_data, *out_data_ts;

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_ADC, &msg, sizeof(msgAdc));

    /* Formattage pour Matlab */
    plhs[0] = mxCreateDoubleMatrix(8, lHeader.size, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    out_data = (double *) mxGetPr(plhs[0]);
    out_data_ts = (double *) mxGetPr(plhs[1]);

    for(h = 0; h < lHeader.size; h++)
    {
        memcpy(&lAdc, msg + sizeof(msgHeader) + h * sizeof(msgAdc), sizeof(msgAdc));
        out_data[h * 8 + 0] = (double) lAdc.canal1;
        out_data[h * 8 + 1] = (double) lAdc.canal2;
        out_data[h * 8 + 2] = (double) lAdc.canal3;
        out_data[h * 8 + 3] = (double) lAdc.canal4;
        out_data[h * 8 + 4] = (double) lAdc.canal5;
        out_data[h * 8 + 5] = (double) lAdc.canal6;
        out_data[h * 8 + 6] = (double) lAdc.canal7;
        out_data[h * 8 + 7] = (double) lAdc.canal8;
        out_data_ts[h] = (double) lAdc.timestamp;
    }

    mxFree(msg);
}

/* */
void logico_serial(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    msgHeader       lHeader;
    msgSerialCmd    lSerie;
    msgSerialAns    lSerieAns;
    int             h;
    char            *out_data;
    char            StrBuffer[101];

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    msg = (char *) mxCalloc(1, sizeof(msgHeader) + sizeof(msgSerialCmd));
    lHeader.type = MSGID_SERIAL_CMD;
    lHeader.info = 0x00;
    lHeader.size = 1;
    lHeader.packetTimestamp = 0.0;

    memcpy(msg, &lHeader, sizeof(msgHeader));

    if(mxGetString(prhs[0], StrBuffer, sizeof(StrBuffer) - 1))
        mexErrMsgTxt("Could not understand port name. It must be a string with less than 100 characters.");

    if((short) mxGetScalar(prhs[5]) > sizeof(lSerie.data))
        mexErrMsgTxt("Send buffer size must be lower than 1024 bytes.");

    if((short) mxGetScalar(prhs[6]) > sizeof(lSerieAns.data))
        mexErrMsgTxt("Receive buffer size must be lower than 1024 bytes.");

    strcpy(lSerie.port, StrBuffer);
    lSerie.speed = (unsigned short) mxGetScalar(prhs[1]);
    lSerie.parity = (char) mxGetScalar(prhs[2]);
    lSerie.stopBits = (char) mxGetScalar(prhs[3]);
    lSerie.sendLength = (short) mxGetScalar(prhs[5]);
    for(h = 0; h < lSerie.sendLength * 2; h += 2)
    {
        lSerie.data[h / 2] = ((unsigned char *) mxGetChars(prhs[4]))[h];
    }

    lSerie.readLength = (short) mxGetScalar(prhs[6]);
    lSerie.readTimeoutSec = (short) mxGetScalar(prhs[7]);
    lSerie.readTimeoutMicro = (long) mxGetScalar(prhs[8]);
    lSerie.closeAfterComm = (char) mxGetScalar(prhs[9]);

    memcpy(msg + sizeof(msgHeader), &lSerie, sizeof(msgSerialCmd));

    send(*main_socket, msg, sizeof(msgHeader) + sizeof(msgSerialCmd), 0);

    recv(*main_socket, msg, sizeof(msgHeader), MSG_PEEK);
    memcpy(&lHeader, msg, sizeof(msgHeader));
    if(lHeader.type != MSGID_SERIAL_ANS) mexErrMsgTxt("Reponse incompatible du robot.");

    msg = (char *) mxCalloc(1, sizeof(msgHeader) + lHeader.size * sizeof(msgSerialAns));

    recv(*main_socket, msg, sizeof(msgHeader) + lHeader.size * sizeof(msgSerialAns), 0);

    /* Formattage pour Matlab */
    memcpy(&lSerieAns, msg + sizeof(msgHeader) + sizeof(msgSerialAns), sizeof(msgSerialAns));
    out_data = mxCalloc(lSerieAns.bufferLength, sizeof(char));
    plhs[0] = mxCreateString(out_data);
    mxFree(msg);
}

/* */
void logico_dout(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    msgHeader       lHeader;
    msgDigitalOut   lDout;
    int             outsend;

    lHeader.type = MSGID_DOUT_CTRL;
    lHeader.info = 0x00;
    lHeader.size = 1;
    lHeader.packetTimestamp = 0.0;

    msg = (char *) mxCalloc(1, sizeof(msgHeader) + sizeof(msgDigitalOut));

    memcpy(msg, &lHeader, sizeof(msgHeader));

    if(nrhs < 4)
    {
        mexErrMsgTxt("Insufficient parameter count (4 needed)");
    }

    lDout.pinD0 = (mxGetScalar(prhs[0]) == 0) ? 0 : 1;
    lDout.pinD1 = (mxGetScalar(prhs[1]) == 0) ? 0 : 1;
    lDout.pinD2 = (mxGetScalar(prhs[2]) == 0) ? 0 : 1;
    lDout.pinD3 = (mxGetScalar(prhs[3]) == 0) ? 0 : 1;

    memcpy(msg + sizeof(msgHeader), &lDout, sizeof(msgDigitalOut));
    outsend = send(*main_socket, msg, sizeof(msgHeader) + sizeof(msgDigitalOut), 0);
}

/* */
void logico_gps(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    msgHeader       lHeader;
    msgGps          lGps;
    unsigned int    h;
    double            *out_data, *out_data_ts;

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_GPS, &msg, sizeof(msgGps));

    /* Formattage pour Matlab */
    plhs[0] = mxCreateDoubleMatrix(7, lHeader.size, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    out_data = (double *) mxGetPr(plhs[0]);
    out_data_ts = (double *) mxGetPr(plhs[1]);

    for(h = 0; h < lHeader.size; h++)
    {
        memcpy(&lGps, msg + sizeof(msgHeader) + h * sizeof(msgGps), sizeof(msgGps));
        out_data[h * 7 + 0] = (double) lGps.state;
        out_data[h * 7 + 1] = (double) lGps.longitude;
        out_data[h * 7 + 2] = (double) lGps.latitude;
        out_data[h * 7 + 3] = (double) lGps.altitude;
        out_data[h * 7 + 4] = (double) lGps.speedModule;
        out_data[h * 7 + 5] = (double) lGps.speedAngle;
        out_data[h * 7 + 6] = (double) lGps.verticalSpeed;
        out_data_ts[h] = (double) lGps.timestamp;
    }

    mxFree(msg);
}

/* */
void logico_camera(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    char            *inPixelSource;
    msgHeader       lHeader;
    msgCam          lCam;
    unsigned int    i;
    unsigned int    a, b, y, x;
    uint32_t        msg_pos;
    unsigned char   *out_data;
    double          *out_data_ts;
    unsigned int    sizeImg = 0;
    unsigned int    cam_size[4] = { 0, 0, 3, 0 };

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_WEBCAM, &msg, sizeof(msgCam));

    memcpy(&lCam, msg + sizeof(msgHeader), sizeof(msgCam));

    cam_size[0] = lCam.height;
    cam_size[1] = lCam.width;
    cam_size[3] = lHeader.size;

    plhs[0] = mxCreateNumericArray(4, cam_size, mxUINT8_CLASS, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    out_data = (unsigned char *) mxGetData(plhs[0]);
    out_data_ts = (double *) mxGetPr(plhs[1]);

    msg_pos = sizeof(msgHeader) + sizeof(msgCam) * lHeader.size;

    for(i = 0; i < lHeader.size; i++)
    {
        /* On passe sur toutes les images */
        memcpy(&lCam, msg + sizeof(msgHeader) + i * sizeof(msgCam), sizeof(msgCam));

        sizeImg = lCam.width * lCam.height * 3;
        y = 0;

        /* Get image source */
        switch (lCam.compressionType) {
            case R4M_COMP_JPEG:
                /* Decompress the image first and set the pixel source pointer to it */
                inPixelSource = (char *) mxMalloc(sizeImg);
                decodeJPEG(msg + msg_pos, lCam.sizeData, (uint32_t*)inPixelSource);
                msg_pos += lCam.sizeData;
                break;
            default:
            case R4M_COMP_NONE:
                /* Set the pixel source pointer directly to the message header */
                inPixelSource = &msg[sizeof(msgHeader)               /* Skip the packet header */
                                     + sizeof(msgCam) * lHeader.size /* Skip the msgCam Header */
                                     + sizeImg * i];           /* Skip previous images */
        }

        /* Formattage pour Matlab */
        for(y = 0, b = 0; y < lCam.width * lCam.height * 3 - lCam.width * 3; b++, y += lCam.width * 3)
        {
            for(x = y, a = b; x < y + lCam.width * 3; a += lCam.height)
            {
                #define DESTINATION_STRIDE     a + i * sizeImg + lCam.width * lCam.height
                out_data[DESTINATION_STRIDE * 0] = inPixelSource[x++];
                out_data[DESTINATION_STRIDE * 1] = inPixelSource[x++];
                out_data[DESTINATION_STRIDE * 2] = inPixelSource[x++];
            }
        }

        out_data_ts[i] = (double) lCam.timestamp;
    }

    mxFree(msg);
}

/* */
void logico_camera_stereo(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    char            *inPixelSource_l, *inPixelSource_r;
    msgHeader       lHeader;
    msgCam          lCam_l, lCam_r;
    unsigned int    i;
    unsigned int    a, b, y, x;
    uint32_t        msg_pos;
    unsigned char   *out_data_l, *out_data_r;
    double          *out_data_ts;
    unsigned int    sizeImg = 0;
    unsigned int    cam_size[4] = { 0, 0, 3, 0 };

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_WEBCAM_STEREO, &msg, sizeof(msgCam));

    memcpy(&lCam_l, msg + sizeof(msgHeader), sizeof(msgCam));

    cam_size[0] = lCam_l.height;
    cam_size[1] = lCam_l.width;
    cam_size[3] = lHeader.size;

    plhs[0] = mxCreateNumericArray(4, cam_size, mxUINT8_CLASS, mxREAL);
    plhs[1] = mxCreateNumericArray(4, cam_size, mxUINT8_CLASS, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    out_data_l = (unsigned char *) mxGetData(plhs[0]);
    out_data_r = (unsigned char *) mxGetData(plhs[1]);
    out_data_ts = (double *) mxGetPr(plhs[2]);

    msg_pos = sizeof(msgHeader) + sizeof(msgCam) * lHeader.size;

    for(i = 0; i < lHeader.size; i += 2)
    {
        /* On passe sur toutes les images */
        memcpy(&lCam_l, msg + sizeof(msgHeader) + (i + 0) * sizeof(msgCam), sizeof(msgCam));
        memcpy(&lCam_r, msg + sizeof(msgHeader) + (i + 1) * sizeof(msgCam), sizeof(msgCam));

        out_data_ts[i / 2] = (double) lCam_l.timestamp;

        sizeImg = lCam_l.width * lCam_l.height * 3;
        y = 0;

        /* Get image source */
        switch (lCam_l.compressionType) {
            case R4M_COMP_JPEG:
                /* Decompress the image first and set the pixel source pointer to it */
                inPixelSource_l = (char *) mxMalloc(sizeImg);
                inPixelSource_r = (char *) mxMalloc(sizeImg);
                decodeJPEG(msg + msg_pos, lCam_l.sizeData, (uint32_t*)inPixelSource_l);
                msg_pos += lCam_l.sizeData;
                decodeJPEG(msg + msg_pos, lCam_r.sizeData, (uint32_t*)inPixelSource_r);
                msg_pos += lCam_r.sizeData;
                break;
            default:
            case R4M_COMP_NONE:
                /* Set the pixel source pointer directly to the message header */
                inPixelSource_l = &msg[sizeof(msgHeader)               /* Skip the packet header */
                                       + sizeof(msgCam) * lHeader.size /* Skip the msgCam Header */
                                       + sizeImg * i];           /* Skip previous images */
                inPixelSource_r = inPixelSource_l + sizeImg; /* One image further */
        }

        /* Formattage pour Matlab */
        for(y = 0, b = 0; y < lCam_l.width * lCam_l.height * 3 - lCam_l.width * 3; b++, y += lCam_l.width * 3)
        {
            for(x = y, a = b; x < y + lCam_l.width * 3; a += lCam_l.height)
            {
                /* Handling both images at the same time. First R left, then R right, then G left... */
                #define DESTINATION_STRIDE_STEREO     a + i * sizeImg + lCam_l.width * lCam_l.height
                out_data_l[DESTINATION_STRIDE_STEREO * 0] = inPixelSource_l[x];
                out_data_r[DESTINATION_STRIDE_STEREO * 0] = inPixelSource_r[x++];
                out_data_l[DESTINATION_STRIDE_STEREO * 1] = inPixelSource_l[x];
                out_data_r[DESTINATION_STRIDE_STEREO * 1] = inPixelSource_r[x++];
                out_data_l[DESTINATION_STRIDE_STEREO * 2] = inPixelSource_l[x];
                out_data_r[DESTINATION_STRIDE_STEREO * 2] = inPixelSource_r[x++];
            }
        }
    }

    mxFree(msg);
}

/* */
void logico_kinect(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    msgHeader       lHeader;
    msgCam          lCam;
    unsigned int    i;
    unsigned int    a, b, y, x;
    unsigned short  *out_data;
    double          *out_data_ts;
    unsigned int    sizeImg = 0;

    /* TODO: Why not use lCam.channels? */
    unsigned int    cam_size[4] = { 0, 0, 4, 0 };

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_KINECT, &msg, sizeof(msgCam));

    memcpy(&lCam, msg + sizeof(msgHeader), sizeof(msgCam));

    cam_size[0] = lCam.height;
    cam_size[1] = lCam.width;
    cam_size[3] = lHeader.size;

    plhs[0] = mxCreateNumericArray(4, cam_size, mxUINT16_CLASS, mxREAL);    /* 16 bits */
    plhs[1] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    out_data = (unsigned short *) mxGetData(plhs[0]);
    out_data_ts = (double *) mxGetPr(plhs[1]);

    for(i = 0; i < lHeader.size; i++)
    {
        /* On passe sur toutes les images */
        memcpy(&lCam, msg + sizeof(msgHeader) + i * sizeof(msgCam), sizeof(msgCam));

        sizeImg = lCam.width * lCam.height * 3 + lCam.width * lCam.height * sizeof(uint16_t);

        /* Formattage pour Matlab */
        for(y = 0, b = 0; y < lCam.width * lCam.height * 4 - lCam.width * 4; b++, y += lCam.width * 4)
        {
            for(x = y, a = b; x < y + lCam.width * 4; a += lCam.height)
            {
                /* */

                /* SIZE IMG EST FAUX CA VA PETER NE PAS UTILISER */

                /* DANS LA PARTIE DE GAUCHE DE L'EQUATION */

                /* */
                out_data[a + i * sizeImg + lCam.width * lCam.height * 0] = (unsigned short) msg[sizeof(msgHeader) + sizeof(msgCam) * lHeader.size + sizeImg * i + x++];
                out_data[a + i * sizeImg + lCam.width * lCam.height * 1] = (unsigned short) msg[sizeof(msgHeader) + sizeof(msgCam) * lHeader.size + sizeImg * i + x++];
                out_data[a + i * sizeImg + lCam.width * lCam.height * 2] = (unsigned short) msg[sizeof(msgHeader) + sizeof(msgCam) * lHeader.size + sizeImg * i + x++];
                out_data[a + i * sizeImg + lCam.width * lCam.height * 3] = *(unsigned short *) &msg[sizeof(msgHeader) + sizeof(msgCam) * lHeader.size + sizeImg * i + x];
                x += 2;
            }
        }

        out_data_ts[i] = (double) lCam.timestamp;
    }

    mxFree(msg);
}

/* */
void logico_kinect_ir(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    msgHeader       lHeader;
    msgCam          lCam;
    unsigned int    i;
    unsigned int    a, b, y, x;
    unsigned short  *out_data;
    double          *out_data_ts;
    unsigned int    sizeImg = 0;
    unsigned int    cam_size[4] = { 0, 0, 1, 0 };

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_KINECT_DEPTH, &msg, sizeof(msgCam));

    memcpy(&lCam, msg + sizeof(msgHeader), sizeof(msgCam));

    cam_size[0] = lCam.height;
    cam_size[1] = lCam.width;
    cam_size[2] = lHeader.size;

    plhs[0] = mxCreateNumericArray(3, cam_size, mxUINT16_CLASS, mxREAL);    /* 16 bits */
    plhs[1] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    out_data = (unsigned short *) mxGetData(plhs[0]);
    out_data_ts = (double *) mxGetPr(plhs[1]);

    for(i = 0; i < lHeader.size; i++)
    {
        /* On passe sur toutes les images */
        memcpy(&lCam, msg + sizeof(msgHeader) + i * sizeof(msgCam), sizeof(msgCam));

        sizeImg = lCam.width * lCam.height * 1;

        /* Formattage pour Matlab */
        for(y = 0, b = 0; y < lCam.width * lCam.height - lCam.width; b++, y += lCam.width)
        {
            for(x = y, a = b; x < y + lCam.width; a += lCam.height)
            {
                out_data[a + i * sizeImg + 0] = *(unsigned short *) &msg[sizeof(msgHeader) + sizeof(msgCam) * lHeader.size + 2 * sizeImg * i + x++ *2];
            }
        }

        out_data_ts[i] = (double) lCam.timestamp;
    }

    mxFree(msg);
}

/* */
void logico_hokuyo(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    msgHeader       lHeader;
    msgHokuyo       lHokuyo;
    unsigned int    i;
    unsigned int    hokuyo_size[2] = { 0, 0 };
    unsigned char   *out_data;
    double          *out_data_ts, *out_data_infos;

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_HOKUYO, &msg, sizeof(msgHokuyo));
    memcpy(&lHokuyo, msg + sizeof(msgHeader), sizeof(msgHokuyo));

    hokuyo_size[0] = lHokuyo.sizeData / sizeof(float);
    hokuyo_size[1] = lHeader.size;

    /*mexPrintf("Hokuyo size by calc %f and by ref %i", (lHokuyo.angleMax - lHokuyo.angleMin) / lHokuyo.angleIncrement, lHokuyo.sizeData);*/
    plhs[0] = mxCreateNumericArray(2, hokuyo_size, mxSINGLE_CLASS, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1, 6, mxREAL);

    out_data = (unsigned char *) mxGetData(plhs[0]);
    out_data_ts = (double *) mxGetPr(plhs[1]);
    out_data_infos = (double *) mxGetPr(plhs[2]);

    for(i = 0; i < lHeader.size; i++)
    {
        /* On passe sur tous les echantillons */
        memcpy(&lHokuyo, msg + sizeof(msgHeader) + i * sizeof(msgHokuyo), sizeof(msgHokuyo));
        memcpy
        (
            out_data + i * hokuyo_size[0] * sizeof(float),
            msg + sizeof(msgHeader) + lHeader.size * sizeof(msgHokuyo) + i * hokuyo_size[0] * sizeof(float),
            hokuyo_size[0] * sizeof(float)
        );

        out_data_ts[i] = (double) lHokuyo.timestamp;
    }

    out_data_infos[0] = lHokuyo.angleMin;
    out_data_infos[1] = lHokuyo.angleMax;
    out_data_infos[2] = lHokuyo.angleIncrement;
    out_data_infos[3] = lHokuyo.rangeMin;
    out_data_infos[4] = lHokuyo.rangeMax;
    out_data_infos[5] = lHokuyo.timeIncrement;

    mxFree(msg);
}

/* */
void logico_computer(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char            *msg = NULL;
    msgHeader       lHeader;
    msgComputer     lComputer;
    unsigned int    h;
    double          *out_data, *out_data_ts;

    if(initialized == 0) mexErrMsgTxt("No connection established.");

    lHeader = logico_send_data_request(MSGID_COMPUTER, &msg, sizeof(msgComputer));

    /* Formattage pour Matlab */
    plhs[0] = mxCreateDoubleMatrix(15, lHeader.size, mxREAL);
    plhs[1] = mxCreateDoubleMatrix(1, lHeader.size, mxREAL);
    out_data = (double *) mxGetPr(plhs[0]);
    out_data_ts = (double *) mxGetPr(plhs[1]);

    for(h = 0; h < lHeader.size; h++)
    {
        memcpy(&lComputer, msg + sizeof(msgHeader) + h * sizeof(msgComputer), sizeof(msgComputer));
        out_data[h * 15 + 0] = (double) lComputer.state;
        out_data[h * 15 + 1] = (double) lComputer.cpuTemperature;
        out_data[h * 15 + 2] = (double) lComputer.unixLoad[0];
        out_data[h * 15 + 3] = (double) lComputer.unixLoad[1];
        out_data[h * 15 + 4] = (double) lComputer.unixLoad[2];
        out_data[h * 15 + 5] = (double) lComputer.cpuLoad[0];
        out_data[h * 15 + 6] = (double) lComputer.cpuLoad[1];
        out_data[h * 15 + 7] = (double) lComputer.cpuLoad[2];
        out_data[h * 15 + 8] = (double) lComputer.cpuLoad[3];
        out_data[h * 15 + 9] = (double) lComputer.cpuLoad[4];
        out_data[h * 15 + 10] = (double) lComputer.cpuLoad[5];
        out_data[h * 15 + 11] = (double) lComputer.cpuLoad[6];
        out_data[h * 15 + 12] = (double) lComputer.memUsed[0];
        out_data[h * 15 + 13] = (double) lComputer.memUsed[1];
        out_data[h * 15 + 14] = (double) lComputer.memUsed[2];
        out_data_ts[h] = (double) lComputer.timestamp;
    }
}


/*******************************************************************************
 *
 *                               Mex interface
 *
 ******************************************************************************/

struct _cmd_data
{
    char    *pCmd;
    void (*pFunction) (int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]);
}
CmdTable[] =
{
    { "connect", logico_start },
    { "close", logico_close },
    { "subscribe", logico_subscribe },
    { "unsubscribe", logico_unsubscribe },
    { "batterie", logico_battery },
    { "adc", logico_adc },
    { "imu", logico_imu },
    { "gps", logico_gps },
    { "serie", logico_serial },
    { "ordinateur", logico_computer },
    { "camera", logico_camera },
    { "camera_stereo", logico_camera_stereo },
    { "hokuyo", logico_hokuyo },
    { "kinect", logico_kinect },
    { "kinect_ir", logico_kinect_ir },
    { "sorties_numeriques", logico_dout }
};

/* */

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    char        StrBuffer[65];
    static char ErrBuffer[512];
    int         h;

    if(nrhs < 1)
    {
        strcpy(ErrBuffer, "Missing command string. Expecting one of:");
        for(h = 0; h < sizeof(CmdTable) / sizeof(CmdTable[0]); h++)
        {
            sprintf(StrBuffer, "\n %s", CmdTable[h].pCmd);
            strcat(ErrBuffer, StrBuffer);
        }

        mexErrMsgTxt(ErrBuffer);
    }

    if(!mxIsChar(prhs[0])) mexErrMsgTxt("The first argument must be a string.");

    if(mxGetString(prhs[0], StrBuffer, sizeof(StrBuffer) - 1))
        mexWarnMsgTxt("Cannot understand request: String conversion failed.");

    for(h = 0; h < sizeof(CmdTable) / sizeof(CmdTable[0]); h++)
    {
        if(!strcmp(CmdTable[h].pCmd, StrBuffer))
        {
            CmdTable[h].pFunction(nlhs, plhs, nrhs - 1, prhs + 1);
            return;
        }
    }

    mexErrMsgTxt("Invalid command");
}
