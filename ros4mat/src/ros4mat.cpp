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
#include <ros/console.h>
#include <ros/ros.h>

#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>         /* Network */
#include <netinet/in.h>         /* Network */
#include <arpa/inet.h>          /* Network */
#include <netinet/tcp.h>        /* Network */
#include <netdb.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <sys/time.h>
#include <sys/socket.h>

#include <vector>
#include <map>
#include <queue>
#include <sstream>

#include <ros4mat/M_ADC.h>
#include <ros4mat/S_ADC.h>
#include <ros4mat/M_IMU.h>
#include <ros4mat/S_IMU.h>
#include <ros4mat/M_Battery.h>
#include <ros4mat/S_Battery.h>
#include <ros4mat/M_Computer.h>
#include <ros4mat/S_Computer.h>
#include <ros4mat/S_Cam.h>
#include <ros4mat/M_StereoCam.h>
#include <ros4mat/S_StereoCam.h>
#include <ros4mat/S_DigitalOut.h>
#include <sensor_msgs/Image.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>

#include <sensor_msgs/LaserScan.h>

#include <ros4mat/S_Serial.h>

#include "../../exchangeStructs.h"
#include "../../thirdparty/easyzlib.h"

#define SOCKET_ERROR    -1

/* Extract from http://www.ros.org/wiki/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 * subscribe() returns a Subscriber object that you
 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
 * object go out of scope, this callback will automatically be unsubscribed from
 * this topic.
 */
#define LOCAL_SERVER_PORT       1500
#define SOCKET_SEND_TIMEOUT_SEC 5

typedef struct msgCamInternal   msgCamInternal;
struct msgCamInternal
{
    uint32_t        width;
    uint32_t        height;
    unsigned char   channels;
    uint32_t        sizeData;
    char            *cptr;      /* Keeps the image buffer address */
    double          timestamp;  /* Image acquisition timestamp */
};

typedef struct msgStereoCamInternal msgStereoCamInternal;
struct msgStereoCamInternal
{
    uint32_t        width;
    uint32_t        height;
    unsigned char   channels;
    uint32_t        sizeData;
    char            *cptr_L;    /* Keeps the image buffer address (left img) */
    char            *cptr_R;    /* Keeps the image buffer address (right img) */
    double          timestamp;  /* Image acquisition timestamp */
};

typedef struct msgHokuyoInternal    msgHokuyoInternal;
struct msgHokuyoInternal
{
    float       angleMin;
    float       angleMax;
    float       angleIncrement;
    float       timeIncrement;
    float       rangeMin;
    float       rangeMax;
    double      timestamp;
    uint32_t    sizeData;
    char        *cptr;          /* Keeps the ranges buffer address */
};

typedef struct matlabClient matlabClient;
struct matlabClient
{
    int                                                                         id;
    char                                                                        compression;
    std::map<unsigned int, std::pair<ros::Subscriber, std::queue<void *> > >    subscribers;    // subscribers, buffered data
    std::map<unsigned int, uint32_t>                                            buffersInfo;    // Allocated size
};

std::map<int, matlabClient> clients;    // int = Client Socket File Descriptor


void dataAdcReceived(const ros4mat::M_ADC::ConstPtr &msg)
{
    std::map<int, matlabClient>::iterator   lClientIt;
    ROS_DEBUG("Reception de donnees de l'ADC (%d samples)", (int) msg->timestamps.size());

    msgAdc  *lMsg = NULL;
    for(int i = 0; i < msg->timestamps.size(); i++)
    {
        for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
        {
            if((*lClientIt).second.subscribers.count(MSGID_ADC) == 0) continue;
            lMsg = new msgAdc;
            lMsg->timestamp = msg->timestamps[i];
            lMsg->canal1 = (msg->canal1.size() > 0) ? msg->canal1[i] : 0;
            lMsg->canal2 = (msg->canal2.size() > 0) ? msg->canal2[i] : 0;
            lMsg->canal3 = (msg->canal3.size() > 0) ? msg->canal3[i] : 0;
            lMsg->canal4 = (msg->canal4.size() > 0) ? msg->canal4[i] : 0;
            lMsg->canal5 = (msg->canal5.size() > 0) ? msg->canal5[i] : 0;
            lMsg->canal6 = (msg->canal6.size() > 0) ? msg->canal6[i] : 0;
            lMsg->canal7 = (msg->canal7.size() > 0) ? msg->canal7[i] : 0;
            lMsg->canal8 = (msg->canal8.size() > 0) ? msg->canal8[i] : 0;

            // Add to the buffer
            (*lClientIt).second.subscribers[MSGID_ADC].second.push((void *) lMsg);
            lMsg = NULL;

            // Pruning obsolete data
            while((*lClientIt).second.subscribers[MSGID_ADC].second.size() > (*lClientIt).second.buffersInfo[MSGID_ADC])
            {
                ROS_DEBUG("PRUNING ADC DATA");
                delete(msgAdc *) ((*lClientIt).second.subscribers[MSGID_ADC].second.front());
                (*lClientIt).second.subscribers[MSGID_ADC].second.pop();
            }
        }
    }
}


void dataImuReceived(const ros4mat::M_IMU::ConstPtr &msg)
{
    std::map<int, matlabClient>::iterator   lClientIt;
    ROS_DEBUG("Reception de donnees de l'IMU (%d samples)", (int) msg->timestamps.size());

    msgImu  *lMsg = NULL;
    for(unsigned int i = 0; i < msg->timestamps.size(); i++)
    {
        for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
        {
            if((*lClientIt).second.subscribers.count(MSGID_IMU) == 0) continue;

            ROS_DEBUG("Nouveau Client");
            ROS_DEBUG("Buffer Size: %u", (unsigned int) (*lClientIt).second.subscribers[MSGID_IMU].second.size());
            ROS_DEBUG("Max Buffer Size: %u", (*lClientIt).second.buffersInfo[MSGID_IMU]);
            lMsg = new msgImu;
            lMsg->timestamp = msg->timestamps[i];
            lMsg->acceleration[0] = msg->aX[i];
            lMsg->acceleration[1] = msg->aY[i];
            lMsg->acceleration[2] = msg->aZ[i];
            lMsg->gyro[0] = msg->rX[i];
            lMsg->gyro[1] = msg->rY[i];
            lMsg->gyro[2] = msg->rZ[i];
            lMsg->posAngulaire[0] = msg->mX[i];
            lMsg->posAngulaire[1] = msg->mY[i];
            lMsg->posAngulaire[2] = msg->mZ[i];

            // Add to the buffer
            (*lClientIt).second.subscribers[MSGID_IMU].second.push((void *) lMsg);
            lMsg = NULL;

            // Pruning obsolete data
            while((*lClientIt).second.subscribers[MSGID_IMU].second.size() > (*lClientIt).second.buffersInfo[MSGID_IMU])
            {
                ROS_DEBUG("PRUNING IMU DATA");
                delete(msgImu *) ((*lClientIt).second.subscribers[MSGID_IMU].second.front());
                (*lClientIt).second.subscribers[MSGID_IMU].second.pop();
            }
        }
    }

    ROS_DEBUG("Fin de la transmission IMU\n\n");
}


void dataCamReceived(const sensor_msgs::Image::ConstPtr &msg)
{
    char                                    *bufferImg;
    std::map<int, matlabClient>::iterator   lClientIt;
    msgCamInternal                          *lMsg = NULL;
    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_WEBCAM) == 0) continue;
        lMsg = new msgCamInternal;
        lMsg->timestamp = msg->header.stamp.toSec();
        lMsg->width = msg->width;
        lMsg->height = msg->height;
        lMsg->channels = 3;
        lMsg->sizeData = (msg->data).size();
        if(lMsg->sizeData != lMsg->width * lMsg->height * 3)
        {
            ROS_WARN(
                "Incoherence de taille des buffers de camera (taille prevue : %d, obtenue : %d",
                lMsg->width * lMsg->height * 3,
                lMsg->sizeData
            );
        }

        bufferImg = new char[lMsg->sizeData];
        for(unsigned int i = 0; i < lMsg->sizeData; i++) bufferImg[i] = msg->data[i];
        lMsg->cptr = bufferImg;
        (*lClientIt).second.subscribers[MSGID_WEBCAM].second.push((void *) lMsg);
        lMsg = NULL;
        while (
            (*lClientIt).second.subscribers[MSGID_WEBCAM].second.size() >
                (*lClientIt).second.buffersInfo[MSGID_WEBCAM]
        )
        {
            ROS_INFO_ONCE("Beginning PRUNING webcam data");
            delete[]((msgCamInternal *) (*lClientIt).second.subscribers[MSGID_WEBCAM].second.front())->cptr;
            delete(msgCamInternal *) ((*lClientIt).second.subscribers[MSGID_WEBCAM].second.front());
            (*lClientIt).second.subscribers[MSGID_WEBCAM].second.pop();
        }
    }
}


void dataStereoCamReceived(const ros4mat::M_StereoCam::ConstPtr &image)
{
    char                                    *bufferImg_L, *bufferImg_R;
    std::map<int, matlabClient>::iterator   lClientIt;
    msgStereoCamInternal                    *lMsg = NULL;

    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_WEBCAM_STEREO) == 0) continue;
        lMsg = new msgStereoCamInternal;

        lMsg->timestamp = image->timestamp;
        lMsg->width = image->width;
        lMsg->height = image->height;
        lMsg->channels = 3;
        lMsg->sizeData = (image->image_left).size();
        if(lMsg->sizeData != lMsg->width * lMsg->height * 3)
        {
            ROS_WARN(
                "Incoherence de taille des buffers de camera (taille prevue : %d, obtenue : %d",
                lMsg->width * lMsg->height * 3,
                lMsg->sizeData
            );
        }

        if((image->image_left).size() != (image->image_right).size())
        {
            ROS_WARN(
                "Incoherence de taille entre camera gauche et droite (gauche : %d, droite : %d )",
                (image->image_left).size(),
                (image->image_right).size()
            );
        }

        bufferImg_L = new char[lMsg->sizeData];
        bufferImg_R = new char[lMsg->sizeData];
        for(unsigned int i = 0; i < lMsg->sizeData; i++)
        {
            bufferImg_L[i] = image->image_left[i];
            bufferImg_R[i] = image->image_right[i];
        }

        lMsg->cptr_L = bufferImg_L;
        lMsg->cptr_R = bufferImg_R;
        (*lClientIt).second.subscribers[MSGID_WEBCAM_STEREO].second.push((void *) lMsg);
        lMsg = NULL;

        while (
            (*lClientIt).second.subscribers[MSGID_WEBCAM_STEREO].second.size() >
                (*lClientIt).second.buffersInfo[MSGID_WEBCAM_STEREO]
        )
        {
            ROS_INFO_ONCE("Beginning PRUNING stereo webcam data");
            delete[]((msgStereoCamInternal *) (*lClientIt).second.subscribers[MSGID_WEBCAM_STEREO].second.front())->cptr_L;
            delete[]((msgStereoCamInternal *) (*lClientIt).second.subscribers[MSGID_WEBCAM_STEREO].second.front())->cptr_R;
            delete(msgStereoCamInternal *) ((*lClientIt).second.subscribers[MSGID_WEBCAM_STEREO].second.front());
            (*lClientIt).second.subscribers[MSGID_WEBCAM_STEREO].second.pop();
        }
    }
}


void dataGpsReceived(const gps_common::GPSFix::ConstPtr &msg)
{
    std::map<int, matlabClient>::iterator   lClientIt;
    ROS_DEBUG("Reception de donnees du GPS (1 sample)");

    msgGps  *lMsg = NULL;

    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_GPS) == 0) continue;

        ROS_DEBUG("Nouveau Client");
        ROS_DEBUG("Buffer Size: %u", (unsigned int) (*lClientIt).second.subscribers[MSGID_GPS].second.size());
        ROS_DEBUG("Max Buffer Size: %u", (*lClientIt).second.buffersInfo[MSGID_GPS]);
        lMsg = new msgGps;
        lMsg->timestamp = msg->header.stamp.toSec();

        if(msg->status.status == 0)
            lMsg->state = MSGID_GPS_STATE_FIX3D;
        else
            lMsg->state = MSGID_GPS_STATE_NOFIX;

        lMsg->state = msg->status.status;
        lMsg->latitude = msg->latitude;
        lMsg->longitude = msg->longitude;
        lMsg->altitude = msg->altitude;
        lMsg->speedModule = msg->speed;
        lMsg->speedAngle = msg->track;
        lMsg->verticalSpeed = msg->climb;

        // Add to the buffer
        (*lClientIt).second.subscribers[MSGID_GPS].second.push((void *) lMsg);
        lMsg = NULL;

        // Pruning obsolete data
        while((*lClientIt).second.subscribers[MSGID_GPS].second.size() > (*lClientIt).second.buffersInfo[MSGID_GPS])
        {
            ROS_DEBUG("PRUNING GPS DATA");
            delete(msgGps *) ((*lClientIt).second.subscribers[MSGID_GPS].second.front());
            (*lClientIt).second.subscribers[MSGID_GPS].second.pop();
        }
    }

    ROS_DEBUG("Fin de la transmission GPS\n\n");
}


void dataBatteryReceived(const ros4mat::M_Battery::ConstPtr &msg)
{
    std::map<int, matlabClient>::iterator   lClientIt;
    ROS_DEBUG("Reception de donnees de la batterie (1 sample)");

    msgBattery  *lMsg = NULL;

    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_BATTERY) == 0) continue;

        ROS_DEBUG("Nouveau Client");
        ROS_DEBUG("Buffer Size: %u", (unsigned int) (*lClientIt).second.subscribers[MSGID_BATTERY].second.size());
        ROS_DEBUG("Max Buffer Size: %u", (*lClientIt).second.buffersInfo[MSGID_BATTERY]);
        lMsg = new msgBattery;
        lMsg->timestamp = msg->timestamp;
        lMsg->currentDrained = msg->current;
        for(unsigned int i = 0; i < 6; i++) lMsg->cellsVoltage[i] = msg->vcell[i];

        lMsg->state = 0;
        for(unsigned int i = 0; i < 8; i++) lMsg->state |= (msg->batteryState[7 - i]) ? 1 << i : 0;

        // Add to the buffer
        (*lClientIt).second.subscribers[MSGID_BATTERY].second.push((void *) lMsg);
        lMsg = NULL;

        // Pruning obsolete data
        while (
            (*lClientIt).second.subscribers[MSGID_BATTERY].second.size() >
                (*lClientIt).second.buffersInfo[MSGID_BATTERY]
        )
        {
            ROS_DEBUG("PRUNING BATTERY DATA");
            delete(msgBattery *) ((*lClientIt).second.subscribers[MSGID_BATTERY].second.front());
            (*lClientIt).second.subscribers[MSGID_BATTERY].second.pop();
        }
    }

    ROS_DEBUG("Fin de la transmission Battery\n\n");
}


void dataComputerReceived(const ros4mat::M_Computer::ConstPtr &msg)
{
    std::map<int, matlabClient>::iterator   lClientIt;
    ROS_DEBUG("Reception de statistiques sur l'ordinateur (1 sample)");

    msgComputer *lMsg = NULL;

    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_COMPUTER) == 0) continue;

        ROS_DEBUG("Nouveau Client");
        ROS_DEBUG("Buffer Size: %u", (unsigned int) (*lClientIt).second.subscribers[MSGID_COMPUTER].second.size());
        ROS_DEBUG("Max Buffer Size: %u", (*lClientIt).second.buffersInfo[MSGID_COMPUTER]);
        lMsg = new msgComputer;
        lMsg->timestamp = msg->timestamp;
        lMsg->cpuTemperature = msg->cpuTemp;
        lMsg->state = 0x0;
        for(unsigned int i = 0; i < msg->cpuLoad.size(); i++) lMsg->cpuLoad[i] = msg->cpuLoad[i];
        for(unsigned int i = 0; i < msg->memUsed.size(); i++) lMsg->memUsed[i] = msg->memUsed[i];
        for(unsigned int i = 0; i < msg->unixLoad.size(); i++) lMsg->unixLoad[i] = msg->unixLoad[i];

        // Add to the buffer
        (*lClientIt).second.subscribers[MSGID_COMPUTER].second.push((void *) lMsg);
        lMsg = NULL;

        // Pruning obsolete data
        while (
            (*lClientIt).second.subscribers[MSGID_COMPUTER].second.size() >
                (*lClientIt).second.buffersInfo[MSGID_COMPUTER]
        )
        {
            ROS_DEBUG("PRUNING COMPUTER DATA");
            delete(msgComputer *) ((*lClientIt).second.subscribers[MSGID_COMPUTER].second.front());
            (*lClientIt).second.subscribers[MSGID_COMPUTER].second.pop();
        }
    }
}


void dataHokuyoReceived(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    char                                    *bufferRanges;
    std::map<int, matlabClient>::iterator   lClientIt;
    msgHokuyoInternal                       *lMsg = NULL;

    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_HOKUYO) == 0) continue;
        lMsg = new msgHokuyoInternal;
        lMsg->timestamp = msg->header.stamp.toSec();

        lMsg->angleMin = msg->angle_min;
        lMsg->angleMax = msg->angle_max;
        lMsg->angleIncrement = msg->angle_increment;

        lMsg->timeIncrement = msg->time_increment;
        lMsg->rangeMin = msg->range_min;
        lMsg->rangeMax = msg->range_max;
        lMsg->sizeData = sizeof(float) * (msg->ranges).size();

        bufferRanges = new char[lMsg->sizeData];
        for(unsigned int i = 0; i < lMsg->sizeData / sizeof(float); i++)
            memcpy(bufferRanges + sizeof(float) * i, &(msg->ranges[i]), sizeof(float)); // float32 = 4 bytes

        // Note pour Yan : on ne peut pas faire simplement un memcpy entre msg->ranges et bufferRanges
        // parce que msg->ranges est un tableau dynamique
        lMsg->cptr = bufferRanges;

        // Add to the buffer
        (*lClientIt).second.subscribers[MSGID_HOKUYO].second.push((void *) lMsg);
        lMsg = NULL;

        // Pruning obsolete data
        while (
            (*lClientIt).second.subscribers[MSGID_HOKUYO].second.size() >
                (*lClientIt).second.buffersInfo[MSGID_HOKUYO]
        )
        {
            ROS_INFO_ONCE("Beginning PRUNING hokuyo data");
            delete[]((msgHokuyoInternal *) (*lClientIt).second.subscribers[MSGID_HOKUYO].second.front())->cptr;
            delete(msgHokuyoInternal *) ((*lClientIt).second.subscribers[MSGID_HOKUYO].second.front());
            (*lClientIt).second.subscribers[MSGID_HOKUYO].second.pop();
        }
    }
}


void dataKinectDepthReceived(const sensor_msgs::Image::ConstPtr &msg)
{
    char                                    *bufferImg;
    std::map<int, matlabClient>::iterator   lClientIt;
    msgCamInternal                          *lMsg = NULL;

    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_KINECT_DEPTH) == 0) continue;
        lMsg = new msgCamInternal;
        lMsg->timestamp = msg->header.stamp.toSec();
        lMsg->width = msg->width;
        lMsg->height = msg->height;
        lMsg->channels = 1;
        lMsg->sizeData = (msg->data).size();

        if(lMsg->sizeData != lMsg->width * lMsg->height * 1 * 2)
        {   // * 2 parce que ce sont des uint16
            ROS_WARN(
                "Incoherence de taille des buffers de camera (taille prevue : %d, obtenue : %d",
                lMsg->width * lMsg->height * 1 * 2,
                lMsg->sizeData
            );
        }

        bufferImg = new char[lMsg->sizeData];
        for(unsigned int i = 0; i < lMsg->sizeData; i++)
        {
            bufferImg[i] = msg->data[i];
        }

        lMsg->cptr = bufferImg;

        // Add to the buffer
        (*lClientIt).second.subscribers[MSGID_KINECT_DEPTH].second.push((void *) lMsg);
        lMsg = NULL;

        // Pruning obsolete data
        while (
            (*lClientIt).second.subscribers[MSGID_KINECT_DEPTH].second.size() >
                (*lClientIt).second.buffersInfo[MSGID_KINECT_DEPTH]
        )
        {
            delete[]((msgCamInternal *) (*lClientIt).second.subscribers[MSGID_KINECT_DEPTH].second.front())->cptr;
            delete(msgCamInternal *) ((*lClientIt).second.subscribers[MSGID_KINECT_DEPTH].second.front());
            (*lClientIt).second.subscribers[MSGID_KINECT_DEPTH].second.pop();
        }
    }
}


void dataKinectRGBReceived(const sensor_msgs::Image::ConstPtr &msg)
{
    char                                    *bufferImg;
    std::map<int, matlabClient>::iterator   lClientIt;
    msgCamInternal                          *lMsg = NULL;

    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_KINECT) == 0) continue;
        lMsg = new msgCamInternal;
        lMsg->timestamp = msg->header.stamp.toSec();
        lMsg->width = msg->width;
        lMsg->height = msg->height;
        lMsg->channels = 4;
        lMsg->sizeData = (msg->data).size() + 640 * 480 * sizeof(uint16_t); /* Add Depth to packet when sending */

        if((msg->data).size() != lMsg->width * lMsg->height * 3)
        {
            ROS_WARN(
                "Incoherence de taille des buffers de camera (taille prevue : %d, obtenue : %d",
                lMsg->width * lMsg->height * 3,
                (msg->data).size()
            );
        }

        // Only populate the RGB part of the MSGID_KINECT buffer. The depth part will be taken directly from MSGID_KINECT_DEPTH buffer
        bufferImg = new char[(msg->data).size()];
        for(unsigned int i = 0; i < (msg->data).size(); i++)
        {
            bufferImg[i] = msg->data[i];
        }

        lMsg->cptr = bufferImg;

        // Add to the buffer
        (*lClientIt).second.subscribers[MSGID_KINECT].second.push((void *) lMsg);
        lMsg = NULL;

        // Pruning obsolete data
        while (
            (*lClientIt).second.subscribers[MSGID_KINECT].second.size() >
                (*lClientIt).second.buffersInfo[MSGID_KINECT]
        )
        {
            ROS_INFO_ONCE("Beginning PRUNING kinect RGB data");
            delete[]((msgCamInternal *) (*lClientIt).second.subscribers[MSGID_KINECT].second.front())->cptr;
            delete(msgCamInternal *) ((*lClientIt).second.subscribers[MSGID_KINECT].second.front());
            (*lClientIt).second.subscribers[MSGID_KINECT].second.pop();
        }
    }
}


int subscribeTo(msgSubscribe *info, ros::NodeHandle nodeRos, matlabClient &in_client)
{
    /* TODO: ADD Buffer toward publishers*/
    ros::Subscriber         sub;
    bool                    subOnly = false;

    std::stringstream       tmpDevice;

    ros4mat::S_ADC          lParamsSetAdc;
    ros4mat::S_IMU          lParamsSetImu;
    ros4mat::S_Battery      lParamsSetBat;
    ros4mat::S_Cam          lParamsSetCam;
    ros4mat::S_StereoCam    lParamsSetStereoCam;
    ros4mat::S_Computer     lParamsSetComputer;

    ROS_INFO(
        "Reception d'une demande d'inscription pour le capteur %02X.\nFrequence d'acquisition : %i\nFrequence de polling : %i\nTaille du buffer circulaire : %i",
        info->typeCapteur,
        info->freqAcquisition,
        info->freqSend,
        info->bufferSize
    );

    if(info->freqAcquisition == 0)
    {
        /* Special case:
                Parameters don't change (no service call), we only subscribe */
        ROS_INFO("Inscription seulement");
        if(info->bufferSize == 0)
        {
            int                                     bufferNewSize = 1;
            std::map<int, matlabClient>::iterator   lClientIt;
            bool                                    otherClientConnected = false;

            for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
            {
                if((*lClientIt).second.subscribers.count(info->typeCapteur) > 0)
                    bufferNewSize = (*lClientIt).second.buffersInfo[info->typeCapteur];
                otherClientConnected = true;
            }

            if(!otherClientConnected)
            {
                ROS_WARN(
                    "Aucun client actuellement connecte au capteur %02X, inscription seulement ignoree",
                    info->typeCapteur
                );
                return -1;
            }

            info->bufferSize = bufferNewSize;
            ROS_INFO(
                "Pas de taille de buffer envoyee, utilisation de la taille du precedent subscriber (%d echantillons)",
                bufferNewSize
            );
        }

        subOnly = true;
    }

    switch(info->typeCapteur)
    {
    case MSGID_ADC:
        if(!subOnly)
        {
            for(unsigned int i = 0; i < 8; i++)
            {
                lParamsSetAdc.request.adcChannels[i] = (info->paramSupp & (1 << i));
                ROS_INFO("Channel: %i, Val: %u", i, lParamsSetAdc.request.adcChannels[i]);
            }

            lParamsSetAdc.request.adcFreqAcq = info->freqAcquisition;
            lParamsSetAdc.request.adcFreqPoll = info->freqSend;
            lParamsSetAdc.request.adcBufferSize = info->bufferSize;
            lParamsSetAdc.request.subscribe = true;
            if(!ros::service::call("/D_ADC/params", lParamsSetAdc))
            {
                ROS_ERROR(
                    "Le service de parametrage de l'ADC a renvoye une erreur (code %d).",
                    lParamsSetAdc.response.ret
                );
                return -1;
            }
        }
        else
        {
            for(unsigned int i = 0; i < 8; i++)
            {
                lParamsSetAdc.request.adcChannels[i] = 0;
            }

            lParamsSetAdc.request.adcFreqAcq = 0;
            lParamsSetAdc.request.adcFreqPoll = 0;
            lParamsSetAdc.request.adcBufferSize = 0;
            lParamsSetAdc.request.subscribe = true;
            if(!ros::service::call("/D_ADC/params", lParamsSetAdc))
            {
                ROS_ERROR(
                    "Le service de parametrage de l'ADC a renvoye une erreur (code %d).",
                    lParamsSetAdc.response.ret
                );
                return -1;
            }
        }

        sub = nodeRos.subscribe("/D_ADC/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 2000, dataAdcReceived);
        break;

    case MSGID_IMU:
        if(subOnly)
        {
            lParamsSetImu.request.imuFreqAcq = 0;
            lParamsSetImu.request.imuFreqPoll = 0;
            lParamsSetImu.request.imuBufferSize = 0;                    // ?!?
            lParamsSetImu.request.subscribe = true;
            if(!ros::service::call("/D_IMU/params", lParamsSetImu))
            {
                ROS_ERROR(
                    "Le service de parametrage de l'IMU en mode inscription seulement a renvoye une erreur (code %d).",
                    lParamsSetImu.response.ret
                );
                return -1;
            }
        }
        else
        {
            lParamsSetImu.request.imuFreqAcq = info->freqAcquisition;
            lParamsSetImu.request.imuFreqPoll = info->freqSend;
            lParamsSetImu.request.imuBufferSize = info->bufferSize;     // ?!?
            lParamsSetImu.request.subscribe = true;
            if(!ros::service::call("/D_IMU/params", lParamsSetImu))
            {
                ROS_ERROR(
                    "Le service de parametrage de l'IMU a renvoye une erreur (code %d).",
                    lParamsSetImu.response.ret
                );
                return -1;
            }
        }

        sub = nodeRos.subscribe("/D_IMU/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 250, dataImuReceived);
        break;

    case MSGID_BATTERY:
        if(subOnly)
        {
            lParamsSetBat.request.subscribe = true;
            lParamsSetBat.request.battFreqAcq = 0;
            lParamsSetBat.request.battBufferSize = 0;                   // ?!?
            lParamsSetBat.request.sendCmdChargeDecharge = false;
            if(!ros::service::call("/D_Battery/params", lParamsSetBat))
            {
                ROS_ERROR(
                    "Le service de parametrage de la batterie a renvoye une erreur (code %d).",
                    lParamsSetBat.response.ret
                );
                return -1;
            }
        }
        else
        {
            lParamsSetBat.request.subscribe = true;
            lParamsSetBat.request.battFreqAcq = info->freqAcquisition;
            lParamsSetBat.request.battBufferSize = info->bufferSize;    // ?!?
            lParamsSetBat.request.sendCmdChargeDecharge = false;
            if(!ros::service::call("/D_Battery/params", lParamsSetBat))
            {
                ROS_ERROR(
                    "Le service de parametrage de la batterie a renvoye une erreur (code %d).",
                    lParamsSetBat.response.ret
                );
                return -1;
            }
        }

        sub = nodeRos.subscribe("/D_Battery/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataBatteryReceived);
        break;

    case MSGID_GPS:
        sub = nodeRos.subscribe("/extended_fix", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataGpsReceived);
        break;

    case MSGID_HOKUYO:
        sub = nodeRos.subscribe("/scan", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataHokuyoReceived);
        break;

    case MSGID_WEBCAM:
        if(subOnly)
        {
            lParamsSetCam.request.subscribe = true;
            lParamsSetCam.request.device = "";
            lParamsSetCam.request.width = 0;
            lParamsSetCam.request.height = 0;
            lParamsSetCam.request.fps = 0;
            ROS_INFO("Envoi de la requete au service");
            if(!ros::service::call("/D_Cam/params", lParamsSetCam))
            {
                ROS_ERROR(
                    "Le service de parametrage de la camera a renvoye une erreur (code %d).",
                    lParamsSetCam.response.ret
                );
                return -1;
            }
        }
        else
        {
            lParamsSetCam.request.subscribe = true;
            tmpDevice << "/dev/video";
            tmpDevice << (unsigned int) (info->paramSupp2);

            lParamsSetCam.request.device = tmpDevice.str();
            lParamsSetCam.request.width = (unsigned int) (info->paramSupp) * 20;
            lParamsSetCam.request.height = lParamsSetCam.request.width * 3 / 4;
            lParamsSetCam.request.fps = info->freqAcquisition;
            ROS_INFO("Envoi de la requete au service");
            if(!ros::service::call("/D_Cam/params", lParamsSetCam))
            {
                ROS_ERROR(
                    "Le service de parametrage de la camera a renvoye une erreur (code %d).",
                    lParamsSetCam.response.ret
                );
                return -1;
            }
        }

        sub = nodeRos.subscribe("/image_raw", 2 * SOCKET_SEND_TIMEOUT_SEC * 30, dataCamReceived);
        break;

    case MSGID_WEBCAM_STEREO:
        if(subOnly)
        {
            lParamsSetStereoCam.request.subscribe = true;
            lParamsSetStereoCam.request.device_L = "";
            lParamsSetStereoCam.request.device_R = "";
            lParamsSetStereoCam.request.width = 0;
            lParamsSetStereoCam.request.height = 0;
            lParamsSetStereoCam.request.fps = 0;
            ROS_INFO("Envoi de la requete au service");
            if(!ros::service::call("/D_CamStereo/params", lParamsSetStereoCam))
            {
                ROS_ERROR(
                    "Le service de parametrage de la camera stereo a renvoye une erreur (code %d).",
                    lParamsSetStereoCam.response.ret
                );
                return -1;
            }
        }
        else
        {
            lParamsSetStereoCam.request.subscribe = true;
            tmpDevice << "/dev/video";
            tmpDevice << (unsigned int) (info->paramSupp2) % 10;
            lParamsSetStereoCam.request.device_L = tmpDevice.str();

            tmpDevice.str("");
            tmpDevice << "/dev/video";
            tmpDevice << (unsigned int) (info->paramSupp2) / 10;
            lParamsSetStereoCam.request.device_R = tmpDevice.str();
            lParamsSetStereoCam.request.width = (unsigned int) (info->paramSupp) * 20;
            lParamsSetStereoCam.request.height = lParamsSetStereoCam.request.width * 3 / 4;
            lParamsSetStereoCam.request.fps = info->freqAcquisition;
            ROS_INFO("Envoi de la requete au service");
            if(!ros::service::call("/D_CamStereo/params", lParamsSetStereoCam))
            {
                ROS_ERROR(
                    "Le service de parametrage de la camera stereo a renvoye une erreur (code %d).",
                    lParamsSetStereoCam.response.ret
                );
                return -1;
            }
        }

        sub = nodeRos.subscribe("/D_CamStereo/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 30, dataStereoCamReceived);
        break;

    case MSGID_KINECT_DEPTH:
        // Published directly by driver only when ~depth_registration is true (OpenNI registration enabled).
        // On a quelque chose a faire pour activer depth_registration?
        sub = nodeRos.subscribe("/depth/image_raw", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataKinectDepthReceived);
        break;

    case MSGID_KINECT:
        sub = nodeRos.subscribe("/depth/image_raw", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataKinectDepthReceived);
        sub = nodeRos.subscribe("/rgb/image_raw", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataKinectRGBReceived);
        break;

    case MSGID_COMPUTER:
        if(subOnly)
        {
            lParamsSetComputer.request.subscribe = true;
            lParamsSetComputer.request.freqAcq = 0;
            lParamsSetComputer.request.bufferSize = 0;
            if(!ros::service::call("/D_ComputerInfo/params", lParamsSetComputer))
            {
                ROS_ERROR(
                    "Le service de parametrage des statistiques de l'ordinateur a renvoye une erreur (code %d).",
                    lParamsSetComputer.response.ret
                );
                return -1;
            }
        }
        else
        {
            lParamsSetComputer.request.subscribe = true;
            lParamsSetComputer.request.freqAcq = info->freqAcquisition;
            lParamsSetComputer.request.bufferSize = info->bufferSize;
            if(!ros::service::call("/D_ComputerInfo/params", lParamsSetComputer))
            {
                ROS_ERROR(
                    "Le service de parametrage des statistiques de l'ordinateur a renvoye une erreur (code %d).",
                    lParamsSetComputer.response.ret
                );
                return -1;
            }
        }

        sub = nodeRos.subscribe("D_ComputerInfo/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataComputerReceived);
        break;

    default:
        ROS_ERROR("Demande d'inscription a un type de capteur inconnu (%X) sera ignoree!", info->typeCapteur);
        return 1;
    }

    in_client.subscribers[info->typeCapteur].first = sub;
    in_client.buffersInfo[info->typeCapteur] = info->bufferSize;

    return 0;
}


int unsubscribeTo(msgUnsubscribe *info, matlabClient &in_client, bool deleteInMap = true)
{
    ros4mat::S_ADC          lParamsUnsetAdc;
    ros4mat::S_IMU          lParamsUnsetImu;

    //ros4mat::S_GPS lParamsUnsetGps;
    ros4mat::S_Battery      lParamsUnsetBattery;
    ros4mat::S_Cam          lParamsUnsetCam;
    ros4mat::S_StereoCam    lParamsUnsetStereoCam;
    ros4mat::S_Computer     lParamsUnsetComputer;

    ROS_INFO("Reception d'une demande de desinscription pour le capteur %02X", info->typeCapteur);
    if(in_client.subscribers.count(info->typeCapteur) == 0)
    {
        ROS_WARN("Tentative de desinscription sans etre prealablement inscrit sera ignoree.");
        return 1;
    }

    switch(info->typeCapteur)
    {
    case MSGID_ADC:
        lParamsUnsetAdc.request.subscribe = false;
        if(!ros::service::call("/D_ADC/params", lParamsUnsetAdc))
        {
            ROS_ERROR(
                "Le service de parametrage de l'ADC a renvoye une erreur (code %d) lors de la deconnexion.",
                lParamsUnsetAdc.response.ret
            );
            return -1;
        }
        break;

    case MSGID_IMU:
        lParamsUnsetImu.request.subscribe = false;
        if(!ros::service::call("/D_IMU/params", lParamsUnsetImu))
        {
            ROS_ERROR(
                "Le service de parametrage de l'IMU a renvoye une erreur (code %d) lors de la deconnexion.",
                lParamsUnsetImu.response.ret
            );
            return -1;
        }
        break;

    case MSGID_BATTERY:
        lParamsUnsetBattery.request.subscribe = false;
        if(!ros::service::call("/D_Battery/params", lParamsUnsetBattery))
        {
            ROS_ERROR(
                "Le service de parametrage de la batterie a renvoye une erreur (code %d) lors de la deconnexion.",
                lParamsUnsetBattery.response.ret
            );
            return -1;
        }
        break;

    case MSGID_WEBCAM:
        lParamsUnsetCam.request.subscribe = false;
        if(!ros::service::call("/D_Cam/params", lParamsUnsetCam))
        {
            ROS_ERROR(
                "Le service de parametrage de la camera a renvoye une erreur (code %d) lors de la deconnexion.",
                lParamsUnsetCam.response.ret
            );
            return -1;
        }
        break;

    case MSGID_WEBCAM_STEREO:
        lParamsUnsetStereoCam.request.subscribe = false;
        if(!ros::service::call("/D_CamStereo/params", lParamsUnsetStereoCam))
        {
            ROS_ERROR(
                "Le service de parametrage des cameras stereo a renvoye une erreur (code %d) lors de la deconnexion.",
                lParamsUnsetStereoCam.response.ret
            );
            return -1;
        }
        break;

    case MSGID_COMPUTER:
        lParamsUnsetComputer.request.subscribe = false;
        if(!ros::service::call("/D_ComputerInfo/params", lParamsUnsetComputer))
        {
            ROS_ERROR(
                "Le service de parametrage des stats de l'ordinateur a renvoye une erreur (code %d) lors de la deconnexion.",
                lParamsUnsetComputer.response.ret
            );
            return -1;
        }
        break;

    case MSGID_GPS:
    case MSGID_HOKUYO:
    case MSGID_KINECT_DEPTH:
        break;

    default:
        ROS_ERROR("Type de capteur inconnu (unsubscribe %X)!", info->typeCapteur);
    }

    while(!in_client.subscribers[info->typeCapteur].second.empty())
    {
        if(info->typeCapteur == MSGID_WEBCAM)
            delete[]((msgCamInternal *) (in_client.subscribers[info->typeCapteur].second.front()))->cptr;
        else if(info->typeCapteur == MSGID_HOKUYO)
            delete[]((msgHokuyoInternal *) (in_client.subscribers[info->typeCapteur].second.front()))->cptr;

        delete in_client.subscribers[info->typeCapteur].second.front();
        in_client.subscribers[info->typeCapteur].second.pop();
    }

    if(deleteInMap)
    {
        in_client.subscribers.erase(info->typeCapteur);
        in_client.buffersInfo.erase(info->typeCapteur);
    }

    return 0;
}


int unsubscribeAll(int in_client)
{
    std::map<unsigned int, std::pair<ros::Subscriber, std::queue<void *> > >::iterator  lIt;
    msgUnsubscribe                                                                      lUM;
    if(clients.count(in_client) == 0)
    {
        ROS_WARN("Tentative de deconnexion d'un client inexistant!");
        return -1;
    }

    for(lIt = clients[in_client].subscribers.begin(); lIt != clients[in_client].subscribers.end(); lIt++)
    {
        lUM.typeCapteur = (*lIt).first;
        unsubscribeTo(&lUM, clients[in_client], false);
    }

    clients.erase(in_client);
    return 0;
}


int sendSerialCmd(msgSerialCmd *info, matlabClient &in_client, char *answer, char &answerSize)
{
    ros4mat::S_Serial   lParams;
    ROS_INFO(
        "Serial received on Logico Agent : speed = %d / parity = %d / stop = %d / sendLength = %d / readLength = %d / readTimeoutSec = %d / readTimeoutMicro = %d",
        info->speed,
        info->parity,
        info->stopBits,
        info->sendLength,
        info->readLength,
        info->readTimeoutSec,
        info->readTimeoutMicro
    );
    lParams.request.port = std::string(info->port);
    ROS_INFO("Serial port %s", info->port);
    lParams.request.speed = info->speed;
    lParams.request.parity = info->parity;
    lParams.request.stopBits = info->stopBits;
    lParams.request.sendBufferLength = info->sendLength;
    lParams.request.readBufferLength = info->readLength;
    lParams.request.readTimeoutSec = info->readTimeoutSec;
    lParams.request.readTimeoutMicro = info->readTimeoutMicro;
    lParams.request.closeAfterComm = info->closeAfterComm;
    for(unsigned int i = 0; i < info->sendLength; i++) lParams.request.sendBuffer.push_back(info->data[i]);
    answer = new char[lParams.response.receiveBufferLength];
    for(unsigned int i = 0; i < lParams.response.receiveBufferLength; i++)
        answer[i] = lParams.response.receiveBuffer[i];
    answerSize = lParams.response.receiveBufferLength;
    ROS_INFO("Serial OK, sending data");
    if(!ros::service::call("/D_Serial/serialCommand", lParams))
    {
        ROS_ERROR("L'envoi de la commande serie a renvoye une erreur");
        return MSGID_SERIAL_ANS_NO_OPEN;
    }

    return MSGID_SERIAL_ANS_OK;
}


int sendDigitalOutCmd(msgDigitalOut *info)
{
    ros4mat::S_DigitalOut   lParams;
    lParams.request.outputState[0] = info->pinD0;
    lParams.request.outputState[1] = info->pinD1;
    lParams.request.outputState[2] = info->pinD2;
    lParams.request.outputState[3] = info->pinD3;

    if(!ros::service::call("/D_ADC/digitalOutCtrl", lParams))
    {
        ROS_ERROR("L'envoi de la commande digital out a renvoye une erreur");
        return MSGID_SERIAL_ANS_NO_OPEN;
    }

    return 0;
}


int sendDataToClient(int socketFd, msgHeader *inHeader, const char *bufferOut, int sizeBuffer, int compressionType)
{
    /* inHeader contains the message header
     * bufferOut contains the memcpy-ed data
     * compressionType is defined in exchangeStructs.h
     * This function must receive the header since it populates the compressSize variable of the header before sending. */
    long    sizeCompress;
    char    *bufferSend = 0;
    int     sendBytes = 0;

    inHeader->compressionType = compressionType;
    inHeader->uncompressSize = sizeBuffer;

    if(compressionType == MSGID_HEADER_NOCOMPRESSION)
    {
        bufferSend = new char[sizeof(msgHeader) + sizeBuffer];
        inHeader->compressSize = sizeBuffer;

        memcpy(bufferSend, inHeader, sizeof(msgHeader));
        memcpy(bufferSend + sizeof(msgHeader), bufferOut, sizeBuffer);

        sendBytes = send(socketFd, bufferSend, sizeof(msgHeader) + sizeBuffer, 0);
    }
    else if(compressionType == MSGID_HEADER_ZLIBCOMPRESSION)
    {
        if(sizeBuffer > 0)
        {
            sizeCompress = EZ_COMPRESSMAXDESTLENGTH(sizeBuffer);

            char    *bufferCompressOut = new char[sizeCompress];
            int     retVal = ezcompress(
                    (unsigned char *) bufferCompressOut,
                    &sizeCompress,
                    (unsigned char *) bufferOut,
                    sizeBuffer
                );
            if(retVal < 0)
            {
                ROS_ERROR("Erreur lors de la compression, %i retourne par ezlib!", retVal);
                return -1;
            }

            inHeader->compressSize = (unsigned int) sizeCompress;

            bufferSend = new char[sizeof(msgHeader) + inHeader->compressSize];
            memcpy(bufferSend + sizeof(msgHeader), bufferCompressOut, inHeader->compressSize);
            delete[] bufferCompressOut;
        }
        else
        {
            bufferSend = new char[sizeof(msgHeader)];
            inHeader->compressSize = 0;
        }

        memcpy(bufferSend, inHeader, sizeof(msgHeader));
        sendBytes = send(socketFd, bufferSend, sizeof(msgHeader) + inHeader->compressSize, 0);
    }

    ROS_INFO(
        "Data sent by sendDataToClient (%i bytes envoyes, %u bytes devraient l'avoir ete, header.type = %X)",
        sendBytes,
        sizeof(msgHeader) + inHeader->compressSize,
        inHeader->type
    );

    delete[] bufferSend;
    return sendBytes;
}

std::map<unsigned int, unsigned int>    ProtocolMsgSize;



int main(int argc, char **argv)
{
    ProtocolMsgSize[0x00] = sizeof(msgHeader);
    ProtocolMsgSize[MSGID_CONNECT] = sizeof(msgConnect);
    ProtocolMsgSize[MSGID_QUIT] = 0;
    ProtocolMsgSize[MSGID_SUBSCRIBE] = sizeof(msgSubscribe);
    ProtocolMsgSize[MSGID_UNSUBSCRIBE] = sizeof(msgUnsubscribe);
    ProtocolMsgSize[MSGID_SERIAL_CMD] = sizeof(msgSerialCmd);
    ProtocolMsgSize[MSGID_ADC] = sizeof(msgAdc);
    ProtocolMsgSize[MSGID_IMU] = sizeof(msgImu);
    ProtocolMsgSize[MSGID_BATTERY] = sizeof(msgBattery);
    ProtocolMsgSize[MSGID_GPS] = sizeof(msgGps);
    ProtocolMsgSize[MSGID_WEBCAM] = sizeof(msgCam);
    ProtocolMsgSize[MSGID_KINECT] = sizeof(msgCam);
    ProtocolMsgSize[MSGID_KINECT_DEPTH] = sizeof(msgCam);
    ProtocolMsgSize[MSGID_WEBCAM_STEREO] = sizeof(msgCam);
    ProtocolMsgSize[MSGID_HOKUYO] = sizeof(msgHokuyo);
    ProtocolMsgSize[MSGID_SERIAL_ANS] = sizeof(msgSerialAns);
    ProtocolMsgSize[MSGID_COMPUTER] = sizeof(msgComputer);
    ProtocolMsgSize[MSGID_DOUT_CTRL] = sizeof(msgDigitalOut);

    // Initialisation
    ros::init(argc, argv, "C_ros4mat");

    ros::NodeHandle     nodeRos;

    msgHeader           lHeader;

    int                 sd, rc, n, cliLen, flags;
    struct sockaddr_in  cliAddr, servAddr;
    char                *msg = NULL;
    struct timeval      timeout;
    char                *lAnswer = NULL;
    int                 recvBytes = 0;
    int                 i, retval, nbrImgSend, nbrCaptureSend;
    socklen_t           size;
    fd_set              active_fd_set, read_fd_set, write_fd_set, local_fd;

    /* socket creation */
    sd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(sd < 0)
    {
        ROS_FATAL("%s: cannot open socket \n", argv[0]);
        exit(1);
    }

    /* Deactivating Nagle algorithm to minimize latency */
    int flag = 1;
    int result = setsockopt(sd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));

    /* Number of SYN Retransmits */
    int syncnt = 2;
    setsockopt(sd, IPPROTO_TCP, TCP_SYNCNT, &syncnt, sizeof(int));

    /* Time-to-live of an orphan FIN-WAIT-2 */
    int finwaittimeout = 5;
    setsockopt(sd, SOL_SOCKET, TCP_LINGER2, &finwaittimeout, sizeof(int));

    /* send and recv timeout */
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(struct timeval));
    timeout.tv_sec = SOCKET_SEND_TIMEOUT_SEC;
    timeout.tv_usec = 0;
    setsockopt(sd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(struct timeval));

    // Do not fail on bind
    setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(int));

    /* flags = fcntl(sd, F_GETFL);
    flags |= O_NONBLOCK;
    fcntl(sd, F_SETFL, flags); */

    /* bind local server port */
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servAddr.sin_port = htons(LOCAL_SERVER_PORT);
    rc = bind(sd, (struct sockaddr *) &servAddr, sizeof(servAddr));
    if(rc < 0)
    {
        ROS_FATAL("%s: cannot bind port number %d \n", argv[0], LOCAL_SERVER_PORT);
        exit(1);
    }

    rc = listen(sd, 5);
    if(rc < 0)
    {
        ROS_FATAL("Impossible to listen on TCP socket");
        exit(1);
    }

    FD_ZERO(&active_fd_set);
    FD_SET(sd, &active_fd_set);

    ROS_INFO("%s: waiting for data on TCP port %u\n", argv[0], LOCAL_SERVER_PORT);

    ros::Rate   loop_rate = ros::Rate(200.);
    while(1)
    {
        cliLen = sizeof(cliAddr);

        read_fd_set = active_fd_set;
        FD_ZERO(&write_fd_set);

        timeout.tv_sec = 0;
        timeout.tv_usec = 100;
        if(select(FD_SETSIZE, &read_fd_set, &write_fd_set, NULL, &timeout) < 0)
        {
            perror("Select Error");
            exit(EXIT_FAILURE);
        }

        for(i = 0; i < FD_SETSIZE; ++i)
        {
            if(FD_ISSET(i, &read_fd_set))
            {
                if(i == sd)
                {
                    /* Connection request on original socket. */
                    int newcon;
                    size = sizeof(cliAddr);
                    newcon = accept(sd, (struct sockaddr *) &cliAddr, &size);
                    if(newcon < 0)
                    {
                        ROS_ERROR("accept");
                        exit(EXIT_FAILURE);
                    }

                    ROS_INFO(
                        "Server: connect from host %s, port %u.\n",
                        inet_ntoa(cliAddr.sin_addr),
                        ntohs(cliAddr.sin_port)
                    );
                    FD_SET(newcon, &active_fd_set);
                }
                else
                {
                    FD_ZERO(&local_fd);
                    FD_SET(i, &local_fd);
                    recvBytes = 0;

                    int recvReturn = 0;

                    // Header reception
                    msg = new char[sizeof(msgHeader)];
                    while(recvBytes < sizeof(msgHeader))
                    {
                        /* Node data reception */
                        recvReturn = recv(i, msg + recvBytes, sizeof(msgHeader) - recvBytes, 0);
                        if(recvReturn == -1)
                        {
                            // Timeout
                            ROS_ERROR(
                                "Timeout on header reception (Receibed %db / %db expected)",
                                recvBytes,
                                sizeof(msgHeader)
                            );
                            break;
                        }
                        else if(recvReturn == 0)
                        {
                            ROS_ERROR("Connection closed by the client on header reception");
                            unsubscribeAll(i);

                            // Node unsubscribtion
                            clients.erase(i);
                            close(i);
                            FD_CLR(i, &active_fd_set);
                            delete[] msg;
                            break;
                        }

                        recvBytes += recvReturn;
                    }

                    if(recvReturn <= 0) continue;

                    std::map<unsigned int, std::pair<ros::Subscriber, std::queue<void *> > >::iterator  lIt;

                    msgUnsubscribe                                                                      lUM;
                    if(recvBytes < sizeof(msgHeader))
                    {
                        if(n == 0)
                        {
                            ROS_INFO("Closed connexion");
                            unsubscribeAll(i);
                            clients.erase(i);
                            close(i);
                            FD_CLR(i, &active_fd_set);
                        }

                        // If it's a corrupted message or a new connection
                        delete[] msg;
                        break;
                    }

                    msgHeader   lHeader;
                    memcpy(&lHeader, msg, sizeof(msgHeader));
                    delete[] msg;
                    if(ProtocolMsgSize.count(lHeader.type) == 0)
                    {
                        ROS_WARN("Received an unknown message type: %X. Ignoring...", lHeader.type);
                        continue;
                    }

                    msg = new char[ProtocolMsgSize[lHeader.type] * lHeader.size + 1];

                    // Header parsing and data handling
                    recvBytes = 0;
                    while(recvBytes < ProtocolMsgSize[lHeader.type] * lHeader.size)
                    {
                        /* Message reception */
                        recvReturn = recv(i, msg + recvBytes, ProtocolMsgSize[lHeader.type] - recvBytes, 0);
                        if(recvReturn == -1)
                        {
                            // Timeout
                            ROS_ERROR(
                                "Timeout on message reception (header correctly received) : %db received, %db expected!",
                                recvBytes,
                                ProtocolMsgSize[lHeader.type] * lHeader.size
                            );
                            delete[] msg;
                            break;
                        }
                        else if(recvReturn == 0)
                        {
                            ROS_ERROR("Connection closed by client");
                            unsubscribeAll(i);

                            // Node unsubscribtion
                            clients.erase(i);
                            close(i);
                            FD_CLR(i, &active_fd_set);
                            delete[] msg;
                            break;
                        }

                        recvBytes += recvReturn;
                    }

                    if(recvReturn <= 0) continue;

                    n = recvBytes;

                    msgHeader       lAnswerHeader;
                    msgConnect      lConnect;
                    msgSubscribe    lSubscribe;
                    msgComputer     lComputer;
                    msgUnsubscribe  lUnsubscribe;
                    msgDigitalOut   lDigitalCmd;
                    msgSerialCmd    lSerialCmd;
                    msgSerialAns    lSerialAns;
                    char            *lRetour;
                    char            lRetourSize = 0;
                    switch(lHeader.type)
                    {
                    case MSGID_CONNECT:
                        ROS_DEBUG("Received Connect");
                        memcpy(&lConnect, msg, sizeof(msgConnect));
                        lAnswer = new char[sizeof(msgHeader)];
                        lAnswerHeader.type = MSGID_CONNECT_ACK;
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.size = 1;
                        lAnswerHeader.packetTimestamp = 0.0;
                        memcpy(lAnswer, &lAnswerHeader, sizeof(msgHeader));
                        send(i, lAnswer, sizeof(msgHeader), 0);
                        clients[i] = matlabClient();
                        clients[i].compression = lConnect.compression;
                        delete lAnswer;
                        break;

                    case MSGID_QUIT:
                        ROS_DEBUG("Received Quit");
                        unsubscribeAll(i);

                        // Node unsubscribtion
                        clients.erase(i);
                        close(i);
                        FD_CLR(i, &active_fd_set);
                        break;

                    case MSGID_SUBSCRIBE:
                        memcpy(&lSubscribe, msg, sizeof(msgSubscribe));
                        ROS_DEBUG("Received Subscribe for %X", lSubscribe.typeCapteur);
                        subscribeTo(&lSubscribe, nodeRos, clients[i]);
                        break;

                    case MSGID_UNSUBSCRIBE:
                        ROS_DEBUG("Received Unsubscribe");
                        memcpy(&lUnsubscribe, msg, sizeof(msgUnsubscribe));
                        unsubscribeTo(&lUnsubscribe, clients[i]);
                        break;

                    case MSGID_SERIAL_CMD:
                        ROS_DEBUG("Received serial command");
                        memcpy(&lSerialCmd, msg, sizeof(msgSerialCmd));
                        lAnswerHeader.type = MSGID_SERIAL_ANS;
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.size = 1;
                        lAnswerHeader.packetTimestamp = 0.0;
                        lAnswerHeader.compressSize = sizeof(msgSerialAns);
                        lAnswerHeader.uncompressSize = sizeof(msgSerialAns);
                        lAnswerHeader.compressionType = MSGID_HEADER_NOCOMPRESSION;
                        lSerialAns.status = sendSerialCmd(&lSerialCmd, clients[i], lRetour, lRetourSize);
                        lSerialAns.bufferLength = lRetourSize;

                        memcpy(lSerialAns.data, lRetour, lRetourSize);
                        lAnswer = new char[sizeof(msgHeader) + sizeof(msgSerialAns)];
                        memcpy(lAnswer, &lAnswerHeader, sizeof(msgHeader));
                        memcpy(lAnswer + sizeof(msgHeader), &lSerialAns, sizeof(msgSerialAns));
                        send(i, lAnswer, sizeof(msgHeader) + sizeof(msgSerialAns), 0);
                        break;

                    case MSGID_DOUT_CTRL:
                        ROS_DEBUG("Received Digital Out Control");
                        memcpy(&lDigitalCmd, msg, sizeof(msgDigitalOut));
                        sendDigitalOutCmd(&lDigitalCmd);
                        break;

                    case MSGID_WEBCAM:
                    case MSGID_KINECT:
                    case MSGID_KINECT_DEPTH:
                        if(lHeader.type == MSGID_KINECT_DEPTH)
                            ROS_INFO("Received Kinect Depth");
                        else if(lHeader.type == MSGID_KINECT)
                            ROS_INFO("Received Kinect");
                        else
                            ROS_INFO("Received Webcam");

                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                        if(clients[i].subscribers[lHeader.type].second.size() == 0)
                        {
                            // No image to send
                            ROS_INFO("No data to send");
                            lAnswerHeader.size = 0;
                            lAnswer = new char[sizeof(msgHeader)];
                            memcpy(lAnswer, &lAnswerHeader, sizeof(msgHeader));
                            send(i, lAnswer, sizeof(msgHeader), 0);
                        }
                        else
                        {
                            /* Sending protocol: We send an msgHeader containing the number of images 
                                 * contained in the size variable and their resolution (X multiplied by 20,
                                 * as the subscribe protocol).
                                 * Then, we send EVERY imgStruct in one shot, then the images having the 
                                 * same size */
                            nbrImgSend = clients[i].subscribers[lHeader.type].second.size();

                            int imgSizeBytes =
                                ((msgCamInternal *) clients[i].subscribers[lHeader.type].second.front())->sizeData;

                            int sendSize = nbrImgSend * sizeof(msgCam) + nbrImgSend * imgSizeBytes;

                            lAnswerHeader.size = nbrImgSend;
                            lAnswerHeader.info =
                                (((msgCamInternal *) clients[i].subscribers[lHeader.type].second.front())->width) /
                                20;

                            lAnswer = new char[sendSize];

                            msgCam          lCamData;
                            msgCamInternal  *currentCamStruct = 0;
                            for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                            {
                                // msgCam struct copy at the packet beginning
                                currentCamStruct = (msgCamInternal *) clients[i].subscribers[lHeader.type].second.front();
                                lCamData.width = currentCamStruct->width;
                                lCamData.height = currentCamStruct->height;
                                lCamData.channels = currentCamStruct->channels;
                                lCamData.sizeData = currentCamStruct->sizeData;
                                lCamData.timestamp = currentCamStruct->timestamp;

                                // Image subheader copy
                                memcpy(lAnswer + k * sizeof(msgCam), &lCamData, sizeof(msgCam));

                                // Image copy
                                if(lHeader.type == MSGID_KINECT)
                                {
                                    memcpy(
                                        lAnswer + nbrImgSend * sizeof(msgCam) + k * imgSizeBytes,
                                        currentCamStruct->cptr,
                                        imgSizeBytes - 640 * 480 * sizeof(uint16_t)
                                    );
                                    memcpy(
                                        lAnswer + nbrImgSend * sizeof(msgCam) + k * imgSizeBytes,
                                        ((msgCamInternal *) clients[i].subscribers[MSGID_KINECT_DEPTH].second.front())->cptr,
                                        640 * 480 * sizeof(uint16_t)
                                    );
                                }
                                else
                                {
                                    memcpy(
                                        lAnswer + nbrImgSend * sizeof(msgCam) + k * imgSizeBytes,
                                        currentCamStruct->cptr,
                                        imgSizeBytes
                                    );
                                }

                                // Buffer pruning
                                delete[] currentCamStruct->cptr;
                                delete currentCamStruct;
                                clients[i].subscribers[lHeader.type].second.pop();
                                if(lHeader.type == MSGID_KINECT)
                                {
                                    delete[]((msgCamInternal *) clients[i].subscribers[MSGID_KINECT_DEPTH].second.front())->cptr;
                                    delete (msgCamInternal *) clients[i].subscribers[MSGID_KINECT_DEPTH].second.front();
                                    clients[i].subscribers[MSGID_KINECT_DEPTH].second.pop();
                                }
                            }

                            ROS_INFO("Send webcam data: %X (size = %i)", lAnswerHeader.type, sendSize);
                            sendDataToClient(i, &lAnswerHeader, lAnswer, sendSize, clients[i].compression);
                        }

                        delete lAnswer;
                        break;

                    case MSGID_WEBCAM_STEREO:
                        ROS_INFO("Received Webcam Stereo");

                        lAnswerHeader.type = MSGID_WEBCAM_STEREO;
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();

                        if(clients[i].subscribers[MSGID_WEBCAM_STEREO].second.size() == 0)
                        {
                            // No image to send
                            ROS_INFO("No data to send");
                            lAnswerHeader.size = 0;
                            lAnswer = new char[sizeof(msgHeader)];
                            memcpy(lAnswer, &lAnswerHeader, sizeof(msgHeader));
                            send(i, lAnswer, sizeof(msgHeader), 0);
                        }
                        else
                        {
                            /* Sending protocol: We send an msgHeader containing the number of images 
                                 * contained in the size variable and their resolution (X multiplied by 20,
                                 * as the subscribe protocol).
                                 * Then, we send EVERY imgStruct in one shot, then the images having the 
                                 * same size. The left and right images are alternated (one left, one right, one left, ...)
                                 * We send two imgStruct per set of two images */
                            nbrImgSend = clients[i].subscribers[MSGID_WEBCAM_STEREO].second.size();

                            int imgSizeBytes =
                                ((msgStereoCamInternal *) clients[i].subscribers[MSGID_WEBCAM_STEREO].second.front())->sizeData;

                            int sendSize = nbrImgSend * sizeof(msgCam) + nbrImgSend * imgSizeBytes * 2; // *2 because stereo
                            lAnswerHeader.size = nbrImgSend;
                            lAnswerHeader.info =
                                (((msgStereoCamInternal *) clients[i].subscribers[MSGID_WEBCAM_STEREO].second.front())->width) /
                                20;

                            lAnswer = new char[sendSize];

                            msgCam                  lCamData;
                            msgStereoCamInternal    *currentCamStruct = 0;
                            for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                            {
                                // msgCam struct copy at the packet beginning
                                currentCamStruct = (msgStereoCamInternal *) clients[i].subscribers[MSGID_WEBCAM_STEREO].second.front();
                                lCamData.width = currentCamStruct->width;
                                lCamData.height = currentCamStruct->height;
                                lCamData.channels = currentCamStruct->channels;
                                lCamData.sizeData = currentCamStruct->sizeData;
                                lCamData.timestamp = currentCamStruct->timestamp;

                                // Header copy
                                memcpy(lAnswer + (k + 0) * sizeof(msgCam), &lCamData, sizeof(msgCam));
                                memcpy(lAnswer + (k + 1) * sizeof(msgCam), &lCamData, sizeof(msgCam));

                                // Left Image copy
                                memcpy(
                                    lAnswer + nbrImgSend * sizeof(msgCam) * 2 + k * imgSizeBytes * 2,
                                    currentCamStruct->cptr_L,
                                    imgSizeBytes
                                );

                                // Right Image copy
                                memcpy(
                                    lAnswer + nbrImgSend * sizeof(msgCam) * 2 + k * imgSizeBytes * 2 + imgSizeBytes,
                                    currentCamStruct->cptr_R,
                                    imgSizeBytes
                                );

                                // Buffer pruning
                                delete[] currentCamStruct->cptr_L;
                                delete[] currentCamStruct->cptr_R;
                                delete currentCamStruct;
                                clients[i].subscribers[MSGID_WEBCAM_STEREO].second.pop();
                            }

                            ROS_INFO("Send webcam data: %X (size = %i)", lAnswerHeader.type, sendSize);
                            sendDataToClient(i, &lAnswerHeader, lAnswer, sendSize, clients[i].compression);
                        }

                        delete lAnswer;
                        break;

                    case MSGID_HOKUYO:
                        ROS_INFO("Received Hokuyo");
                        lAnswerHeader.type = MSGID_HOKUYO;
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                        if(clients[i].subscribers[MSGID_HOKUYO].second.size() == 0)
                        {
                            // No image to send
                            ROS_INFO("No data to send");
                            lAnswerHeader.size = 0;
                            lAnswer = new char[sizeof(msgHeader)];
                            memcpy(lAnswer, &lAnswerHeader, sizeof(msgHeader));
                            send(i, lAnswer, sizeof(msgHeader), 0);
                        }
                        else
                        {
                            /* Sending protocol: We send an msgHeader containing the number of depth scans 
                                 * contained in the size variable.
                                 * Then, we send EVERY rangeStruct in one shot, then the ranges having the 
                                 * same size */
                            nbrCaptureSend = clients[i].subscribers[MSGID_HOKUYO].second.size();

                            int rangesSizeBytes =
                                ((msgHokuyoInternal *) clients[i].subscribers[MSGID_HOKUYO].second.front())->sizeData;

                            int sendSize = nbrCaptureSend * sizeof(msgHokuyo) + nbrCaptureSend * rangesSizeBytes;

                            lAnswerHeader.size = nbrCaptureSend;

                            lAnswer = new char[sendSize];

                            msgHokuyo           lHokuyoData;
                            msgHokuyoInternal   *currentStruct = 0;
                            for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                            {
                                // msgCam struct copy at the packet beginning
                                currentStruct = (msgHokuyoInternal *) clients[i].subscribers[MSGID_HOKUYO].second.front();
                                lHokuyoData.angleMin = currentStruct->angleMin;
                                lHokuyoData.angleMax = currentStruct->angleMax;
                                lHokuyoData.angleIncrement = currentStruct->angleIncrement;
                                lHokuyoData.timeIncrement = currentStruct->timeIncrement;
                                lHokuyoData.rangeMin = currentStruct->rangeMin;
                                lHokuyoData.rangeMax = currentStruct->rangeMax;
                                lHokuyoData.sizeData = currentStruct->sizeData;
                                lHokuyoData.timestamp = currentStruct->timestamp;
                                memcpy(lAnswer + k * sizeof(msgHokuyo), &lHokuyoData, sizeof(msgHokuyo));

                                // Image copy
                                memcpy(
                                    lAnswer + nbrCaptureSend * sizeof(msgHokuyo) + k * rangesSizeBytes,
                                    currentStruct->cptr,
                                    rangesSizeBytes
                                );

                                // Buffer pruning
                                delete[] currentStruct->cptr;
                                delete currentStruct;
                                clients[i].subscribers[MSGID_HOKUYO].second.pop();
                            }

                            ROS_INFO("Send hokuyo data: %X (size = %i)", lAnswerHeader.type, sendSize);
                            sendDataToClient(i, &lAnswerHeader, lAnswer, sendSize, clients[i].compression);
                        }

                        delete lAnswer;
                        break;

                    // TODO: Unify all these together
                    case MSGID_BATTERY:
                        ROS_DEBUG("Received Battery");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = clients[i].subscribers[lHeader.type].second.size();
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();

                        lAnswer = new char[lAnswerHeader.size * sizeof(msgBattery)];
                        for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                        {
                            memcpy(
                                lAnswer + k * sizeof(msgBattery),
                                clients[i].subscribers[lHeader.type].second.front(),
                                sizeof(msgBattery)
                            );
                            delete (msgBattery *) clients[i].subscribers[lHeader.type].second.front();
                            clients[i].subscribers[lHeader.type].second.pop();
                        }

                        sendDataToClient(
                            i,
                            &lAnswerHeader,
                            lAnswer,
                            lAnswerHeader.size * sizeof(msgBattery),
                            clients[i].compression
                        );
                        delete lAnswer;
                        break;

                    case MSGID_GPS:
                        ROS_DEBUG("Received GPS");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = clients[i].subscribers[lHeader.type].second.size();
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();

                        lAnswer = new char[lAnswerHeader.size * sizeof(msgGps)];
                        for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                        {
                            memcpy(
                                lAnswer + k * sizeof(msgGps),
                                clients[i].subscribers[lHeader.type].second.front(),
                                sizeof(msgGps)
                            );
                            delete (msgGps *) clients[i].subscribers[lHeader.type].second.front();
                            clients[i].subscribers[lHeader.type].second.pop();
                        }

                        sendDataToClient(
                            i,
                            &lAnswerHeader,
                            lAnswer,
                            lAnswerHeader.size * sizeof(msgGps),
                            clients[i].compression
                        );
                        delete lAnswer;
                        break;

                    case MSGID_COMPUTER:
                        ROS_DEBUG("Received Computer");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = clients[i].subscribers[lHeader.type].second.size();
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();

                        lAnswer = new char[lAnswerHeader.size * sizeof(msgComputer)];
                        for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                        {
                            memcpy(
                                lAnswer + k * sizeof(msgComputer),
                                clients[i].subscribers[lHeader.type].second.front(),
                                sizeof(msgComputer)
                            );
                            delete (msgComputer *) clients[i].subscribers[lHeader.type].second.front();
                            clients[i].subscribers[lHeader.type].second.pop();
                        }

                        sendDataToClient(
                            i,
                            &lAnswerHeader,
                            lAnswer,
                            lAnswerHeader.size * sizeof(msgComputer),
                            clients[i].compression
                        );

                        delete lAnswer;
                        break;

                    case MSGID_ADC:
                        ROS_DEBUG("Received ADC");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = clients[i].subscribers[lHeader.type].second.size();
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();

                        lAnswer = new char[lAnswerHeader.size * sizeof(msgAdc)];
                        for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                        {
                            memcpy(
                                lAnswer + k * sizeof(msgAdc),
                                clients[i].subscribers[lHeader.type].second.front(),
                                sizeof(msgAdc)
                            );
                            delete (msgAdc *) clients[i].subscribers[lHeader.type].second.front();
                            clients[i].subscribers[lHeader.type].second.pop();
                        }

                        sendDataToClient(
                            i,
                            &lAnswerHeader,
                            lAnswer,
                            lAnswerHeader.size * sizeof(msgAdc),
                            clients[i].compression
                        );
                        delete lAnswer;
                        break;

                    case MSGID_IMU:
                        ROS_DEBUG("Received IMU");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = clients[i].subscribers[lHeader.type].second.size();
                        lAnswerHeader.info = 0x00;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();

                        lAnswer = new char[lAnswerHeader.size * sizeof(msgImu)];
                        for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                        {
                            memcpy(
                                lAnswer + k * sizeof(msgImu),
                                clients[i].subscribers[lHeader.type].second.front(),
                                sizeof(msgImu)
                            );
                            delete (msgImu *) clients[i].subscribers[lHeader.type].second.front();
                            clients[i].subscribers[lHeader.type].second.pop();
                        }

                        sendDataToClient(
                            i,
                            &lAnswerHeader,
                            lAnswer,
                            lAnswerHeader.size * sizeof(msgImu),
                            clients[i].compression
                        );
                        delete lAnswer;
                        break;

                    default:
                        ROS_ERROR("Unknown request: %d", lHeader.type);
                        break;
                    }

                    delete[] msg;
                }   // If inbound data
            }       // Isset
        }           // Data For

        ros::spinOnce();
        loop_rate.sleep();
    }               // While

    return 0;
}
