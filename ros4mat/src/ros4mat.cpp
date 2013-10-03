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

#include <iostream>
#include <fstream>

#include <ros4mat/M_ADC.h>
#include <ros4mat/S_ADC.h>
#include <ros4mat/M_IMU.h>
#include <ros4mat/S_IMU.h>
#include <ros4mat/M_Battery.h>
#include <ros4mat/S_Battery.h>
#include <ros4mat/M_Computer.h>
#include <ros4mat/S_Computer.h>
#include <ros4mat/M_Kinect.h>
#include <ros4mat/S_Kinect.h>
#include <ros4mat/M_Cam.h>
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
#define SOCKET_SEND_TIMEOUT_SEC 10

typedef struct msgCamInternal   msgCamInternal;
struct msgCamInternal
{
    uint32_t        width;
    uint32_t        height;
    unsigned char   channels;
    uint32_t        sizeData;
    char            *cptr;      /* Keeps the image buffer address */
    char            compressionType;
    double          timestamp;  /* Image acquisition timestamp */
};

typedef struct msgStereoCamInternal msgStereoCamInternal;
struct msgStereoCamInternal
{
    uint32_t        width_L;
    uint32_t        width_R;
    uint32_t        height_L;
    uint32_t        height_R;
    unsigned char   channels;
    uint32_t        sizeData_L;
    uint32_t        sizeData_R;
    char            *cptr_L;    /* Keeps the image buffer address (left img) */
    char            *cptr_R;    /* Keeps the image buffer address (right img) */
    char            compressionType;
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

typedef struct msgKinectInternal    msgKinectInternal;
struct msgKinectInternal
{
    msgCamInternal infoDepth;
    msgCamInternal infoRGB;
};

typedef struct matlabClient matlabClient;
struct matlabClient
{
    int                                                                         id;
    char                                                                        compression;
    std::map<unsigned int, std::pair<ros::Subscriber, std::queue<void *> > >    subscribers;    // subscribers, buffered data
    std::map<unsigned int, uint32_t>                                            buffersInfo;    // Allocated size
    std::string                                                                 lasterror;
    std::string                                                                 lasterror_issued_by;
};

std::map<uint32_t, matlabClient> clients;    // uint = unique client id

void dataAdcReceived(const ros4mat::M_ADC::ConstPtr &msg)
{
    std::map<uint32_t, matlabClient>::iterator   lClientIt;
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
    std::map<uint32_t, matlabClient>::iterator   lClientIt;
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


void dataCamReceived(const ros4mat::M_Cam::ConstPtr &msg)
{
    char                                    *bufferImg;
    std::map<uint32_t, matlabClient>::iterator   lClientIt;
    msgCamInternal                          *lMsg = NULL;
    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_WEBCAM) == 0) continue;
        lMsg = new msgCamInternal;
        lMsg->timestamp = msg->header.stamp.toSec();
        lMsg->width = msg->width;
        lMsg->height = msg->height;
        lMsg->channels = 3;
        lMsg->sizeData = (msg->image).size();
        lMsg->compressionType = msg->compressionRatio;

        if(lMsg->sizeData != lMsg->width * lMsg->height * 3 && lMsg->compressionType == 0)
        {
            ROS_WARN(
                "Incoherence de taille des buffers de camera (taille prevue : %d, obtenue : %d",
                lMsg->width * lMsg->height * 3,
                lMsg->sizeData
            );
        }

        bufferImg = new char[lMsg->sizeData];
        for(unsigned int i = 0; i < lMsg->sizeData; i++) bufferImg[i] = msg->image[i];
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

int kk = 0;
void dataStereoCamReceived(const ros4mat::M_StereoCam::ConstPtr &image)
{
    char                                    *bufferImg_L, *bufferImg_R;
    std::map<uint32_t, matlabClient>::iterator   lClientIt;
    msgStereoCamInternal                    *lMsg = NULL;

    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_WEBCAM_STEREO) == 0) continue;
        lMsg = new msgStereoCamInternal;

        // TODO : Differencier proprietes de l'image gauche et droite
        lMsg->timestamp = image->timestamp;
        lMsg->width_L = image->width;
        lMsg->width_R = image->width;
        lMsg->height_L = image->height;
        lMsg->height_R = image->height;
        lMsg->channels = 3;
        lMsg->sizeData_L = (image->image_left).size();
        lMsg->sizeData_R = (image->image_right).size();
        ROS_INFO("Size left / Size right = %d / %d", lMsg->sizeData_L, lMsg->sizeData_R);
        lMsg->compressionType = image->compressionRatio;
        if(lMsg->sizeData_L != lMsg->width_L * lMsg->height_L * 3 && lMsg->compressionType == 0)
        {
            ROS_WARN(
                "Incoherence de taille des buffers de camera (taille prevue : %d, obtenue : %d",
                lMsg->width_L * lMsg->height_L * 3,
                lMsg->sizeData_L
            );
        }

        if((image->image_left).size() != (image->image_right).size() && lMsg->compressionType == 0)
        {
            ROS_WARN(
                "Incoherence de taille entre camera gauche et droite (gauche : %d, droite : %d )",
                (image->image_left).size(),
                (image->image_right).size()
            );
        }

        bufferImg_L = new char[lMsg->sizeData_L];
        bufferImg_R = new char[lMsg->sizeData_R];
        for(unsigned int i = 0; i < lMsg->sizeData_L; i++)
        {
            bufferImg_L[i] = image->image_left[i];
        }

        for(unsigned int i = 0; i < lMsg->sizeData_R; i++)
        {
            bufferImg_R[i] = image->image_right[i];
        }

        lMsg->cptr_L = bufferImg_L;
        lMsg->cptr_R = bufferImg_R;

        if(kk == 0){
            std::ofstream outTestJpg1, outTestJpg2;
            ROS_INFO("Writing JPG to files");
            outTestJpg1.open ("SEMIRAW_testgauche.jpg");
            outTestJpg1.write((char *)lMsg->cptr_L, lMsg->sizeData_L);
            outTestJpg1.close();
            ROS_INFO("Writing JPG to files");
            outTestJpg2.open ("SEMIRAW_testdroite.jpg");
            outTestJpg2.write((char *)lMsg->cptr_R, lMsg->sizeData_R);
            outTestJpg2.close();
            kk = 1;
        }


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
    std::map<uint32_t, matlabClient>::iterator   lClientIt;
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



void dataComputerReceived(const ros4mat::M_Computer::ConstPtr &msg)
{
    std::map<uint32_t, matlabClient>::iterator   lClientIt;
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
    std::map<uint32_t, matlabClient>::iterator   lClientIt;
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


void dataKinectReceived(const ros4mat::M_Kinect::ConstPtr &msg)
{
    char                                    *bufferRgb, *bufferDepth;
    std::map<uint32_t, matlabClient>::iterator   lClientIt;
    msgKinectInternal                          *lMsg = NULL;

    for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
    {
        if((*lClientIt).second.subscribers.count(MSGID_KINECT) == 0) continue;
        lMsg = new msgKinectInternal;
        lMsg->infoRGB.timestamp = msg->header.stamp.toSec();
        lMsg->infoDepth.timestamp = msg->header.stamp.toSec();

        lMsg->infoRGB.width = msg->width_rgb;
        lMsg->infoRGB.height = msg->height_rgb;
        lMsg->infoRGB.channels = 3;
        lMsg->infoRGB.sizeData = (msg->rgb).size();

        lMsg->infoDepth.width = msg->width_depth;
        lMsg->infoDepth.height = msg->height_depth;
        lMsg->infoDepth.sizeData = (msg->depth).size();

        lMsg->infoRGB.compressionType = msg->compressionRatio;
        lMsg->infoDepth.compressionType = 0;

        /*if(lMsg->sizeData != lMsg->width * lMsg->height * 1 * 2)
        {   // * 2 parce que ce sont des uint16
            ROS_WARN(
                "Incoherence de taille des buffers de camera (taille prevue : %d, obtenue : %d",
                lMsg->width * lMsg->height * 1 * 2,
                lMsg->sizeData
            );
        }*/
        
        // TODO : Ne transmettre que les images demandees (p. ex. pas de RGB)
        bufferRgb = new char[lMsg->infoRGB.sizeData];
        bufferDepth = new char[lMsg->infoDepth.sizeData];
        for(unsigned int i = 0; i < lMsg->infoRGB.sizeData; i++)
        {
            bufferRgb[i] = (msg->rgb)[i];
        }
        for(unsigned int i = 0; i < lMsg->infoDepth.sizeData; i++)
        {
            bufferDepth[i] = (msg->depth)[i];
        }

        lMsg->infoRGB.cptr = bufferRgb;
        lMsg->infoDepth.cptr = bufferDepth;

        // Add to the buffer
        (*lClientIt).second.subscribers[MSGID_KINECT].second.push((void *) lMsg);
        lMsg = NULL;

        // Pruning obsolete data
        while (
            (*lClientIt).second.subscribers[MSGID_KINECT].second.size() >
                (*lClientIt).second.buffersInfo[MSGID_KINECT]
        )
        {
            ROS_DEBUG("Pruning Kinect data");
            //ROS_INFO("Adresse RGB = %p, adresse Depth = %p", ((msgKinectInternal *) (*lClientIt).second.subscribers[MSGID_KINECT].second.front())->infoRGB.cptr, ((msgKinectInternal *) (*lClientIt).second.subscribers[MSGID_KINECT].second.front())->infoDepth.cptr);
            delete[] ((msgKinectInternal *) (*lClientIt).second.subscribers[MSGID_KINECT].second.front())->infoRGB.cptr;
            delete[] ((msgKinectInternal *) (*lClientIt).second.subscribers[MSGID_KINECT].second.front())->infoDepth.cptr;
            delete(msgKinectInternal *) ((*lClientIt).second.subscribers[MSGID_KINECT].second.front());
            (*lClientIt).second.subscribers[MSGID_KINECT].second.pop();
        }
    }
}



int subscribeTo(unsigned char typeCapteur, uint32_t bufferSize, char* info, bool subOnly, ros::NodeHandle nodeRos, matlabClient &in_client)
{
    /* TODO: ADD Buffer toward publishers*/
    /* C'est quoi ce TODO la?? */
    ros::Subscriber         sub;

    std::stringstream       tmpDevice;

    ros4mat::S_ADC          lParamsSetAdc;
    ros4mat::S_IMU          lParamsSetImu;
    ros4mat::S_Battery      lParamsSetBat;
    ros4mat::S_Cam          lParamsSetCam;
    ros4mat::S_StereoCam    lParamsSetStereoCam;
    ros4mat::S_Kinect       lParamsSetKinect;
    ros4mat::S_Computer     lParamsSetComputer;

    if(subOnly){
        /* Special case:
                Parameters don't change (no service call), we only subscribe */
        ROS_INFO("Inscription seulement");
        if(bufferSize == 0)
        {
            int                                     bufferNewSize = 1;
            std::map<uint32_t, matlabClient>::iterator   lClientIt;
            bool                                    otherClientConnected = false;

            for(lClientIt = clients.begin(); lClientIt != clients.end(); lClientIt++)
            {
                if((*lClientIt).second.subscribers.count(typeCapteur) > 0){
                    bufferNewSize = (*lClientIt).second.buffersInfo[typeCapteur];
                    otherClientConnected = true;
                }
            }

            if(!otherClientConnected)
            {
                // TODO : CHECK if we cannot connect anyway
                ROS_WARN(
                    "Aucun client actuellement connecte au capteur %02X, inscription seulement ignoree",
                    typeCapteur
                );
                in_client.lasterror = "No client currently connected to the sensor requested for silent subscribe. Ignoring.";
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }

            bufferSize = bufferNewSize;
            ROS_INFO(
                "Pas de taille de buffer envoyee, utilisation de la taille du precedent subscriber (%d echantillons)",
                bufferNewSize
            );
        }
    }


    ROS_INFO(
        "Reception d'une demande d'inscription pour le capteur %02X.\nTaille du buffer circulaire : %i",
        typeCapteur,
        bufferSize
    );


    paramsAdc lStructAdc;
    paramsImu lStructImu;
    paramsGps lStructGps;
    paramsCamera lStructCamera;
    paramsStereoCam lStructStereoCam;
    paramsKinect lStructKinect;
    paramsHokuyo lStructHokuyo;
    paramsComputer lStructComputer;

    FILE *console;
    char bufR[10];

    switch(typeCapteur)
    {
    case MSGID_ADC:
        if(!(console = popen("rosservice list | grep /D_ADC/params | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si le node ADC est lance!");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node ADC not started!");
            in_client.lasterror = "The ADC node is not started or crashed (no service at /D_ADC/params). Impossible to subscribe";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }
        if(subOnly)
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
                in_client.lasterror = "The ADC silent subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetAdc.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }
        else
        {       
            memcpy(&lStructAdc, info, sizeof(lStructAdc));
            ROS_INFO("Channels info %X", lStructAdc.channels);
            for(unsigned int i = 0; i < 8; i++)
            {
                lParamsSetAdc.request.adcChannels[i] = (lStructAdc.channels & (1 << i));
                ROS_INFO("Channel: %i, Val: %u", i, lParamsSetAdc.request.adcChannels[i]);
            }

            lParamsSetAdc.request.adcFreqAcq = lStructAdc.freqAcquisition;
            lParamsSetAdc.request.adcFreqPoll = lStructAdc.freqSend;
            lParamsSetAdc.request.adcBufferSize = bufferSize;
            lParamsSetAdc.request.subscribe = true;
            if(!ros::service::call("/D_ADC/params", lParamsSetAdc))
            {
                ROS_ERROR(
                    "Le service de parametrage de l'ADC a renvoye une erreur (code %d).",
                    lParamsSetAdc.response.ret
                );
                in_client.lasterror = "The ADC subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetAdc.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }

        if(!(console = popen("rostopic list | grep /D_ADC/data | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si l'ADC est deja en fonction...");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node ADC not started!");
            in_client.lasterror = "The ADC topic is not present! Impossible to subscribe to /D_ADC/data";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }
        sub = nodeRos.subscribe("/D_ADC/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 2000, dataAdcReceived);
        break;

    case MSGID_IMU:
        if(!(console = popen("rosservice list | grep /D_IMU/params | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si le node IMU est lance!");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node IMU not started!");
            in_client.lasterror = "The IMU node is not started or crashed (no service at /D_IMU/params). Impossible to subscribe";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }

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
                in_client.lasterror = "The IMU silent subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetImu.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }
        else
        {
            memcpy(&lStructImu, info, sizeof(lStructImu));
            lParamsSetImu.request.imuFreqAcq = lStructImu.freqAcquisition;
            lParamsSetImu.request.imuFreqPoll = lStructImu.freqSend;
            lParamsSetImu.request.imuBufferSize = bufferSize;     // ?!?
            lParamsSetImu.request.subscribe = true;
            if(!ros::service::call("/D_IMU/params", lParamsSetImu))
            {
                ROS_ERROR(
                    "Le service de parametrage de l'IMU a renvoye une erreur (code %d).",
                    lParamsSetImu.response.ret
                );
                in_client.lasterror = "The IMU subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetImu.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }

        if(!(console = popen("rostopic list | grep /D_IMU/data | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si l'IMU est deja en fonction...");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node IMU not started!");
            in_client.lasterror = "The IMU topic is not present! Impossible to subscribe to /D_IMU/data";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }
        sub = nodeRos.subscribe("/D_IMU/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 250, dataImuReceived);
        break;

    case MSGID_GPS:
        if(!(console = popen("rostopic list | grep /extended_fix | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si le GPS est deja en fonction...");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node GPS not started!");
            in_client.lasterror = "The GPS node is not started! Impossible to subscribe to its topic.";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }
        sub = nodeRos.subscribe("/extended_fix", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataGpsReceived);
        break;

    case MSGID_HOKUYO:
        if(!(console = popen("rostopic list | grep /scan | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si le Hokuyo est deja en fonction...");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node Hokuyo not started!");
            in_client.lasterror = "The Hokuyo node is not started! Impossible to subscribe to its topic.";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }
        sub = nodeRos.subscribe("/scan", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataHokuyoReceived);
        break;

    case MSGID_WEBCAM:
        if(!(console = popen("rosservice list | grep /D_Cam/params | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si le node Camera est lance!");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node Camera not started!");
            in_client.lasterror = "The camera node is not started or crashed (no service at /D_Cam/params). Impossible to subscribe";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }
        if(subOnly)
        {
            lParamsSetCam.request.subscribe = true;
            lParamsSetCam.request.device = "";
            lParamsSetCam.request.width = 0;
            lParamsSetCam.request.height = 0;
            lParamsSetCam.request.fps = 0;
            lParamsSetCam.request.exposure = 0;
            ROS_INFO("Envoi de la requete au service");
            if(!ros::service::call("/D_Cam/params", lParamsSetCam))
            {
                ROS_ERROR(
                    "Le service de parametrage de la camera a renvoye une erreur (code %d).",
                    lParamsSetCam.response.ret
                );
                in_client.lasterror = "The silent Camera subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetCam.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }
        else
        {

            memcpy(&lStructCamera, info, sizeof(lStructCamera));
            lParamsSetCam.request.subscribe = true;
            tmpDevice << "/dev/video";
            tmpDevice << (unsigned int) (lStructCamera.id);

            lParamsSetCam.request.device = tmpDevice.str();
            lParamsSetCam.request.width = lStructCamera.width;
            lParamsSetCam.request.height = lStructCamera.height;
            lParamsSetCam.request.fps = lStructCamera.fps;
            lParamsSetCam.request.compressionRatio = lStructCamera.compression;
            lParamsSetCam.request.exposure = lStructCamera.exposure;
            ROS_INFO("Envoi de la requete au service");
            if(!ros::service::call("/D_Cam/params", lParamsSetCam))
            {
                ROS_ERROR(
                    "Le service de parametrage de la camera a renvoye une erreur (code %d).",
                    lParamsSetCam.response.ret
                );
                in_client.lasterror = "The Camera subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetCam.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }

        if(!(console = popen("rostopic list | grep /D_Cam/data | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si la camera est deja en fonction...");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node Camera not started!");
            in_client.lasterror = "The camera topic is not present! Impossible to subscribe to /D_Cam/data";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }

        sub = nodeRos.subscribe("/D_Cam/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 30, dataCamReceived);
        break;

    case MSGID_WEBCAM_STEREO:
        if(!(console = popen("rosservice list | grep /D_CamStereo/params | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si le node StereoCam est lance!");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node StereoCam not started!");
            in_client.lasterror = "The Stereo Camera node is not started or crashed (no service at /D_CamStereo/params). Impossible to subscribe";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }

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
                in_client.lasterror = "The silent StereoCamera subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetStereoCam.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }
        else
        {
            memcpy(&lStructStereoCam, info, sizeof(lStructStereoCam));
            lParamsSetStereoCam.request.subscribe = true;
            tmpDevice << "/dev/video";
            tmpDevice << (unsigned int) lStructStereoCam.idLeft;
            lParamsSetStereoCam.request.device_L = tmpDevice.str();

            tmpDevice.str("");
            tmpDevice << "/dev/video";
            tmpDevice << (unsigned int) lStructStereoCam.idRight;
            lParamsSetStereoCam.request.device_R = tmpDevice.str();
            lParamsSetStereoCam.request.width = lStructStereoCam.width;
            lParamsSetStereoCam.request.height = lStructStereoCam.height;
            lParamsSetStereoCam.request.fps = lStructStereoCam.fps;
            lParamsSetStereoCam.request.compressionRatio = lStructStereoCam.compression;
            lParamsSetStereoCam.request.exposure_L = lStructStereoCam.exposureLeft;
            lParamsSetStereoCam.request.exposure_R = lStructStereoCam.exposureRight;
            ROS_INFO("Envoi de la requete au service");
            if(!ros::service::call("/D_CamStereo/params", lParamsSetStereoCam))
            {
                ROS_ERROR(
                    "Le service de parametrage de la camera stereo a renvoye une erreur (code %d).",
                    lParamsSetStereoCam.response.ret
                );
                in_client.lasterror = "The StereoCamera subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetStereoCam.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }

        if(!(console = popen("rostopic list | grep /D_CamStereo/data | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si la camera est deja en fonction...");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node StereoCamera not started!");
            in_client.lasterror = "The Stereo Camera topic is not present! Impossible to subscribe to /D_CamStereo/data";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }


        sub = nodeRos.subscribe("/D_CamStereo/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 30, dataStereoCamReceived);
        break;

    case MSGID_KINECT:
        if(!(console = popen("rosservice list | grep /D_Kinect/params/params | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si le node Kinect est lance!");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node Kinect not started!");
            in_client.lasterror = "The Kinect node is not started or crashed (no service at /D_Kinect/params/params). Impossible to subscribe";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }

        if(subOnly)
        {
            lParamsSetKinect.request.subscribe = true;
            lParamsSetKinect.request.width_rgb = 0;
            lParamsSetKinect.request.height_rgb = 0;
            lParamsSetKinect.request.width_depth = 0;
            lParamsSetKinect.request.height_depth = 0;
            lParamsSetKinect.request.fps = 0;
            ROS_INFO("Envoi de la requete au service");
            if(!ros::service::call("/D_Kinect/params", lParamsSetKinect))
            {
                ROS_ERROR(
                    "Le service de parametrage de la kinect a renvoye une erreur (code %d).",
                    lParamsSetKinect.response.ret
                );
                in_client.lasterror = "The silent Kinect subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetKinect.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }
        else
        {
            memcpy(&lStructKinect, info, sizeof(lStructKinect));
            lParamsSetKinect.request.subscribe = true;
            lParamsSetKinect.request.width_rgb = lStructKinect.widthRGB;
            lParamsSetKinect.request.height_rgb = lStructKinect.heightRGB;
            lParamsSetKinect.request.width_depth = lStructKinect.widthRGB;
            lParamsSetKinect.request.height_depth = lStructKinect.heightRGB;
            lParamsSetKinect.request.fps = lStructKinect.fpsRGB;
            lParamsSetKinect.request.compressionRatio = lStructKinect.compressionRGB;
            ROS_INFO("Envoi de la requete au service");
            if(!ros::service::call("/D_Kinect/params", lParamsSetKinect))
            {
                ROS_ERROR(
                    "Le service de parametrage de la kinect a renvoye une erreur (code %d).",
                    lParamsSetKinect.response.ret
                );
                in_client.lasterror = "The Kinect subscription did not execute properly!\n";
                in_client.lasterror += lParamsSetKinect.response.errorDesc;
                in_client.lasterror_issued_by = "subscribeTo";
                return -1;
            }
        }
        if(!(console = popen("rostopic list | grep /D_Kinect/data | wc -l", "r"))){
            ROS_WARN("Impossible de verifier si la Kinect est deja en fonction...");
        }
        fgets(bufR, sizeof(bufR), console);
        if(bufR[0] == '0'){
            ROS_ERROR("Node Kinect not started!");
            in_client.lasterror = "The Kinect topic is not present! Impossible to subscribe to /D_Kinect/data";
            in_client.lasterror_issued_by = "subscribeTo";
            return -1;
        }

        sub = nodeRos.subscribe("/D_Kinect/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataKinectReceived);
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
            memcpy(&lStructComputer, info, sizeof(lStructComputer));
            lParamsSetComputer.request.subscribe = true;
            lParamsSetComputer.request.freqAcq = lStructComputer.freqAcquisition;
            lParamsSetComputer.request.bufferSize = bufferSize;
            if(!ros::service::call("/D_ComputerInfo/params", lParamsSetComputer))
            {
                ROS_ERROR(
                    "Le service de parametrage des statistiques de l'ordinateur a renvoye une erreur (code %d).",
                    lParamsSetComputer.response.ret
                );
                return -1;
            }
        }

        sub = nodeRos.subscribe("/D_ComputerInfo/data", 2 * SOCKET_SEND_TIMEOUT_SEC * 10, dataComputerReceived);
        break;

    default:
        ROS_ERROR("Demande d'inscription a un type de capteur inconnu (%X) sera ignoree!", typeCapteur);
        return 1;
    }

    in_client.subscribers[typeCapteur].first = sub;
    in_client.buffersInfo[typeCapteur] = bufferSize;

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
    ros4mat::S_Kinect       lParamsUnsetKinect;

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

    case MSGID_KINECT:
        lParamsUnsetKinect.request.subscribe = false;
        if(!ros::service::call("/D_Kinect/params", lParamsUnsetKinect))
        {
            ROS_ERROR(
                "Le service de parametrage de la Kinect a renvoye une erreur (code %d) lors de la deconnexion.",
                lParamsUnsetKinect.response.ret
            );
            return -1;
        }
        break;


    case MSGID_GPS:
    case MSGID_HOKUYO:
        break;

    default:
        ROS_ERROR("Type de capteur inconnu (unsubscribe %X)!", info->typeCapteur);
    }

    while(!in_client.subscribers[info->typeCapteur].second.empty())
    {
        if(info->typeCapteur == MSGID_WEBCAM){
            delete[]((msgCamInternal *) (in_client.subscribers[info->typeCapteur].second.front()))->cptr;
        }
        else if(info->typeCapteur == MSGID_HOKUYO){
            delete[]((msgHokuyoInternal *) (in_client.subscribers[info->typeCapteur].second.front()))->cptr;
        }
        else if(info->typeCapteur == MSGID_KINECT){
            delete[] ((msgKinectInternal *) in_client.subscribers[info->typeCapteur].second.front())->infoRGB.cptr;
            delete[] ((msgKinectInternal *) in_client.subscribers[info->typeCapteur].second.front())->infoDepth.cptr;
        }

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


int sendSerialCmd(msgSerialCmd *info, char* port, char* data, matlabClient &in_client, char **answer, uint32_t &answerSize)
{
    ros4mat::S_Serial   lParams;
    ROS_INFO(
        "Serial received on Logico Agent : speed = %d / parity = %d / stop = %d / sendLength = %d / readLength = %d / readTimeoutSec = %d / readTimeoutMicro = %d / namelength = %d",
        info->speed,
        info->parity,
        info->stopBits,
        info->sendLength,
        info->readLength,
        info->readTimeoutSec,
        info->readTimeoutMicro,
        info->portBufferLength
    );
    lParams.request.port = std::string(port);
    ROS_INFO("Serial port %s", port);
    lParams.request.speed = info->speed;
    lParams.request.parity = info->parity;
    lParams.request.stopBits = info->stopBits;
    lParams.request.sendBufferLength = info->sendLength;
    lParams.request.readBufferLength = info->readLength;
    lParams.request.readTimeoutSec = info->readTimeoutSec;
    lParams.request.readTimeoutMicro = info->readTimeoutMicro;
    lParams.request.closeAfterComm = info->closeAfterComm;
    for(unsigned int i = 0; i < info->sendLength; i++)                  // TODO : OPTIMIZE
        lParams.request.sendBuffer.push_back(data[i]);
    ROS_INFO("Serial OK, sending data");
    if(!ros::service::call("/D_Serial/serialCommand", lParams))
    {
        ROS_ERROR("L'envoi de la commande serie a renvoye une erreur");
        return MSGID_SERIAL_ANS_NO_OPEN;
    }
    *answer = new char[lParams.response.receiveBufferLength];
    for(unsigned int i = 0; i < lParams.response.receiveBufferLength; i++)  // TODO : OPTIMIZE
        (*answer)[i] = lParams.response.receiveBuffer[i];
    answerSize = lParams.response.receiveBufferLength;

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

int sendError(int socketFd, char reqType, char errCode, std::string errorStr){
    msgHeader lErrorHeader;
    lErrorHeader.type = reqType;
    lErrorHeader.size = 0;
    lErrorHeader.error = errCode;
    lErrorHeader.compressSize = errorStr.length()+1;
    lErrorHeader.uncompressSize = errorStr.length()+1;
    lErrorHeader.compressionType = 0;
    lErrorHeader.packetTimestamp = ros::Time::now().toSec();

    char *buf = new char[sizeof(msgHeader) + errorStr.length() + 1];    // We include a trailing \0

    memcpy(buf, &lErrorHeader, sizeof(msgHeader));
    memcpy(buf + sizeof(msgHeader), errorStr.c_str(), errorStr.length() + 1);

    int sendBytes = send(socketFd, buf, sizeof(msgHeader) + errorStr.length() + 1, 0);

    delete[] buf;
    return sendBytes;
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
                    recvReturn = recv(i, msg, sizeof(msgHeader), MSG_WAITALL);
                    if(recvReturn == -1)
                    {
                        // Timeout
                        ROS_ERROR(
                            "Timeout on header reception (Received %db / %db expected)",
                            recvBytes,
                            sizeof(msgHeader)
                        );
                        continue;
                    }
                    else if(recvReturn == 0)
                    {
                        ROS_ERROR("Connection closed by the client on header reception");
                       
                        // On ne peut plus faire ca
                        //unsubscribeAll(i);
                        // Node unsubscribtion
                        //clients.erase(i);
                        
                        close(i);
                        FD_CLR(i, &active_fd_set);
                        delete[] msg;
                        continue;
                    }
                    else if(recvReturn < sizeof(msgHeader))
                    {
                        if(n == 0)
                        {
                            ROS_INFO("Closed connexion");
                            // On ne peut plus faire ca
                            //unsubscribeAll(i);
                            //clients.erase(i);
                            close(i);
                            FD_CLR(i, &active_fd_set);
                        }

                        // If it's a corrupted message or a new connection
                        delete[] msg;
                        continue;
                    }

                    std::map<unsigned int, std::pair<ros::Subscriber, std::queue<void *> > >::iterator  lIt;
                    msgUnsubscribe                                                                      lUM;
                    
                    msgHeader lHeader;
                    memcpy(&lHeader, msg, sizeof(msgHeader));
                    delete[] msg;


                    if(ProtocolMsgSize.count(lHeader.type) == 0)
                    {
                        ROS_WARN("Received an unknown message type: %X. Ignoring...", lHeader.type);
                        continue;
                    }

                    if(lHeader.type != MSGID_CONNECT && clients.count(lHeader.clientId) == 0){
                        ROS_WARN("Received an invalid ID from client, and the client is not requesting for connection. Ignoring...");
                        continue;
                    }


                    // Message reception
                    msg = new char[lHeader.uncompressSize];
                    if(lHeader.size > 0){
                        recvReturn = recv(i, msg, lHeader.uncompressSize, MSG_WAITALL);
                        if(recvReturn == -1)
                        {
                            // Timeout
                            ROS_ERROR(
                                "Timeout on message reception (header correctly received) : %db received, %db expected!",
                                recvReturn,
                                lHeader.uncompressSize
                            );
                            delete[] msg;
                            continue;
                        }
                        else if(recvReturn == 0)
                        {
                            ROS_ERROR("Connection closed by client");
                            unsubscribeAll(lHeader.clientId);

                            // Node unsubscribtion
                            clients.erase(lHeader.clientId);
                            close(i);
                            FD_CLR(i, &active_fd_set);
                            delete[] msg;
                            continue;
                        }
                    }


                    // Header parsing and data handling

                    n = recvBytes;

                    msgHeader       lAnswerHeader;
                    msgConnect      lConnect;
                    msgSubscribe    lSubscribe;
                    msgComputer     lComputer;
                    msgUnsubscribe  lUnsubscribe;
                    msgDigitalOut   lDigitalCmd;
                    msgSerialCmd    lSerialCmd;
                    msgSerialAns    lSerialAns;

                    memset(&lAnswerHeader, 0, sizeof(msgHeader));
                    memset(&lSerialAns, 0, sizeof(msgSerialAns));

                    char            *lRetour;
                    uint32_t        lRetourSize = 0;
                    char            *bufferSubscribe, *bufferData, *bufferPort;

                    if(lHeader.type & 0xF0 && clients[lHeader.clientId].subscribers[lHeader.type].second.size() == 0)
                    {
                        // No data to send
                        ROS_INFO("No data to send");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = 0;
                        lAnswerHeader.error = 0;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                        lAnswer = new char[sizeof(msgHeader)];
                        memcpy(lAnswer, &lAnswerHeader, sizeof(msgHeader));
                        send(i, lAnswer, sizeof(msgHeader), 0);
                        delete[] msg;
                        continue;
                    }

                    switch(lHeader.type)
                    {
                    case MSGID_CONNECT:
                        ROS_INFO("Received Connect");
                        memcpy(&lConnect, msg, sizeof(msgConnect));
                        lAnswer = new char[sizeof(msgHeader)];
                        lAnswerHeader.type = MSGID_CONNECT_ACK;
                        lAnswerHeader.error = 0;
                        lAnswerHeader.size = 0;
                        lAnswerHeader.packetTimestamp = 0.0;

                        if(lHeader.clientId == 0){
                            // Le client demande un ID (premiere connexion)
                            if(clients.empty()){                               
                                lAnswerHeader.clientId = 1;
                            }
                            else{
                                std::map<uint32_t,matlabClient>::reverse_iterator rend;
                                rend = clients.rbegin();
                                lAnswerHeader.clientId = rend->first + 1;
                            }                           
                            clients[lAnswerHeader.clientId] = matlabClient();
                        }
                        else{
                            if(clients.count(lHeader.clientId) == 0){
                                ROS_WARN("Client said he already has an ID, but this ID do not exist in memory!");
                                clients[lAnswerHeader.clientId] = matlabClient();
                                // Envoyer message d'erreur au client?
                            }
                            lAnswerHeader.clientId = lHeader.clientId;    // On fait "confiance" au client
                        }

                        memcpy(lAnswer, &lAnswerHeader, sizeof(msgHeader));
                        send(i, lAnswer, sizeof(msgHeader), 0);
                            
                        clients[lAnswerHeader.clientId].compression = lConnect.compression;
                        delete[] lAnswer;
                        break;

                    case MSGID_QUIT:
                        ROS_DEBUG("Received Quit");
                        unsubscribeAll(lHeader.clientId);

                        // Node unsubscribtion
                        clients.erase(lHeader.clientId);
                        close(i);
                        FD_CLR(i, &active_fd_set);
                        break;

                    case MSGID_SUBSCRIBE:
                        memcpy(&lSubscribe, msg, sizeof(msgSubscribe));
                        ROS_DEBUG("Received Subscribe for %X", lSubscribe.typeCapteur);

                        if(lSubscribe.paramsSize == 0){
                            if(lSubscribe.silentSubscribe != 0){
                                subscribeTo(lSubscribe.typeCapteur, lSubscribe.bufferSize, 0, true, nodeRos, clients[lHeader.clientId]);
                            }
                            else{
                                clients[lHeader.clientId].lasterror = "Subscribe received without any parameters struct. Sensor will not be subscribed.";
                                ROS_WARN("Subscribe received a paramsSize of 0. Invalid!");
                            }
                        }
                        else{
                            bufferSubscribe = new char[lSubscribe.paramsSize];
                            memcpy(bufferSubscribe, msg+sizeof(msgSubscribe), lSubscribe.paramsSize);
                            subscribeTo(lSubscribe.typeCapteur, lSubscribe.bufferSize, bufferSubscribe, lSubscribe.silentSubscribe == 1, nodeRos, clients[lHeader.clientId]);
                            delete[] bufferSubscribe;
                        }                        

                        if(clients[lHeader.clientId].lasterror != ""){
                            sendError(i, MSGID_SUBSCRIBE_ACK, 1, clients[lHeader.clientId].lasterror);
                            clients[lHeader.clientId].lasterror = "";
                            clients[lHeader.clientId].lasterror_issued_by = "";
                        }
                        else{
                            lAnswer = new char[sizeof(msgHeader)];
                            lAnswerHeader.type = MSGID_SUBSCRIBE_ACK;
                            lAnswerHeader.error = 0;
                            lAnswerHeader.size = 0;
                            lAnswerHeader.compressSize = 0;
                            lAnswerHeader.uncompressSize = 0;
                            lAnswerHeader.clientId = lHeader.clientId; 
                            lAnswerHeader.compressionType = MSGID_HEADER_NOCOMPRESSION;
                            lAnswerHeader.packetTimestamp = 0.0;
                            memcpy(lAnswer, &lAnswerHeader, sizeof(msgHeader));
                            send(i, lAnswer, sizeof(msgHeader), 0);
                            delete[] lAnswer;
                        }

                        break;

                    case MSGID_UNSUBSCRIBE:
                        ROS_DEBUG("Received Unsubscribe");
                        memcpy(&lUnsubscribe, msg, sizeof(msgUnsubscribe));
                        unsubscribeTo(&lUnsubscribe, clients[lHeader.clientId]);
                        break;

                    case MSGID_SERIAL_CMD:
                        ROS_DEBUG("Received serial command");
                        memcpy(&lSerialCmd, msg, sizeof(msgSerialCmd));

                        bufferPort = new char[lSerialCmd.portBufferLength+1];
                        memcpy(bufferPort, msg + sizeof(msgSerialCmd), lSerialCmd.portBufferLength);
                        bufferPort[lSerialCmd.portBufferLength] = 0;        // To terminate string

                        if(lSerialCmd.sendLength > 0){
                            bufferData = new char[lSerialCmd.sendLength];
                            memcpy(bufferData, msg + sizeof(msgSerialCmd) + lSerialCmd.portBufferLength, lSerialCmd.sendLength);
                        }
                        if(lSerialCmd.readLength > 0){
                            lRetour = new char[lSerialCmd.readLength];
                        }

                        // int sendSerialCmd(msgSerialCmd *info, char* port, char* data, matlabClient &in_client, char **answer, char &answerSize)
                        lSerialAns.status = sendSerialCmd(&lSerialCmd, bufferPort, bufferData, clients[lHeader.clientId],
                                                            &lRetour, lRetourSize);

                        lAnswerHeader.type = MSGID_SERIAL_ANS;
                        lAnswerHeader.error = 0;
                        lAnswerHeader.size = 1;
                        lAnswerHeader.packetTimestamp = 0.0;
                        lAnswerHeader.clientId = lHeader.clientId; 
                        lAnswerHeader.compressSize = sizeof(msgSerialAns) + lRetourSize;
                        lAnswerHeader.uncompressSize = sizeof(msgSerialAns) + lRetourSize;
                        lSerialAns.dataLength = lRetourSize;

                        lAnswer = new char[sizeof(msgSerialAns) + lRetourSize];
                        memcpy(lAnswer, &lSerialAns, sizeof(msgSerialAns));
                        memcpy(lAnswer + sizeof(msgSerialAns), lRetour, lRetourSize);

                        sendDataToClient(i, &lAnswerHeader, lAnswer, sizeof(msgSerialAns) + lRetourSize, clients[lHeader.clientId].compression);

                        delete[] bufferPort;
                        if(lSerialCmd.sendLength > 0)
                            delete[] bufferData;
                        if(lSerialCmd.readLength > 0)
                            delete[] lRetour;

                        delete[] lAnswer;
                        break;

                    case MSGID_DOUT_CTRL:
                        ROS_DEBUG("Received Digital Out Control");
                        memcpy(&lDigitalCmd, msg, sizeof(msgDigitalOut));
                        sendDigitalOutCmd(&lDigitalCmd);
                        break;

                    case MSGID_WEBCAM:
                        {
                            lAnswerHeader.type = MSGID_WEBCAM;
                            lAnswerHeader.error = 0;
                            lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                            lAnswerHeader.clientId = lHeader.clientId; 

                            nbrImgSend = clients[lHeader.clientId].subscribers[MSGID_WEBCAM].second.size();
                            lAnswerHeader.size = nbrImgSend;

                            // malloc because we use realloc subsequently
                            lRetourSize = nbrImgSend * sizeof(msgCam);
                            lAnswer = (char*)malloc(lRetourSize);
                            
                            msgCam          lCamData;
                            msgCamInternal  *currentCamStruct = 0;
                            for(unsigned int k = 0; k < lAnswerHeader.size; k++){
                                // msgCam struct copy at the packet beginning
                                currentCamStruct = (msgCamInternal *) clients[lHeader.clientId].subscribers[MSGID_WEBCAM].second.front();
                                lCamData.width = currentCamStruct->width;
                                lCamData.height = currentCamStruct->height;
                                lCamData.channels = currentCamStruct->channels;
                                lCamData.sizeData = currentCamStruct->sizeData;
                                lCamData.compressionType = currentCamStruct->compressionType;
                                lCamData.timestamp = currentCamStruct->timestamp;

                                // Image subheader copy
                                memcpy(lAnswer + k * sizeof(msgCam), &lCamData, sizeof(msgCam));

                                // On realloc pour chaque image car elles peuvent etre de taille differente
                                lRetourSize += currentCamStruct->sizeData;
                                lAnswer = (char*)realloc(lAnswer, lRetourSize);

                                memcpy(lAnswer + lRetourSize - currentCamStruct->sizeData, currentCamStruct->cptr, currentCamStruct->sizeData);

                                 // Buffer pruning
                                delete[] currentCamStruct->cptr;
                                delete currentCamStruct;
                                clients[lHeader.clientId].subscribers[lHeader.type].second.pop();
                                }
                        
                            ROS_INFO("Send webcam data: %X (size = %i)", lAnswerHeader.type, lRetourSize);
                            sendDataToClient(i, &lAnswerHeader, lAnswer, lRetourSize, clients[lHeader.clientId].compression);

                            free(lAnswer);
                        }
                        break;

                    case MSGID_WEBCAM_STEREO:
                        {
                            lAnswerHeader.type = MSGID_WEBCAM_STEREO;
                            lAnswerHeader.error = 0;
                            lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                            lAnswerHeader.clientId = lHeader.clientId;

                            nbrImgSend = clients[lHeader.clientId].subscribers[MSGID_WEBCAM_STEREO].second.size();
                            lAnswerHeader.size = nbrImgSend;

                            // malloc because we use realloc subsequently
                            lRetourSize = nbrImgSend * sizeof(msgCam) * 2;
                            lAnswer = (char*)malloc(lRetourSize);

                            msgCam                  lCamData;
                            msgStereoCamInternal    *currentCamStruct = 0;
                            std::ofstream                outTestJpg1, outTestJpg2;
                            for(unsigned int k = 0; k < lAnswerHeader.size; k += 1)
                            {
                                // msgCam struct copy at the packet beginning
                                currentCamStruct = (msgStereoCamInternal *) clients[lHeader.clientId].subscribers[MSGID_WEBCAM_STEREO].second.front();
                                lCamData.width = currentCamStruct->width_L;
                                lCamData.height = currentCamStruct->height_L;
                                lCamData.channels = currentCamStruct->channels;
                                lCamData.sizeData = currentCamStruct->sizeData_L;
                                lCamData.compressionType = currentCamStruct->compressionType;
                                lCamData.timestamp = currentCamStruct->timestamp;

                                // Header copy for left image
                                memcpy(lAnswer + (k*2 + 0) * sizeof(msgCam), &lCamData, sizeof(msgCam));

                                // Header copy for right image
                                lCamData.width = currentCamStruct->width_R;
                                lCamData.height = currentCamStruct->height_R;
                                lCamData.sizeData = currentCamStruct->sizeData_R;
                                memcpy(lAnswer + (k*2 + 1) * sizeof(msgCam), &lCamData, sizeof(msgCam));

                                // On realloc pour chaque image car elles peuvent etre de taille differente
                                lRetourSize += currentCamStruct->sizeData_L + currentCamStruct->sizeData_R;
                                lAnswer = (char*)realloc(lAnswer, lRetourSize);

                                // Left Image copy
                                memcpy(
                                    lAnswer + lRetourSize - (currentCamStruct->sizeData_L + currentCamStruct->sizeData_R),
                                    currentCamStruct->cptr_L,
                                    currentCamStruct->sizeData_L
                                );

                                // Right Image copy
                                memcpy(
                                    lAnswer + lRetourSize - currentCamStruct->sizeData_R,
                                    currentCamStruct->cptr_R,
                                    currentCamStruct->sizeData_R
                                );

                                if(k == 0){
                                    ROS_INFO("Writing JPG to files");
                                    outTestJpg1.open ("testgauche.jpg");
                                    outTestJpg1.write(currentCamStruct->cptr_L, currentCamStruct->sizeData_L);
                                    outTestJpg1.close();
                                    outTestJpg2.open ("testdroite.jpg");
                                    outTestJpg2.write(currentCamStruct->cptr_R, currentCamStruct->sizeData_R);
                                    outTestJpg2.close();
                                }

                                // Buffer pruning
                                delete[] currentCamStruct->cptr_L;
                                delete[] currentCamStruct->cptr_R;
                                delete currentCamStruct;
                                clients[lHeader.clientId].subscribers[MSGID_WEBCAM_STEREO].second.pop();
                            }

                            ROS_INFO("Send stereo webcam data: %X (size = %i)", lAnswerHeader.type, lRetourSize);
                            sendDataToClient(i, &lAnswerHeader, lAnswer, lRetourSize, clients[lHeader.clientId].compression);

                            free(lAnswer);
                        }
                        break;

                    case MSGID_KINECT:
                        {
                            lAnswerHeader.type = MSGID_KINECT;lAnswerHeader.error = 0;
                            lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                            lAnswerHeader.clientId = lHeader.clientId; 

                            nbrImgSend = clients[lHeader.clientId].subscribers[MSGID_KINECT].second.size();
                            lAnswerHeader.size = nbrImgSend;

                            // malloc because we use realloc subsequently
                            lRetourSize = nbrImgSend * sizeof(msgKinect);
                            lAnswer = (char*)malloc(lRetourSize);

                            msgKinectInternal *currentKinectStruct = 0;
                            msgKinect lKinectData;
                            for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                            {
                                currentKinectStruct = (msgKinectInternal *) clients[lHeader.clientId].subscribers[MSGID_KINECT].second.front();
                                lKinectData.infoRGB.width = currentKinectStruct->infoRGB.width;
                                lKinectData.infoRGB.height = currentKinectStruct->infoRGB.height;
                                lKinectData.infoRGB.channels = currentKinectStruct->infoRGB.channels;
                                lKinectData.infoRGB.sizeData = currentKinectStruct->infoRGB.sizeData;
                                lKinectData.infoRGB.compressionType = currentKinectStruct->infoRGB.compressionType;
                                lKinectData.infoRGB.timestamp = currentKinectStruct->infoRGB.timestamp;

                                lKinectData.infoDepth.width = currentKinectStruct->infoDepth.width;
                                lKinectData.infoDepth.height = currentKinectStruct->infoDepth.height;
                                lKinectData.infoDepth.channels = currentKinectStruct->infoDepth.channels;
                                lKinectData.infoDepth.sizeData = currentKinectStruct->infoDepth.sizeData;
                                lKinectData.infoDepth.compressionType = currentKinectStruct->infoDepth.compressionType;
                                lKinectData.infoDepth.timestamp = currentKinectStruct->infoDepth.timestamp;

                                // Image subheader copy
                                memcpy(lAnswer + k * sizeof(msgKinect), &lKinectData, sizeof(msgKinect));

                                // On realloc pour chaque image car elles peuvent etre de taille differente
                                lRetourSize += currentKinectStruct->infoRGB.sizeData + currentKinectStruct->infoDepth.sizeData;
                                lAnswer = (char*)realloc(lAnswer, lRetourSize);

                                // RGB image copy
                                if(currentKinectStruct->infoRGB.sizeData > 0){
                                    memcpy(
                                        lAnswer + lRetourSize - (currentKinectStruct->infoRGB.sizeData + currentKinectStruct->infoDepth.sizeData),
                                        currentKinectStruct->infoRGB.cptr,
                                        currentKinectStruct->infoRGB.sizeData
                                    );
                                    delete[] currentKinectStruct->infoRGB.cptr;
                                }

                                // Depth image copy
                                if(currentKinectStruct->infoDepth.sizeData > 0){
                                    memcpy(
                                        lAnswer + lRetourSize - currentKinectStruct->infoDepth.sizeData,
                                        currentKinectStruct->infoDepth.cptr,
                                        currentKinectStruct->infoDepth.sizeData
                                    );
                                    delete[] currentKinectStruct->infoDepth.cptr;
                                }

                                delete currentKinectStruct;
                                clients[lHeader.clientId].subscribers[MSGID_KINECT].second.pop();
                            }

                            ROS_INFO("Send kinect data: %X (size = %i)", lAnswerHeader.type, lRetourSize);
                            sendDataToClient(i, &lAnswerHeader, lAnswer, lRetourSize, clients[lHeader.clientId].compression);

                            free(lAnswer);
                        }
                        break;

                    case MSGID_HOKUYO:
                        {
                            lAnswerHeader.type = MSGID_HOKUYO;
                            lAnswerHeader.error = 0;
                            lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                            lAnswerHeader.clientId = lHeader.clientId; 
                            
                            /* Sending protocol: We send an msgHeader containing the number of depth scans 
                                 * contained in the size variable.
                                 * Then, we send EVERY rangeStruct in one shot, then the ranges having the 
                                 * same size */
                            nbrCaptureSend = clients[lHeader.clientId].subscribers[MSGID_HOKUYO].second.size();

                            int rangesSizeBytes =
                                ((msgHokuyoInternal *) clients[lHeader.clientId].subscribers[MSGID_HOKUYO].second.front())->sizeData;

                            int sendSize = nbrCaptureSend * sizeof(msgHokuyo) + nbrCaptureSend * rangesSizeBytes;

                            lAnswerHeader.size = nbrCaptureSend;

                            lAnswer = new char[sendSize];

                            msgHokuyo           lHokuyoData;
                            msgHokuyoInternal   *currentStruct = 0;
                            for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                            {
                                // msgCam struct copy at the packet beginning
                                currentStruct = (msgHokuyoInternal *) clients[lHeader.clientId].subscribers[MSGID_HOKUYO].second.front();
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
                                clients[lHeader.clientId].subscribers[MSGID_HOKUYO].second.pop();
                            }

                            ROS_INFO("Send hokuyo data: %X (size = %i)", lAnswerHeader.type, sendSize);
                            sendDataToClient(i, &lAnswerHeader, lAnswer, sendSize, clients[lHeader.clientId].compression);
                            

                            delete[] lAnswer;
                        }
                        break;

                    // TODO: Unify all these together

                    case MSGID_GPS:
                        ROS_DEBUG("Received GPS");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = clients[lHeader.clientId].subscribers[lHeader.type].second.size();
                        lAnswerHeader.error = 0;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                        lAnswerHeader.clientId = lHeader.clientId; 

                        lAnswer = new char[lAnswerHeader.size * sizeof(msgGps)];
                        for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                        {
                            memcpy(
                                lAnswer + k * sizeof(msgGps),
                                clients[lHeader.clientId].subscribers[lHeader.type].second.front(),
                                sizeof(msgGps)
                            );
                            delete (msgGps *) clients[lHeader.clientId].subscribers[lHeader.type].second.front();
                            clients[lHeader.clientId].subscribers[lHeader.type].second.pop();
                        }

                        sendDataToClient(
                            i,
                            &lAnswerHeader,
                            lAnswer,
                            lAnswerHeader.size * sizeof(msgGps),
                            clients[lHeader.clientId].compression
                        );
                        delete[] lAnswer;
                        break;

                    case MSGID_COMPUTER:
                        ROS_DEBUG("Received Computer");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = clients[lHeader.clientId].subscribers[lHeader.type].second.size();
                        lAnswerHeader.error = 0;
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                        lAnswerHeader.clientId = lHeader.clientId; 

                        lAnswer = new char[lAnswerHeader.size * sizeof(msgComputer)];
                        for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                        {
                            memcpy(
                                lAnswer + k * sizeof(msgComputer),
                                clients[lHeader.clientId].subscribers[lHeader.type].second.front(),
                                sizeof(msgComputer)
                            );
                            delete (msgComputer *) clients[lHeader.clientId].subscribers[lHeader.type].second.front();
                            clients[lHeader.clientId].subscribers[lHeader.type].second.pop();
                        }

                        sendDataToClient(
                            i,
                            &lAnswerHeader,
                            lAnswer,
                            lAnswerHeader.size * sizeof(msgComputer),
                            clients[lHeader.clientId].compression
                        );

                        delete[] lAnswer;
                        break;

                    case MSGID_ADC:
                        ROS_DEBUG("Received ADC");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = clients[lHeader.clientId].subscribers[lHeader.type].second.size();
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                        lAnswerHeader.error = 0;
                        lAnswerHeader.clientId = lHeader.clientId; 

                        lAnswer = new char[lAnswerHeader.size * sizeof(msgAdc)];
                        for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                        {
                            memcpy(
                                lAnswer + k * sizeof(msgAdc),
                                clients[lHeader.clientId].subscribers[lHeader.type].second.front(),
                                sizeof(msgAdc)
                            );
                            delete (msgAdc *) clients[lHeader.clientId].subscribers[lHeader.type].second.front();
                            clients[lHeader.clientId].subscribers[lHeader.type].second.pop();
                        }

                        sendDataToClient(
                            i,
                            &lAnswerHeader,
                            lAnswer,
                            lAnswerHeader.size * sizeof(msgAdc),
                            clients[lHeader.clientId].compression
                        );
                        delete[] lAnswer;
                        break;

                    case MSGID_IMU:
                        ROS_DEBUG("Received IMU");
                        lAnswerHeader.type = lHeader.type;
                        lAnswerHeader.size = clients[lHeader.clientId].subscribers[lHeader.type].second.size();
                        lAnswerHeader.packetTimestamp = ros::Time::now().toSec();
                        lAnswerHeader.error = 0;
                        lAnswerHeader.clientId = lHeader.clientId; 

                        lAnswer = new char[lAnswerHeader.size * sizeof(msgImu)];
                        for(unsigned int k = 0; k < lAnswerHeader.size; k++)
                        {
                            memcpy(
                                lAnswer + k * sizeof(msgImu),
                                clients[lHeader.clientId].subscribers[lHeader.type].second.front(),
                                sizeof(msgImu)
                            );
                            delete (msgImu *) clients[lHeader.clientId].subscribers[lHeader.type].second.front();
                            clients[lHeader.clientId].subscribers[lHeader.type].second.pop();
                        }

                        sendDataToClient(
                            i,
                            &lAnswerHeader,
                            lAnswer,
                            lAnswerHeader.size * sizeof(msgImu),
                            clients[lHeader.clientId].compression
                        );
                        delete[] lAnswer;
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
