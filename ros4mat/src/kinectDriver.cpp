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
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string>
#include <sstream>
#include <cmath>
#include <ros4mat/S_Kinect.h>
#include <ros4mat/M_Kinect.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include "jpge.h"       // Compression JPG. On ne peut pas utiliser celle de image_transport pour certaines raisons


bool initialised = false;
bool running = false;

ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()
ros::NodeHandle* n = 0;
ros::Publisher kinectPublisher;
int nbrKinectSubscribers = 0;
unsigned int dataId = 0;
unsigned char compression = 0;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;	

void dataKinectSync(const sensor_msgs::Image::ConstPtr& imageRGB, const sensor_msgs::Image::ConstPtr& imageDepth){
	ros4mat::M_Kinect msg;
	msg.header.seq = dataId++;
	msg.header.stamp = ros::Time::now();
	msg.width_rgb = imageRGB->width;
	msg.width_depth = imageDepth->width;
	msg.height_rgb = imageRGB->height;
	msg.height_depth = imageDepth->height;
	msg.channels = 3;

	msg.timestamp = imageDepth->header.stamp.toSec();


	msg.compressionRatio = compression;

	if(compression == 0){	// No compression
		msg.rgb = imageRGB->data;
	}
	else{
		// JPEG compression
		unsigned char *dataImg = new unsigned char[msg.width_rgb*msg.height_rgb*msg.channels];
		for(unsigned int i=0; i < imageRGB->data.size(); i++)
			dataImg[i] = imageRGB->data[i];

		unsigned char *bufjpeg = new unsigned char[msg.width_rgb*msg.height_rgb*msg.channels];
		int outsize = msg.width_rgb*msg.height_rgb*msg.channels;
		struct jpge::params paramsCompression = jpge::params();
		paramsCompression.m_quality = (int)compression;

		bool ok = jpge::compress_image_to_jpeg_file_in_memory(bufjpeg, outsize, msg.width_rgb, msg.height_rgb, msg.channels, dataImg, paramsCompression);

		for(unsigned int i=0; i < outsize; i++)
			msg.rgb.push_back(bufjpeg[i]);
		delete[] bufjpeg;
		delete[] dataImg;
	}

	msg.depth = imageDepth->data;

	kinectPublisher.publish(msg);
}

bool newConfKinect(ros4mat::S_Kinect::Request& request, ros4mat::S_Kinect::Response& response){
	int ret = 0;
	FILE *console;
	char bufR[10];
	std::stringstream ssRequest;

	if(request.subscribe){
		ROS_INFO("Configuration de la Kinect...");
		nbrKinectSubscribers++;
		if(request.fps == 0){
            ROS_INFO("Configuration kinect reussie en mode inscription seulement, demarrage...");
            response.ret = 0;
            return true;
        }

		if(nbrKinectSubscribers == 1){
			ROS_INFO("Pas de publisher disponible, creation de celui-ci...");
			kinectPublisher = n->advertise<ros4mat::M_Kinect>("D_Kinect/data", request.kinectBufferSize);
		}

		
		*loop_rate = ros::Rate((double)request.fps);
		running = true;

		compression = request.compressionRatio;


		
		ROS_INFO("Configuration de la kinect terminee avec succes...");
		response.ret = ret;
		return true;
	}
	else{
		ROS_INFO("Deconnexion du subscriber (kinect)...");
		if(--nbrKinectSubscribers <= 0){
			// Il ne reste plus qu'un seul subscriber, donc on peut supprimer sans risque
			running = false;
			
			nbrKinectSubscribers = 0;
		}
		response.ret = 0;
		return true;
	}
}



int main(int argc, char* argv[])
{
	// Initialisation
    ros::init(argc, argv, "D_Kinect");
    ros::NodeHandle l_n;
	n = &l_n;
	ROS_INFO("Service kinect synchronizer demarre...");

	ros::ServiceServer service = n->advertiseService("D_Kinect/params", newConfKinect);

    ros::Rate l_loop_rate = ros::Rate(30.);
	// Ok, c'est epouvantable, mais je vois pas comment faire autrement
	loop_rate = &l_loop_rate;

        /* Connecting to a maybe-not-already-published topic. This hack is to handle the sync()
        variable that seems to fall out of scope in the external function. */
		message_filters::Subscriber<sensor_msgs::Image> imageRGB_sub(*n, "/rgb/image_raw", 10);
        message_filters::Subscriber<sensor_msgs::Image> imageDepth_sub(*n, "/depth_registered/image_raw", 10);

        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)8
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageRGB_sub, imageDepth_sub);

        sync.registerCallback(boost::bind(dataKinectSync, _1, _2));
	
	
    initialised = true;
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate->sleep();
	}
    return 0;
}


