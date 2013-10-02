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
 #include <unistd.h>
#include <sys/ioctl.h>
#include <string>
#include <sstream>
#include <cmath>
#include <ros4mat/M_Cam.h>
#include <ros4mat/S_Cam.h>
#include <sensor_msgs/Image.h>

#include "jpge.h"       // Compression JPG. On ne peut pas utiliser celle de image_transport pour certaines raisons


bool initialised = false;
bool running = false;

ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()
ros::NodeHandle* n = 0;
ros::Publisher camPublisher;
int nbrSubscribers = 0;
unsigned int dataId = 0;
unsigned char compression = 0;


void dataCamSync(const sensor_msgs::Image::ConstPtr& image){
	ros4mat::M_Cam msg;
	msg.header.seq = dataId++;
	msg.header.stamp = ros::Time::now();
	msg.width = image->width;
	msg.height = image->height;
	msg.channels = 3;
	msg.timestamp = image->header.stamp.toSec();

	//ROS_INFO("Encoding de l'image : %s", image->encoding.c_str());


	msg.compressionRatio = compression;

	if(compression == 0){	// No compression
		msg.image = image->data;
	}
	else{
		// JPEG compression
		unsigned char *dataImg = new unsigned char[msg.width*msg.height*msg.channels];
		for(unsigned int i=0; i < image->data.size(); i++)
			dataImg[i] = image->data[i];

		unsigned char *bufjpeg = new unsigned char[msg.width*msg.height*msg.channels];
		int outsize = msg.width*msg.height*msg.channels;
		struct jpge::params paramsCompression = jpge::params();
		paramsCompression.m_quality = (int)compression;

		bool ok = jpge::compress_image_to_jpeg_file_in_memory(bufjpeg, outsize, msg.width, msg.height, msg.channels, dataImg, paramsCompression);

		for(unsigned int i=0; i < outsize; i++)
			msg.image.push_back(bufjpeg[i]);
		delete[] bufjpeg;
		delete[] dataImg;
	}

	camPublisher.publish(msg);
}

bool newConfReceived(ros4mat::S_Cam::Request& request, ros4mat::S_Cam::Response& response){
	int ret = 0;
	FILE *console;
	char bufR[10];
	std::stringstream ssRequest;

	
	if(request.subscribe){
		ROS_INFO("Configuration de la camera...");
		// TODO protection des frequences
		
		nbrSubscribers++;
        if(request.fps == 0){
            ROS_INFO("Configuration reussie en mode inscription seulement, demarrage...");
            response.ret = 0;
            return true;
        }

        // Check if /dev/video* is available
        ssRequest << "ls /dev | grep video" << request.device << " | wc -l";
        if(!(console = popen(ssRequest.str().c_str(), "r"))){
			ROS_WARN("Impossible de verifier si le device existe!");
		}
		
		fgets(bufR, sizeof(bufR), console);
		if(bufR[0] == '0'){
            response.ret = -1;
            ssRequest.str("");
            ssRequest << "Device /dev/video" << request.device << " is not present! Check USB connection and camera ID.";
            ROS_INFO(ssRequest.str().c_str());
            response.errorDesc = ssRequest.str();
            return false;
		}

		
		if(!(console = popen("rosnode list | grep /uvc_camera | wc -l", "r"))){
				ROS_WARN("Impossible de verifier si la camera est deja en fonction...");
			}
		
		fgets(bufR, sizeof(bufR), console);
		if(bufR[0] == '1'){
            ROS_INFO("Kill node uvc_camera");
			ret = system("rosnode kill /uvc_camera && sleep 0.5");
		}

		if(nbrSubscribers == 1){
			ROS_INFO("Pas de publisher disponible, creation de celui-ci...");
			camPublisher = n->advertise<ros4mat::M_Cam>("D_Cam/data", request.camBufferSize);
		}

		ssRequest.str("");
		ssRequest << "rosrun uvc_camera camera_node _width:=" << request.width << " _height:=" << request.height << " _device:=" << request.device << " _fps:=" << (unsigned int)request.fps << " &";
		ROS_INFO("Lancement de la commande : %s", ssRequest.str().c_str());
		ret = system(ssRequest.str().c_str());
		ROS_INFO("Fin de la commande (status %i)", ret);
		
		ssRequest.str("");
		ssRequest << "uvcdynctrl --device=" << request.device.substr(5) << " -s \"Exposure (Absolute)\"" << request.exposure;
		sleep(4);		// On attend que la camera soit configuree
		ret = system(ssRequest.str().c_str()); // Bon ok c'est un peu arbitraire. Plus le nombre est petit, plus c'est clair

		compression = request.compressionRatio;

		*loop_rate = ros::Rate((double)request.fps);
		running = true;
		
		ROS_INFO("Compression value set to %d", compression);	
		ROS_INFO("Configuration de la camera terminee avec succes...");
		response.ret = ret;
		return true;
	}
	else{
		ROS_INFO("Deconnexion du subscriber...");
		if(--nbrSubscribers <= 0){
			// Il ne reste plus qu'un seul subscriber, donc on peut supprimer sans risque
			running = false;
			ret = system("rosnode kill /uvc_camera && sleep 0.2");
			
			nbrSubscribers = 0;
			ROS_INFO("Plus de subscribers, kill node uvc_camera et mise en attente...");
		}
		response.ret = 0;
		return true;
	}
}

int main(int argc, char* argv[])
{
	// Initialisation
    ros::init(argc, argv, "D_Cam");
    ros::NodeHandle l_n;
	n = &l_n;
	ROS_INFO("Service camera demarre...");
	
	ros::ServiceServer service = n->advertiseService("D_Cam/params", newConfReceived);

    ros::Rate l_loop_rate = ros::Rate(30.);
	// Ok, c'est epouvantable, mais je vois pas comment faire autrement
	loop_rate = &l_loop_rate;
	
    initialised = true;

    ros::Subscriber sub = l_n.subscribe("/image_raw", 2 * 10 * 30, dataCamSync);
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate->sleep();
	}
    return 0;
}


