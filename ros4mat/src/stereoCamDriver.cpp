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
#include <ros4mat/S_StereoCam.h>
#include <ros4mat/M_StereoCam.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>


bool initialised = false;
bool running = false;

ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()
ros::NodeHandle* n = 0;
ros::Publisher stereoPublisher;
int nbrStereoSubscribers = 0;
unsigned int dataId = 0;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;	

void dataCameraSync(const sensor_msgs::Image::ConstPtr& imageL, const sensor_msgs::Image::ConstPtr& imageR){
	ros4mat::M_StereoCam msg;
	msg.header.seq = dataId++;
	msg.header.stamp = ros::Time::now();
	msg.width = imageL->width;
	msg.height = imageL->height;
	msg.channels = 3;

	msg.timestamp = imageL->header.stamp.toSec();

	msg.image_left = imageL->data;
	msg.image_right = imageR->data;

	stereoPublisher.publish(msg);
}

bool newConfStereo(ros4mat::S_StereoCam::Request& request, ros4mat::S_StereoCam::Response& response){
	int ret = 0;
	FILE *console;
	char bufR[10];
	std::stringstream ssRequest;

	if(request.subscribe){
		ROS_INFO("Configuration de la paire stereo...");
		nbrStereoSubscribers++;
		if(request.fps == 0){
            ROS_INFO("Configuration stereo reussie en mode inscription seulement, demarrage...");
            response.ret = 0;
            return true;
        }

        if(!(console = popen("rosnode list | grep uvc_camera_stereo | wc -l", "r"))){
				ROS_WARN("Impossible de verifier si la paire stereo est deja en fonction...");
			}
		fgets(bufR, sizeof(bufR), console);
		if(bufR[0] == '1'){
            ROS_INFO("Kill node uvc_camera/stereo_node");
			ret = system("rosnode kill /uvc_camera_stereo && sleep 0.2");
		}

		ssRequest << "rosrun uvc_camera stereo_node _width:=" << request.width << " _height:=" << request.height << " _left/device:=" << request.device_L << " _right/device:=" << request.device_R << " _fps:=" << (unsigned int)request.fps << " &";
		ROS_INFO("Lancement de la commande : %s", ssRequest.str().c_str());
		ret = system(ssRequest.str().c_str());
		ROS_INFO("Fin de la commande (status %i)", ret);


		if(nbrStereoSubscribers == 1){
			ROS_INFO("Pas de publisher disponible, creation de celui-ci...");
			stereoPublisher = n->advertise<ros4mat::M_StereoCam>("D_CamStereo/data", request.camBufferSize);
		}

		
		*loop_rate = ros::Rate((double)request.fps);
		running = true;

		/*message_filters::Subscriber<sensor_msgs::Image> imageL_sub(*n, "/left/image_raw", request.fps);
        message_filters::Subscriber<sensor_msgs::Image> imageR_sub(*n, "/right/image_raw", request.fps);

        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(request.fps), imageL_sub, imageR_sub);

        sync.registerCallback(boost::bind(dataCameraSync, _1, _2));*/


		
		ROS_INFO("Configuration de la camera terminee avec succes...");
		response.ret = ret;
		return true;
	}
	else{
		ROS_INFO("Deconnexion du subscriber (stereo)...");
		if(--nbrStereoSubscribers <= 0){
			// Il ne reste plus qu'un seul subscriber, donc on peut supprimer sans risque
			running = false;
			ret = system("rosnode kill /uvc_camera_stereo && sleep 0.2");
			
			nbrStereoSubscribers = 0;
			ROS_INFO("Plus de subscribers au stereo, kill node uvc_camera et mise en attente...");
		}
		response.ret = 0;
		return true;
	}
}



int main(int argc, char* argv[])
{
	// Initialisation
    ros::init(argc, argv, "D_CamStereo");
    ros::NodeHandle l_n;
	n = &l_n;
	ROS_INFO("Service camera stereo demarre...");

	ros::ServiceServer service = n->advertiseService("D_CamStereo/params", newConfStereo);

    ros::Rate l_loop_rate = ros::Rate(30.);
	// Ok, c'est epouvantable, mais je vois pas comment faire autrement
	loop_rate = &l_loop_rate;

        /* Connecting to a maybe-not-already-published topic. This hack is to handle the sync()
        variable that seems to fall out of scope in the external function. */
		message_filters::Subscriber<sensor_msgs::Image> imageL_sub(*n, "/left/image_raw", 10);
        message_filters::Subscriber<sensor_msgs::Image> imageR_sub(*n, "/right/image_raw", 10);

        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imageL_sub, imageR_sub);

        sync.registerCallback(boost::bind(dataCameraSync, _1, _2));
	
	
    initialised = true;
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate->sleep();
	}
    return 0;
}


