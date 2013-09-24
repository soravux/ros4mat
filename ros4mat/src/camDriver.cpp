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
#include <ros4mat/S_Cam.h>
#include <sensor_msgs/Image.h>


bool initialised = false;
bool running = false;

ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()
ros::NodeHandle* n = 0;
int nbrSubscribers = 0, nbrStereoSubscribers = 0;


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
		
		if(!(console = popen("rosnode list | grep /uvc_camera | wc -l", "r"))){
				ROS_WARN("Impossible de verifier si la camera est deja en fonction...");
			}
		
		fgets(bufR, sizeof(bufR), console);
		if(bufR[0] == '1'){
            ROS_INFO("Kill node uvc_camera");
			ret = system("rosnode kill /uvc_camera && sleep 0.2");
		}

		ssRequest << "rosrun uvc_camera camera_node _width:=" << request.width << " _height:=" << request.height << " _device:=" << request.device << " _fps:=" << (unsigned int)request.fps << " &";
		ROS_INFO("Lancement de la commande : %s", ssRequest.str().c_str());
		ret = system(ssRequest.str().c_str());
		ROS_INFO("Fin de la commande (status %i)", ret);
		
		*loop_rate = ros::Rate((double)request.fps);
		running = true;
		
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
	
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate->sleep();
	}
    return 0;
}


