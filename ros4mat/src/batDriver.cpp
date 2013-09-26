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
#include <sstream>
#include <ros4mat/M_Battery.h>
#include <ros4mat/S_Battery.h>
#include "batHelper.cpp"


ros::Publisher battPublisher;

bool initialised = false;
bool running = false;
unsigned int dataId = 0;
int nbrSubscribers = 0;

ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()
ros::NodeHandle* n = 0;

int readData(void){
	if(!initialised || !running){
		ROS_ERROR("Impossible de recevoir des donnees sans etre connecte et/ou running");
		return -1;
	}
	
	ros4mat::M_Battery msg;
	msg.header.seq = dataId++;
	msg.header.stamp = ros::Time::now();
	
	
	msg.current = GetCurrent();
	
	BQ_State stateBatt = GetState();
	msg.batteryState.push_back(stateBatt.charge);
	msg.batteryState.push_back(stateBatt.discharge);
	msg.batteryState.push_back(stateBatt.vprog);
	msg.batteryState.push_back(stateBatt.overtemp);
	msg.batteryState.push_back(stateBatt.undervoltage);
	msg.batteryState.push_back(stateBatt.overvoltage);
	msg.batteryState.push_back(stateBatt.overcurrent);
	msg.batteryState.push_back(stateBatt.shortcircuit);
	
	msg.vcell.push_back(GetCellVoltage(0));
	msg.vcell.push_back(GetCellVoltage(1));
	msg.vcell.push_back(GetCellVoltage(2));
	msg.vcell.push_back(GetCellVoltage(3));
	msg.vcell.push_back(GetCellVoltage(4));
	msg.vcell.push_back(GetCellVoltage(5));
	
	msg.vtotal = msg.vcell[0]+msg.vcell[1]+msg.vcell[2]+msg.vcell[3]+msg.vcell[4]+msg.vcell[5];
	
	msg.timestamp = ros::Time::now().toSec();
	
	battPublisher.publish(msg);
	return 0;
}

bool newConfReceived(ros4mat::S_Battery::Request& request, ros4mat::S_Battery::Response& response){
	int ret;
    std::string topicName = "D_Battery/data";		// Chemin pouvant etre utilise pour recuperer les donnees
	
	if(request.subscribe){
		ROS_INFO("Configuration de la batterie...");
        nbrSubscribers++;
        if(request.battFreqAcq == 0){
            ROS_INFO("Configuration reussie en mode inscription seulement, demarrage...");
            response.ret = 0;
            return true;
        }
        
		if(request.sendCmdChargeDecharge){
			SetDischargeState(request.dechargeState);
			SetChargeState(request.chargeState);
		}
		
		// TODO protection des frequences
        battPublisher = n->advertise<ros4mat::M_Battery>(topicName, 20);
		*loop_rate = ros::Rate(request.battFreqAcq);
		response.ret = 0;
		running = true;
		return true;
	}
	else{
		if(--nbrSubscribers <= 0){
			// Il ne reste plus qu'un seul subscriber, donc on peut supprimer sans risque
			running = false;
			
			ROS_INFO("Plus de subscribers, mise en attente...");
            nbrSubscribers = 0;
		}
		response.ret = 0;
		return true;
	}
}

int main(int argc, char* argv[])
{
	// Initialisation
    ros::init(argc, argv, "D_Battery");
    ros::NodeHandle l_n;
	n = &l_n;
	
	ros::ServiceServer service = n->advertiseService("D_Battery/params", newConfReceived);
	
	ROS_INFO("[D_Battery] Service de batterie demarre...");
	
    ros::Rate l_loop_rate = ros::Rate(2.);
	// Ok, c'est epouvantable, mais je vois pas comment faire autrement
	loop_rate = &l_loop_rate;

    if (AttachI2C() == 0){
	
		ROS_INFO("[D_Battery] Connexion I2C reussie... en attente de commandes.");
        initialised = true;

        while (ros::ok())
        {
			if(running)
				readData();
            ros::spinOnce();
            loop_rate->sleep();
        }
		DetachI2C();
    }
	else{
		ROS_ERROR("Impossible de se connecter a la puce I2C de controle de la batterie!");
	}

    return 0;
}


