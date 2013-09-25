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
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <ros4mat/M_Computer.h>
#include <ros4mat/S_Computer.h>
#include "../../exchangeStructs.h"

int handleGps = -1;
ros::Publisher computerInfoPublisher;
int nbrSubscribers = 0;

bool initialised = false;
bool running = false;

ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()
ros::NodeHandle* n = 0;


int publishInfos(void){
	std::ifstream fHandle;
    std::string fLine, parsedLine = "";
    std::stringstream lineStream;
    float tmpValue = 0.;
	int tmpIntValue = 0;
    char spaceCarac;
	char bufTmp[100];
	int retCode;
    
	if(!initialised){
		ROS_ERROR("Impossible de recevoir des donnees sans etre connecte et/ou running");
		return -1;
	}
	
	ros4mat::M_Computer msg;
	
    // Load
	double loadavg[3] = {0.};
	getloadavg(loadavg, 3);
	for(unsigned int i = 0; i < 3; i++)
		msg.unixLoad.push_back(loadavg[i]);
    
	// CPU
	msg.timestamp = ros::Time::now().toSec();
	retCode = system("top -b -n 3 -d 0.1 | grep 'Cpu(s):' > /tmp/logicoCpuInfo");
	fHandle.open("/tmp/logicoCpuInfo");
	getline(fHandle,fLine);
    getline(fHandle,fLine);
    getline(fHandle,fLine);     // Seule la 3e ligne nous interesse
	for(unsigned int i = 0; i < fLine.size(); i++){
		if((fLine[i] <= 0x39 && fLine[i] >= 0x30) || fLine[i] == '.' || fLine[i] == '%'){
			parsedLine.append(1, fLine[i]);
		}
	}
	lineStream.clear();
	lineStream.str(parsedLine);
	ROS_INFO_STREAM("1) " << parsedLine);
	
	for(unsigned int i = 0; i < 7; i++){
		lineStream >> tmpValue;
		msg.cpuLoad.push_back(tmpValue);
		lineStream >> spaceCarac;	// Les % separent les valeurs
		}
	fHandle.close();
	
	retCode = system("top -b -n 3 -d 0.1 | grep Mem: > /tmp/logicoMemInfo");
	fHandle.open("/tmp/logicoMemInfo");
	fLine = "";
	parsedLine = "";
	getline(fHandle,fLine);
	for(unsigned int i = 0; i < fLine.size(); i++){
		if((fLine[i] <= 0x39 && fLine[i] >= 0x30) || fLine[i] == 'k'){
			parsedLine.append(1, fLine[i]);
		}
	}
    lineStream.clear();
	lineStream.str(parsedLine);
	//ROS_INFO_STREAM("2) " << lineStream.str());
	for(unsigned int i = 0; i < 3; i++){
		lineStream >> tmpIntValue;
		msg.memUsed.push_back(tmpIntValue);
		lineStream >> spaceCarac;	// Les k separent les valeurs
		}
	fHandle.close();
    
    // Temperatures
    retCode = system("sensors | grep Physical > /tmp/logicoTempInfo");
    fHandle.open("/tmp/logicoTempInfo");
	fLine = "";
	getline(fHandle,fLine);
	fLine = fLine.substr(17,4);
    lineStream.clear();
	lineStream.str(fLine);
	//ROS_INFO_STREAM("3) " << lineStream.str());
	lineStream >> tmpValue;
	msg.cpuTemp = tmpValue;
    fHandle.close();
    
    msg.state = MSGID_COMPUTER_STATE_OK;
    
    computerInfoPublisher.publish(msg);   
	return 0;
}

bool newConfReceived(ros4mat::S_Computer::Request& request, ros4mat::S_Computer::Response& response){
	int ret = 0;

    std::string topicName = "D_ComputerInfo/data";		// Chemin pouvant etre utilise pour recuperer les donnees
	
	if(request.subscribe){
		ROS_INFO("[D_ComputerInfo] Configuration...");
        nbrSubscribers++;
        if(request.freqAcq == 0){
            ROS_INFO("Configuration reussie en mode inscription seulement, demarrage...");
            response.ret = 0;
            return true;
        }
        
		computerInfoPublisher = n->advertise<ros4mat::M_Computer>(topicName, request.bufferSize);
		*loop_rate = ros::Rate(request.freqAcq);
		running = true;
		
		ROS_INFO("Configuration terminee avec succes...");
		response.ret = ret;
		return true;
	}
	else{
		ROS_INFO("Deconnexion du subscriber...");
		if(--nbrSubscribers <= 0){
			// Il ne reste plus qu'un seul subscriber, donc on peut supprimer sans risque
			running = false;
			nbrSubscribers = 0;
			ROS_INFO("Plus de subscribers, mise en attente...");
		}
		response.ret = 0;
		return true;
	}
}

int main(int argc, char* argv[])
{
	// Initialisation
    ros::init(argc, argv, "D_ComputerInfo");
    ros::NodeHandle l_n;
	n = &l_n;
	ROS_INFO("[D_ComputerInfo] Service demarre...");
	
	ros::ServiceServer service = n->advertiseService("D_ComputerInfo/params", newConfReceived);

    ros::Rate l_loop_rate = ros::Rate(5.);
	// Ok, c'est epouvantable, mais je vois pas comment faire autrement
	loop_rate = &l_loop_rate;
	
    ROS_INFO("Demarrage effectue avec succes... en attente d'une commande.");

    initialised = true;

    while (ros::ok())
    {
        if(running)
            publishInfos();
        ros::spinOnce();
        loop_rate->sleep();
    }

    return 0;
}


