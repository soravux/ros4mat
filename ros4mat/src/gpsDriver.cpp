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
#include <ros4mat/M_GPS.h>
#include <ros4mat/S_GPS.h>
#include "exchangeStructs.h"

int handleGps = -1;
ros::Publisher gpsPublisher;

bool initialised = false;
bool running = false;

ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()
ros::NodeHandle* n = 0;

bool connectGps(const char* deviceName){
	handleGps = open(deviceName, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(handleGps == -1){
		ROS_INFO("[D_GPS] ERREUR : Impossible de se connecter au GPS");
		return false;
	}
	fcntl(handleGps, F_SETFL, 0);
	
	// Configure Serial link
	struct termios port_settings;
	
	tcgetattr(handleGps, &port_settings);    // apply the settings to the port
	cfsetispeed(&port_settings, B38400);    // set baud rates
	cfsetospeed(&port_settings, B38400);
	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;
    port_settings.c_oflag &= ~OPOST;

	port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tcsetattr(handleGps, TCSANOW, &port_settings);    // apply the settings to the port
	
	ROS_INFO("[D_GPS] GPS Skytrack connected...");
	initialised = true;
	
	return true;
}

int readGps(int timeoutSec = 0, int timeoutMicro = 250000){
	
	char* bufferRead;
	
	int selectResult, bytesRead, bytesReady;
	
	struct timeval Timeout;
    Timeout.tv_usec = timeoutMicro;  /* microsecondes */
    Timeout.tv_sec  = timeoutSec;  /* secondes */
	
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(handleGps, &rfds);
	
	if(!initialised){
		ROS_INFO("[D_GPS] ERREUR : Impossible de recevoir des donnees sans etre connecte et/ou running");
		return -1;
	}
	
	ros4mat::M_GPS msg;
	
	selectResult = select(handleGps+1, &rfds, NULL, NULL, &Timeout);
	if(selectResult == -1){
		ROS_INFO("[D_GPS] ERREUR : Echec de lecture (-1 renvoye par select)");
		return -1;
	}
	if(!selectResult){
		ROS_INFO("[D_GPS] AVERTISSEMENT : Timeout lors de la lecture (0 renvoye par select)");
		return 0;
	}
	
	ioctl(handleGps, FIONREAD, &bytesReady);
	
	bufferRead = new char[bytesReady];
	
	bytesRead = read(handleGps, bufferRead, bytesReady);
	if(bytesRead == -1){
		ROS_INFO("[D_GPS] ERREUR : Echec de lecture (-1 renvoye par read)");
		delete[] bufferRead;
		return -1;
	}
	
	std::string strBuf(bufferRead, bytesRead);
	std::stringstream lineStream;
	std::string sTmp;
	size_t currentPos = 0;
	size_t nextPos = strBuf.find("\n", currentPos);
	
	char readChar;
	float dummyFloat;
	float utcTime = 0.;
	float latitude = 0.;
	float longitude = 0.;
	float altitude = 0.;
	float vitesseAngle = 0.;
	float vitesseModule = 0.;
	short fixType = 0;
	unsigned short nbrSats = 0;
	
	while(nextPos != std::string::npos){
		sTmp = strBuf.substr(currentPos,nextPos);
		if(sTmp.substr(0,6) == "$GPGGA"){
			// Temps, position et statut
			lineStream.str(sTmp.substr(7));
			lineStream >> utcTime;
			lineStream >> readChar;
			lineStream >> latitude;
			lineStream >> readChar;
			lineStream >> readChar;		// N/S
			lineStream >> readChar;
			lineStream >> longitude;
			lineStream >> readChar;		// W/E
			lineStream >> readChar;
			lineStream >> fixType;
			lineStream >> readChar;
			lineStream >> nbrSats;
			lineStream >> readChar;
			lineStream >> dummyFloat;
			lineStream >> readChar;
			lineStream >> altitude;
		}
		else if(sTmp.substr(0,6) == "$GPVTG"){
			// Vitesse
			lineStream.str(sTmp.substr(7));
			lineStream >> vitesseAngle;
			lineStream >> readChar;
			lineStream >> readChar;		// T
			lineStream >> readChar;
			lineStream >> readChar;		// 2 , de suite
			lineStream >> readChar;		// M
			lineStream >> vitesseModule;		// Vitesse... en knots (!)
			lineStream >> readChar;
			lineStream >> readChar;		// N
			lineStream >> readChar;
			lineStream >> vitesseModule;		// Vitesse en km/h
		}
		// On ignore les autres messages

		currentPos = nextPos+1;
		if(nextPos == strBuf.size())
			break;
		
		nextPos = strBuf.find("\n", currentPos);
	}
	
	msg.status = fixType;
	msg.latitude = latitude / 100.;		// Le format envoye par le GPS est ddmm.mmmm
	msg.longitude = longitude / 100.;	// ou dd est le degre et mm les minutes
	msg.altitude = altitude;
	msg.HvitesseModule = vitesseModule;
	msg.HvitesseAngle = vitesseAngle;
	msg.satUsed = nbrSats;
	
	msg.timestamp = ros::Time::now().toSec();
    
    gpsPublisher.publish(msg);
	delete[] bufferRead;
	return 0;
}

bool newConfReceived(ros4mat::S_GPS::Request& request, ros4mat::S_GPS::Response& response){
	int ret = 0;

    std::string topicName = "D_GPS/data";		// Chemin pouvant etre utilise pour recuperer les donnees
	
	if(request.subscribe){
		ROS_INFO("[D_GPS] Configuration du GPS...");
		// TODO protection des frequences
		gpsPublisher = n->advertise<ros4mat::M_GPS>(topicName, request.gpsBufferSize);
		*loop_rate = ros::Rate(request.gpsFreqAcq);
		running = true;
		
		ROS_INFO("[D_GPS] Configuration du GPS terminee avec succes...");
		response.ret = ret;
		response.gpsStatus = 0;
		tcflush(handleGps, TCIFLUSH);
		return true;
	}
	else{
		ROS_INFO("[D_GPS] Deconnexion du subscriber...");
		if(gpsPublisher.getNumSubscribers() <= 1){
			// Il ne reste plus qu'un seul subscriber, donc on peut supprimer sans risque
			running = false;
			
			ROS_INFO("[D_GPS] Plus de subscribers, mise en attente...");
		}
		response.ret = 0;
		return true;
	}
}

int main(int argc, char* argv[])
{
	// Initialisation
    ros::init(argc, argv, "D_GPS");
    ros::NodeHandle l_n;
	n = &l_n;
	ROS_INFO("[D_GPS] Service GPS demarre...");
	
	ros::ServiceServer service = n->advertiseService("D_GPS/params", newConfReceived);

    ros::Rate l_loop_rate = ros::Rate(5.);
	// Ok, c'est epouvantable, mais je vois pas comment faire autrement
	loop_rate = &l_loop_rate;
	
    if (connectGps("/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_D-if00-port0")) {
		ROS_INFO("[D_GPS] GPS connecte avec succes... en attente d'une commande.");
        initialised = true;

        while (ros::ok())
        {
			if(running)
				readGps();
            ros::spinOnce();
            loop_rate->sleep();
        }

    }
    return 0;
}
