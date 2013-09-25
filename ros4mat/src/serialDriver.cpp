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
#include <map>
#include <ros4mat/S_Serial.h>


std::map<std::string, int> handlesMap;

bool initialised = false;
bool running = false;

ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()
ros::NodeHandle* n = 0;

bool connectDevice(const char* deviceName, unsigned int speed, bool parity, unsigned char stopBits){
	int lHandle = open(deviceName, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if(lHandle == -1){
		ROS_INFO("[D_Serial] ERREUR : Impossible de se connecter au peripherique serie : %d", lHandle);
		return false;
	}
	fcntl(lHandle, F_SETFL, 0);
	
	handlesMap[std::string(deviceName)] = lHandle;
	
	// Configure Serial link
	struct termios port_settings;
	
	tcgetattr(lHandle, &port_settings);    // apply the settings to the port
	switch(speed){
		case 50:
			cfsetispeed(&port_settings, B50);
			cfsetospeed(&port_settings, B50);
		break;
		case 75:
			cfsetispeed(&port_settings, B75);
			cfsetospeed(&port_settings, B75);
		break;
		case 110:
			cfsetispeed(&port_settings, B110);
			cfsetospeed(&port_settings, B110);
		break;
		case 150:
			cfsetispeed(&port_settings, B150);
			cfsetospeed(&port_settings, B150);
		break;
		case 200:
			cfsetispeed(&port_settings, B200);
			cfsetospeed(&port_settings, B200);
		break;
		case 300:
			cfsetispeed(&port_settings, B300);
			cfsetospeed(&port_settings, B300);
		break;
		case 600:
			cfsetispeed(&port_settings, B600);
			cfsetospeed(&port_settings, B600);
		break;
		case 1200:
			cfsetispeed(&port_settings, B1200);
			cfsetospeed(&port_settings, B1200);
		break;
		case 1800:
			cfsetispeed(&port_settings, B1800);
			cfsetospeed(&port_settings, B1800);
		break;
		case 2400:
			cfsetispeed(&port_settings, B2400);
			cfsetospeed(&port_settings, B2400);
		break;
		case 4800:
			cfsetispeed(&port_settings, B4800);
			cfsetospeed(&port_settings, B4800);
		break;
		case 9600:
			cfsetispeed(&port_settings, B9600);
			cfsetospeed(&port_settings, B9600);
		break;
		case 19200:
			cfsetispeed(&port_settings, B19200);
			cfsetospeed(&port_settings, B19200);
		break;
		case 38400:
			cfsetispeed(&port_settings, B38400);
			cfsetospeed(&port_settings, B38400);
		break;
		case 57600:
			cfsetispeed(&port_settings, B57600);
			cfsetospeed(&port_settings, B57600);
		break;
		case 115200:
			cfsetispeed(&port_settings, B115200);
			cfsetospeed(&port_settings, B115200);
		break;
		default:
			ROS_WARN("Vitesse %d bauds invalide; la vitesse par defaut de 19200 bauds sera utilisee", speed);
			cfsetispeed(&port_settings, B19200);
			cfsetospeed(&port_settings, B19200);
		break;
	}
	
	if(!parity)
		port_settings.c_cflag &= ~PARENB;
	else
		port_settings.c_cflag |= PARENB;
		
	if(stopBits == 2)
		port_settings.c_cflag |= CSTOPB;
	else
		port_settings.c_cflag &= ~CSTOPB;
		
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;
    port_settings.c_oflag &= ~OPOST;

	port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	tcsetattr(lHandle, TCSANOW, &port_settings);    // apply the settings to the port
	
	ROS_INFO("Peripherique serie connecte avec succes...");
	initialised = true;
	
	return true;
}


bool serialEvent(ros4mat::S_Serial::Request& request, ros4mat::S_Serial::Response& response){
	int bytesWritten, bytesToRead, bytesRead;
	if(handlesMap.count(request.port) == 0){
		// On doit d'abord ouvrir le port
		if(!connectDevice(request.port.c_str(), request.speed, request.parity, request.stopBits)){
			ROS_INFO("[D_Serial] Erreur de connexion, abort");
			return false;
		}
	}
	
	fd_set rfds;
	FD_ZERO(&rfds);
	FD_SET(handlesMap[request.port], &rfds);

	if(request.sendBufferLength != 0){
		// On envoie des donnees
		char* bufferEnvoi = new char[request.sendBufferLength];
		for(unsigned int i=0; i<request.sendBufferLength; i++)
			bufferEnvoi[i] = request.sendBuffer[i];
		bytesWritten = write(handlesMap[request.port], bufferEnvoi, request.sendBufferLength);
		ROS_INFO("Donnees envoyees : %d bytes ecrits", bytesWritten);
		for(int i = 0; i<request.sendBufferLength; i++)
			ROS_INFO("%X",(unsigned char)bufferEnvoi[i]);
		ROS_INFO("End");
		delete[] bufferEnvoi;
	}
	
	if(request.readBufferLength != 0){
		// On recoit des donnees
		struct timeval Timeout;
		Timeout.tv_usec = request.readTimeoutMicro;  /* microsecondes */
		Timeout.tv_sec  = request.readTimeoutSec;  /* secondes */
		
		char* bufferReception = new char[request.readBufferLength];
		ROS_INFO("Tentative de reception (fd = %d) avec timeout %d.%d", handlesMap[request.port], Timeout.tv_sec, Timeout.tv_usec);
		int selectResult = select(handlesMap[request.port]+1, &rfds, NULL, NULL, &Timeout);
		if(selectResult == -1){
			ROS_INFO("[D_Serial] ERREUR : Echec de lecture (-1 renvoye par select)");
			close(handlesMap[request.port]);
			handlesMap.erase(request.port);
			response.timeoutTrigged = false;
			return false;
		}
		if(!selectResult){
			ROS_INFO("[D_Serial] AVERTISSEMENT : Timeout lors de la lecture (0 renvoye par select)");
			response.timeoutTrigged = true;
		}
		else{
			
			bytesRead = read(handlesMap[request.port], bufferReception, request.readBufferLength);
			if(bytesRead == -1){
				ROS_INFO("[D_Serial] ERREUR : Echec de lecture (-1 renvoye par read)");
				close(handlesMap[request.port]);
				handlesMap.erase(request.port);
				return false;
			}
			ROS_INFO("Donnees recues : %d bytes demandes, %d bytes lus", request.readBufferLength, bytesRead);
			response.timeoutTrigged = false;
			response.receiveBufferLength = request.readBufferLength;
			response.receiveBuffer.reserve(request.readBufferLength);
			for(unsigned int i=0; i<request.readBufferLength; i++)
				response.receiveBuffer.push_back(bufferReception[i]);
		}			
		delete[] bufferReception;
	}
	
	if(request.sendBufferLength == 0 && request.readBufferLength == 0){
		// Cas special, on flush les buffers
		tcflush(handlesMap[request.port], TCIOFLUSH);
	}
	
	if(request.closeAfterComm){
		return true;
		// On ferme le port
		close(handlesMap[request.port]);
		handlesMap.erase(request.port);
	}
	
	return true;
}

int main(int argc, char* argv[])
{
	// Initialisation
    ros::init(argc, argv, "D_Serial");
    ros::NodeHandle l_n;
	n = &l_n;
	ROS_INFO("[D_Serial] Service port serie demarre...");
	
	std::map<std::string, int>::iterator lHandlesIterator;
	
	ros::ServiceServer service = n->advertiseService("D_Serial/serialCommand", serialEvent);

    ros::Rate l_loop_rate = ros::Rate(20.);
	// Ok, c'est epouvantable, mais je vois pas comment faire autrement
	loop_rate = &l_loop_rate;
	
	ROS_INFO("[D_Serial] Service port serie demarre avec succes... en attente d'une commande.");
	initialised = true;

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate->sleep();
	}

	for(lHandlesIterator = handlesMap.begin() ; lHandlesIterator != handlesMap.end(); lHandlesIterator++ )
		close((*lHandlesIterator).second);
	
	ROS_INFO("[D_Serial] Tous les ports fermes avec succes, termine.");
    return 0;
}


