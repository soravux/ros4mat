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
#include <ros4mat/S_Serial.h>


int main(int argc, char **argv)
{
	char* bufR;
	// Initialisation
  ros::init(argc, argv, "testSerial");

  ros::NodeHandle n;
	
	unsigned char data0[] = {128};
	unsigned char data1[] = {131};
	unsigned char data2[] = {137, 130, 200, 128, 00};
	unsigned char data3[] = {137, 0, 0, 0, 0};
	
	ros4mat::S_Serial lParams, lParams2;
	lParams.request.port = "/dev/ttyS0";
	lParams.request.speed = 57600;
	lParams.request.parity = false;
	lParams.request.stopBits = 1;
	lParams.request.sendBufferLength = 1;
	lParams.request.readBufferLength = 0;
	lParams.request.readTimeoutSec = 1;
	lParams.request.readTimeoutMicro = 0;
	lParams.request.closeAfterComm = false;
	for(unsigned int i=0; i<lParams.request.sendBufferLength; i++)	// TEMPORAIRE (le i < 100)
		lParams.request.sendBuffer.push_back(data0[i]);
	
	ROS_INFO("Serial OK, sending data");
	if(!ros::service::call("/D_Serial/serialCommand", lParams)){
		ROS_INFO("[LogicoAgent] ERREUR : l'envoi de la commande serie a renvoye une erreur");
	}
	ROS_INFO("Data sent");
	ROS_INFO("%i : %d", lParams.response.timeoutTrigged, lParams.response.receiveBufferLength);
	
	
	lParams.request.sendBufferLength = 1;
	lParams.request.readBufferLength = 0;
	lParams.request.readTimeoutSec = 1;
	lParams.request.readTimeoutMicro = 0;
	lParams.request.closeAfterComm = false;
	lParams.request.sendBuffer.clear();
	for(unsigned int i=0; i<lParams.request.sendBufferLength; i++)	// TEMPORAIRE (le i < 100)
		lParams.request.sendBuffer.push_back(data1[i]);
	
	ROS_INFO("Serial OK, sending data");
	if(!ros::service::call("/D_Serial/serialCommand", lParams)){
		ROS_INFO("[LogicoAgent] ERREUR : l'envoi de la commande serie a renvoye une erreur");
	}
	ROS_INFO("Data sent");
	ROS_INFO("%i : %d", lParams.response.timeoutTrigged, lParams.response.receiveBufferLength);
	/*bufR = new char[lParams.response.receiveBufferLength+1];
	bufR[lParams.response.receiveBufferLength] = 0;
	for(int i = 0; i < lParams.response.receiveBufferLength; i++)
		bufR[i] = lParams.response.receiveBuffer[i];
	
	ROS_INFO("%s", bufR);
	delete[] bufR;*/
	
	lParams.request.sendBufferLength = 5;
	lParams.request.closeAfterComm = false;
	lParams.request.sendBuffer.clear();
	for(unsigned int i=0; i<lParams.request.sendBufferLength; i++)	// TEMPORAIRE (le i < 100)
		lParams.request.sendBuffer.push_back(data2[i]);
	
	ROS_INFO("Serial OK, sending data");
	if(!ros::service::call("/D_Serial/serialCommand", lParams)){
		ROS_INFO("[LogicoAgent] ERREUR : l'envoi de la commande serie a renvoye une erreur");
	}
	ROS_INFO("Data sent");
	ROS_INFO("%i : %d", lParams.response.timeoutTrigged, lParams.response.receiveBufferLength);
	
	sleep(2);
	lParams.request.sendBufferLength = 5;
	lParams.request.closeAfterComm = false;
	lParams.request.sendBuffer.clear();
	for(unsigned int i=0; i<lParams.request.sendBufferLength; i++)	// TEMPORAIRE (le i < 100)
		lParams.request.sendBuffer.push_back(data3[i]);
	
	ROS_INFO("Serial OK, sending data");
	if(!ros::service::call("/D_Serial/serialCommand", lParams)){
		ROS_INFO("[LogicoAgent] ERREUR : l'envoi de la commande serie a renvoye une erreur");
	}
	ROS_INFO("Data sent");
	ROS_INFO("%i : %d", lParams.response.timeoutTrigged, lParams.response.receiveBufferLength);
	
	/*bufR = new char[lParams2.response.receiveBufferLength+1];
	bufR[lParams2.response.receiveBufferLength] = 0;
	for(int i = 0; i < lParams2.response.receiveBufferLength; i++)
		bufR[i] = lParams2.response.receiveBuffer[i];
	
	ROS_INFO("%s", bufR);
	delete[] bufR;*/
	

  return 0;
}


