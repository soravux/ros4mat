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
#include <ros/console.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sstream>
#include <ros4mat/M_ADC.h>
#include <ros4mat/S_ADC.h>
#include <ros4mat/S_DigitalOut.h>

#define CAN_THROW_SAMPLES_AWAY true


int handleAdc = -1;
ros::Publisher adcPublisher;
bool initialised = false;
bool running = false;
int freqAcquisition = 0;

char slistData[8] = {-1};		// Contient le channel associe a chaque numero de la slist
int nbrActiveChannels = 0;
ros::Time beginTime;
int nbrSubscribers = 0;

ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()
ros::NodeHandle* n = 0;

unsigned int dataId = 0;

int execCmd(const char* cmd, int bytesToWrite, int bytesToRead, char* bufferReception, int timeoutSec = 1, int timeoutMicro = 0){
	int selectResult, bytesRead, bytesWritten;
	struct timeval Timeout;
    Timeout.tv_usec = timeoutMicro;  /* microsecondes */
    Timeout.tv_sec  = timeoutSec;  /* secondes */
	
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(handleAdc, &rfds);
    
    
	bytesWritten = write(handleAdc, cmd, bytesToWrite);
	if(bytesWritten < 0){
		ROS_ERROR("Echec d'ecriture");
		return -1;
	}
    
	if(bytesToRead == 0){
		ROS_DEBUG("Commande envoyee avec succes");
		return 0;
	}
	
	selectResult = select(handleAdc+1, &rfds, NULL, NULL, &Timeout);
	if(selectResult == -1){
		ROS_ERROR("Echec de lecture lors de l'envoi d'une commande (-1 renvoye par select)");
		return -1;
	}
	if(!selectResult){
		ROS_WARN("Timeout lors de la lecture lors de l'envoi d'une commande (0 renvoye par select)");
		return 0;
	}
	
	bytesRead = read(handleAdc, bufferReception, bytesToRead);
    if(bytesRead == -1){
		ROS_ERROR("Echec de lecture lors de l'envoi d'une commande (-1 renvoye par read)");
		return -1;
	}
	
	ROS_DEBUG("Commande envoyee avec succes");
	return 0;
}

int attachAdc(const char* deviceName){
	char checkCmd1[] = "info 0\r";
	char checkCmd2[] = "info 1\r";
	char checkFirmwareCmd[] = "info 2\r";
	char checkSerialNbrCmd[] = "info 6\r";

	char answerCmd1[] = "info 0 DATAQ\r";
	char answerCmd2[] = "info 1 1490\r";

	char* recvBuffer;

	int deviceFlags;
	int stateCmd;
	int errorOccured = 0;
	handleAdc = open(deviceName, O_RDWR | O_NOCTTY);
	if(handleAdc == -1){
		ROS_ERROR("ADC connection error on open() : %s", strerror(errno));
		return -1;
	}
	//deviceFlags = fcntl(handleAdc, F_GETFL);
	//fcntl(handleAdc, F_SETFL, deviceFlags | FNDELAY);	// No delay on read
	
	// Configure Serial link
	struct termios port_settings;
	memset(&port_settings, 0, sizeof(struct termios));
	
	cfsetispeed(&port_settings, B115200);    // set baud rates
	cfsetospeed(&port_settings, B115200);
	port_settings.c_cflag |= (CLOCAL | CREAD);
	port_settings.c_cflag |= CS8;

	tcsetattr(handleAdc, TCSANOW, &port_settings);    // apply the settings to the port
	
	ROS_INFO("Serial link configured...");
	tcflush(handleAdc, TCIOFLUSH);			// Make sure that there is no garbage in the input buffers

	recvBuffer = new char[strlen(answerCmd1)];
	memset(recvBuffer, 0, strlen(answerCmd1));
	stateCmd = execCmd(checkCmd1, strlen(checkCmd1), strlen(answerCmd1), recvBuffer, 5);	// Allow a 5 seconds timeout
	if(stateCmd < 0 || memcmp(answerCmd1, recvBuffer, strlen(answerCmd1)) != 0){
		ROS_WARN("ADC communication test failed! Will try to continue anyway.");
		if(stateCmd < 0)
			ROS_WARN("Invalid execCmd return state");
		else
			ROS_WARN("First bytes received : %c%c%c%c", recvBuffer[0], recvBuffer[1], recvBuffer[2], recvBuffer[3]);
		errorOccured = 1;
	}
	delete[] recvBuffer;

	recvBuffer = new char[strlen(answerCmd2)];
	memset(recvBuffer, 0, strlen(answerCmd2));
	stateCmd = execCmd(checkCmd2, strlen(checkCmd2), strlen(answerCmd2), recvBuffer);
	if(stateCmd < 0 || memcmp(answerCmd2, recvBuffer, strlen(answerCmd2)) != 0){
		ROS_WARN("ADC identification test failed! Will try to continue anyway.");
		if(stateCmd < 0)
			ROS_WARN("Invalid execCmd return state");
		else
			ROS_WARN("First byte received : %c%c%c%c", recvBuffer[0], recvBuffer[1], recvBuffer[2], recvBuffer[3]);
		errorOccured = 1;
	}
	delete[] recvBuffer;

	if(!errorOccured)
		ROS_INFO("ADC DataQ DI-149 identification OK.");

	initialised = true;
	return 0;
}


int detachAdc(void){
	char buffer[40];
	if(running){
		// Si l'ADC envoie du data, on doit d'abord l'arreter
		execCmd("stop\r", 5, 5, buffer);
	}
	close(handleAdc);
	return 0;
}

void alignRead(void){
	char buffer[40];
	int readRet, totalRead;

	if(!running || !initialised){
		ROS_ERROR("Impossible d'aligner l'input sans etre running/initialized");
		return;
	}
	ROS_INFO("Input alignment on the first byte result...");
	tcflush(handleAdc, TCIFLUSH);	// S'assure que les buffers d'entree sont bien vides

	do{		// On s'aligne
		readRet = read(handleAdc, buffer, 1);
		//ROS_INFO("Value in first byte : %X", ((unsigned int)(buffer[0])) & 0xFF);
	} while(readRet != 1 || (buffer[0] & 0x01));
	totalRead = 1;
	while(totalRead < nbrActiveChannels*2){
		readRet = read(handleAdc, buffer, nbrActiveChannels*2-totalRead);
		if(readRet == -1){
			ROS_WARN("Read error while aligning!");
			continue;
		}
		totalRead += readRet;
	}
	beginTime =  ros::Time::now();
	ROS_INFO("Alignment done successfully");
}

int setParamsAdc(int freq, const bool channelsState[]){
	// channelsState contient 8 booleen, pour les 8 canaux
	// Si valeur=0, alors channel inactif
	// Sinon, il acquisitionne a 'freq' Hz
	// La frequence doit etre entre 12 et 2000
	// La frequence peut etre approximative (surtout pour les petits taux)
	char* buffer;
	std::stringstream stmp;
	int currentSlistIndex = 0;

	buffer = new char[40];
	
	ROS_INFO("ADC parameters update...");
	ROS_INFO("Freq = %d", freq);
	for(unsigned int i=0; i<8; i++)
		ROS_INFO("Channel %i = %d", i, channelsState[i]);
	
	if(running){
		// Si l'ADC envoie du data, on doit d'abord l'arreter
		ROS_INFO("Stopping ADC...");
		execCmd("stop\r", 5, 5, buffer);
		running = false;
	}
	
	freqAcquisition = freq;
	
	nbrActiveChannels = 0;
	execCmd("asc\r", 4, 4, buffer);
	
	for(unsigned int i=0; i<8; i++){
		if(channelsState[i]){
			stmp.str("");
			stmp.clear();
			stmp << "slist " << (currentSlistIndex) << " x000" << i << '\r';
			execCmd(stmp.str().c_str(), stmp.str().length(), stmp.str().length(), buffer);
			slistData[currentSlistIndex] = i;
			nbrActiveChannels++;
			currentSlistIndex++;
		}
	}

	stmp.str("");		// Notify the end of the slist
	stmp.clear();
	stmp << "slist " << (currentSlistIndex) << " xffff" << '\r';
	execCmd(stmp.str().c_str(), stmp.str().length(), stmp.str().length(), buffer);

	stmp.str("");
	stmp.clear();
	stmp << "srate " << 750000/freq << '\r';
	execCmd(stmp.str().c_str(), stmp.str().length(), stmp.str().length(), buffer);

	execCmd("bin\r", 4, 4, buffer);

	execCmd("start\r", 6, 6, buffer);

	running = true;

	ROS_INFO("End of configuration...");
	delete[] buffer;
	return 0;
}

int readData(int timeoutSec = 0, int timeoutMicro = 250000){
	char* bufferRead;
	int selectResult, bytesRead, bytesReady;
	int currentWordIndex = 0;
	short rawValue;
	double valueVolt;
	bool samplesOk = true;
	
	struct timeval Timeout;
    Timeout.tv_usec = timeoutMicro;  /* microsecondes */
    Timeout.tv_sec  = timeoutSec;  /* secondes */

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(handleAdc, &rfds);
	
	if(!initialised || !running){
		ROS_ERROR("Impossible de recevoir des donnees sans etre connecte et/ou running");
		return -1;
	}
	//ROS_INFO("Read DATA!");
	double periodeAcquisition = 1./(double)(freqAcquisition);
	double timestampSample = beginTime.toSec();
	ros4mat::M_ADC msg;

	// On peut sauter les premiers samples mais c'est sans importance
	int sizeToRead = nbrActiveChannels*2;
	bufferRead = new char[sizeToRead];

	while(true){
		bytesRead = read(handleAdc, bufferRead, sizeToRead);
		if(bytesRead == 0)
			break;
		while(bytesRead < sizeToRead){
			//ROS_WARN("Bytes read : %i / %i", bytesRead, sizeToRead);
			bytesRead += read(handleAdc, bufferRead+bytesRead, sizeToRead-bytesRead);
		}
		if(bufferRead[0] & 0x01){
			ROS_WARN("Synchronization error (first byte is %X)! Will try to resync on the next pass, but samples may have been lost.", bufferRead[0]);
			for(int i=0; i < sizeToRead-1; i++)
				ROS_INFO("[%i] = %X", i, ((int)bufferRead[i]) & 0xFF);
			alignRead();
			break;
		}

		currentWordIndex = 0;
		for(int i=0; i<sizeToRead-1; i+=2){

			if(currentWordIndex != 0 && !(bufferRead[i] & 0x01)){
				ROS_WARN("Synchronization error : index %i is not the initial byte, but sync bit is set to 0 (1 expected) : %X This sample will be skipped.", i, (unsigned char)(bufferRead[i]) & 0xFF);
				samplesOk = false;
			}

			// rawValue est un short en little endian
			// donc l'adresse la plus BASSE contient les bits les MOINS significatifs
			
			// A4  A3  A2  A1  A0  D1  D0  0
			// A11  A10  A9  A8  A7  A6  A5  1
			short* lInt = (short*)&bufferRead[i];
			(*lInt) = ((*lInt & 0xFE00) | ((*lInt & 0x00F8) << 1)) >> 4;
			rawValue = (*lInt);
			
			valueVolt = rawValue;
			valueVolt /= 204.7;
			valueVolt -= 10.0;

			switch(slistData[currentWordIndex]){
				case 0:
					msg.canal1.push_back(valueVolt);
					break;
				case 1:
					msg.canal2.push_back(valueVolt);
					break;
				case 2:
					msg.canal3.push_back(valueVolt);
					break;
				case 3:
					msg.canal4.push_back(valueVolt);
					break;
				case 4:
					msg.canal5.push_back(valueVolt);
					break;
				case 5:
					msg.canal6.push_back(valueVolt);
					break;
				case 6:
					msg.canal7.push_back(valueVolt);
					break;
				case 7:
					msg.canal8.push_back(valueVolt);
					break;
				default:
					ROS_WARN("Reception de donnee d'une entree inconnue (%d / %d)!", currentWordIndex, slistData[currentWordIndex]);
			}

			if(++currentWordIndex == nbrActiveChannels){
				currentWordIndex = 0;
				// Ajout d'un timestamp
				// Est-ce que ces deux lignes doivent etre inversees?
				msg.timestamps.push_back(beginTime.toSec());
				beginTime = beginTime + ros::Duration(periodeAcquisition);
			}
		
		}
	}
	if(samplesOk){	// Publish only if we are in sync
		msg.header.seq = dataId++;
		msg.header.stamp = ros::Time::now();
		
		adcPublisher.publish(msg);
		}
	delete[] bufferRead;
	return 0;
}

bool setDigitalOut(ros4mat::S_DigitalOut::Request& request, ros4mat::S_DigitalOut::Response& response){
	unsigned char dataSend = 0;
	char data[4] = {0};
	
	for(unsigned int i = 0; i < 4; i++)
		dataSend += (request.outputState[i]) ? 0 : (1 << i);		// Logique negative sur le DataQ
	
	response.ret = execCmd(data, 4, 0, NULL);
	response.timestampSet = ros::Time::now().toSec();
	return true;
}

bool newConfReceived(ros4mat::S_ADC::Request& request, ros4mat::S_ADC::Response& response){
	bool channels[8] = {false};
	char bufferComm[5] = {0};
    std::string topic_path = "D_ADC/data";		// Chemin pouvant etre utilise pour recuperer les donnees
	int ret, nbrChan = 0;
	
	if(request.subscribe){
        
        nbrSubscribers++;
        if(request.adcFreqAcq == 0){
            ROS_INFO("Configuration reussie en mode inscription seulement, demarrage...");
            response.ret = 0;
            return true;
        }
        
		//ROS_INFO("%d", request.adcFreqAcq);
		for(unsigned int i=0; i<8; i++){
			channels[i] = request.adcChannels[i];
			if(channels[i])
				nbrChan++;
			//ROS_INFO(">>> [%d] = %d", i, channels[i]);
		}
		
		if(nbrChan == 0){
			ROS_ERROR("Aucun canal de l'ADC actif!");
			response.errorDesc = "No ADC channel was set active!";
			response.ret = -1;
			return false;
		}
		
		if(request.adcFreqPoll == 0){
			ROS_ERROR("Frequence de polling de 0!");
			response.errorDesc = "Polling frequency set to 0 Hz!";
			response.ret = -1;
			return false;
		}
		if(request.adcFreqAcq == 0){
			ROS_ERROR(" Frequence d'acquisition de 0!");
			response.errorDesc = "Sample frequency set to 0 Hz!";
			response.ret = -1;
			return false;
		}
		ret = setParamsAdc((request.adcFreqAcq<12) ? 12 : request.adcFreqAcq, channels);
		// Si c'est le premier subscriber, on cree le publisher
		if(nbrSubscribers == 1){
			ROS_INFO("Pas de publisher disponible, creation de celui-ci...");
			adcPublisher = n->advertise<ros4mat::M_ADC>(topic_path, request.adcBufferSize);
		}
		*loop_rate = ros::Rate((double)request.adcFreqPoll);
		response.ret = ret;
		return true;
		
	}
	else{
		ROS_INFO("Deconnexion d'un subscriber (il reste %d subscribers)...", adcPublisher.getNumSubscribers());
		if(--nbrSubscribers <= 0){
			// Il ne reste plus qu'un seul subscriber, donc on arrete
			running = false;
			execCmd("stop\r", 5, 5, bufferComm);
			
			tcflush(handleAdc, TCIOFLUSH);
			ROS_INFO("Plus de subscribers, arret de la communication avec l'ADC...");
            nbrSubscribers = 0;
		}
		response.ret = 0;
		return true;
	}
}

int main(int argc, char* argv[])
{
	// Initialisation
    ros::init(argc, argv, "D_ADC");
    ros::NodeHandle l_n;
	n = &l_n;
	
	ros::ServiceServer service = n->advertiseService("D_ADC/params", newConfReceived);
	ros::ServiceServer service_dout = n->advertiseService("D_ADC/digitalOutCtrl", setDigitalOut);
	
	ros::Rate l_loop_rate = ros::Rate(20.);
	// Ok, c'est epouvantable, mais je vois pas comment faire autrement
	loop_rate = &l_loop_rate;


    if (attachAdc("/dev/serial/by-id/usb-0683_1490-if00") == 0) {
        initialised = true;

        while (ros::ok())
        {
			if(running)
				readData();
				
            ros::spinOnce();
            loop_rate->sleep();
        }

        detachAdc();
    }
    return 0;
}


