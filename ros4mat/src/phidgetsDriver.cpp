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
#include <phidget21.h>
#include <ros4mat/M_IMU.h>
#include <ros4mat/S_IMU.h>


CPhidgetSpatialHandle phid;


ros::NodeHandle* n = 0;
ros::Publisher imuPublisher;

bool initialised = false;
bool running = false;
int nbrSubscribers = 0;

unsigned int dataId = 0;

ros::Time beginAcquisitionTime;
unsigned int periodeAcquisition = 0;
ros::Rate* loop_rate = 0;		// Pointeur car ne peut pas etre initialise avant ros::init()


int AttachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("[D_IMU] %s Phidgets reconnu (serial : %d)!", name, serial_number);

    return 0;
}

int DetachHandler(CPhidgetHandle phid, void *userptr)
{
    int serial_number;
    const char *name;

    CPhidget_getDeviceName (phid, &name);
    CPhidget_getSerialNumber(phid, &serial_number);
    ROS_INFO("[D_IMU] %s Phidgets debranche (serial : %d)!", name, serial_number);

    return 0;
}

int ErrorHandler(CPhidgetHandle phid, void *userptr, int ErrorCode, const char *Description)
{
    ROS_INFO("[D_IMU] ERREUR (et lamentations) %d - %s", ErrorCode, Description);
    return 0;
}

int SpatialDataHandler(CPhidgetSpatialHandle spatial, void *userptr, CPhidgetSpatial_SpatialEventDataHandle *data, int count)
{
    static double magneticFieldRem[3] = {0., 0., 0.};
    if (initialised) {
        ros4mat::M_IMU s;
        for (int i = 0; i < count; i++) {
			// Renseigner le header est essentiel pour rxplot
			s.header.seq = dataId++;
			s.header.stamp = ros::Time::now();
			
			// On rentre les datas RAW (pas de calcul d'angle par exemple)
			s.aX.push_back(data[i]->acceleration[0]);
			s.aY.push_back(data[i]->acceleration[1]);
			s.aZ.push_back(data[i]->acceleration[2]);
			
			s.rX.push_back(data[i]->angularRate[0]);
			s.rY.push_back(data[i]->angularRate[1]);
			s.rZ.push_back(data[i]->angularRate[2]);
			
			if(data[i]->magneticField[0] == PUNK_DBL){
				// Cas special, calibration du magnetometre a tous les 28 ms
				s.mX.push_back(magneticFieldRem[0]);
				s.mY.push_back(magneticFieldRem[1]);
				s.mZ.push_back(magneticFieldRem[2]);
			}
			else{
				magneticFieldRem[0] = data[i]->magneticField[0];
				magneticFieldRem[1] = data[i]->magneticField[1];
				magneticFieldRem[2] = data[i]->magneticField[2];
				s.mX.push_back(data[i]->magneticField[0]);
				s.mY.push_back(data[i]->magneticField[1]);
				s.mZ.push_back(data[i]->magneticField[2]);
			}
			
			s.timestamps.push_back(ros::Time::now().toSec()-(count-i-1)*periodeAcquisition);
			//s.timestamps.push_back(beginAcquisitionTime.toSec() +(double)(data[i]->timestamp.seconds) + (data[i]->timestamp.microseconds/1000000.0));
        }
        imuPublisher.publish(s);
    }
    return 0;
}


bool attach(CPhidgetSpatialHandle &phid)
{
    int result;
    const char *err;
	ROS_INFO("[D_IMU] Demarrage du driver...");
    CPhidgetSpatial_create(&phid);

    //Set the handlers
    CPhidget_set_OnAttach_Handler((CPhidgetHandle)phid, AttachHandler, NULL);
    CPhidget_set_OnDetach_Handler((CPhidgetHandle)phid, DetachHandler, NULL);
    CPhidget_set_OnError_Handler((CPhidgetHandle)phid, ErrorHandler, NULL);

    // Callback function (le dernier argument est pour passer un pointeur, inutile ici)
    CPhidgetSpatial_set_OnSpatialData_Handler(phid, SpatialDataHandler, NULL);
	ROS_INFO("[D_IMU] Creation des handler effectuee...");
	
	ROS_INFO("[D_IMU] Pret a recevoir des commandes...");
    return true;
}

void disconnect(
    CPhidgetSpatialHandle &phid)
{
    ROS_INFO("En cours de fermeture...");
    CPhidget_close((CPhidgetHandle)phid);
    CPhidget_delete((CPhidgetHandle)phid);
}

bool newConfReceived(ros4mat::S_IMU::Request& request, ros4mat::S_IMU::Response& response){
	int result, milliS;
    const char *err;
	
    std::string topic_path = "D_IMU/data";		// Chemin pouvant etre utilise pour recuperer les donnees
	
	if(request.subscribe){	
		ROS_INFO("[D_IMU] Configuration de l'IMU...");
		
		nbrSubscribers++;
		if(request.imuFreqAcq == 0){
			ROS_INFO("Configuration reussie en mode inscription seulement, demarrage...");
			response.ret = 0;
			return true;
		}
		
		if(running)
			CPhidget_close((CPhidgetHandle)phid);
		
		// Si c'est le premier subscriber, on cree le publisher
		if(nbrSubscribers == 1)
			imuPublisher = n->advertise<ros4mat::M_IMU>(topic_path, request.imuBufferSize);

		//On tente de recuperer les datas
		CPhidget_open((CPhidgetHandle)phid, -1);
		//En attente du signal
		ROS_INFO("[D_IMU] En attente de la reponse du driver...");
		if ((result = CPhidget_waitForAttachment((CPhidgetHandle)phid, 10000))) {
			CPhidget_getErrorDescription(result, &err);
			ROS_INFO("[D_IMU] Erreur lors de la connexion a l'IMU : %s\n", err);
			response.ret = result;
			running = false;
			return false;
		}
		else{
			milliS = (request.imuFreqAcq > 250) ? 4 : (int)(1./((double)(request.imuFreqAcq) / 1000));
			if(milliS > 16 && milliS %8 != 0)
				milliS = milliS - milliS %8;
			
            periodeAcquisition = milliS;
			beginAcquisitionTime = ros::Time::now();
			CPhidgetSpatial_setDataRate(phid, milliS);
			
			*loop_rate = ros::Rate(request.imuFreqPoll);
			response.ret = 0;
			running = true;
		}
		ROS_INFO("Configuration reussie en mode complet, demarrage...");
		return true;
	}
	else{
		ROS_INFO("Deconnexion du subscriber...");
		if(--nbrSubscribers <= 0){
			// Il ne reste plus qu'un seul subscriber, donc on peut supprimer sans risque
			if(running){
				CPhidget_close((CPhidgetHandle)phid);
				running = false;
			}
			
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
    ros::init(argc, argv, "D_IMU");
	ros::NodeHandle l_n;
	n = &l_n;
	
	ros::ServiceServer service = n->advertiseService("D_IMU/params", newConfReceived);

    ros::Rate l_loop_rate = ros::Rate(20.);
	// Ok, c'est epouvantable, mais je vois pas comment faire autrement
	loop_rate = &l_loop_rate;

    if (attach(phid)) {
        initialised = true;

        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate->sleep();
        }

        disconnect(phid);
    }
    return 0;
}


