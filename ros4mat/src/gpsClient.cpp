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
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>

void dataGpsReceived(const gps_common::GPSFix::ConstPtr& msg)
{	
	ROS_INFO("Reception de donnees du GPS at %6f", msg->header.stamp.toSec());
	ROS_INFO("Latitude : %6f", msg->latitude);
	ROS_INFO("Longitude : %6f", msg->longitude);
	ROS_INFO("Altitude : %6f", msg->altitude);
	ROS_INFO("Module de la vitesse : %6f", msg->speed);
	ROS_INFO("Angle de la vitesse : %6f", msg->track);
	ROS_INFO("Nombre de satellites utilises : %d", msg->status.satellites_used);
	ROS_INFO("Reception terminee\n");
}


int main(int argc, char **argv)
{
	// Initialisation
  ros::init(argc, argv, "lecteurGPS");

  ros::NodeHandle n;
        //Package_Logico::S_GPS lParamsSetGps;

	// On indique apres qui on attend
	/*lParamsSetGps.request.subscribe = true;
	lParamsSetGps.request.gpsFreqAcq = 1;
	lParamsSetGps.request.gpsBufferSize = 300;     // ?!?
	if(!ros::service::call("/D_GPS/params", lParamsSetGps)){
			ROS_ERROR("Le service de parametrage du GPS a renvoye une erreur (code %d).", lParamsSetGps.response.ret);
			return -1;
	}*/
	
  ros::Subscriber sub = n.subscribe("/extended_fix", 300, dataGpsReceived);

// Boucle tant qu'il y a des messages
  ros::spin();

  return 0;
}


