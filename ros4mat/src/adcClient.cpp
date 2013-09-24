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
#include <ros4mat/M_ADC.h>

void dataAdcReceived(const ros4mat::M_ADC::ConstPtr& msg)
{	
	ROS_INFO("Reception de donnees de l'ADC (%d samples)", (int)msg->timestamps.size());
    for (int i = 0; i < msg->timestamps.size(); i++) {
        ROS_INFO("[Timestamp] Canal 0, Canal 1...");
        ROS_INFO("[%6f] %3f, %3f, %3f, %3f, %3f, %3f", msg->timestamps[i], msg->canal1[i], msg->canal2[i], msg->canal3[i], msg->canal4[i], msg->canal5[i], msg->canal6[i]);
    }	
	ROS_INFO("Fin de la transmission\n\n");
}


int main(int argc, char **argv)
{
	// Initialisation
  ros::init(argc, argv, "lecteurADC");

  ros::NodeHandle n;

	// Specify the data we are waiting for
  ros::Subscriber sub = n.subscribe("D_ADC/data", 1000, dataAdcReceived);

  // Loop while there are messages
  ros::spin();

  return 0;
}


