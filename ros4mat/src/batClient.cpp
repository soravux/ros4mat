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
#include <ros4mat/M_Battery.h>

void dataBatReceived(const ros4mat::M_Battery::ConstPtr& msg)
{	
	ROS_INFO("Reception de donnees de la batterie at %6f", msg->timestamp);
	ROS_INFO("Courant circulant de/vers la batterie : %5f A", msg->current);
	ROS_INFO("Tension totale de la batterie : %4f V", msg->vtotal);
	ROS_INFO("Tension des cellules :");
    for (int i = 0; i < 6; i++) {
        ROS_INFO("Cellule %i : %4f V", i+1, msg->vcell[i]);
    }
	ROS_INFO("Etat de la batterie :");
	ROS_INFO("Charge : %d / Decharge : %d / Prog : %d / Over Temp. : %d / Undervoltage : %d / Overvoltage : %d / Overcurrent : %d / Short : %d", msg->batteryState[0], msg->batteryState[1], msg->batteryState[2], msg->batteryState[3], msg->batteryState[4], msg->batteryState[5], msg->batteryState[6], msg->batteryState[7]);
	ROS_INFO("Fin de la transmission\n\n");
}


int main(int argc, char **argv)
{
	// Initialisation
  ros::init(argc, argv, "lecteurBatterie");

  ros::NodeHandle n;

	// On indique apres qui on attend
  ros::Subscriber sub = n.subscribe("D_Battery/data", 100, dataBatReceived);

// Boucle tant qu'il y a des messages
  ros::spin();

  return 0;
}


