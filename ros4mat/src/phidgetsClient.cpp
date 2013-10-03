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
#include <ros4mat/M_IMU.h>

void dataIMUReceived(const ros4mat::M_IMU::ConstPtr& msg)
{
	ROS_INFO("Reception de donnees de l'IMU");
	ROS_INFO("Acceleration> x: %6f  y: %6f  x: %6f\n", msg->aX[0], msg->aY[0], msg->aZ[0]);
	ROS_INFO("Angular Rate> x: %6f  y: %6f  x: %6f\n", msg->rX[0], msg->rY[0], msg->rZ[0]);
	ROS_INFO("Magnetic Field> x: %6f  y: %6f  x: %6f\n", msg->mX[0], msg->mY[0], msg->mZ[0]);
	ROS_INFO("Timestamp> %8f\n", msg->timestamps[0]);
	ROS_INFO("Fin de la transmission");
}


int main(int argc, char **argv)
{
	// Initialisation
  ros::init(argc, argv, "lecteurIMU");

  ros::NodeHandle n;

	// On indique apres qui on attend
  ros::Subscriber sub = n.subscribe("/D_IMU/data", 1000, dataIMUReceived);

// Boucle tant qu'il y a des messages
  ros::spin();

  return 0;
}


