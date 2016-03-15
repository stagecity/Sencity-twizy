/*
 * Copyright © 2015  TELECOM Nancy
 *
 * This file is part of velodyne-savedata ROS module.
 * velodyne-savedata ROS module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * velodyne-savedata ROS module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with velodyne-savedata ROS module.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include "savedata.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "savedata_node");
	ros::NodeHandle node;
	ros::NodeHandle private_nh("~");

	velodyne_savedata::Savedata save(node, private_nh);

	ros::spin();

	return 0;
}
