/*
 * Copyright © 2015  TELECOM Nancy
 *
 * This file is part of wifi ROS module.
 * wifi ROS module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * wifi ROS module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with wifi ROS module.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _WIFI_DRIVER_H_
#define _WIFI_DRIVER_H_

#include <ros/ros.h>
#include <vector>
#include <string>

namespace wifi_driver
{

   
	class WifiDriver
	{
	public:
	
		WifiDriver(ros::NodeHandle node,
				ros::NodeHandle private_nh);
		~WifiDriver() {}
	
		void poll(void);
	
	private:
		
		ros::Publisher output_;     // object to publish ROS messages
		std::string port;           // WiFi interface name
	
	    /** Read the output of a process
	     *
	     * @param fd file descriptor of the process output
	     * @param list output as a list of string
	     *
	     * @return true if success
	     */
		bool readPipe(int fd, std::vector<std::string> &list);
		
	};

} // namespace wifi_driver

#endif // _WIFI_DRIVER_H_
