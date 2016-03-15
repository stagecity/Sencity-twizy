/*
 * Copyright © 2015  TELECOM Nancy
 *
 * This file is part of gps ROS module.
 * gps ROS module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * gps ROS module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gps ROS module.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _GPS_DRIVER_H_
#define _GPS_DRIVER_H_

#include <string.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <vector>

namespace gps_driver
{
	
	class GpsDriver
	{
	public:
	
		GpsDriver(ros::NodeHandle node,
				ros::NodeHandle private_nh);
		~GpsDriver() {}
	
		bool poll(void);
	
	private:
		ros::Publisher output_;     // ROS message publisher
		serial::Serial serial_;     // Serial object to communicate with the GPS
		std::string port;           // Unix serial port of the GPS (/dev/ttyACM0)
		int baud;                   // Serial speed
		std::string frame_id;       // TF name
		
		/** Check integrity of GPS NMEA messages
		 *
		 * @param nmeaSentence NMEA messages
		 * @param sumString Checksum of the NMEA message to check integrity
		 * @return true if integrity ok, false otherwise.
		 */
		bool checksum(const std::string &nmeaSentence,const std::string & sumString);
		
		/** Split a string with a delimiter
		 * 
		 * @param str the string to split
		 * @param delimiter the delimiter to use
		 * 
		 * @return vector of sub-string
		 */
		std::vector<std::string> split(std::string str, char delimiter);
	
	
	};

} // namespace gps_driver

#endif // _GPS_DRIVER_H_
