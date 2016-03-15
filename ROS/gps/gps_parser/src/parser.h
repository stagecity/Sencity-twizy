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

#ifndef _GPS_PARSER_H_
#define _GPS_PARSER_H_

#include <string>
#include <ros/ros.h>
#include <vector>
#include <gps_msgs/GpsSentences.h>
#include <gps_msgs/GpsData.h>

namespace gps_parser
{
	
	class GpsParser
	{
	public:
	
		GpsParser(ros::NodeHandle node,
				ros::NodeHandle private_nh);
		~GpsParser() {}
	
	private:
		ros::Subscriber gps_sentences_;     // nmea sentenes listener object
		ros::Publisher output_;             // parsed data publisher
		std::string frame_id;               // tf id
		
		/** Convert a string into float
		 * take care of unwanted error with Not A Number
		 *
		 * @param str float as a string
		 */
		float safeFloat(std::string str);
		
		/** Convert a string into double
		 * take care of unwanted error with Not A Number
		 *
		 * @param str double as a string
		 */
		double safeDouble(std::string str);
		
		/** Convert a string into integer
		 * take care of unwanted error with 0
		 *
		 * @param str integer as a string
		 */
		int safeInt(std::string str);
		
		
		/** Split a string in tokens with a delimiter
		 * 
		 * @param str the string to split
		 * @param delimiter the delimiter to use
		 */
		std::vector<std::string> split(std::string str, char delimiter);
		
		/** @brief Callback for raw gps messages. */
		void processScan(const gps_msgs::GpsSentences::ConstPtr &gpsSentencesMsg);
	
	};

} // namespace gps_driver

#endif // _GPS_DRIVER_H_
