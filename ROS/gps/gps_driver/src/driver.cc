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

#include <ros/ros.h>
#include <gps_msgs/GpsSentences.h>
#include <tf/transform_listener.h>
#include "driver.h"

namespace gps_driver
{
	
	GpsDriver::GpsDriver(ros::NodeHandle node,
			ros::NodeHandle private_nh)
	{
		private_nh.param("frame_id", frame_id, std::string("gps"));
		std::string tf_prefix = tf::getPrefixParam(private_nh);
		frame_id = tf::resolve(tf_prefix, frame_id);
	
		// use private node handle to get parameters
		private_nh.param("baud", baud, 38400);
		private_nh.param("port", port, std::string("/dev/ttyUSB0"));
		// Try to set up the Serial communication
		try
		{
			serial_.setPort(port);
			serial_.setBaudrate(baud);
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			serial_.setTimeout(to);
			serial_.open();
		}
		catch (serial::IOException& e)
		{
			ROS_ERROR_STREAM("Unable to open port ");
		}
		if(serial_.isOpen()){
			ROS_INFO_STREAM("Serial Port initialized");
		}
	
		// raw data output topic
		output_ = node.advertise<gps_msgs::GpsSentences>("gps_sentences", 1);
	}
	
	/** poll the device
	 *
	 *  @returns true unless end of file reached
	 */
	bool GpsDriver::poll(void)
	{
		// Allocate a new shared pointer for zero-copy sharing with other nodelets.
		gps_msgs::GpsSentencesPtr sentences(new gps_msgs::GpsSentences);
        
		static std::string swp; // for reading serial
		static std::vector<std::string> tokens; // to split NMEA sentence and checksum
        
        // We configured GPS to write 6 NMEA sentences each second
		for (int i = 0; i < 6; ++i)
		{
		    // GPRMC is the first sentence send in each second
		    // So we wait since we get one
			do {
				try {
					swp = serial_.readline(82);
				} catch(serial::IOException& e) {
					return false;
				} catch(serial::PortNotOpenedException& e) {
					return false;
				}
	
			} while((i == 0) && (swp.find("$GPRMC") != 0));
	
	        // Each sentence end with '\r'
	        // We don't need it
			swp.erase(swp.find_last_of ("\r"));
	
	        // We get the NMEA sentence and the checksum
			tokens = split(swp, '*');   
	
	        // There should only be 2 tokens
			if(tokens.size() != 2)
				return true;
	
	        // Check the integrity
			if(!checksum(tokens.at(0), tokens.at(1)))
				return true;
			// Each sentence is sent in a defined order
			switch(i) {
			case 0:
			    // The time of this ROS message is the time of the first GPS sentence
				sentences->header.stamp = ros::Time::now();
				sentences->rmc = tokens.at(0);
				break;
			case 1:
				sentences->gga = tokens.at(0); 
				break;
			case 2:
				sentences->gsa = tokens.at(0);
				break;
			case 3:
				sentences-> pgrme = tokens.at(0);
				break;
			case 4:
				sentences->vtg = tokens.at(0);
				break;
			case 5:
				sentences->pgrmv = tokens.at(0);
				break;
			}   
		}
	
	
		sentences->header.frame_id = frame_id;
		output_.publish(sentences);
	
		return true;
	}
	
	/** Check integrity of GPS NMEA messages
	 *
	 * @param nmeaSentence NMEA messages
	 * @param sumString Checksum of the NMEA message to check integrity
	 * @return true if integrity ok, false otherwise.
	 */
		 
	bool GpsDriver::checksum(const std::string &nmeaSentence,const std::string & sumString) {
	
		const char* nmea = nmeaSentence.c_str();
	
		int checksum = 0;
	       
	    // Sum is a XOR between each byte
		for(int i = 1; i < nmeaSentence.length(); i++) {
			checksum ^= nmea[i];
		}
	
	    // Convert the provided sum in integer
		int sum;   
		std::stringstream ss;
		ss << std::hex << sumString;
		ss >> sum;
	
		return checksum == sum;
	
	}
	
	/** Split a string with a delimiter
	 * 
	 * @param str the string to split
	 * @param delimiter the delimiter to use
	 * 
	 * @return vector of sub-string
	 */
	//http://code.runnable.com/VHb0hWMZp-ws1gAr/splitting-a-string-into-a-vector-for-c%2B%2B
	std::vector<std::string> GpsDriver::split(std::string str, char delimiter) {
		std::vector<std::string> internal;
		std::stringstream ss(str); // Turn the string into a stream.
		std::string tok;
	
		while(std::getline(ss, tok, delimiter)) {
			internal.push_back(tok);
		}
	
		return internal;
	}
} // namespace gps_driver
