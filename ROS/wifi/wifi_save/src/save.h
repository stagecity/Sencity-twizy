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

#ifndef _WIFI_SAVE_H_
#define _WIFI_SAVE_H_ 

#include <ros/ros.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <wifi_msgs/WifiScan.h>

// Maximum length of MAC address string
#define ADDRESS_SIZE 	18
// Maximum length of SSID string
#define SSID_SIZE 		30
// Maximum length of security string
#define SECURITY_SIZE	50
// Maximum length of groupCipher string
#define GROUP_SIZE 		20
// Maximum length of pairwiseCiphers string
#define PAIR_SIZE 		20
// Maximum length of WiFi protocol string
#define PROTOCOL_SIZE	20

namespace wifi_save
{
    /**
     * @brief WiFi binary data from ROS WiFi module
     *
     *
     */
    typedef struct {
        uint64_t stamp;                 /**< ROS timestamp in nano seconds */
        uint16_t frequency;             /**<  WiFi frequency (channel) in MHz */
        char address[ADDRESS_SIZE];     /**< WiFi access point MAC address */
        char ssid[SSID_SIZE];           /**< WiFi SSID */
        char security[SECURITY_SIZE];   /**< WiFi security protocol */
        char groupCipher[GROUP_SIZE];   /**< WiFi security group cipher */
        char pairwiseCiphers[PAIR_SIZE];/**< WiFi security pair ciphers */
        char protocol[PROTOCOL_SIZE];   /**< WiFi protocol */
        uint8_t signal;                 /**< signal strengh form 0 to 100(highest) */
        uint8_t encryption;             /**< is WiFi encrypted (true is non null) */
    } BinaryData;

	
	class WifiSave
	{
	public:
	
		WifiSave(ros::NodeHandle node, ros::NodeHandle private_nh);
		~WifiSave();
	
	private:
	    // Output directory path
		std::string dirPath;
		// Maximum size of stored files
		int maxSize;
		// Structure saved in file
		BinaryData binaryData;
		// Output stream (write in file)
		std::ofstream ofstreamBin;
		// Listener object to WiFi messages
		ros::Subscriber wifi_scan_; 
		
		/** @brief Data message callback */
		void processData(const wifi_msgs::WifiScan::ConstPtr &scanMsg);
		/** @brief Write WiFi data in a save file */
		void writeBinary();
	};

} 

#endif 
