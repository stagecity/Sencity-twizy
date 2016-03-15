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

#include <math.h>
#include "save.h"


namespace wifi_save
{
	WifiSave::WifiSave(ros::NodeHandle node, ros::NodeHandle private_nh)
	{ 
	    // Get parameters
		private_nh.param("outputDir", dirPath, std::string("~/capture/"));
		private_nh.param("maxsize", maxSize, 1073741824);

        // Listen to WiFi messages
		wifi_scan_ =
				node.subscribe("wifi/data", 3,
						&WifiSave::processData, (WifiSave *) this,
						ros::TransportHints().tcpNoDelay(true));
				
	    // Initialize structure timestamp		
		binaryData.stamp = 0;
	
	}
	
	WifiSave::~WifiSave() {
	    // Close and flush output stream
		ofstreamBin.close();
	}
	
	/** @brief Data message callback */
	void WifiSave::processData(const wifi_msgs::WifiScan::ConstPtr &scanMsg)
	{
	    // Set timestamp
		binaryData.stamp = scanMsg->header.stamp.toNSec();
		
		// Write each scan in file
		for(int i = 0; i < scanMsg->data.size(); ++i) {
			wifi_msgs::WifiData wifiData = scanMsg->data.at(i);
			
			// Convert frequency from float (GHz) to int (MHz)
			binaryData.frequency = (uint16_t) round(wifiData.frequency * 1000.f);
			binaryData.signal = wifiData.signal;
			binaryData.encryption = (wifiData.encryption) ? 1 : 0;
			std::strncpy(binaryData.address, wifiData.address.c_str(), ADDRESS_SIZE);
			std::strncpy(binaryData.ssid, wifiData.ssid.c_str(), SSID_SIZE);
			std::strncpy(binaryData.security, wifiData.security.c_str(), SECURITY_SIZE);
			std::strncpy(binaryData.groupCipher, wifiData.groupCipher.c_str(), GROUP_SIZE);
			std::strncpy(binaryData.pairwiseCiphers, wifiData.pairwiseCiphers.c_str(), PAIR_SIZE);
			std::strncpy(binaryData.protocol, wifiData.protocol.c_str(), PROTOCOL_SIZE);
			
			writeBinary();
		}		
	}
	
	/** @brief Write WiFi data in a save file */
	void WifiSave::writeBinary()
	{
	    // Initialisation at maximum file size
	    // in order to force creation of file
		static int fileSize = maxSize;
		// at each call we increase a size indicator
		fileSize += sizeof(BinaryData);
	
	    // if we reach the maximum
		if(fileSize > maxSize) {
		    // Get a new file name and the file path
			std::ostringstream path;
			path.clear();
			path << dirPath << "WIFI_" << ros::Time::now() << ".bin";
			ofstreamBin.close();
			// Create the output stream
			ofstreamBin.open(path.str().c_str(), std::ios::out|std::ios::binary|std::ios::app);
	
	        // reset the size indicator
			fileSize = sizeof(BinaryData);
		}
	
	    // write the data to the file
		ofstreamBin.write((char*) &binaryData, sizeof(BinaryData));
	}
	
} 
