/*
 * Copyright © 2015  TELECOM Nancy
 *
 * This file is part of imu ROS module.
 * imu ROS module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * imu ROS module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with imu ROS module.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "save.h"


namespace imu_save
{
	ImuSave::ImuSave(ros::NodeHandle node, ros::NodeHandle private_nh)
	{ 
	    // get parameters
		private_nh.param("outputDir", dirPath, std::string("~/capture/"));
		private_nh.param("maxsize", maxSize, 1073741824);

        // listen to imu messages
		imu_data_ =
				node.subscribe("imu/data_raw", 3,
						&ImuSave::processData, (ImuSave *) this,
						ros::TransportHints().tcpNoDelay(true));
						
		imu_mag_ =
				node.subscribe("imu/mag", 3,
						&ImuSave::processDataMag, (ImuSave *) this,
						ros::TransportHints().tcpNoDelay(true));
		binaryData.stamp = 0;
	
	}
	
	ImuSave::~ImuSave() {
	    // close the file
		ofstreamBin.close();
	}
	
	
	void ImuSave::processData(const sensor_msgs::Imu::ConstPtr &dataMsg)
	{
	    binaryData.gyro[0] = dataMsg->angular_velocity.x;
	    binaryData.gyro[1] = dataMsg->angular_velocity.y;
	    binaryData.gyro[2] = dataMsg->angular_velocity.z;
	    
	    binaryData.accel[0] = dataMsg->linear_acceleration.x;
	    binaryData.accel[1] = dataMsg->linear_acceleration.y;
	    binaryData.accel[2] = dataMsg->linear_acceleration.z;
	
	    // Check if Compass data is already seted
		if(binaryData.stamp == dataMsg->header.stamp.toNSec())
			writeBinary();
		else
			binaryData.stamp = dataMsg->header.stamp.toNSec();
		
	}
	
	void ImuSave::processDataMag(const sensor_msgs::MagneticField::ConstPtr &dataMsg) {
		
	    binaryData.mag[0] = dataMsg->magnetic_field.x;
	    binaryData.mag[1] = dataMsg->magnetic_field.y;
	    binaryData.mag[2] = dataMsg->magnetic_field.z;
	
	    // Check if IMU data is already seted
		if(binaryData.stamp == dataMsg->header.stamp.toNSec())
			writeBinary();
		else
			binaryData.stamp = dataMsg->header.stamp.toNSec();
		
	}
	
	
	
	void ImuSave::writeBinary()
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
			path << dirPath << "IMU_" << ros::Time::now() << ".bin";
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
