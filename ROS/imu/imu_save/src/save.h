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

#ifndef _IMU_SAVE_H_
#define _IMU_SAVE_H_ 

#include <ros/ros.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
namespace imu_save
{
    /* @brief Binary data saved in file */
	typedef struct {
    uint64_t stamp;     /**< ROS timestamp in nanoseconds */
    double accel[3];    /**< 3D accelerometer vector. Composant in meter/second^2 (SI); 0:x - 1:y - 2:z */
    double gyro[3];     /**< 3D gyroscope vector. Composant in rad/second; 0:x - 1:y - 2:z  */
    double mag[3];      /**< 3D magnetometer vector. Composant in Tesla; 0:x - 1:y - 2:z  */
	} BinaryData;
	
	class ImuSave
	{
	public:
	
		ImuSave(ros::NodeHandle node, ros::NodeHandle private_nh);
		~ImuSave();
	
	private:
		std::string dirPath;            // Directory where to save data
		int maxSize;                    // Maximum size of save file
		BinaryData binaryData;          // Structure writed to file
		std::ofstream ofstreamBin;      // Output stream
		ros::Subscriber imu_data_, imu_mag_;    // ROS messages listeners
		
		/** @brief IMU Data message callback */
		void processData(const sensor_msgs::Imu::ConstPtr &dataMsg);
		/** @brief Compass Data message callback */
		void processDataMag(const sensor_msgs::MagneticField::ConstPtr &dataMsg);
		/** Write IMU data in a save file */
		void writeBinary();
		
	};

} 

#endif 
