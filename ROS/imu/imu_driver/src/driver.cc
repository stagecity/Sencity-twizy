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

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <math.h>
#include "driver.h"

namespace imu_driver
{
	
	ImuDriver::ImuDriver(ros::NodeHandle node,
			ros::NodeHandle private_nh)
	{
		private_nh.param("frame_id", frame_id, std::string("imu"));
		std::string tf_prefix = tf::getPrefixParam(private_nh);
		frame_id = tf::resolve(tf_prefix, frame_id);
	
		// use private node handle to get parameters
		private_nh.param("baud", baud, 1000000);
		private_nh.param("port", port, std::string("/dev/ttyACM0"));
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
		output_ = node.advertise<sensor_msgs::Imu>("imu/data_raw", 3);
		outputMag_ = node.advertise<sensor_msgs::MagneticField>("imu/mag", 3);
    
        /* 
		 * Coefficient to convert gyroscope data
		 * from degrees/unit to radian
		 */
		degToRadCoef = (M_PI * 500.0)/(65535.0 *180.0);
		/* 
		 * Coefficient to convert accelerometer data
		 * from g/unit to m/s²
		 */
		gToSI = (8. * 9.80665) / 65535.0;
		/* 
		 * Coefficient to convert compass data
		 * from µTesla/unit to Tesla
		 */
		magCoef = (4912. / 32760.) * 1e-6 ;
	}
	
	/** poll the device
	 *
	 *  @returns true unless end of file reached
	 */
	bool ImuDriver::poll(void)
	{
		// Allocate a new shared pointer for zero-copy sharing with other nodelets.
		sensor_msgs::ImuPtr imuMsg(new sensor_msgs::Imu);
		sensor_msgs::MagneticFieldPtr magMsg(new sensor_msgs::MagneticField);
		
		// Read serial while we don't get
		// the header of the data structure
		uint8_t tmp;
		do {
			serial_.read(&tmp ,1);
		} while(tmp != SERIAL_HEADER);

        // set Frame id of the message
		imuMsg->header.frame_id = frame_id;
		// We get the message now
		imuMsg->header.stamp = ros::Time::now();
		magMsg->header.frame_id = frame_id;
		// Same time as imuMsg
		magMsg->header.stamp = imuMsg->header.stamp;
		
		// Read the data structure and the checksum
		try {
			serial_.read(&(data.value[0]), 20);
			serial_.read(&sum, 1);
		} catch(serial::IOException& e) {
			return false;
		} catch(serial::PortNotOpenedException& e) {
			return false;
		}
		
		// A bad checksum might be a serial error
		if(!checksum())
		    return true;
		
        // Define covariance, as defined in sensor_msgs::Imu
		imuMsg->orientation_covariance[0] = -1.0;	// no orientation
		imuMsg->orientation_covariance[4] = 10000000000000.0;
		imuMsg->orientation_covariance[8] = 10000000000000.0;


        // Set data in messages in respect of REP 103
		imuMsg->angular_velocity.x = data.gyro.x * degToRadCoef;
		imuMsg->angular_velocity.y = data.gyro.y * degToRadCoef;
		imuMsg->angular_velocity.z = data.gyro.z * degToRadCoef;
		imuMsg->angular_velocity_covariance[0] = 0;
		imuMsg->angular_velocity_covariance[4] = 0;
		imuMsg->angular_velocity_covariance[8] = 0;

		imuMsg->linear_acceleration.x = gToSI * data.accel.x;
		imuMsg->linear_acceleration.y = gToSI * data.accel.y;
		imuMsg->linear_acceleration.z = gToSI * data.accel.z;
		imuMsg->linear_acceleration_covariance[0] = 0;
		imuMsg->linear_acceleration_covariance[4] = 0;
		imuMsg->linear_acceleration_covariance[8] = 0;

		magMsg->magnetic_field.x = magCoef * data.mag.y;
		magMsg->magnetic_field.y = magCoef * data.mag.x;
		magMsg->magnetic_field.z = -(magCoef * data.mag.z);
		
		magMsg->magnetic_field_covariance[0] = 0.;
		magMsg->magnetic_field_covariance[4] = 0.;
		magMsg->magnetic_field_covariance[8] = 0.;

        // Publish data
		output_.publish(imuMsg);
		outputMag_.publish(magMsg);
	
		return true;
	}
	
	bool ImuDriver::checksum() {
		uint8_t checksum = 0;
	    
	    // Perform xor
		for(uint8_t i = 0; i < 20; i++) {
			checksum ^= data.value[i];
		}
		return checksum == sum;
	
	}
} // namespace imu_driver
