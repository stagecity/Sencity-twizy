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

#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_

#include <ros/ros.h>
#include <serial/serial.h>

#define SERIAL_HEADER 0xA5

namespace imu_driver
{
	struct Int16Vect3D {
        int16_t x;  /**< x axis of the vector */
        int16_t y;  /**< y axis of the vector */
        int16_t z;  /**< z axis of the vector */
    };
    

	class ImuDriver
	{
	public:
	
		ImuDriver(ros::NodeHandle node,
				ros::NodeHandle private_nh);
		~ImuDriver() {}
	    /** poll the device
	     *
	     *  @returns true unless end of file reached
	     */
		bool poll(void);
	
	private:
		// Object to pulish ROS messages
		ros::Publisher output_, outputMag_;
		// Object to read/write serial
		serial::Serial serial_;
		// Serial port (like "/dev/ttyACM0")
		std::string port;
		// Serial Speed (bauds)
		int baud;
		// TF id
		std::string frame_id;
		/* 
		 * Coefficient to convert gyroscope data
		 * from degrees/unit to radian
		 */
		float degToRadCoef;
	    /* 
		 * Coefficient to convert accelerometer data
		 * from g/unit to m/s²
		 */
		float gToSI;
		/* 
		 * Coefficient to convert compass data
		 * from µTesla/unit to Tesla
		 */
		float magCoef;
		
		/*
		 * Data stucture sent over serial by Arduino
		 */
		union {
		    struct {
		      struct Int16Vect3D accel;     /**< Accelerometer data */
		      struct Int16Vect3D gyro;      /**< Gyroscope data */
		      struct Int16Vect3D mag;       /**< Compass data */
		      int16_t temp;                 /**< temperature */
		    };
		    uint8_t value[20];  // Structure as a 20 byte array
		} data;

        // Contain checksum get from serial
		uint8_t sum;
		
		/* @brief Check the checksum
		 *
		 * @return true is checksum valid
		 */
		bool checksum();
	
	};

} // namespace imu_driver

#endif // _IMU_DRIVER_H_
