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

#ifndef _GPS_SAVE_H_
#define _GPS_SAVE_H_ 

#include <ros/ros.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <gps_msgs/GpsData.h>

namespace gps_save
{
    typedef struct {
        uint64_t stamp;         /**< ROS timestamp in nano seconds */
        double latitude;        /**< GPS latitude in degrees */
        double longitude;       /**< GPS longitude in degrees */
        float seaHeight;        /**< Height above sea in meters */
        float geoidHeight;      /**< Geoidal height in meters */
        float positionDilutionOfPrecision;      /**< Position dilution of precision */
        float horizontalDilutionOfPrecision;    /**< Horizontal dilution of precision */
        float verticalDilutionOfPrecision;      /**< Vertical dilution of precision */
        float magneticVariation;                /**< Magnetic variation of direction in degrees*/
        float trueCourseOverGround;             /**< true course over ground in degrees */
        float magneticCourseOverGround;         /**<  magnetic course over ground in degrees */
        float speedOverGround;                  /**< speed over ground in kilometers/hour */
        float estimatedHorizontalPosError;      /**< Estimated horizontal position error in meters */
        float estimatedVerticalPosError;        /**< Estimated vertical position error in meters */
        float estimatedPosError;        /**< Estimated position error in meters */
        float trueEstVelocity;          /**< True est velocity in meters/second */
        float trueNorthVelocity;        /**< True north velocity in meters/second */
        float upVelocity;       /**< Up velocity in meters/second */
        uint8_t gpsQuality;     /**< GPS quality :GPS quality indication, 0 = fix not available, 1 = Non-differential GPS fix available, 2 = Differential GPS (WAAS) fix available, 6 = Estimated */
        uint8_t nbSat;          /**< Number of satellite in view */
        uint8_t gsaModeAuto;    /**< Automatic mode */
        uint8_t fixType;        /**< Fix type, 1 = not available, 2 = 2D, 3 = 3D */
        uint8_t utcHour;        /**< UTC hour */
        uint8_t utcMin;         /**< UTC minute */
        uint8_t utcSec;         /**< UTC second */
        uint8_t validPosition;  /**< is position valid : boolean  */
        uint8_t utcDay;         /**< UTC day */
        uint8_t utcMounth;      /**< UTC mounth */
        uint8_t utcYear;        /**< UTC Year */
        uint8_t mode;           /**< Mode indicator : A = Autonomous, D = Differential, E = Estimated, N = Data not valid */
        uint8_t prnUsed[12];    /**< PRN number, 01 to 32, of satellite used in solution, up to 12 transmitted */
    } BinaryData;
	
	class GpsSave
	{
	public:
	
		GpsSave(ros::NodeHandle node, ros::NodeHandle private_nh);
		~GpsSave();
	
	private:
		std::string dirPath;    // Directory where to save data
		int maxSize;            // Maximum size of save file
		std::ofstream ofstreamBin;      // Output stream
		ros::Subscriber gps_data_;      // Parsed GPS data listener
		
		/** @brief Data message callback */
		void processData(const gps_msgs::GpsData::ConstPtr &dataMsg);
		
		/** Write GPS data in a save file
		 *
		 * @param binaryData reference to data structure
		 */
		void writeBinary(const BinaryData &binaryData);
	};

} 

#endif 
