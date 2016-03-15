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

#include "save.h"


namespace gps_save
{
	GpsSave::GpsSave(ros::NodeHandle node, ros::NodeHandle private_nh)
	{ 
	    // get parameters
		private_nh.param("outputDir", dirPath, std::string("~/capture/"));
		private_nh.param("maxsize", maxSize, 1073741824);

        // listen to GPS data
		gps_data_ =
				node.subscribe("gps_data", 1,
						&GpsSave::processData, (GpsSave *) this,
						ros::TransportHints().tcpNoDelay(true));
	
	}
	
	GpsSave::~GpsSave() {
	    // Close and flush the save file
		ofstreamBin.close();
	}
	
	/** @brief Data message callback */
	void GpsSave::processData(const gps_msgs::GpsData::ConstPtr &dataMsg)
	{
	    // Data structure to store GPS data
	    // Don't need to create it at each callback -> static
		static BinaryData binaryData;
	
		binaryData.stamp = dataMsg->header.stamp.toNSec();
		binaryData.latitude = dataMsg->latitude;
		binaryData.longitude = dataMsg->longitude;
		binaryData.seaHeight = dataMsg->seaHeight;
		binaryData.geoidHeight = dataMsg->geoidHeight;
		binaryData.positionDilutionOfPrecision = dataMsg->positionDilutionOfPrecision;
		binaryData.horizontalDilutionOfPrecision = dataMsg->horizontalDilutionOfPrecision;
		binaryData.verticalDilutionOfPrecision = dataMsg->verticalDilutionOfPrecision;
		binaryData.magneticVariation = dataMsg->magneticVariation;
		binaryData.trueCourseOverGround = dataMsg->trueCourseOverGround;
		binaryData.magneticCourseOverGround = dataMsg->magneticCourseOverGround;
		binaryData.speedOverGround = dataMsg->speedOverGround;
		binaryData.estimatedHorizontalPosError = dataMsg->estimatedHorizontalPosError;
		binaryData.estimatedVerticalPosError = dataMsg->estimatedVerticalPosError;
		binaryData.estimatedPosError = dataMsg->estimatedPosError;
		binaryData.trueEstVelocity = dataMsg->trueEstVelocity;
		binaryData.trueNorthVelocity = dataMsg->trueNorthVelocity;
		binaryData.upVelocity = dataMsg->upVelocity;
		binaryData.gpsQuality = dataMsg->gpsQuality;
		binaryData.nbSat = dataMsg->nbSat;
		binaryData.gsaModeAuto = dataMsg->gsaModeAuto;
		binaryData.fixType = dataMsg->fixType;
		binaryData.utcHour = dataMsg->utcHour;
		binaryData.utcMin = dataMsg->utcMin;
		binaryData.utcSec = dataMsg->utcSec;
		binaryData.validPosition = dataMsg->validPosition;
		binaryData.utcDay = dataMsg->utcDay;
		binaryData.utcMounth = dataMsg->utcMounth;
		binaryData.utcYear = dataMsg->utcYear;
		binaryData.mode = dataMsg->mode;
		for(int i = 0; i < 12; i++)
			binaryData.prnUsed[i] = dataMsg->prnUsed[i];
	
		writeBinary(binaryData);
	}
	
	/** Write GPS data in a save file
	 *
	 * @param binaryData reference to data structure
	 */
	void GpsSave::writeBinary(const BinaryData &binaryData)
	{
		static int fileSize = maxSize;
		fileSize += sizeof(BinaryData);
	    
	    // Check if we have to make another file
		if(fileSize > maxSize) {
		    // Make the new filepath
			std::ostringstream path;
			path.clear();
			path << dirPath << "GPS_" << ros::Time::now() << ".bin";
			ofstreamBin.close();
			ofstreamBin.open(path.str().c_str(), std::ios::out|std::ios::binary|std::ios::app);
	
			fileSize = sizeof(BinaryData);
		}
	    
	    // append data structure to the file
		ofstreamBin.write((char*) &binaryData, sizeof(BinaryData));
	}

} 
