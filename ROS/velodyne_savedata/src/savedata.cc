/*
 * Copyright © 2015  TELECOM Nancy
 *
 * This file is part of velodyne-savedata ROS module.
 * velodyne-savedata ROS module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * velodyne-savedata ROS module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with velodyne-savedata ROS module.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "savedata.h"


namespace velodyne_savedata
{
	/** @brief Constructor. */
	Savedata::Savedata(ros::NodeHandle node, ros::NodeHandle private_nh)
	{ 
	    // Get parameters
		private_nh.param("outputDir", dirPath, std::string("~/capture/"));
		private_nh.param("maxsize", maxSize, 1073741824);
		// subscribe to VelodyneScan packets
		velodyne_scan_ =
				node.subscribe("velodyne_points", 10,
						&Savedata::processScan, (Savedata *) this,
						ros::TransportHints().tcpNoDelay(true));
	
	}
	
	Savedata::~Savedata() {
	    // close and flush to save file.
		ofstreamBin.close();
	}
	
	/** @brief Callback for velodyne scan messages. */
	void Savedata::processScan(const sensor_msgs::PointCloud2::ConstPtr &pcMsg)
	{
		VPointCloud cloud;      // cloud of points
		pcl::fromROSMsg (*pcMsg, cloud);    // convert point_cloud from ROS message to PointCloudLibrary format
		// Allocation of an array of Points
		Point *pts = (Point*) malloc(cloud.width * sizeof(Point));  
		VPoint point;
		// For each points
		for (uint32_t i = 0; i < cloud.width; ++i)
		{
			point = cloud.at(i);
			pts[i].x = point.x;
			
			pts[i].y = point.y;
			
			pts[i].z = point.z;
			
			pts[i].intensity = point.intensity;
			
			pts[i].stamp = ros::Time(point.stamp).toNSec();
			
			pts[i].ring = point.ring;
			
		}

        // write data to file
		writeBinaryPacket(pts, pcMsg->width);
	
		free(pts);
	}
	
	/** Write points into the save file
     * 
     * @param pts array of Points
     * @param size number of Points
     */
	void Savedata::writeBinaryPacket(const Point* const pts, const int size)
	{
	    // Initialisation at maximum file size
	    // in order to force creation of file
		static int fileSize = maxSize;
		int scanSize = sizeof(Point) * size;
		// at each call we increase a size indicator
		fileSize += scanSize;
	
	    // if we reach the maximum
		if(fileSize > maxSize) {
		    // Get a new file name and the file path
			std::ostringstream path;
			path.clear();
			path << dirPath << "VLP-16_" << ros::Time::now() << ".bin";
			ofstreamBin.close();
			// Create the output stream
			ofstreamBin.open(path.str().c_str(), std::ios::out|std::ios::binary|std::ios::app);
			// reset the size indicator
			fileSize = scanSize;
		}
		
		// write the data to the file
		ofstreamBin.write((char*) pts, scanSize);
	}

} // namespace velodyne_savedata
