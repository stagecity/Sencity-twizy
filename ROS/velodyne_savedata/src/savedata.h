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

#ifndef _VELODYNE_SAVEDATA_Savedata_H_
#define _VELODYNE_SAVEDATA_Savedata_H_ 1

#include <ros/ros.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl_ros/point_cloud.h>
namespace velodyne_savedata
{	

	typedef velodyne_pointcloud::PointXYZIR VPoint;
	typedef pcl::PointCloud<VPoint> VPointCloud;
	/** Point structure
	 * this structure is stored for each point in a file    
	 */
    typedef struct {
        uint64_t stamp;     /**< ROS timestamp in nano seconds */
        float x;            /**< Forward (relative to car), in meter from LIDAR center */
        float y;            /**< Left (relative to car), in meter from LIDAR center */
        float z;            /**< Up (relative to car), in meter from LIDAR center */
        uint8_t ring;       /**< Velodyne laser number */
        uint8_t intensity;  /**< Laser intensity measured */
    } __attribute__((packed)) Point;
	
	class Savedata
	{
	public:
	
		Savedata(ros::NodeHandle node, ros::NodeHandle private_nh);
		~Savedata();
	
	private:
	    // Path of directory where to save data
		std::string dirPath;
		// Maximum file size of stored data
		int maxSize;
		// Output stream (write file)
		std::ofstream ofstreamBin;
		// Velodyne scan message listener
	    ros::Subscriber velodyne_scan_;  
		// Messages callback
		void processScan(const sensor_msgs::PointCloud2::ConstPtr &pcMsg);
		/** Write points into the save file
		 * 
		 * @param pts array of Points
		 * @param size number of Points
		 */
		void writeBinaryPacket(const Point* const pts, const int size);
	};

} // namespace velodyne_savedata

#endif // _VELODYNE_SAVEDATA_Savedata_H_
