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

#ifndef _GPS_FIX_H_
#define _GPS_FIX_H_ 

#include <ros/ros.h>
#include <string.h>
#include <gps_msgs/GpsData.h>


namespace gps_fix
{
    
    class GpsFix
    {
    public:
        /* GpsFix constructor
         *
         * @param node public node
         * @param private_nh private node
         */
        GpsFix(ros::NodeHandle node, ros::NodeHandle private_nh);
        ~GpsFix();
    
    private:
        std::string frame_id;       // tf name
        ros::Subscriber gps_data_;  // gps parsed data
        ros::Publisher navSatFix_;  // NavSatFic message publisher
        ros::Publisher twistStamped_;   // Speed message publisher
        ros::Publisher timeReference_;  // Time synchronisation message publisher
        
        /* Process incoming data from Gps Parser
         * 
         * @param dataMsg parsed message from gps_parser node
         */
        void processData(const gps_msgs::GpsData::ConstPtr &dataMsg);
        
        /* Process incoming data from Gps Parser
         * Extract and publish the gps Fix data
         *
         * @param dataMsg parsed message from gps_parser node
         */
        void publishFix(const gps_msgs::GpsData::ConstPtr &dataMsg);
        
        /* Process incoming data from Gps Parser
         * Extract and publish the velocity data
         *
         * @param dataMsg parsed message from gps_parser node
         */
        void publishVel(const gps_msgs::GpsData::ConstPtr &dataMsg);
        
        /* Process incoming data from Gps Parser
         * Extract and publish gps time data
         * 
         * @param dataMsg parsed message from gps_parser node
         */
        void publishTimeRef(const gps_msgs::GpsData::ConstPtr &dataMsg);
    };

} 

#endif 
