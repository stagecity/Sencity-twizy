/* 
 * Copyright © 2015  TELECOM Nancy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "fix.h"
#include <tf/transform_listener.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/TimeReference.h>

namespace gps_fix
{
    /** @brief Constructor. */
    GpsFix::GpsFix(ros::NodeHandle node, ros::NodeHandle private_nh)
    {
        // Get tf prefix
        private_nh.param("frame_id", frame_id, std::string("gps"));
        std::string tf_prefix = tf::getPrefixParam(private_nh);
        frame_id = tf::resolve(tf_prefix, frame_id);

        // Instanciate publishers
        navSatFix_ = node.advertise<sensor_msgs::NavSatFix>("fix", 1);
        twistStamped_ = node.advertise<geometry_msgs::TwistStamped>("vel", 1);
        timeReference_ = node.advertise<sensor_msgs::TimeReference>("time_reference", 1);

        // Subscribe to parsed GPS data
        gps_data_ = node.subscribe("gps_data", 1,
                        &GpsFix::processData, (GpsFix *) this,
                        ros::TransportHints().tcpNoDelay(true));

    }

    GpsFix::~GpsFix() {}

    /* Process incoming data from Gps Parser
     *
     * @param dataMsg parsed message from gps_parser node
     */
    void GpsFix::processData(const gps_msgs::GpsData::ConstPtr &dataMsg)
    {
        // Process each type of data
        publishFix(dataMsg);
        publishVel(dataMsg);
        publishTimeRef(dataMsg);
    }

    /* Process incoming data from Gps Parser
     * Extract and publish the gps Fix data
     *
     * @param dataMsg parsed message from gps_parser node
     */
    void GpsFix::publishFix(const gps_msgs::GpsData::ConstPtr &dataMsg) {
        if (navSatFix_.getNumSubscribers() == 0)         // no one listening?
                return;                                  // avoid much work

        // Create new message (pointer)
        sensor_msgs::NavSatFixPtr fixMsg(new sensor_msgs::NavSatFix);

        // Fix time is time of data message
        fixMsg->header.stamp = dataMsg->header.stamp;
        // set tf id
        fixMsg->header.frame_id = frame_id;

        // Parse GPS quality
        switch(dataMsg->gpsQuality) {
        case gps_msgs::GpsData::GPS_QUALITY_NOT_AVAILABLE:
            fixMsg->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            break;
        case gps_msgs::GpsData::GPS_QUALITY_NON_DIFFERENTIAL:
            fixMsg->status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
            break;
        case gps_msgs::GpsData::GPS_QUALITY_DIFFERENTIAL:
            fixMsg->status.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
            break;
        case gps_msgs::GpsData::GPS_QUALITY_ESTIMATED:
            fixMsg->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
            break;
        default:
            fixMsg->status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        }

        // We get the Fix from a GPS
        fixMsg->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        // Set coordinates
        fixMsg->latitude = dataMsg->latitude;;
        fixMsg->longitude = dataMsg->longitude;

        // Set altitude
        fixMsg->altitude = dataMsg->geoidHeight;

        // @see http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
        fixMsg->position_covariance[0] = dataMsg->horizontalDilutionOfPrecision * dataMsg->horizontalDilutionOfPrecision;
        fixMsg->position_covariance[4] = fixMsg->position_covariance[0];
        fixMsg->position_covariance[8] = (2 * dataMsg->horizontalDilutionOfPrecision) * (2 * dataMsg->horizontalDilutionOfPrecision);
        fixMsg->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

        // publish the message
        navSatFix_.publish(fixMsg);
    }

    /* Process incoming data from Gps Parser
     * Extract and publish the velocity data
     *
     * @param dataMsg parsed message from gps_parser node
     */
    void GpsFix::publishVel(const gps_msgs::GpsData::ConstPtr &dataMsg){
        if (twistStamped_.getNumSubscribers() == 0)     // no one listening?
            return;                                     // avoid much work

        if(!dataMsg->validPosition)     // Position not valid
            return;                     // Don't send speed

        // Make a TwistStamped message
        geometry_msgs::TwistStampedPtr velMsg(new geometry_msgs::TwistStamped);

        // Velocity time is data message time
        velMsg->header.stamp = dataMsg->header.stamp;
        // set tf id
        velMsg->header.frame_id = frame_id;

        // ROS convention http://www.ros.org/reps/rep-0103.html
        // x = Est
        // y = North
        // z = Up
        velMsg->twist.linear.y = dataMsg->trueNorthVelocity;
        velMsg->twist.linear.x = dataMsg->trueEstVelocity;
        velMsg->twist.linear.z = dataMsg->upVelocity;

        // publih the message
        twistStamped_.publish(velMsg);
    }

    /* Process incoming data from Gps Parser
     * Extract and publish gps time data
     *
     * @param dataMsg parsed message from gps_parser node
     */
    void GpsFix::publishTimeRef(const gps_msgs::GpsData::ConstPtr &dataMsg) {
        if (timeReference_.getNumSubscribers() == 0)    // no one listening?
            return;                                     // avoid much work

        // Make a TimeReference message
        sensor_msgs::TimeReferencePtr timeMsg(new sensor_msgs::TimeReference);

        // GPS Time was at data message time
        timeMsg->header.stamp = dataMsg->header.stamp;
        // set tf id
        timeMsg->header.frame_id = frame_id;

        // Convert GPS UTC time to Unix time
        boost::posix_time::ptime t1(boost::gregorian::date(dataMsg->utcYear+2000,dataMsg->utcMounth,dataMsg->utcDay),
                boost::posix_time::time_duration(dataMsg->utcHour,dataMsg->utcMin,dataMsg->utcSec));

        // set Time reference (from UTC)
        timeMsg->time_ref = ros::Time::fromBoost(t1);

    	// publish message
        timeReference_.publish(timeMsg);
    }

} 
