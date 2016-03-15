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

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "driver.h"

namespace imu_driver
{
	
	class ImuDriverNodelet: public nodelet::Nodelet
	{
	public:
	
		ImuDriverNodelet():
			running_(false)
	{}
	
		~ImuDriverNodelet()
		{
			if (running_)
			{
				NODELET_INFO("shutting down driver thread");
				running_ = false;
				deviceThread_->join();
				NODELET_INFO("driver thread stopped");
			}
		}
	
	private:
	
		virtual void onInit(void);
		virtual void devicePoll(void);
	
		volatile bool running_;               ///< device thread is running
		boost::shared_ptr<boost::thread> deviceThread_;
	
		boost::shared_ptr<ImuDriver> dvr_; ///< driver implementation class
	};
	
	void ImuDriverNodelet::onInit()
	{
		// start the driver
		dvr_.reset(new ImuDriver(getNodeHandle(), getPrivateNodeHandle()));
	
		// spawn device poll thread
		running_ = true;
		deviceThread_ = boost::shared_ptr< boost::thread >
		(new boost::thread(boost::bind(&ImuDriverNodelet::devicePoll, this)));
	}
	
	/** @brief Device poll thread main loop. */
	void ImuDriverNodelet::devicePoll()
	{
		while(ros::ok())
		{
			// poll device until end of file
			running_ = dvr_->poll();
			if (!running_)
				break;
		}
		running_ = false;
	}

} 

// Register this plugin with pluginlib.
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(imu_driver, ImuDriverNodelet,
		imu_driver::ImuDriverNodelet, nodelet::Nodelet);
