/*
 * Copyright © 2015  TELECOM Nancy
 *
 * This file is part of wifi ROS module.
 * wifi ROS module is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * wifi ROS module is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with wifi ROS module.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "driver.h"

namespace wifi_driver
{
	
	class WifiDriverNodelet: public nodelet::Nodelet
	{
	public:
	
		WifiDriverNodelet():
			running_(false)
	{}
	
		~WifiDriverNodelet()
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
	
		boost::shared_ptr<WifiDriver> dvr_; ///< driver implementation class
	};
	
	void WifiDriverNodelet::onInit()
	{
		// start the driver
		dvr_.reset(new WifiDriver(getNodeHandle(), getPrivateNodeHandle()));
	
		// spawn device poll thread
		running_ = true;
		deviceThread_ = boost::shared_ptr< boost::thread >
		(new boost::thread(boost::bind(&WifiDriverNodelet::devicePoll, this)));
	}
	
	/** @brief Device poll thread main loop. */
	void WifiDriverNodelet::devicePoll()
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
PLUGINLIB_DECLARE_CLASS(wifi_driver, WifiDriverNodelet,
		wifi_driver::WifiDriverNodelet, nodelet::Nodelet);
