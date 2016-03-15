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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "fix.h"

namespace gps_fix
{
	class GpsFixNodelet: public nodelet::Nodelet
	{
	public:
	
		GpsFixNodelet() {}
		~GpsFixNodelet() {}
	
	private:
	
		virtual void onInit();
		boost::shared_ptr<GpsFix> fix_;
	};
	
	/** @brief Nodelet initialization. */
	void GpsFixNodelet::onInit()
	{
		fix_.reset(new GpsFix(getNodeHandle(),getPrivateNodeHandle()));
	}

}

// Register this plugin with pluginlib. 
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(gps_fix, GpsFixNodelet,
		gps_fix::GpsFixNodelet, nodelet::Nodelet);
