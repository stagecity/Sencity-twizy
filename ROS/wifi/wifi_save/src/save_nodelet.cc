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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "save.h"

namespace wifi_save
{
	class WifiSaveNodelet: public nodelet::Nodelet
	{
	public:
	
		WifiSaveNodelet() {}
		~WifiSaveNodelet() {}
	
	private:
	
		virtual void onInit();
		boost::shared_ptr<WifiSave> save_;
	};
	

	void WifiSaveNodelet::onInit()
	{
		save_.reset(new WifiSave(getNodeHandle(),getPrivateNodeHandle()));
	}

}


// Register this plugin with pluginlib.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(wifi_save, WifiSaveNodelet,
		wifi_save::WifiSaveNodelet, nodelet::Nodelet);
