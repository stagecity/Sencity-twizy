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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "savedata.h"

namespace velodyne_savedata
{
	class SavedataNodelet: public nodelet::Nodelet
	{
	public:
	
		SavedataNodelet() {}
		~SavedataNodelet() {}
	
	private:
	
		virtual void onInit();
		boost::shared_ptr<Savedata> save_;
	};
	
	void SavedataNodelet::onInit()
	{
		save_.reset(new Savedata(getNodeHandle(),getPrivateNodeHandle()));
	}

} // namespace velodyne_savedata


// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne_savedata, SavedataNodelet,
		velodyne_savedata::SavedataNodelet, nodelet::Nodelet);
