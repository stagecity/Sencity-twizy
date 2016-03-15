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
#include <boost/lexical_cast.hpp>
#include <tf/transform_listener.h>
#include <sstream>
#include <cmath>
#include "parser.h"


namespace gps_parser
{
	
	GpsParser::GpsParser(ros::NodeHandle node,
			ros::NodeHandle private_nh)
	{
	    // set tf id
		private_nh.param("frame_id", frame_id, std::string("gps"));
		std::string tf_prefix = tf::getPrefixParam(private_nh);
		frame_id = tf::resolve(tf_prefix, frame_id);
		
		// parsed data output topic
		output_ = node.advertise<gps_msgs::GpsData>("gps_data", 1);
		
		// raw data topic listening
		gps_sentences_ = node.subscribe("gps_sentences", 1,
				&GpsParser::processScan, (GpsParser *) this,
				ros::TransportHints().tcpNoDelay(true));
	
	}
	
	/** @brief Callback for raw gps messages. */
	void GpsParser::processScan(const gps_msgs::GpsSentences::ConstPtr &gpsSentencesMsg)
	{
		if (output_.getNumSubscribers() == 0)         // no one listening?
			return;                                     // avoid much work
	
	    // Make a new GpsData message pointer
		gps_msgs::GpsDataPtr dataMsg(new gps_msgs::GpsData);
	
	    // Data time is the same as raw data message time
		dataMsg->header.stamp = gpsSentencesMsg->header.stamp;
		dataMsg->header.frame_id = frame_id;
	
	    // vector that will contain splited NMEA sentences tokens
		std::vector<std::string> tokens;
	
		//--------- GGA ---------//
		tokens = split(gpsSentencesMsg->gga, ',');
		//latitude
		dataMsg->latitude = safeDouble(tokens.at(2).substr(0,2)) + (safeDouble(tokens.at(2).substr(2)) / 60.);
	    // South is negative
		if(tokens.at(3).c_str()[0] == 'S')
			dataMsg->latitude = -dataMsg->latitude;
	
		//longitude
		dataMsg->longitude = safeDouble(tokens.at(4).substr(0,3)) + (safeDouble(tokens.at(4).substr(3)) / 60.);
	    // West is negative
		if(tokens.at(5).c_str()[0] == 'W')
			dataMsg->longitude = -dataMsg->longitude;
			
			
		// GPS quality
		dataMsg->gpsQuality = safeInt(tokens.at(6));     
	
		// Sattelites in view
		dataMsg->nbSat = safeInt(tokens.at(7));        
	
		// seaHeight
		dataMsg->seaHeight = safeFloat(tokens.at(9));
	
		// geoidHeight
		dataMsg->geoidHeight = safeFloat(tokens.at(11));
	
	
		//--------- GSA ---------//
 		tokens = split(gpsSentencesMsg->gsa, ',');
	
		// Mode
		dataMsg->gsaModeAuto = (tokens.at(1).c_str()[0] == 'A');
		// Fix type
		dataMsg->fixType = safeInt(tokens.at(2));
	
		// PRN (used sattelites number)
		for(int i = 0, j = 3; i < 12; i++, j++)
			dataMsg->prnUsed[i] = safeInt(tokens.at(j));
	
		// Position dilution of precision
		dataMsg->positionDilutionOfPrecision = safeFloat(tokens.at(15));
	
		// horizontal Dilution Of Precision
		dataMsg->horizontalDilutionOfPrecision = safeFloat(tokens.at(16));
	
		// vertical Dilution Of Precision
		try {
			dataMsg->verticalDilutionOfPrecision = safeFloat(tokens.at(17));
		} catch (std::out_of_range &) {
			dataMsg->verticalDilutionOfPrecision = NAN;
		}
	
		//--------- RMC ---------//
		tokens = split(gpsSentencesMsg->rmc, ',');
		// UTC Time
		dataMsg->utcHour = safeInt(tokens.at(1).substr (0,2));
		dataMsg->utcMin = safeInt(tokens.at(1).substr (2,2));
		dataMsg->utcSec = safeInt(tokens.at(1).substr (4,2));
	
		// Status
		dataMsg->validPosition = (tokens.at(2).c_str()[0] == 'A');
		// UTC date
		dataMsg->utcDay = safeInt(tokens.at(9).substr (0,2));
		dataMsg->utcMounth = safeInt(tokens.at(9).substr (2,2));
		dataMsg->utcYear = safeInt(tokens.at(9).substr (4,2));
	
		// Magnetic variation
		dataMsg->magneticVariation = safeFloat(tokens.at(10));
	    // East is negative
		if(tokens.at(11).c_str()[0] == 'E') 
			dataMsg->magneticVariation = -dataMsg->magneticVariation;
			
		// Mode
		dataMsg->mode = tokens.at(12).c_str()[0];
	
	
		//--------- VTG ---------//
		tokens = split(gpsSentencesMsg->vtg, ',');
		// True course over ground
		dataMsg->trueCourseOverGround = safeFloat(tokens.at(1));
	
		// Magnetic course over ground
		dataMsg->magneticCourseOverGround = safeFloat(tokens.at(3));
	
		// speed over ground
		dataMsg->speedOverGround = safeFloat(tokens.at(7));        
	
		//--------- PGRME ---------//
		tokens = split(gpsSentencesMsg->pgrme, ',');
		// estimated horizontal position error
		dataMsg->estimatedHorizontalPosError = safeFloat(tokens.at(1));
	
		// estimated vertical position error
		dataMsg->estimatedVerticalPosError = safeFloat(tokens.at(3));
	
		// estimated position error
		dataMsg->estimatedPosError = safeFloat(tokens.at(5));
	
		//--------- PGRMV ---------//
		tokens = split(gpsSentencesMsg->pgrmv, ',');
		// true est velocity
		dataMsg->trueEstVelocity = safeFloat(tokens.at(1));
	
		// true north velocity
		dataMsg->trueNorthVelocity = safeFloat(tokens.at(2));
	
		// Up velocity
		try {
			dataMsg->upVelocity = safeFloat(tokens.at(3));
		} catch (std::out_of_range &) {
			dataMsg->upVelocity = NAN;
		}
	
	    // publish message
		output_.publish(dataMsg);
	}
	
	
	/** Split a string in tokens with a delimiter
	 * 
	 * @param str the string to split
	 * @param delimiter the delimiter to use
	 */
	 //http://code.runnable.com/VHb0hWMZp-ws1gAr/splitting-a-string-into-a-vector-for-c%2B%2B
	std::vector<std::string> GpsParser::split(std::string str, char delimiter) {
		std::vector<std::string> internal;
		std::stringstream ss(str); // Turn the string into a stream.
		std::string tok;
	
		while(std::getline(ss, tok, delimiter)) {
			internal.push_back(tok);
		}
	
		return internal;
	}
	
	/** Convert a string into float
	 * take care of unwanted error with Not A Number
	 *
	 * @param str float as a string
	 */
	float GpsParser::safeFloat(std::string str) {
		try {
			return boost::lexical_cast<float> (str);
		} catch(boost::bad_lexical_cast &) {
			return NAN;
		}
	}
	
	/** Convert a string into double
	 * take care of unwanted error with Not A Number
	 *
	 * @param str double as a string
	 */
	double GpsParser::safeDouble(std::string str) {
		try {
			return boost::lexical_cast<double>(str);
		} catch(boost::bad_lexical_cast &) {
			return NAN;
		}
	}
	
	/** Convert a string into integer
     * take care of unwanted error with 0
     *
     * @param str integer as a string
     */
	int GpsParser::safeInt(std::string str) {
		try {
			return boost::lexical_cast<int>(str);
		} catch(boost::bad_lexical_cast &) {
			return 0;
		}
	}

} // namespace gps_driver
