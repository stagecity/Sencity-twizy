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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/wait.h>
#include <boost/lexical_cast.hpp>
#include <wifi_msgs/WifiScan.h>

#include "driver.h"
#define BUFFER_SIZE 1024    // maximum line length

namespace wifi_driver
{
	
	WifiDriver::WifiDriver(ros::NodeHandle node,
			ros::NodeHandle private_nh)
	{
		// use private node handle to get parameters
		private_nh.param("port", port, std::string("wlan0"));
		// raw data output topic
		output_ = node.advertise<wifi_msgs::WifiScan>("wifi/data", 3);
	}
	
	/** poll the device
	 *
	 */
	void WifiDriver::poll(void)
	{
        pid_t childProc;        // will contain PID of iwlist
        int fd[2];              // file descriptors for pipe with iwlist

        //try to make a pipe
        if(pipe(fd) == -1)
        {
          ROS_ERROR_STREAM("Pipe error");
          ROS_ERROR_STREAM(strerror(errno));
          return;
        }
         
        // Here we fork
        if((childProc = fork())) //Father
        {
            // close pipe's "stdout", father side
            close(fd[1]);
            if(childProc == -1) //No child created
            {
                close(fd[0]);
                ROS_ERROR_STREAM("Error when creating child");
                ROS_ERROR_STREAM(strerror(errno));
                return;
            }
            std::vector<std::string> list;
            // try to read iwlist output
            if(!readPipe(fd[0], list)) {
                return;
            }   
            
            // Allocate a new shared pointer for zero-copy sharing with other nodelets.
			wifi_msgs::WifiScanPtr wifiMsg(new wifi_msgs::WifiScan);
			wifiMsg->header.stamp = ros::Time::now();
            
            // Message for 1 WiFi network information
            wifi_msgs::WifiData wifiData;
            // Cursor to parse iwlist output
            size_t pos;
            // Parse each line of iwlist output
            for(int i = 0; i < list.size(); ++i) {
				std::string &line = list.at(i);

                // iwlist network information begin with Address
                if((pos = line.find(" - Address: ")) !=  std::string::npos) {
					if(i > 1) {
					    wifiMsg->data.push_back(wifiData);
					    wifiData.address = "";
					    wifiData.ssid = "";
					    wifiData.signal = 0;
					    wifiData.encryption = false;
					    wifiData.frequency = NAN;
					    wifiData.security = "";
					    wifiData.groupCipher = "";
					    wifiData.pairwiseCiphers = "";
					    wifiData.protocol = "";
					}
				    // Delete " - Address: "
					wifiData.address = line.substr(pos + 12);
				} 
				else if((pos = line.find("ESSID:\"")) !=  std::string::npos) {
				    // Delete "ESSID:\""
					wifiData.ssid = line.substr(pos + 7);
					// Delete final "
					wifiData.ssid.erase(wifiData.ssid.length()-1);
				}
                else if((pos = line.find("Frequency:")) !=  std::string::npos) {
					try {
					    // Delete "Frequency:"
						std::string tmp = line.substr(pos + 10);
						pos = tmp.find(" GHz ");
						// Convert number in float on string with " GHz" deleted
						wifiData.frequency = boost::lexical_cast<float> (tmp.erase(pos));
					} catch(boost::bad_lexical_cast &) {
						wifiData.frequency =  NAN;
					}
				}
                else if((pos = line.find("Encryption key:")) !=  std::string::npos) {
                    // Delete "Encryption key:" and test if it is "on"
					wifiData.encryption = line.substr(pos + 15) == "on";
				}
                else if((pos = line.find("IE: ")) !=  std::string::npos) {
                    // Delete "IE: "
					wifiData.security = line.substr(pos + 4);
				}
                else if((pos = line.find("Group Cipher : ")) !=  std::string::npos) {
                    // Delete "Group Cipher : "
					wifiData.groupCipher = line.substr(pos + 15);
				}
                else if((pos = line.find("Pairwise Ciphers (1) : ")) !=  std::string::npos) {
                    // Delete "Pairwise Ciphers (1) : "
					wifiData.pairwiseCiphers = line.substr(pos + 23);
				}
                else if((pos = line.find("Signal level=")) !=  std::string::npos) {
					try {
					    //Delete "Signal level="
						std::string tmp = line.substr(pos + 13);
						pos = tmp.find("/100");
						// Convert signal level in integer on string with "/100" deleted
						wifiData.signal =  boost::lexical_cast<int>(tmp.erase(pos));
					} catch(boost::bad_lexical_cast &) {
						wifiData.signal =  0;
					}
				}
                else if((pos = line.find("Protocol:")) !=  std::string::npos) {
                    // Delete "Protocol:"
					wifiData.protocol = line.substr(pos + 9);
				}
            }
            // Push the network into the list of scanned network
            wifiMsg->data.push_back(wifiData);
            
            output_.publish(wifiMsg);
            // Close pipe
            close(fd[0]);
        }
        else // Child
        {
            // Close pipe's' "stdin"
            close(fd[0]);

            // My stdout is pipe's "stdout"
            if(dup2(fd[1], 1) == -1)
            {
                 ROS_ERROR_STREAM("Child : redirection error");
                 ROS_ERROR_STREAM(strerror(errno));
                 exit(1);
            }

            // exec iwlist
            if(execl("/sbin/iwlist", "/sbin/iwlist", port.c_str(), "scan", (char*) NULL) == -1)
            {
                ROS_ERROR_STREAM("Child : exec error");
                ROS_ERROR_STREAM(strerror(errno));
                exit(1);
            }
        }
	}
	
	/** Read the output of a process
     *
     * @param fd file descriptor of the process output
     * @param list output as a list of string
     *
     * @return true if success
     */
	bool WifiDriver::readPipe(int fd, std::vector<std::string> &list)
    {
        char buffer[BUFFER_SIZE];       // buffer to read process output
        buffer[BUFFER_SIZE - 1] = '\0'; // be sure that there is an end of string

        list.clear();       // clean the list before filling it

        // read output character by charracter
        for(int i = 0; i < BUFFER_SIZE; i++)
        {
            int charRead = read(fd, &buffer[i], 1);
            if(charRead == -1)
            {
                // if it was an interrupt
                if(errno == EINTR)
                {
                    // continue to read 
                    i--;
                    continue;
                }
                ROS_ERROR_STREAM("Read error");
                ROS_ERROR_STREAM(strerror(errno));
                return false;
            }
            else if(charRead == 0)
            {
                // no more to read : we stop
                // (last line of iwlist must have a '\n')
                break;
            }

            // replace newline by end of line
            if(buffer[i] == '\n') {
                buffer[i] = '\0';
                // push the string to the list
                list.push_back(std::string(buffer));
                // read the next line
                i = -1;
            }            

        }

        return true;
    }
	
} // namespace wifi_driver
