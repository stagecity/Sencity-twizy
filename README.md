# Sencity-twizy

This repository contains programs, documentation and other stuff needed by the project Sencity to work with the car.
This work is from the school [TELECOM Nanncy](http://telecomnancy.univ-lorraine.fr)

## Sencity project
Sencity is an educational collaborative and innovative project around the Internet of Things and
smart city. It relies on the development of an experimental platform to allow students to design,
develop and test new smart services in interaction with citizens and their environment. As part
of the project, an electric car was bought to collect data across the city.

## Environment 
### Operating system
It is recommanded to use Ubuntu LTS (14.04).
Download it here : http://www.ubuntu.com/download/desktop

Follow the installation instructions : http://www.ubuntu.com/download/desktop/install-ubuntu-desktop

**You will need to use the serial, so ya have to add your user to the group dialout**
 
### Network 
#### LIDAR
The Velodyne LIDAR Puck send data over ethernet with UDP packet.
You will need to set up your ethernet interface as follow :
 * IP address: 192.168.1.70 (70 as example, any number except 201 works)
 * Gateway: 255.255.255.0

**Don't forget to allow incoming UDP packet from 192.168.1.201 in your firewall**

#### GoPro
To remotly control the GoPro, you will need to configure it as a WiFi hotspot and then connect it.
If you use the one that was configured during the internship, use the following login :
 * SSID : TNTwizy1
 * Passphrase (WPA-PSK) : telecom54

#### WiFi
In order to perform WiFi scan with the ROS WiFi module, you need to set the setuid on iwlist :
```bash
chmod +s /sbin/iwlist
```

### Robot Operating System
#### Installation
Robot Operating System is the main component of this work.
Follow intructions from this page : http://wiki.ros.org/indigo/Installation/Ubuntu
You have to install the following dependancies :

```bash
apt-get install  ros-indigo-desktop-full ros-indigo-imu-filter-madgwick ros-indigo-rviz-imu-plugin ros-indigo-nmea-msgs libpcap-dev
```

#### Workspace
To set your workspace up, follow the ROS wiki :
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment


#### To learn
You should learn ROS basis from their Tutorials :
http://wiki.ros.org/ROS/Tutorials

#### Compilation
In the src folder, copy all packages from the folder ROS/.

Then compile them with ROS build system :
```bash
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo gps_msgs_generate_messages wifi_msgs_generate_messages
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

To install files in your workspace use the following command :

```bash
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo install
```
### Install script
To setup your environment, you can just install the Operating system and execute [setup.sh](setup.sh).
```bash
wget "https://github.com/stagecity/Sencity-twizy/raw/master/setup.sh"
chmod +x setup.sh
./setup.sh
```

### Arduino
You will need arduino to read data from the Inertial Measurement Unit
Official website : https://www.arduino.cc/

The recommanded IDE is [Arduino IDE](https://www.arduino.cc/en/Main/Software)

You may want to read its [manual](https://www.arduino.cc/en/Guide/Environment)!

#### Librarie
You will have to copy the folder "Arduino/libraries/MPU9250" as "Mpu9250Spi"in your arduino libraries folder.

#### Compilation
Open the sketch Arduino/imu/imu.ino with arduino IDE and upload it on your board.

## Data specification
see [Data.md](Data.md)

## Hardware specification and configuration
see [Hardware.md](Hardware.md)

## Use
To start the saving system :
ROS only :
```bash
roslaunch twizy save.launch maxSize:=MaximalFileSizeInByte outputDir:=/path/to/record/folder/ wifiPort:=wifiCardName gpsPort:=/dev/tty{ACM|USB}[0-9] imuPort:=/dev/tty{ACM|USB}[0-9]
```
ROS and GoPro acquisition :
```bash
./launch.sh maxSize:=MaximalFileSizeInByte outputDir:=/path/to/record/folder/ wifiPort:=wifiCardName gpsPort:=/dev/tty{ACM|USB}[0-9] imuPort:=/dev/tty{ACM|USB}[0-9]
```

To view the recorded data :
```bash
roslaunch twizy viewBag.launch file:=/path/to/recorde/Rosbag_yyyy-mm-dd-hh-mm-ss.bag
```

You will find more arguments in the ROS twizy package dcumentation.

Other launch files are avaible : 
 * saveAndView.launch : record data and show them in real-time
 * streamView.launch : show sensors data in real-time
 * view.launch : launch ROS viewing software with configurations

