#!/bin/bash

readonly PROGNAME=$(basename $0)
readonly PROGDIR=$(readlink -m $(dirname $0))
readonly ARGS="$@"

is_indigo() {
    local ver=$1
    
    [[ "$ver" == "indigo" ]]
}

is_file() {
    local file=$1

    [[ -f "$file" ]]
}

is_directory() {
    local directory=$1

    [[ -d "$directory" ]]
}

add_to_serial_group() {
    groups | grep "dialout" &> /dev/null \
        || sudo usermod -a -G dialout $USER
}

create_lidar_connection() {
    local name="LIDAR"
    nmcli con show | egrep "^$name" &> /dev/null \
        || nmcli con add con-name "$name" ifname eth0 autoconnect yes save yes \
            type ethernet ip4 192.168.1.70/24 gw4 192.168.1.201
}

create_goPro_connection() {
    local name="GoPro"
    nmcli con show | egrep "^$name" &> /dev/null \
        || nmcli con add con-name "$name" ifname wlan0 autoconnect yes save yes \
            type wifi ssid TNTwizy1 \
        && nmcli con modify "$name" wifi-sec.key-mgmt wpa-psk \
        && nmcli con modify "$name" wifi-sec.psk telecom54
}

activate_wifi() {
    nmcli radio wifi on
}

set_iwlist_uid() {
    sudo chmod +s /sbin/iwlist
}

add_ros_repositories() {
    # Add ROS repositories
    is_file "/etc/apt/sources.list.d/ros-latest.list" \
        || sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

    # Add keys
    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

    # Retrieve repositories information
    sudo apt-get update
}

install_dependencies_package() {
    sudo apt-get install --assume-yes libpcap-dev git arduino build-essential 
}

install_ros_packages() {
    local version="$1"
    sudo apt-get install --assume-yes ros-$version-desktop-full ros-$version-nmea-msgs
    is_indigo $version \
	    && sudo apt-get install --assume-yes ros-$version-imu-filter-madgwick ros-$version-rviz-imu-plugin
}

setup_ROS() {
    local version="$1"
    
    add_ros_repositories 
    sudo apt-get dist-upgrade --assume-yes
    install_ros_packages $version
}

setup_ROS_Workspace() {
     local version="$1"
     local workspace="$2"
    # Installing and Configuring Your ROS Environment
    # @see http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

    source /opt/ros/$version/setup.bash
    mkdir -p "$workspace"/src
    cd "$workspace"/src
    catkin_init_workspace
    cd "$workspace"
    catkin_make
    source "$workspace"/devel/setup.bash

    # Make this workspace default 
    echo "source $workspace/devel/setup.bash" >> ~/.bashrc
}

get_sources() {
	local version="$1"
    local sourceDest="$2"
    local workspaceDest="$3"

    # Retrieve previous work
	git clone --recursive https://github.com/stagecity/Sencity-twizy.git "$sourceDest"
    
    is_indigo $version \
    	|| git clone -b indigo https://github.com/ccny-ros-pkg/imu_tools.git "$workspaceDest/imu_tools"
}

link_sources_to_ROS_workspace() {
    local src="$1"
    local dest="$2"
    
    for module in "$src/"*
    do
        is_directory $module \
	        && ln -s "$module" "$dest/${module##*/}"
    done
}

compile_ROS_modules() {
    local workspace="$1"
    cd "$workspace"
    
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo gps_msgs_generate_messages wifi_msgs_generate_messages
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo install
}

main() {
    local readonly rosVersion="indigo"
    local readonly rosWorkspace="/home/$USER/catkin_ws"
    local readonly projectSourceFolder="/home/$USER/Sencity-twizy"


    # User rights
    echo "Add your user to serial group"
    add_to_serial_group
    echo "Setuid of iwlist"
    set_iwlist_uid
    
    # Network
    echo "Create LIDAR and GoPro network settings"
    create_lidar_connection
    create_goPro_connection
    activate_wifi
    
    # ROS core
    echo "Setup ROS core and workspace"
    setup_ROS "$rosVersion"
    setup_ROS_Workspace "$rosVersion" "$rosWorkspace" 

    # ROS modules for Twizy
    echo "Get sources"
    get_sources "$rosVersion" "$projectSourceFolder" "$rosWorkspace/src"
    link_sources_to_ROS_workspace "$projectSourceFolder/ROS" "$rosWorkspace/src"
    
    echo "Install dependencies and compile modules"
    install_dependencies_package
    compile_ROS_modules "$rosWorkspace"
}

main
