cmake_minimum_required(VERSION 2.8.3)
project(wifi_driver)

set(${PROJECT_NAME}_CATKIN_DEPS
    nodelet
    roscpp
    roslib
    wifi_msgs)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS} )
find_package(Boost COMPONENTS signals)


include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS)
    
add_subdirectory(src)

install(FILES nodelet_wifi.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  

