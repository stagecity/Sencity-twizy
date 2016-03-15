add_executable(imu_driver_node imu_node.cc driver.cc)
target_link_libraries(imu_driver_node
                      ${catkin_LIBRARIES})
install(TARGETS imu_driver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(imu_driver_nodelet nodelet.cc driver.cc)
target_link_libraries(imu_driver_nodelet
                      ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(imu_driver_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()
install(TARGETS imu_driver_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
