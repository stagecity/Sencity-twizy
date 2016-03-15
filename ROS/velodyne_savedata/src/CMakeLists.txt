add_executable(velodyne_savedata_node savedata_node.cc savedata.cc)
target_link_libraries(velodyne_savedata_node
                      ${catkin_LIBRARIES})
install(TARGETS velodyne_savedata_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(velodyne_savedata_nodelet savedata_nodelet.cc savedata.cc)
target_link_libraries(velodyne_savedata_nodelet
                      ${catkin_LIBRARIES})
install(TARGETS velodyne_savedata_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
