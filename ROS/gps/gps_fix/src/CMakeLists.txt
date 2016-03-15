add_executable(gps_fix_node fix_node.cc fix.cc)
target_link_libraries(gps_fix_node
                      ${catkin_LIBRARIES})
install(TARGETS gps_fix_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(gps_fix_nodelet fix_nodelet.cc fix.cc)
target_link_libraries(gps_fix_nodelet
                      ${catkin_LIBRARIES})
install(TARGETS gps_fix_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
