add_executable(gps_parser_node gps_node.cc parser.cc)
target_link_libraries(gps_parser_node
                      ${catkin_LIBRARIES})
install(TARGETS gps_parser_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(gps_parser_nodelet nodelet.cc parser.cc)
target_link_libraries(gps_parser_nodelet
                      ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(gps_parser_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()
install(TARGETS gps_parser_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
