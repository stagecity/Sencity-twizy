add_executable(wifi_driver_node wifi_node.cc driver.cc)
target_link_libraries(wifi_driver_node
                      ${catkin_LIBRARIES})
install(TARGETS wifi_driver_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(wifi_driver_nodelet nodelet.cc driver.cc)
target_link_libraries(wifi_driver_nodelet
                      ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(wifi_driver_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()
install(TARGETS wifi_driver_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
