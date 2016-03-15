add_executable(wifi_save_node save_node.cc save.cc)
target_link_libraries(wifi_save_node
                      ${catkin_LIBRARIES})
install(TARGETS wifi_save_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(wifi_save_nodelet save_nodelet.cc save.cc)
target_link_libraries(wifi_save_nodelet
                      ${catkin_LIBRARIES})
install(TARGETS wifi_save_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
