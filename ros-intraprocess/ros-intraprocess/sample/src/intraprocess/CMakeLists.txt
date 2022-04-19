cmake_minimum_required(VERSION 2.8.3)
project(intraprocess)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  std_msgs
)

catkin_package(
  LIBRARIES intraprocess_publish
  # DEPENDS system_lib
  CATKIN_DEPENDS nodelet pluginlib roscpp std_msgs)

catkin_package(
  LIBRARIES intraprocess_subscribe
  # DEPENDS system_lib
  CATKIN_DEPENDS nodelet pluginlib roscpp std_msgs)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(intraprocess_publish
  src/publish.cpp
)
add_library(intraprocess_subscribe
  src/subscribe.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(intraprocess_publish
  ${catkin_LIBRARIES}
)
target_link_libraries(intraprocess_subscribe
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# if(catkin_EXPORTED_LIBRARIES)
#   add_dependencies(intraprocess_publish ${catkin_EXPORTED_LIBRARIES})
#   add_dependencies(intraprocess_subscribe ${catkin_EXPORTED_LIBRARIES})
# endif()

install(TARGETS intraprocess_publish
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS intraprocess_subscribe
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   intraprocess_publish.xml
   intraprocess_subscribe.xml
   intraprocess.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_intraprocess.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
