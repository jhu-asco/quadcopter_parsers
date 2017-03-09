cmake_minimum_required(VERSION 2.8.3)
project(quad_simulator_parser)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules /usr/local/lib/OGRE/cmake)
option(BUILD_SIM_CAMERA "builds camera simulator" OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  parsernode
  pluginlib
  roscpp
  std_msgs
  cv_bridge
  tf
  tf_conversions
)

## System dependencies are found with CMake's conventions
find_package(GCOP REQUIRED systems)
include_directories(${GCOP_INCLUDE_DIRS})
set(LIBS ${LIBS} ${GCOP_LIBRARIES})

find_package(TinyXML REQUIRED)
include_directories(${TINYXML_INCLUDE_DIR})
set(LIBS ${LIBS} ${TINYXML_LIBRARIES})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

if(BUILD_SIM_CAMERA)
  find_package(ObjectRenderer REQUIRED)
  include_directories(${OBJECT_RENDERER_INCLUDE_DIRS})

  find_package(OGRE REQUIRED)
  find_package(OIS REQUIRED)
  include_directories( ${OIS_INCLUDE_DIRS}
    ${OGRE_INCLUDE_DIRS}
  )
endif(BUILD_SIM_CAMERA)
  

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quad_simulator
#  CATKIN_DEPENDS geometry_msgs parsernode pluginlib roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(quad_simulator
   src/${PROJECT_NAME}/quad_simulator.cpp
 )

 target_link_libraries(quad_simulator
   ${LIBS}
   ${catkin_LIBRARIES}
 )

 add_library(quad_simulator_parser
   src/${PROJECT_NAME}/quad_simulator_parser.cpp
 )

 target_link_libraries(quad_simulator_parser
   quad_simulator
 )


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(quad_simulator_parser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(quad_simulator_parser_node src/quad_simulator_parser_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(quad_simulator_parser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(quad_simulator_parser_node
  ${catkin_LIBRARIES}
)

if(BUILD_SIM_CAMERA)
  add_library(camera_simulator
    src/${PROJECT_NAME}/camera_simulator.cpp
  )

  target_link_libraries(camera_simulator
    ${LIBS}
    ${catkin_LIBRARIES}
    ${OBJECT_RENDERER_LIBS}
  )
  add_executable(camera_simulator_node src/camera_simulator_node.cpp)
  target_link_libraries(camera_simulator_node
    camera_simulator
    ${catkin_LIBRARIES}
  )
endif(BUILD_SIM_CAMERA)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS quad_simulator_parser quad_simulator_parser_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-quad-simulator-test tests/quad_simulator_tests.cpp)
if(TARGET ${PROJECT_NAME}-quad-simulator-test)
  target_link_libraries(${PROJECT_NAME}-quad-simulator-test quad_simulator)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
