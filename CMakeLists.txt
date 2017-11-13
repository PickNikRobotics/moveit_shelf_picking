cmake_minimum_required(VERSION 2.8.3)
project(moveit_shelf_picking)

# C++ 11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  bolt_moveit
  roslint
)

find_package(OMPL REQUIRED)
find_package(Boost REQUIRED serialization)

catkin_package(
  CATKIN_DEPENDS
    bolt_moveit
    geometric_shapes
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
)

# If enabled, show debug message output
add_definitions(-DENABLE_DEBUG_MACRO)

###########
## Build ##
###########

include_directories(SYSTEM
  ${Boost_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
  src/moveit_shelf_picking.cpp
)
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Main program
add_executable(${PROJECT_NAME}_moveit_shelf_picking_main
  src/moveit_shelf_picking_main.cpp
)
# Rename C++ executable without namespace
set_target_properties(${PROJECT_NAME}_moveit_shelf_picking_main
  PROPERTIES OUTPUT_NAME moveit_shelf_picking_main PREFIX "")
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_moveit_shelf_picking_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )

#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_moveit_shelf_picking_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark roslaunch files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark config files for installation
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)