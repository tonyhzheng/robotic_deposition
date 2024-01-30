cmake_minimum_required(VERSION 2.8.3)
project(ur_kinematics)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs  
   pluginlib tf_conversions)

# find_package(Boost REQUIRED COMPONENTS system)
FIND_PACKAGE( Boost 1.65 COMPONENTS program_options REQUIRED )
find_package(PythonLibs 3.7 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur3_kin ur5_kin ur10_kin   
  CATKIN_DEPENDS roscpp geometry_msgs   
    pluginlib tf_conversions
  DEPENDS Boost
)

###########
## Build ##
###########

# set(Boost_INCLUDE_DIR /home/docker/bind_mount/libraries/boost_1_68_0)
# set(Boost_LIBRARY_DIR /home/docker/bind_mount/libraries/boost_1_68_0/stage/lib)
# find_package(Boost COMPONENTS system filesystem REQUIRED)
# include_directories(${Boost_INCLUDE_DIR})
# link_directories(${Boost_LIBRARY_DIR})

include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ur3_kin src/ur_kin.cpp)
set_target_properties(ur3_kin PROPERTIES COMPILE_DEFINITIONS "UR3_PARAMS")
 
add_library(ur5_kin src/ur_kin.cpp)
set_target_properties(ur5_kin PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")

add_library(ur10_kin src/ur_kin.cpp)
set_target_properties(ur10_kin PROPERTIES COMPILE_DEFINITIONS "UR10_PARAMS")


PYTHON_ADD_MODULE(ur_kin_py src/ur_kin_py.cpp)
target_link_libraries(ur_kin_py ur5_kin boost_python boost_numpy ${PYTHON_LIBRARIES} ${Boost_LIBRARIES} )

set_target_properties(ur_kin_py PROPERTIES 
                      LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})


#############
## Install ##
#############

install(TARGETS ur3_kin ur5_kin ur10_kin 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


install(TARGETS ur_kin_py
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
