cmake_minimum_required(VERSION 2.6.3)
project(orocos_ros_interface)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Use 'source orocos_toolchain/env.sh' if the command below fails:
find_package(catkin REQUIRED COMPONENTS
  roscpp rtt_ros actionlib actionlib_msgs genmsg rtt_roscomm
  # ADDITIONAL ROS PACKAGES
  )


find_package(OROCOS-RTT QUIET)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-typekit)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()
find_package(RST-RT REQUIRED)
message(STATUS "RST-RT version: ${RST-RT_VERSION}")
include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
add_definitions(${RST-RT_CFLAGS})
link_directories(${RST-RT_LIBRARY_DIR} ${RST-RT_LIBRARY_DIRS})

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})


########################
find_package(gazebo REQUIRED)
if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()

find_package(Boost COMPONENTS thread REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

########################




### Orocos Dependencies ###
# Note that orocos_use_package() does not need to be called for any dependency
# listed in the package.xml file

include_directories(${USE_OROCOS_INCLUDE_DIRS})


add_definitions(-DRTT_COMPONENT)
orocos_component(orocos_ros_interface include/orocos_ros_interface.hpp src/orocos_ros_interface.cpp)
target_link_libraries(orocos_ros_interface ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${RST-RT_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


# orocos_library(my_library src/my_library.cpp)
# target_link_libraries(my_library ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_service(my_service src/my_service.cpp)
# target_link_libraries(my_service ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_plugin(my_plugin src/my_plugin.cpp)
# target_link_libraries(my_plugin ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

# orocos_typekit(my_typekit src/my_typekit.cpp)
# target_link_libraries(my_typekit ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

### Orocos Package Exports and Install Targets ###
orocos_install_headers(include/orocos_ros_interface.hpp)

# Generate install targets for header files

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})

# Export package information (replaces catkin_package() macro) 
orocos_generate_package()

install(FILES 
  orocos_ros_interface.ops 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
