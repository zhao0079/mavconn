cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
INCLUDE_DIRECTORIES(
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
)


# Set the build type. Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

PROJECT(cbproject)
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ~/pixhawk/embedded/eclipsebuild/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ~/pixhawk/embedded/eclipsebuild/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(px_rviz src/px_rviz.cc)
TARGET_LINK_LIBRARIES(px_rviz
  lcmext
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_THREAD_LIBRARY}
)

rosbuild_add_executable(px_house src/px_house.cc)
TARGET_LINK_LIBRARIES(px_house
  lcmext
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

ADD_EXECUTABLE(wp_test_send src/wp_test_send.cc)
TARGET_LINK_LIBRARIES(wp_test_send
  lcmext
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

