INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(ROS_IncludeSearchPaths
  ~/ros/ros/core/roscpp/include/ros
  ~/ros/ros/core/rosbuild
)

FIND_PATH(ROS_INCLUDE_DIR
  NAMES ros.h
  PATHS ${ROS_IncludeSearchPaths}
)

FIND_PATH(ROS_CMAKE_DIR
  NAMES rosbuild.cmake
  PATHS ${ROS_IncludeSearchPaths}

)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ROS "Could NOT find ros.h (ROS). It is only a requirement for some processes. Its safe to continue."
  ROS_INCLUDE_DIR
)

MARK_AS_ADVANCED(
  ROS_INCLUDE_DIR
  ROS_CMAKE_DIR
)
