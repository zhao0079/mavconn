IF (ROS_FOUND)
ADD_SUBDIRECTORY(visualization)

INCLUDE_DIRECTORIES(~/ros/ros/core/roscpp/include)
INCLUDE_DIRECTORIES(~/ros/stacks/geometry/tf/include)

ENDIF ()

