ADD_SUBDIRECTORY(calibration)

IF (UNIX)
  SET_SOURCE_FILES(TIMER_SRC_FILES ../../core/timer/OgreTimer.Unix.cpp ../../core/Clock.cc)
ELSEIF (WIN32)
  SET_SOURCE_FILES(TIMER_SRC_FILES ../../core/timer/OgreTimer.WIN32.cpp)
ENDIF ()

SET_SOURCE_FILES(CAMERA_SRC_FILES
  Main.cc
  Camera.cc
  CameraOperations.cc
  FPSTimer.cc
  PauseInput.cc
  CameraInput.cc
  DisplayOutput.cc
  FrameInput.cc
  FrameOutput.cc
  SequenceInput.cc
  SequenceOutput.cc
  VideoInput.cc
  VideoOutput.cc
  LcmInput.cc
  LcmOutput.cc
)

PIXHAWK_EXECUTABLE_CONDITIONAL(px_camera CONDITION OPENCV_FOUND FILES ${TIMER_SRC_FILES} ${CAMERA_SRC_FILES})
PIXHAWK_LINK_LIBRARIES(px_camera
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${CV_LIBRARY}
  ${CXCORE_LIBRARY}
  ${HIGHGUI_LIBRARY}
  mavconn_lcm
)

