ADD_SUBDIRECTORY(calibration)

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

PIXHAWK_EXECUTABLE_CONDITIONAL(px_camera CONDITION OPENCV_FOUND FILES ${CAMERA_SRC_FILES})
PIXHAWK_LINK_LIBRARIES(px_camera
#  ${CXCORE_LIBRARY}
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY}
  ${CV_LIBRARY}
  ${HIGHGUI_LIBRARY}
  mavconn_lcm
  corelib
)

