SET_SOURCE_FILES(CAMERA_CALIBRATION_SRC_FILES
  PxCameraCalibration.cc
  PxCameraCalibrationStandard.cc
  PxCameraCalibrationOmnidirectional.cc
  PxCameraStereoCalibration.cc
)

INCLUDE_DIRECTORIES(.)

#PIXHAWK_LIBRARY(cameracalibration SHARED ${CAMERA_CALIBRATION_SRC_FILES})
#PIXHAWK_LINK_LIBRARIES(cameracalibration 
# ${CXCORE_LIBRARY}
#  ${CV_LIBRARY}
#  ${HIGHGUI_LIBRARY}
#  )
