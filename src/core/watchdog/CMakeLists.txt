SET_SOURCE_FILES(WATCHDOG_SRC_FILES
  Main.cc
  Watchdog.cc
  Process.cc
  Command.cc
)

PIXHAWK_EXECUTABLE(watchdog ${WATCHDOG_SRC_FILES})
PIXHAWK_LINK_LIBRARIES(watchdog
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  corelib
  mavconn_lcm
)

PIXHAWK_EXECUTABLE_CONDITIONAL(watchdogcontrol CONDITION OPENCV_FOUND FILES WatchdogControl.cc)
PIXHAWK_LINK_LIBRARIES(watchdogcontrol
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  corelib
  mavconn_lcm
  ${HIGHGUI_LIBRARY}
  ${CV_LIBRARY}
  ${CXCORE_LIBRARY}
)

