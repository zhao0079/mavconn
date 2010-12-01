
PIXHAWK_EXECUTABLE(px_messenger px_messenger.cc)
PIXHAWK_LINK_LIBRARIES(px_messenger
   mavconn_lcm
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
)
