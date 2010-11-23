INCLUDE_DIRECTORIES(
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
)

set (CMAKE_C_FLAGS "-g -std=gnu99 -Wall -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE -D_REENTRANT -Wno-unused-parameter -Wno-format-zero-length -Wshadow")

PIXHAWK_EXECUTABLE(lcm_logger
  glib_util.c
  lcm_logger.c
)

PIXHAWK_LINK_LIBRARIES(lcm_logger
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  lcm
)

PIXHAWK_EXECUTABLE(lcm_logplayer
  lcm_logplayer.c
)

PIXHAWK_LINK_LIBRARIES(lcm_logplayer
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
  lcm
)
