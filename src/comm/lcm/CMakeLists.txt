INCLUDE_DIRECTORIES(.
  ${MAVLINK_INCLUDE_DIR}
  ${MAVLINK_TYPES_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
)

SET_SOURCE_FILES(LCMEXT_SRC_FILES
  mavlink_message_t.c
  gl_overlay_message_t.c
  obstacle_map_message_t.c
  point_cloud_message_t.c
  target_position_message_t.c
)
PIXHAWK_LIBRARY(lcmext SHARED ${LCMEXT_SRC_FILES})
SET_TARGET_PROPERTIES(lcmext PROPERTIES COMPILE_FLAGS "-D_REENTRANT -Wno-pointer-sign")
PIXHAWK_LINK_LIBRARIES(lcmext
  lcm
)
