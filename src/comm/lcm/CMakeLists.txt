INCLUDE_DIRECTORIES(.
  ${MAVLINK_INCLUDE_DIR}
  ${MAVLINK_TYPES_INCLUDE_DIR}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${LCM_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
)

SET_SOURCE_FILES(LCMEXT_SRC_FILES
  mavlink_message_t.c
  camera_image_message_t.c
  gl_overlay_message_t.c
  obstacle_map_message_t.c
  point_cloud_message_t.c
  virtual_scan_message_t.c
)
PIXHAWK_LIBRARY(mavconn_lcm SHARED ${LCMEXT_SRC_FILES})
SET_TARGET_PROPERTIES(mavconn_lcm PROPERTIES COMPILE_FLAGS "-D_REENTRANT -Wno-pointer-sign")
PIXHAWK_LINK_LIBRARIES(mavconn_lcm
  ${LCM_LIBRARY}
)
