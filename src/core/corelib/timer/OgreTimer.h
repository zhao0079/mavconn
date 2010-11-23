#ifndef __OgreTimer_H__
#define __OgreTimer_H__

#include "MAVCONNConfig.h"

#if defined(MAVCONN_PLATFORM_WIN32)
# include "OgreTimerImp.WIN32.h"
#elif defined(MAVCONN_PLATFORM_LINUX)
# include "OgreTimerImp.GLX.h"
#elif defined(MAVCONN_PLATFORM_APPLE)
# include "OgreTimerImp.OSX.h"
#endif

#endif /* __OgreTimer_H__ */
