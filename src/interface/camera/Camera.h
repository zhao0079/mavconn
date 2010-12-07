/*======================================================================

MAVCONN Micro Air Vehicle Flying Robotics Toolkit
Please see our website at <http://MAVCONN.ethz.ch>

Original Authors:
  @author Fabian Landau <mavteam@student.ethz.ch>
Contributing Authors (in alphabetical order):

Todo:

(c) 2009 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

This file is part of the MAVCONN project

    MAVCONN is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    MAVCONN is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MAVCONN. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/

#ifndef _Camera_H__
#define _Camera_H__

#include <cv.h>
#include <iostream>
#include <list>
#include <vector>
#include <unistd.h>

// forward declaration
namespace boost
{
    namespace program_options
    {
        class variables_map;
    }
}

namespace MAVCONN
{
    class InputOperation;
    class OutputOperation;
    class CameraOperationFactory;
    class ParameterExecutor;

    class Camera
    {
        public:
            Camera();
            ~Camera();

            void parseConfigValues(int argc, char* argv[]);
            void createCameraOperations();
            void run();

            void registerCameraOperation(InputOperation* input);
            void registerCameraOperation(OutputOperation* output);
            void unregisterCameraOperation(InputOperation* input);
            void unregisterCameraOperation(OutputOperation* output);

            void startPlayback(InputOperation* input);
            void stopPlayback(InputOperation* input);

            void processImage(IplImage* image);

            static CameraOperationFactory& declareInputOperation(CameraOperationFactory* factory);
            static CameraOperationFactory& declareOutputOperation(CameraOperationFactory* factory);

            inline void setInputModeName(const std::string& name)
                { this->inputModeName_ = name; }
            inline const std::string& getInputModeName() const
                { return this->inputModeName_; }

            inline void setInputResourceName(const std::string& name)
                { this->inputResourceName_ = name; }
            inline const std::string& getInputResourceName() const
                { return this->inputResourceName_; }

            inline void setHasIPC(bool bHasIPC)
                { this->bHasIPC_ = bHasIPC; }
            inline bool hasIPC() const
                { return this->bHasIPC_; }

            inline void togglePause()
                { this->bToggledPause_ = !this->bToggledPause_; }
            inline bool isPaused() const
                { return ((this->bPaused_ && !this->bToggledPause_) || (!this->bPaused_ && this->bToggledPause_)); }
            inline bool isStoped() const
                { return (this->bStoped_ || !this->bPlaybackActive_); }

            inline void requestRestart()
                { this->bRequestedRestart_ = true; }
            inline void requestExit()
                { this->bRequestedExit_ = true; }

            const std::string& getSnapshotMask() const;

            inline void setExitCode(int code)
                { this->exitCode_ = code; }
            inline int getExitCode() const
                { return this->exitCode_; }

        private:
            static std::string getArgumentName(const std::string& argument);
            static std::string getCameraOperationDescription(CameraOperationFactory* factory);
            static std::string getCameraOperationDescriptionList(const std::vector<CameraOperationFactory*>& factories);
            static std::string getCameraOperationDetails(CameraOperationFactory* factory);
            static std::string getParameterDescription(ParameterExecutor* parameter);
            static std::string getParameterDescriptionList(const std::vector<ParameterExecutor*>& parameters);
            static std::string getWrapedLine(const std::string& line, size_t indentation, size_t maxWidth = 80);

            void pause(IplImage* image);
            bool restart();

            InputOperation*               input_;               ///< The input operation - provides an IplImage
            InputOperation*               pausedInput_;         ///< A temporary storage for a paused input operation
            CameraOperationFactory*       inputFactory_;        ///< The factory of the input operation
            std::list<OutputOperation*>   output_;              ///< The output operations - process IplImages
            bool                          bPlaybackActive_;     ///< True if the input operation is sending images
            std::string                   inputModeName_;       ///< The name of the input-mode (camera, video, ...)
            std::string                   inputResourceName_;   ///< The name of the input-resource (camera-name, filename, ...)
            bool                          bHasIPC_;             ///< True if the camera is connected to IPC
            bool                          bToggledPause_;       ///< True if someone requested the camera to pause or unpause
            bool                          bPaused_;             ///< True if the camera input is paused
            bool                          bStoped_;             ///< True if the camera playback has stoped
            bool                          bRequestedRestart_;   ///< True if someone requested the camera to restart the playback
            bool                          bRequestedExit_;      ///< True if someone requested the camera to exit
            int                           exitCode_;            ///< The exit code of the camera

            boost::program_options::variables_map* vm_;         ///< The program options value map

            static std::vector<CameraOperationFactory*>& getInputOperations();  ///< A static list of all supported input operations
            static std::vector<CameraOperationFactory*>& getOutputOperations(); ///< A static list of all supported output operations
    };
}

#endif /* _Camera_H__ */
