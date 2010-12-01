/*======================================================================

MAVCONN mcvlib - The Micro Computer Vision Library
Please see our website at <http://MAVCONN.ethz.ch>

Original Authors:
  Fabian Landau
Contributing Authors (in alphabetical order):

Todo:

(c) 2009 MAVCONN PROJECT  <http://MAVCONN.ethz.ch>

This file is part of the MAVCONN project

    mcvlib is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    mcvlib is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mcvlib. If not, see <http://www.gnu.org/licenses/>.

========================================================================*/

#include "CameraInput.h"
#include "Sleep.h"
/*
#if LIBDC1394_VERSION >= 0200
#  include <dc1394/dc1394.h>
#endif
*/
namespace MAVCONN
{
    RegisterInputOperation(CameraInput)
        .setDescription("Captures frames from a PointGrey Firefly MV / Chameleon camera")
        .addName("camera")
        .addName("c")
        .addParameter("index", &CameraInput::setIndex, "Unique ID of the camera (label on the back)", 0)
        .addParameter("delay", &CameraInput::setDelay, "Initial delay in ms", 250)
        .makeDefault()
    ;

    CameraInput::CameraInput(Camera* camera) : InputOperation(camera)
    {
        this->index_ = 0;
        this->capture_ = 0;
        this->delay_ = 250;

        this->getCamera()->setInputModeName("DC1394");
        this->getCamera()->setInputResourceName("unknown camera");


        // Read the camera information only if libdc1394 is present and in version 2.0.0
        dc1394_t* dc1394 = dc1394_new(); // Initialize libdc1394
        dc1394camera_list_t* list;
        dc1394error_t error = dc1394_camera_enumerate (dc1394, &list); // Find cameras

        if (!error)
        {
            if (list->num == 1)
                std::cout << "Detected " << list->num << " camera with IEEE 1394 drivers" << std::endl;
            else if (list->num > 1)
                std::cout << "Detected " << list->num << " cameras with IEEE 1394 drivers" << std::endl;

            if (list->num > 0)
            {
                dc1394camera_t* camera = dc1394_camera_new(dc1394, list->ids[0].guid);
                if (camera)
                    this->getCamera()->setInputResourceName(camera->model);
            }
        }

        dc1394_camera_free_list(list);
        dc1394_free(dc1394);
    }

    CameraInput::~CameraInput()
    {
        if (this->capture_)
            cvReleaseCapture(&this->capture_);
    }

    void CameraInput::startPlayback()
    {
        this->capture_ = cvCreateCameraCapture(this->index_);
        if (!this->capture_)
        {
            std::cout << "Error: Couldn't open camera (index " << this->index_ << ")" << std::endl;
            delete this;
            return;
        }
        this->getCamera()->setInputResourceName("index " + convertValueToString(this->index_));
        msleep(this->delay_);
    }

    void CameraInput::tick()
    {
        IplImage* image = cvQueryFrame(this->capture_);
        if (image)
        {
            this->getCamera()->processImage(image);
        }
        else
        {
            std::cout << "Error: Couldn't retrieve image from camera" << std::endl;
            this->getCamera()->setExitCode(EXIT_FAILURE);
            delete this;
        }
    }
}
