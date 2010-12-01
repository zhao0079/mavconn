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

#include "IpcInput.h"

#include <cmuipc/ipc.h>
#include <ipc_core.sig>

namespace MAVCONN
{
    RegisterInputOperation(IpcInput)
        .setDescription("Retrieves frames over IPC (central must be running)")
        .addName("ipc")
        .addName("i")
    ;

    static IpcInput* operation_s = 0;
    static void getImage(IplImage* image)
    {
        if (operation_s)
            operation_s->getCamera()->processImage(image);
    }

    /**
     *  @brief Gets called if a module disconnects. Check if it's the image server stop playback.
     */
    static void serverDisconnected(const char* moduleName, void* clientData)
    {
        if (strcmp(moduleName, IMGSERVERNAME) == 0)
        {
            if (operation_s)
                operation_s->getCamera()->stopPlayback(operation_s);
        }
    }

    IpcInput::IpcInput(Camera* camera) : InputOperation(camera)
    {
        this->getCamera()->setInputModeName("IPC");

        operation_s = this;
    }

    IpcInput::~IpcInput()
    {
        this->imageClient_.disconnect();

        operation_s = 0;
    }

    void IpcInput::startPlayback()
    {
        if (!this->imageClient_.connect("CameraIpcInput"))
        {
            std::cout << "Couldn't start the IPC image-client module" << std::endl;
            delete this;
            return;
        }

        this->imageClient_.start(&getImage);
        this->getCamera()->setHasIPC(true);

        IPC_subscribeDisconnect(serverDisconnected, NULL);
    }

    void IpcInput::tick()
    {
        this->imageClient_.pollForMessages(10);
    }
}
