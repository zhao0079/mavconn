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

#include "CameraOperations.h"
#include "Camera.h"

namespace MAVCONN
{
    // CameraOperation //
    CameraOperation::CameraOperation(Camera* camera)
    {
        this->camera_ = camera;
    }

    // InputOperation //
    InputOperation::InputOperation(Camera* camera) : CameraOperation(camera)
    {
        this->getCamera()->registerCameraOperation(this);
    }

    InputOperation::~InputOperation()
    {
        this->getCamera()->unregisterCameraOperation(this);
    }

    // OutputOperation //
    OutputOperation::OutputOperation(Camera* camera) : CameraOperation(camera)
    {
        this->bRequestedDestruction_ = false;
        this->getCamera()->registerCameraOperation(this);
    }

    OutputOperation::~OutputOperation()
    {
        this->getCamera()->unregisterCameraOperation(this);
    }

    // CameraOperationFactory //
    CameraOperationFactory& CameraOperationFactory::addName(const std::string& name)
    {
        this->names_.push_back(name);
        return *this;
    }

    bool CameraOperationFactory::hasName(const std::string& name) const
    {
        for (size_t i = 0; i < this->names_.size(); ++i)
            if (this->names_[i] == name)
                return true;

        return false;
    }

    std::string CameraOperationFactory::getFirstName() const
    {
        if (this->names_.size() > 0)
            return this->names_[0];
        else
            return "";
    }

    void CameraOperationFactory::applyParameters(CameraOperation* operation, const std::string& arguments)
    {
        std::vector<std::string> values(this->parameters_.size());

        size_t index = 0;
        size_t end = 0;
        size_t start = arguments.find(':');
        while (start != std::string::npos && index < this->parameters_.size())
        {
            end = arguments.find(':', start + 1);
            std::string parameter = arguments.substr(start + 1, end - start - 1);

            values[index] = parameter;

            ++index;
            start = end;
        }

        if (start != std::string::npos && start != (arguments.size() - 1))
        {
            if (dynamic_cast<InputOperation*>(operation))
                std::cout << "Warning: Superfluous arguments \"" << arguments.substr(start + 1) << "\" were ignored. The input operation \"" << this->getFirstName() <<"\" takes only " << this->parameters_.size() << " parameters." << std::endl;
            else
                std::cout << "Warning: Superfluous arguments \"" << arguments.substr(start + 1) << "\" were ignored. The output operation \"" << this->getFirstName() <<"\" takes only " << this->parameters_.size() << " parameters." << std::endl;
        }

        for (index = 0; index < this->parameters_.size(); ++index)
        {
            if (values[index] != "")
                (*this->parameters_[index])(operation, values[index]);
            else if (this->parameters_[index]->hasDefaultValue())
                (*this->parameters_[index])(operation);
            else
            {
                if (dynamic_cast<InputOperation*>(operation))
                    std::cout << "Warning: Input operation \"" << this->getFirstName() <<"\": No value for parameter \"" << this->parameters_[index]->getName() << "\" at position " << (index + 1) << " given." << std::endl;
                else
                    std::cout << "Warning: Output operation \"" << this->getFirstName() <<"\": No value for parameter \"" << this->parameters_[index]->getName() << "\" at position " << (index + 1) << " given." << std::endl;
            }
        }

    }
}
