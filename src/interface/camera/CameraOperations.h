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

#ifndef _CameraOperations_H__
#define _CameraOperations_H__

#define RegisterInputOperation(classname) \
    MAVCONN::CameraOperationFactory* __##classname##_camop = &MAVCONN::Camera::declareInputOperation(new MAVCONN::TCameraOperationFactory<classname>())

#define RegisterOutputOperation(classname) \
    MAVCONN::CameraOperationFactory* __##classname##_camop = &MAVCONN::Camera::declareOutputOperation(new MAVCONN::TCameraOperationFactory<classname>())

#include <string>
#include <sstream>
#include <typeinfo>
#include "Camera.h"

namespace MAVCONN
{
    class Camera;

    class CameraOperation
    {
        public:
            CameraOperation(Camera* camera);
            virtual ~CameraOperation() {}

            inline Camera* getCamera() const
                { return this->camera_; }

        private:
            Camera* camera_;
    };

    class InputOperation : public CameraOperation
    {
        public:
            InputOperation(Camera* camera);
            virtual ~InputOperation();

            virtual void startPlayback() = 0;
            virtual void tick() = 0;
    };

    class OutputOperation : public CameraOperation
    {
        public:
            OutputOperation(Camera* camera);
            virtual ~OutputOperation();

            virtual void startProcessing() {}
            virtual void stopProcessing() {}
            virtual void endProcessing() { delete this; }

            virtual void processImage(IplImage* image) = 0;
            virtual void tick() {}

            inline void requestDestruction()
                { this->bRequestedDestruction_ = true; }
            inline bool requestedDestruction() const
                { return this->bRequestedDestruction_; }

        private:
            bool bRequestedDestruction_;
    };

    // Helper classes //

    template <class T> struct TypeStripper
        { typedef T RawType; };
    template <class T> struct TypeStripper<T&>
        { typedef T RawType; };
    template <class T> struct TypeStripper<const T>
        { typedef T RawType; };
    template <class T> struct TypeStripper<const T&>
        { typedef T RawType; };

    template <typename T> inline std::string getTypeDescription()
        { return typeid(T).name(); }
    template <> inline std::string getTypeDescription<bool>()
        { return "bool"; }
    template <> inline std::string getTypeDescription<int>()
        { return "int"; }
    template <> inline std::string getTypeDescription<unsigned int>()
        { return "uint"; }
    template <> inline std::string getTypeDescription<float>()
        { return "float"; }
    template <> inline std::string getTypeDescription<double>()
        { return "double"; }
    template <> inline std::string getTypeDescription<std::string>()
        { return "string"; }

    template <class T>
    bool convertStringToValue(T* target, const std::string& value)
    {
        std::istringstream iss(value);
        iss >> (*target);
        return !iss.fail();
    }

    template <>
    inline bool convertStringToValue<std::string>(std::string* target, const std::string& value)
    {
        *target = value;
        return true;
    }

    template <class T>
    std::string convertValueToString(T value)
    {
        std::ostringstream oss;
        oss << value;
        return oss.str();
    }

    class ParameterExecutor
    {
        public:
            ParameterExecutor(const std::string& name, const std::string& description = "", bool bHasDefaultValue = false) : name_(name), description_(description), bHasDefaultValue_(bHasDefaultValue) {}
            virtual ~ParameterExecutor() {}

            virtual void operator()(CameraOperation* operation) = 0;
            virtual void operator()(CameraOperation* operation, const std::string& parameter) = 0;

            inline const std::string& getName() const
                { return this->name_; }
            inline const std::string& getDescription() const
                { return this->description_; }
            virtual std::string getTypeDescription() const = 0;

            inline bool hasDefaultValue() const
                { return this->bHasDefaultValue_; }
            virtual std::string getDefaultValue() const = 0;

        private:
            std::string name_;
            std::string description_;
            bool bHasDefaultValue_;
    };

    template <class C, typename P>
    class TParameterExecutor : public ParameterExecutor
    {
        public:
            TParameterExecutor(const std::string& name, void (C::*function) (P), const std::string& description)
                : ParameterExecutor(name, description), function_(function) {}
            TParameterExecutor(const std::string& name, void (C::*function) (P), const std::string& description, const typename TypeStripper<P>::RawType& defaultValue)
                : ParameterExecutor(name, description, true), function_(function), defaultValue_(defaultValue) {}

            void operator()(CameraOperation* operation)
            {
                this->callFunction(operation, this->defaultValue_);
            }

            void operator()(CameraOperation* operation, const std::string& parameter)
            {
                typename TypeStripper<P>::RawType value;

                if (convertStringToValue(&value, parameter))
                {
                    this->callFunction(operation, value);
                }
                else
                {
                    std::cout << "Internal error in " << typeid(*this).name() << ": Couldn't convert parameter \"" << parameter << "\" to " << typeid(value).name() << std::endl;
                }
            }

            inline std::string getTypeDescription() const
                { return MAVCONN::getTypeDescription<typename TypeStripper<P>::RawType>(); }

            inline std::string getDefaultValue() const
                { return convertValueToString(this->defaultValue_); }

        private:
            void callFunction(CameraOperation* operation, typename TypeStripper<P>::RawType parameter)
            {
                C* target = dynamic_cast<C*>(operation);

                if (!target)
                {
                    std::cout << "Internal error in " << typeid(*this).name() << ": Couldn't cast pointer" << std::endl;
                    return;
                }

                (target->*this->function_)(parameter);
            }

            void (C::*function_) (P);
            typename TypeStripper<P>::RawType defaultValue_;
    };

    class CameraOperationFactory
    {
        public:
            CameraOperationFactory() : bDefaultOperation_(false) {}
            virtual ~CameraOperationFactory() {}

            virtual CameraOperation* create(Camera* camera) = 0;
            void applyParameters(CameraOperation* operation, const std::string& arguments);

            CameraOperationFactory& addName(const std::string& name);
            bool hasName(const std::string& name) const;
            std::string getFirstName() const;
            inline const std::vector<std::string>& getNames() const
                { return this->names_; }

            inline CameraOperationFactory& setDescription(const std::string& description)
                { this->description_ = description; return *this; }
            inline const std::string& getDescription() const
                { return this->description_; }

            template <class C, typename P>
            CameraOperationFactory& addParameter(const std::string& name, void (C::*function) (P), const std::string& description)
            {
                this->parameters_.push_back(static_cast<ParameterExecutor*>(new TParameterExecutor<C, P>(name, function, description)));
                return *this;
            }
            template <class C, typename P>
            CameraOperationFactory& addParameter(const std::string& name, void (C::*function) (P), const std::string& description, const typename TypeStripper<P>::RawType& defaultValue)
            {
                this->parameters_.push_back(static_cast<ParameterExecutor*>(new TParameterExecutor<C, P>(name, function, description, defaultValue)));
                return *this;
            }
            inline const std::vector<ParameterExecutor*>& getParameters() const
                { return this->parameters_; }

            inline CameraOperationFactory& makeDefault()
                { this->bDefaultOperation_ = true; return *this; }
            inline bool isDefault() const
                { return this->bDefaultOperation_; }

        private:
            std::vector<std::string> names_;
            std::vector<ParameterExecutor*> parameters_;
            std::string description_;
            bool bDefaultOperation_;
    };

    template <class T>
    class TCameraOperationFactory : public CameraOperationFactory
    {
        public:
            CameraOperation* create(Camera* camera)
            {
                return static_cast<CameraOperation*>(new T(camera));
            }
    };
}

#endif /* _CameraOperations_H__ */
