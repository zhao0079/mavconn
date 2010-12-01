/*
 *   ORXONOX - the hottest 3D action shooter ever to exist
 *                    > www.orxonox.net <
 *
 *
 *   License notice:
 *
 *   This program is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU General Public License
 *   as published by the Free Software Foundation; either version 2
 *   of the License, or (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 *   Author:
 *      Reto Grieder
 *   Co-authors:
 *      ...
 *
 */

// 5/10/2009: Adapted to the MAVCONN Project by Fabian Landau

/**
@file
@brief
    Declaration of the Exception class.
*/

#ifndef _Exception_H__
#define _Exception_H__

#include "MAVCONNConfig.h"

#include <string>
#include <exception>
#include <cassert>
#include "Debug.h"

namespace MAVCONN
{
    class Exception : public std::exception
    {
        public:

            Exception(const std::string& description, int lineNumber,
                      const char* filename, const char* functionName);
            Exception(const std::string& description);

            /// Needed for  compatibility with std::exception (from Ogre::Exception)
            virtual ~Exception() throw() { }

            virtual const std::string& getFullDescription() const;
            virtual std::string        getTypeName()        const = 0;
            virtual const std::string& getDescription()     const { return this->description_; }
            virtual int                getLineNumber()      const { return this->lineNumber_; }
            virtual const std::string& getFunctionName()    const { return this->functionName_; }

            /// Override std::exception::what (from Ogre::Exception)
            const char* what() const throw() { return getFullDescription().c_str(); }

        protected:
            std::string description_;
            int lineNumber_;
            std::string functionName_;
            std::string filename_;
            // mutable because "what()" is a const method
            mutable std::string fullDescription_;
    };


#define CREATE_MAVCONN_EXCEPTION(ExceptionName)                                     \
    class ExceptionName##Exception : public Exception                               \
    {                                                                               \
    public:                                                                         \
        ExceptionName##Exception(const std::string& description, int lineNumber,    \
                  const char* filename, const char* functionName)                   \
                  : Exception(description, lineNumber, filename, functionName)      \
        { }                                                                         \
                                                                                    \
        ExceptionName##Exception(const std::string& description)                    \
                  : Exception(description)                                          \
        { }                                                                         \
                                                                                    \
        ~ExceptionName##Exception() throw() { }                                     \
                                                                                    \
        std::string getTypeName() const { return #ExceptionName; }                  \
    };

    // Creates all possible exception types.
    // If you want to add a new type, simply copy and adjust a new line here.
    CREATE_MAVCONN_EXCEPTION(General);
    CREATE_MAVCONN_EXCEPTION(FileNotFound);
    CREATE_MAVCONN_EXCEPTION(Argument);
    CREATE_MAVCONN_EXCEPTION(ParseError);
    CREATE_MAVCONN_EXCEPTION(PluginsNotFound);
    CREATE_MAVCONN_EXCEPTION(InitialisationFailed);
    CREATE_MAVCONN_EXCEPTION(NotImplemented);

    /**
    @brief
        Helper function that creates an exception, displays the message, but doesn't throw it.
    */
    template <class T>
    inline const T& InternalHandleException(const T& exception)
    {
        // let the catcher decide whether to display the message below level 4
        COUT(4) << exception.getFullDescription() << std::endl;
        return exception;
    }
}

#define ThrowException(type, description) \
    throw MAVCONN::InternalHandleException(type##Exception(description, __LINE__, __FILE__, __FUNCTIONNAME__))

/*
// define an assert macro that can display a message
#ifndef NDEBUG
#define OrxAssert(Assertion, ErrorMessage) \
    Assertion ? ((void)0) : (void)(MAVCONN::OutputHandler::getOutStream().setOutputLevel(ORX_ERROR) << ErrorMessage << std::endl); \
    assert(Assertion)
#else
#define OrxAssert(condition, errorMessage)  ((void)0)
#endif
*/
#endif /* _Exception_H__ */
