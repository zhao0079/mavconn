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

#include "Camera.h"

#include <fstream>
#include <boost/program_options.hpp>


#define CAPTURED_FRAME_WIDTH 640 // ((65536 - 4 - 4) / 3) // 752 // 100
#define CAPTURED_FRAME_HEIGHT 480 // 1 // 480 // 64
#define CAPTURED_FRAME_N_COLOR_CHANNELS 1

#define IMG_BYTE_SIZE CAPTURED_FRAME_WIDTH * CAPTURED_FRAME_HEIGHT * CAPTURED_FRAME_N_COLOR_CHANNELS


#include "CameraOperations.h"
#include "PauseInput.h"

namespace config = boost::program_options;

namespace MAVCONN
{
    Camera::Camera()
    {
        this->vm_ = new boost::program_options::variables_map();
        this->input_ = 0;
        this->pausedInput_ = 0;
        this->inputFactory_ = 0;
        this->bPlaybackActive_ = false;
        this->bRequestedRestart_ = false;
        this->bRequestedExit_ = false;
        this->bHasIPC_ = false;
        this->bPaused_ = false;
        this->bStoped_ = false;
        this->bToggledPause_ = false;
        this->exitCode_ = EXIT_SUCCESS;
    }

    Camera::~Camera()
    {
        if (this->input_)
            delete this->input_;

        if (this->pausedInput_)
            delete this->pausedInput_;

        while (this->output_.size() > 0)
            delete (*this->output_.begin());

        delete this->vm_;
    }

    void Camera::parseConfigValues(int argc, char* argv[])
    {
        std::string default_input;
        for (size_t i = 0; i < Camera::getInputOperations().size(); ++i)
            if (Camera::getInputOperations()[i]->isDefault())
                default_input = Camera::getInputOperations()[i]->getFirstName();

        std::vector<std::string> default_output;
        std::string default_output_string;
        for (size_t i = 0; i < Camera::getOutputOperations().size(); ++i)
        {
            if (Camera::getOutputOperations()[i]->isDefault())
            {
                default_output.push_back(Camera::getOutputOperations()[i]->getFirstName());
                if (default_output.size() > 1)
                    default_output_string += ", ";
                default_output_string += Camera::getOutputOperations()[i]->getFirstName();
            }
        }

        config::options_description visible("Allowed options");
        visible.add_options()
            ("input,i", config::value<std::string>()->default_value(default_input), "The input resource (exactly one required)\n Use 'camera --help input' for a list of\n allowed input resources")
            ("output,o", config::value<std::vector<std::string> >()->default_value(default_output, default_output_string), "The output devices (mutliple allowed)\n Use 'camera --help output' for a list of\n allowed output devices")
            ("presets,p", config::value<std::string>(), "A presets file containing program options")
            ("snapshot,s", config::value<std::string>()->default_value(""), "The mask for the filename of snapshots\nSee frame input for more details")
            ("help,h", "Print help about a topic (see below)")
        ;

        config::options_description hidden("Hidden options");
        hidden.add_options()
            ("helptopic", config::value<std::vector<std::string> >(), "Help topic")
        ;

        config::options_description alloptions;
        alloptions.add(visible).add(hidden).add(hidden);

        config::positional_options_description positional;
        positional.add("helptopic", 2);

        try
        {
            config::store(config::command_line_parser(argc, argv).options(alloptions).positional(positional).run(), *this->vm_);
        }
        catch (std::exception& e)
        {
            std::cout << "Error: An exception occurred while parsing the command line arguments:" << std::endl << "       " << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }

        if (this->vm_->count("help"))
        {
            if (this->vm_->count("helptopic") == 0)
            {
                // display allowed program options (boost::program_options)
                std::cout << visible << std::endl;
            }
            else
            {
                const std::vector<std::string>& helptopic = (*this->vm_)["helptopic"].as<std::vector<std::string> >();

                if (helptopic[0] == "input")
                {
                    if (helptopic.size() == 1)
                    {
                        // list input resources
                        std::cout << "Input resources:" << std::endl;
                        std::cout << Camera::getCameraOperationDescriptionList(Camera::getInputOperations());
                        std::cout << std::endl;
                        std::cout << "Use 'camera --help input <name>' to get more information about a specific input resource." << std::endl;
                        std::cout << std::endl;

                        exit(EXIT_SUCCESS);
                    }
                    else
                    {
                        // print details about a specific input resource
                        for (size_t i = 0; i < Camera::getInputOperations().size(); ++i)
                        {
                            if (Camera::getInputOperations()[i]->hasName(helptopic[1]))
                            {
                                std::cout << "Detailed information about the input resource \"" << helptopic[1] << "\":" << std::endl;
                                std::cout << Camera::getCameraOperationDetails(Camera::getInputOperations()[i]);
                                std::cout << std::endl;

                                exit(EXIT_SUCCESS);
                            }
                        }

                        std::cout << "Error: Unknown input resource \"" << helptopic[1] << "\"" << std::endl << std::endl;
                    }
                }
                else if (helptopic[0] == "output")
                {
                    if (helptopic.size() == 1)
                    {
                        // list output resources
                        std::cout << "Output devices:" << std::endl;
                        std::cout << Camera::getCameraOperationDescriptionList(Camera::getOutputOperations());
                        std::cout << std::endl;
                        std::cout << "Use 'camera --help output <name>' to get more information about a specific output device." << std::endl;
                        std::cout << std::endl;

                        exit(EXIT_SUCCESS);
                    }
                    else
                    {
                        // print details about a specific output device
                        for (size_t i = 0; i < Camera::getOutputOperations().size(); ++i)
                        {
                            if (Camera::getOutputOperations()[i]->hasName(helptopic[1]))
                            {
                                std::cout << "Detailed information about the output device \"" << helptopic[1] << "\":" << std::endl;
                                std::cout << Camera::getCameraOperationDetails(Camera::getOutputOperations()[i]);
                                std::cout << std::endl;

                                exit(EXIT_SUCCESS);
                            }
                        }

                        std::cout << "Error: Unknown output device \"" << helptopic[1] << "\"" << std::endl << std::endl;
                    }
                }
                else
                {
                    std::cout << "Error: Unknown help topic \"" << helptopic[0] << "\"" << std::endl << std::endl;
                }
            }

            std::cout << "Allowed help topics (usage: 'camera --help <topic>'):" << std::endl;
            std::cout << "  input" << std::endl;
            for (size_t i = 0; i < Camera::getInputOperations().size(); ++i)
                std::cout << "  input " << Camera::getInputOperations()[i]->getFirstName() << std::endl;
            std::cout << "  output" << std::endl;
            for (size_t i = 0; i < Camera::getOutputOperations().size(); ++i)
                std::cout << "  output " << Camera::getOutputOperations()[i]->getFirstName() << std::endl;
            std::cout << std::endl;

            if (this->vm_->count("helptopic") == 0)
            {
                std::cout << "Example I: (All four commands express the same)" << std::endl;
                std::cout << "  > camera --input camera --output display [long options, long arguments]" << std::endl;
                std::cout << "  > camera --input c --output d            [long options, short arguments]" << std::endl;
                std::cout << "  > camera -icamera -odisplay              [short options, long arguments]" << std::endl;
                std::cout << "  > camera -ic -od                         [short options, short arguments]" << std::endl;
                std::cout << std::endl;
                std::cout << "Example II: (Use parameters)" << std::endl;
                std::cout << "  > camera -if:/home/me/pictures/sample.jpg:1:10 -od:0 -ov:sample.avi:10" << std::endl;
                std::cout << std::endl;
                std::cout << "  Input: Image file, repeated with 10 fps" << std::endl;
                std::cout << "  Output 1: Display without information overlay" << std::endl;
                std::cout << "  Output 2: Video with 10 fps" << std::endl;
                std::cout << std::endl;
            }

            exit(EXIT_SUCCESS);
        }

        if (this->vm_->count("presets"))
        {
            std::filebuf file;
            file.open((*this->vm_)["presets"].as<std::string>().c_str(), std::ios::in);

            if (file.is_open())
            {
                std::istream stream(&file);
                config::store(config::parse_config_file(stream, alloptions), *this->vm_);
            }
            else
            {
                std::cout << "Error: Presets file \"" << (*this->vm_)["presets"].as<std::string>() << "\" doesn't exist." << std::endl;
            }
        }
    }

    /* static */ std::string Camera::getCameraOperationDescription(CameraOperationFactory* factory)
    {
        std::string output;
        const std::vector<std::string>& names = factory->getNames();

        if (names.size() >= 2)
            output += names[1] + " ";

        output += "[ ";
        for (size_t i = 0; i < names.size(); ++i)
        {
            if (i == 1)
                continue;

            if (i > 0)
                output += ", ";

            output += names[i];
        }
        output += " ]";

        if (factory->isDefault())
            output += " (default)";

        return output;
    }

    /* static */ std::string Camera::getCameraOperationDescriptionList(const std::vector<CameraOperationFactory*>& factories)
    {
        std::string output;
        std::vector<std::string> descriptions;
        size_t maxlength = 0;

        for (size_t i = 0; i < factories.size(); ++i)
        {
            std::string description = Camera::getCameraOperationDescription(factories[i]);
            if (description.size() > maxlength)
                maxlength = description.size();
            descriptions.push_back(description);
        }

        assert(descriptions.size() == factories.size());

        for (size_t i = 0; i < factories.size(); ++i)
        {
            output += "  ";
            output += descriptions[i];

            for (size_t spacing = descriptions[i].size(); spacing < maxlength; ++spacing)
                output += ' ';

            std::string description = factories[i]->getDescription();

            if (description.size() > 0)
            {
                output += ' ';
                output += Camera::getWrapedLine(description, maxlength + 3);
            }

            output += '\n';
        }

        return output;
    }

    /* static */ std::string Camera::getParameterDescription(ParameterExecutor* parameter)
    {
        std::string output;

        output += parameter->getName();
        output += " [";
        output += parameter->getTypeDescription();
        output += "]";

        if (parameter->hasDefaultValue())
        {
            output += " (=";
            output += parameter->getDefaultValue();
            output += ")";
        }

        return output;
    }

    /* static */ std::string Camera::getParameterDescriptionList(const std::vector<ParameterExecutor*>& parameters)
    {
        std::string output;
        std::vector<std::string> descriptions;
        size_t maxlength = 0;

        for (size_t i = 0; i < parameters.size(); ++i)
        {
            std::string description = Camera::getParameterDescription(parameters[i]);

            if (description.size() > maxlength)
                maxlength = description.size();

            descriptions.push_back(description);

            output += ":";
            output += parameters[i]->getName();
        }

        if (parameters.size() > 0)
        {
            output += "\n\n";
        }

        assert(descriptions.size() == parameters.size());

        for (size_t i = 0; i < parameters.size(); ++i)
        {
            output += "    #";
            output += convertValueToString(i + 1);
            output += ": ";
            output += descriptions[i];

            for (size_t spacing = descriptions[i].size(); spacing < maxlength; ++spacing)
                output += ' ';

            std::string description = Camera::getWrapedLine(parameters[i]->getDescription(), maxlength + 9);

            if (description.size() > 0)
            {
                output += ' ';
                output += description;
            }

            output += '\n';
        }

        return output;
    }

    /* static */ std::string Camera::getCameraOperationDetails(CameraOperationFactory* factory)
    {
        std::string output;

        const std::vector<std::string>& names = factory->getNames();
        if (names.size() > 0)
        {
            output += "  Name: ";
            output += names[0];
            output += '\n';

            if (names.size() > 1)
            {
                output += "  Shortcut: ";
                output += names[1];
                output += '\n';

                if (names.size() > 2)
                {
                    if (names.size() == 3)
                        output += "  Alias: ";
                    else
                        output += "  Aliases: ";

                    for (size_t n = 2; n < names.size(); ++n)
                    {
                        if (n > 2)
                            output += ", ";

                        output += names[n];
                    }
                    output += '\n';
                }
            }
        }

        std::string description = factory->getDescription();
        if (description.size() > 0)
        {

            output += '\n';
            output += "  Description: ";
            output += Camera::getWrapedLine(description, 15);
            output += '\n';
        }

        const std::vector<ParameterExecutor*>& parameters = factory->getParameters();
        if (parameters.size() > 0)
        {
            output += '\n';
            output += "  Parameters:\n";
            output += "    ";
            output += factory->getFirstName();
            output += Camera::getParameterDescriptionList(parameters);
        }

        return output;
    }

    /* static */ std::string Camera::getWrapedLine(const std::string& line, size_t indentation, size_t maxWidth)
    {
        std::string output;

        size_t width = maxWidth - indentation;
        size_t start = 0;

        while (true)
        {
            size_t end = line.find_first_of('\n', start);
            bool bAutowrap = false;

            if (end > start + width)
            {
                end = start + width;
                bAutowrap = true;
            }

            output += line.substr(start, end - start);

            if (end < line.size())
            {
                output += '\n';

                if (bAutowrap)
                    for (start = end; start < line.size() && line[start] == ' '; ++start);
                else
                    start = end + 1;

                for (size_t i = 0; i < indentation; ++i)
                    output += ' ';

                continue;
            }
            else
            {
                break;
            }
        }

        return output;
    }

    void Camera::createCameraOperations()
    {
        // create the input operation
        if (this->vm_->count("input"))
        {
            std::string name = this->getArgumentName((*this->vm_)["input"].as<std::string>());

            bool success = false;
            for (size_t i = 0; i < Camera::getInputOperations().size(); ++i)
            {
                if (Camera::getInputOperations()[i]->hasName(name))
                {
                    this->inputFactory_ = Camera::getInputOperations()[i];
                    success = this->restart();
                    break;
                }
            }

            if (!success)
                std::cout << "Error: \"" << name << "\" is not a valid input operation" << std::endl;

            if (!this->input_)
                return;
        }
        else
        {
            std::cout << "Error: No input operation specified" << std::endl;
            return;
        }

        // create the output operation
        if (this->vm_->count("output"))
        {
            const std::vector<std::string>& output = (*this->vm_)["output"].as<std::vector<std::string> >();
            for (size_t i = 0; i < output.size(); ++i)
            {
                std::string arguments = output[i];
                std::string name = this->getArgumentName(arguments);

                bool success = false;
                for (size_t i = 0; i < Camera::getOutputOperations().size(); ++i)
                {
                    if (Camera::getOutputOperations()[i]->hasName(name))
                    {
                        CameraOperation* operation = Camera::getOutputOperations()[i]->create(this);
                        Camera::getOutputOperations()[i]->applyParameters(operation, arguments);
                        success = true;
                        break;
                    }
                }

                if (!success)
                    std::cout << "Warning: \"" << name << "\" is not a valid output operation" << std::endl;
            }

            if (this->output_.size() == 0)
            {
                std::cout << "Error: No valid output operation specified" << std::endl;
                return;
            }
        }
        else
        {
            std::cout << "Error: No output operation specified" << std::endl;
            return;
        }

        this->input_->startPlayback();
    }

    /* static */ std::string Camera::getArgumentName(const std::string& argument)
    {
        return argument.substr(0, argument.find(':'));
    }

    void Camera::run()
    {
        while (this->output_.size() > 0 && !this->bRequestedExit_)
        {
            if (this->input_)
                this->input_->tick();

            for (std::list<OutputOperation*>::iterator it = this->output_.begin(); it != this->output_.end(); )
                (*(it++))->tick(); // use it++ because the OutputOperation may delete itself during tick()

            for (std::list<OutputOperation*>::iterator it = this->output_.begin(); it != this->output_.end(); )
            {
                if ((*it)->requestedDestruction())
                    (*(it++))->endProcessing(); // use it++ because the OutputOperation may delete itself during endProcessing()
                else
                    ++it;
            }

            ::usleep(100);

            if (this->bRequestedRestart_)
            {
                if (this->restart())
                    this->input_->startPlayback();
            }
        }
    }

    void Camera::processImage(IplImage* image)
    {
        if (!this->bPlaybackActive_)
            this->startPlayback(this->input_);

        for (std::list<OutputOperation*>::iterator it = this->output_.begin(); it != this->output_.end(); )
            (*(it++))->processImage(image); // use it++ because the OutputOperation may delete itself during processImage()

        if (this->bToggledPause_)
            this->pause(image);
    }

    void Camera::pause(IplImage* image)
    {
        this->bToggledPause_ = false;
        this->bPaused_ = !this->bPaused_;

        if (this->bPaused_)
        {
            if (this->pausedInput_)
                delete this->pausedInput_;
            this->pausedInput_ = this->input_;
            this->input_ = 0;
            new PauseInput(this, image);
        }
        else
        {
            InputOperation* temp = this->input_;
            this->input_ = this->pausedInput_;
            this->pausedInput_ = 0;
            if (temp)
                delete temp;
        }
    }

    bool Camera::restart()
    {
        this->bRequestedRestart_ = false;

        if (!this->input_ && this->inputFactory_)
        {
            CameraOperation* operation = this->inputFactory_->create(this);
            this->inputFactory_->applyParameters(operation, (*this->vm_)["input"].as<std::string>());
            this->bStoped_ = false;
            return true;
        }

        return false;
    }

    void Camera::startPlayback(InputOperation* input)
    {
        if (input == this->input_ && !this->bPlaybackActive_)
        {
            this->bPlaybackActive_ = true;

            for (std::list<OutputOperation*>::iterator it = this->output_.begin(); it != this->output_.end(); ++it)
                (*it)->startProcessing();
        }
    }

    void Camera::stopPlayback(InputOperation* input)
    {
        if (input == this->input_ && this->bPlaybackActive_)
        {
            this->bPlaybackActive_ = false;

            for (std::list<OutputOperation*>::iterator it = this->output_.begin(); it != this->output_.end(); ++it)
                (*it)->stopProcessing();
        }
    }

    void Camera::registerCameraOperation(InputOperation* input)
    {
        if (this->input_)
            delete this->input_;

        this->input_ = input;
    }

    void Camera::unregisterCameraOperation(InputOperation* input)
    {
        if (this->input_ == input)
        {
//            if (this->bPlaybackActive_)
//                this->stopPlayback(input);

            this->input_ = 0;
            this->bStoped_ = true;

            for (std::list<OutputOperation*>::iterator it = this->output_.begin(); it != this->output_.end(); )
                (*(it++))->endProcessing(); // use it++ because the OutputOperation may delete itself during endProcessing()
        }
    }

    void Camera::registerCameraOperation(OutputOperation* output)
    {
        this->output_.push_back(output);

        if (this->bPlaybackActive_)
            output->startProcessing();
    }

    void Camera::unregisterCameraOperation(OutputOperation* output)
    {
        for (std::list<OutputOperation*>::iterator it = this->output_.begin(); it != this->output_.end(); ++it)
        {
            if (*it == output)
            {
                this->output_.erase(it);
                return;
            }
        }
    }

    std::vector<CameraOperationFactory*>& Camera::getInputOperations()
    {
        static std::vector<CameraOperationFactory*> inputOperations;
        return inputOperations;
    }

    std::vector<CameraOperationFactory*>& Camera::getOutputOperations()
    {
        static std::vector<CameraOperationFactory*> outputOperations;
        return outputOperations;
    }

    CameraOperationFactory& Camera::declareInputOperation(CameraOperationFactory* factory)
    {
        Camera::getInputOperations().push_back(factory);
        return *factory;
    }

    CameraOperationFactory& Camera::declareOutputOperation(CameraOperationFactory* factory)
    {
        Camera::getOutputOperations().push_back(factory);
        return *factory;
    }

    const std::string& Camera::getSnapshotMask() const
    {
        return (*this->vm_)["snapshot"].as<std::string>();
    }
}
