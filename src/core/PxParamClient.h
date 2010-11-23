/*
 * PxParamClient.h
 *
 *  Created on: 24.04.2010
 *      Author: user
 */

#ifndef PXPARAMCLIENT_H_
#define PXPARAMCLIENT_H_

#include <inttypes.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <tr1/unordered_map>
#include <list>

#include "mavconn.h"
#include "ParamClientCallbacks.h"

using namespace std::tr1;

typedef std::tr1::unordered_map< std::string, float > PxParameterMap;
typedef std::tr1::unordered_map< std::string, PCCallback* > PxParameterCallbackMap;
typedef std::list<PCCallback*> PxCallbackList;
typedef PxParameterMap::value_type ParamPair;

class PxParamClient
{
public:
	PxParamClient(int systemid, int componentid, lcm_t* lcm, const std::string& configFileName="", bool verbose=false) :
		systemid(systemid),
		componentid(componentid),
		lcm(lcm),
		verbose(verbose),
		configFileName(configFileName)
	{
		if (configFileName.size() != 0)
		{
			readParamsFromFile(configFileName);
		}
	}

	~PxParamClient()
	{
		for (PxCallbackList::iterator iter = callbacks.begin(); iter != callbacks.end(); ++iter)
			delete (*iter);
		for (PxParameterCallbackMap::iterator iter = paramCallbacks.begin(); iter != paramCallbacks.end(); ++iter)
			delete iter->second;
	}

protected:
	PxParameterMap params;
	int systemid;
	int componentid;
	lcm_t* lcm;
	bool verbose;
	std::string configFileName;
	PxCallbackList callbacks;
	PxParameterCallbackMap paramCallbacks;


public:

	void handleMAVLinkPacket(const mavlink_message_t* msg)
	{
		switch (msg->msgid)
		{
		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		{
			// Start sending parameters
			if (verbose) printf("PxParamClient: Requested parameters, sending them now..\n");

			uint16_t i = 0;
			PxParameterMap::const_iterator iter = params.begin();
			while(iter != params.end())
			{
				mavlink_message_t response;
				mavlink_msg_param_value_pack(systemid, componentid, &response, (int8_t*)((*iter).first.c_str()), (*iter).second, params.size(), i);
				mavlink_message_t_publish (lcm, "MAVLINK", &response);
				if (verbose) std::cout << "Sending param " << (*iter).first  << ':' << (*iter).second << std::endl;
				++iter;
				i++;
			}
		}
		break;
		case MAVLINK_MSG_ID_PARAM_SET:
		{
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			// Check if this message is for this system
			if ((uint8_t) set.target_system
					== (uint8_t) systemid && (uint8_t) set.target_component
					== componentid)
			{
				const char* key = (char*) set.param_id;
				std::string paramName(key);

				if (params.count(paramName) > 0)
				{
					params.erase(paramName);
					params.insert(std::make_pair(paramName, set.param_value));
				}

				uint16_t i = 0;
				for (PxCallbackList::iterator iter = callbacks.begin(); iter != callbacks.end(); ++iter)
				{
					(**iter)(paramName, set.param_value);
					i++;
				}
				i--;
				if (paramCallbacks.count(paramName) > 0)
					(*paramCallbacks[paramName])(paramName, set.param_value);

				// Report back new value
				mavlink_message_t response;
				mavlink_msg_param_value_pack(systemid, componentid, &response, set.param_id, set.param_value, params.size(), i);
				mavlink_message_t_publish (lcm, "MAVLINK", &response);
			}
		}
		case MAVLINK_MSG_ID_ACTION:
		{
			mavlink_action_t action;
			mavlink_msg_action_decode(msg, &action);
			switch (action.action)
			{
			case MAV_ACTION_STORAGE_READ:
				readParamsFromFile(configFileName);
				if (verbose) printf("Reading parameters from file %s", configFileName.c_str());
				break;
			case MAV_ACTION_STORAGE_WRITE:
				if (verbose) printf("Writing parameters from file %s", configFileName.c_str());
				writeParamsToFile(configFileName);
				break;
			}
		}
		break;
		}
	}

	float getParamValue(const std::string& key)
	{
		return params.find(key)->second;
	}

	bool setParamValue(const std::string& paramName, float value)
	{
		bool updated = false;
		if (params.count(paramName) > 0)
		{
			params.erase(paramName);
			updated = true;
		}
		params.insert(std::make_pair(paramName, value));

		for (PxCallbackList::iterator iter = callbacks.begin(); iter != callbacks.end(); ++iter)
			(**iter)(paramName, value);
		if (paramCallbacks.count(paramName) > 0)
			(*paramCallbacks[paramName])(paramName, value);

		return updated;
	}

	bool setParamValue(const std::string& paramName, float value, PCCallback* callback)
	{
	    this->setCallback(paramName, callback);
	    return this->setParamValue(paramName, value);
	}

	template <typename T>
	bool registerVariable(const std::string& paramName, T* variable)
	{
		this->setCallback(paramName, createPCCallback(variable));
		return this->setParamValue(paramName, *variable);
	}

	template <typename T>
	bool registerVariable(const std::string& paramName, T* variable, T value)
	{
		this->setCallback(paramName, createPCCallback(variable));
		return this->setParamValue(paramName, value);
	}

	void setCallback(PCCallback* callback)
	{
		callbacks.push_back(callback);
	}

	void setCallback(const std::string& paramName, PCCallback* callback)
	{
		if (paramCallbacks.count(paramName) > 0)
		{
			delete paramCallbacks[paramName];
			paramCallbacks.erase(paramName);
		}
		paramCallbacks[paramName] = callback;
	}

	void printParams()
	{
		std::tr1::hash< std::string > hashfunc = params.hash_function();
		//	    hashmap::const_iterator e;
		//	    for( hashmap::const_iterator i = params.begin(), e = params.end() ; i != e ; ++i )
		//	    {
		//	        std::cout << i->first << " -> " << i->second << " (hash = " << hashfunc( i->first ) << ")" << std::endl;
		//	    }

		PxParameterMap::const_iterator iter = params.begin();
		while(iter != params.end())
		{
			std::cout << (*iter).first  << ':' << (*iter).second << std::endl;
			++iter;
		}
	}

	void readParamsFromFile(const std::string& fileName)
	{
		// Load the calibration file
		std::ifstream infile;
		infile.open(fileName.c_str());
		if (!infile)
		{
			fprintf(stderr, "Unable to open file %s\n", fileName.c_str());
			return;	// terminate with error
		}

		std::string paramName;
		float paramValue;
		while (!infile.eof())
		{
			char c = (char)infile.peek();
			if (c == '#')
			{
				infile.ignore(1024, '\n');
			}
			else if (c == '\n' || c == ' ' || c == '\r' /* windows... */)
			{
				infile.ignore(1);
			}
			else
			{
				paramName.clear();
				infile >> paramName;
				infile >> paramValue;

				if(paramName.length() > 0)
				{
					setParamValue(paramName, paramValue);
				}
			}
		}
		infile.close();
	}

	void writeParamsToFile(const std::string& fileName)
	{
		std::ofstream outfile;
		outfile.open(fileName.c_str());
		if (!outfile)
		{
			fprintf(stderr, "Unable to open file %s\n", fileName.c_str());
			return;
		}

		// Write all parameters to file
		PxParameterMap::const_iterator iter = params.begin();
		while(iter != params.end())
		{
			outfile << (*iter).first  << "\t" << (*iter).second << std::endl;
			++iter;
		}
		outfile.close();
	}

	inline void setLcm(lcm_t* lcm)
	{
		this->lcm = lcm;
	}
};

#endif /* PXPARAMCLIENT_H_ */
