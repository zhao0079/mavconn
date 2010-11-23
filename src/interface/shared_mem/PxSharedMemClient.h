/*
 * PxSharedMemClient.h
 *
 *  Created on: 24.04.2010
 *      Author: user
 */

#ifndef PXSHAREDMEMCLIENT_H_
#define PXSHAREDMEMCLIENT_H_

#include <cv.hpp>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <inttypes.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>

#include "mavconn.h"

class PxSharedMemClient
{
public:
	PxSharedMemClient()
	{
		shms = new char*[256];
		shm_sizes = new int[256];

		for (size_t i = 0; i < 256; ++i)
		    shms[i] = NULL;
		for (size_t i = 0; i < 256; ++i)
		    shm_sizes[i] = 0;
	}

	~PxSharedMemClient()
	{
		// Detach shared memory segment
		shmdt(shm);

		delete[] shms;
		delete[] shm_sizes;
	}

protected:
	void attachSharedMem(int key, int shm_size, char** r_shm)
	{
		int shmid;
		// Create shared memory segment
		key_t m_key = 2011;
		if ((shmid = shmget(m_key, shm_size, IPC_CREAT | 0666)) < 0)
		{
			perror("shmget");
			exit(EXIT_FAILURE);
		}

		// Attach shared memory segment
		if((*r_shm = (char*)shmat(shmid, NULL, SHM_RDONLY)) == (char*)-1)
			//if((r_shm = (char*)shmat(shmid, NULL, 0)) == (char*)-1)
		{
			perror("shmat");
			exit(EXIT_FAILURE);
		}
		printf("\nRECEIVING IMAGES SUCCESSFULLY FROM NEW CAMERA, ATTACHED NEW BUFFER (key: %d, size: %d)\n", key, shm_size);
	}

public:

	uint64_t getTimestamp(const mavlink_message_t* msg)
	{
		// Decode message
		if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
		{
			// Instantly return if MAVLink message did not contain an image
			return 0;
		}
		else
		{
			// Extract the image meta information and pointer location from the image
			mavlink_image_available_t img;
			mavlink_msg_image_available_decode(msg, &img);
			return img.timestamp;
		}
	}

	uint64_t getValidUntil(const mavlink_message_t* msg)
	{
		// Decode message
		if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
		{
			// Instantly return if MAVLink message did not contain an image
			return 0;
		}
		else
		{
			// Extract the image meta information and pointer location from the image
			mavlink_image_available_t img;
			mavlink_msg_image_available_decode(msg, &img);
			return img.valid_until;
		}
	}

	uint64_t getCameraID(const mavlink_message_t* msg)
	{
		// Decode message
		if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
		{
			// Instantly return if MAVLink message did not contain an image
			return 0;
		}
		else
		{
			// Extract the image meta information and pointer location from the image
			mavlink_image_available_t img;
			mavlink_msg_image_available_decode(msg, &img);
			return img.cam_id;
		}
	}

	int getCameraNo(const mavlink_message_t* msg)
	{
		// Decode message
		if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
		{
			// Instantly return if MAVLink message did not contain an image
			return -1;
		}
		else
		{
			// Extract the image meta information and pointer location from the image
			mavlink_image_available_t img;
			mavlink_msg_image_available_decode(msg, &img);
			return img.cam_no;
		}
	}

	bool getRollPitch(const mavlink_message_t* msg, float *roll, float *pitch)
	{
		// Decode message
		if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
		{
			// Instantly return if MAVLink message did not contain an image
			return false;
		}
		else
		{
			// Extract the image meta information and pointer location from the image
			mavlink_image_available_t img;
			mavlink_msg_image_available_decode(msg, &img);

			if (roll)
				*roll = img.roll;

			if (pitch)
				*pitch = img.pitch;

			return true;
		}
	}

	bool sharedMemCopyImage(const mavlink_message_t* msg, cv::Mat& img)
	{
		IplImage iplImg = img;  // only header created, no data copied
		return sharedMemCopyImage(msg, &iplImg);
	}

	bool sharedMemCopyImage(const mavlink_message_t* msg, IplImage* ret1)
	{
		// Decode message
		if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
		{
			// Instantly return if MAVLink message did not contain an image
			return NULL;
		}
		else
		{
			// Extract the image meta information and pointer location from the image
			mavlink_image_available_t img;
			mavlink_msg_image_available_decode(msg, &img);
			int cam = (int)img.cam_no;
			//int size = img.width * img.height * img.depth/8 * img.channels;
			int size = (img.width * img.height * img.depth * img.channels) / 8;
			//std::cout << "cam #" << (int)img.cam_no << " key: " << img.key << " size: " << size << " width: " << (int)img.width << " height: " << (int)img.height << " depth: " << (int)img.depth << " channels: " << (int)img.channels << " " << std::endl;

			// Check if shared memory for this camera is already attached
			if (shms[cam] == NULL || shm_sizes[cam] != size)
			{
				//attachSharedMem((int)img.key, size, &shm);
				shm_sizes[cam] = size;
			}

			shms[cam] = shm;
			char* ptr;// = shm;
			key_t m_key = img.key;

			int shmid;
			if ((shmid = shmget(m_key, size, IPC_CREAT | 0666)) < 0)
			{
				perror("shmget");
				return false;
			}

			// Attach shared memory segment
			if((ptr = (char*)shmat(shmid, NULL, SHM_RDONLY)) == (char*)-1)
				//if((r_shm = (char*)shmat(shmid, NULL, 0)) == (char*)-1)
			{
				perror("shmat");
				return false;
			}

			//cvCreateImageHeader(cvSize(img.width,img.height),IPL_DEPTH_8U, img.channels);
			//char* raw = (char*) malloc(size);
			// Check if memory could be allocated
			//if (raw == NULL || ptr == NULL) return NULL;

			// Copy message into IPLImage
			// Copy image at position img_buf_index (multi-image buffer)
			// FIXME should check the img.valid_until field to determine if the image
			// is still valid
			//memcpy(ret1->imageData, ptr+size*img.img_buf_index, size);	// original before img_buf_index change for fast stereo hack
			//memcpy(ret1->imageData, ptr+(img.cam_no*size), size);
			memcpy(ret1->imageData, ptr, size);

			//memcpy(raw, ptr, size);
			shmdt(ptr);
			//cvSetData(ret, raw, (int)img.width);
			return true;
		}
	}

	// Wrapper to use openCV 2.x cv::Mat as image container
	bool sharedMemCopyStereoImage(const mavlink_message_t* msg, cv::Mat& imgLeft, cv::Mat& imgRight)
	{
		IplImage iplImgLeft = imgLeft;
		IplImage iplImgRight = imgRight;	// only header created, no data copied
		return sharedMemCopyStereoImage(msg, &iplImgLeft, &iplImgRight);
	}

	bool sharedMemCopyStereoImage(const mavlink_message_t* msg, IplImage* ret1, IplImage* ret2)
	{
		// Decode message
		if (msg->msgid != MAVLINK_MSG_ID_IMAGE_AVAILABLE)
		{
			// Instantly return if MAVLink message did not contain an image
			return false;
		}
		else
		{
			// Extract the image meta information and pointer location from the image
			mavlink_image_available_t img;
			mavlink_msg_image_available_decode(msg, &img);

			if (img.img_buf_index != 2)
				return false;

			int cam = (int)img.cam_no;
			int size = img.width * img.height * img.depth/8 * img.channels;

			// Check if shared memory for this camera is already attached
			if (shms[cam] == NULL || shm_sizes[cam] != size*2)
			{
				//attachSharedMem((int)img.key, size, &shm);
				shm_sizes[cam] = size*2;
			}

			shms[cam] = shm;
			char* ptr;
			key_t m_key = img.key;

			int shmid;
			if ((shmid = shmget(m_key, size*2, IPC_CREAT | 0666)) < 0)
			{
				perror("shmget");
				return false;
			}

			// Attach shared memory segment
			if((ptr = (char*)shmat(shmid, NULL, SHM_RDONLY)) == (char*)-1)
			{
				perror("shmat");
				return false;
			}

			// Copy message into IPLImage
			memcpy(ret1->imageData, ptr, size);
			memcpy(ret2->imageData, ptr+size, size);

			shmdt(ptr);
			return true;
		}
	}
/*
	char* sharedMemGetShmPtr(const mavlink_message_t* imgMsg)
											{
		// Decode message
		image_available_t img;
		//int size = img.width * img.height * img.depth * img.channels;

		// Pointer to shared memory
		char* shm = NULL;

		// Return shared mem pointer
		return shm;
											}
*/
private:
	char* shm;
	char** shms;
	int* shm_sizes;
};

#endif /* PXSHAREDMEMCLIENT_H_ */
