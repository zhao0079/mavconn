/*
 * PxSharedMemServer.h
 *
 *  Created on: 24.04.2010
 *      Author: user
 */

#ifndef PxSharedMemServer_H_
#define PxSharedMemServer_H_

#include <cvtypes.h>
#include <cv.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <inttypes.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>

#include <mavlink.h>
#include "mavconn.h"

int img_buf_size = 2;  ///< Number of images in buffer, each image stays img_buf_size/frame rate in the buffer, e.g. 5/30 = 167 ms

class PxSharedMemServer
{
public:
	PxSharedMemServer(int sysid, int compid, int width, int height, int depth, int channels, int key_ = 2011) :
	sysid(sysid),
	compid(compid)
	{
		// Shared memory key
		this->key = key_;
		this->img_seq = 0;
		this->img_buf_index = 0;
		this->shmid = -1;
		this->shm = 0;

		this->shm_size = (width * height * img_buf_size * depth * channels) / 8;//; ///< Monochrome 640x480 images
	}

	~PxSharedMemServer()
	{
		// Detach shared memory segment
		if (this->shm)
            shmdt(this->shm);
		// Remove shared memory segment
		if (this->shmid != -1)
		{
            if (shmctl(this->shmid, IPC_RMID, &this->shmid_ds) == -1)
            {
                perror("shmctl: IPC_RMID");
                exit(EXIT_FAILURE);
            }
		}
	}

	void sharedMemWriteImage(const IplImage* frame, uint64_t cam_id, uint32_t cam_no, uint64_t timestamp, float roll, float pitch, uint32_t exposure, lcm_t* lcm)
	{
		//printf("WRITING CAM %d\n", cam_no);
		// FIXME Calculate properly
		struct timeval tv;
		gettimeofday(&tv, NULL);
		uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
		uint64_t valid_until = now + (uint64_t)(100000);

		// Get image metadata
		char* data;
		int step;
		CvSize img_size;
		cvGetRawData(frame, (uchar**)(&data), &step, &img_size);

		// Create shared memory segment
		if (this->shmid == -1)
		{
			if ((this->shmid = shmget(this->key, this->shm_size, IPC_CREAT | 0666)) < 0)
			{
				perror("shmget");
				exit(EXIT_FAILURE);
			}
		}

		// Attach shared memory segment
		if((this->shm = (char*)shmat(this->shmid, NULL, 0)) == (char*)-1) {
			perror("shmat");
			exit(EXIT_FAILURE);
		}

		// Copy image raw data into shared memory
		uint32_t size = this->shm_size / img_buf_size;
		memcpy(this->shm+(cam_no*size), data, this->shm_size / img_buf_size);

		// Send out data at 1 Hz
		//sprintf(s, "Image #%d", img_seq);
		mavlink_image_available_t imginfo;
		imginfo.cam_id = cam_id;
		imginfo.cam_no = cam_no;
		imginfo.timestamp = timestamp;
		imginfo.valid_until = valid_until;
		imginfo.img_seq = this->img_seq;
		imginfo.img_buf_index = 1;	//FIXME
		imginfo.width = img_size.width;
		imginfo.height = img_size.height;
		imginfo.depth = frame->depth;
		imginfo.channels = frame->nChannels;
		imginfo.key = (int)this->key;
		imginfo.exposure = exposure;
		imginfo.gain = 1;//gain;
		imginfo.roll = roll;
		imginfo.pitch = pitch;

		//std::cout << "cam #" << (int)imginfo.cam_no << " key: " << imginfo.key << " size: " << shm_size << " width: " << (int)imginfo.width << " height: " << (int)imginfo.height << " depth: " << (int)imginfo.depth << " channels: " << (int)imginfo.channels << " " << std::endl;

		mavlink_message_t msg;
		mavlink_msg_image_available_encode(this->sysid, this->compid, &msg, &imginfo);
		mavlink_message_t_publish (lcm, "IMAGES", &msg);
		this->img_seq++;

		// Detach shared memory segment
		shmdt(this->shm);
	}

	void sharedMemWriteStereoImage(const IplImage* frame, uint64_t cam_id, uint32_t cam_no, const IplImage* frame_right, uint64_t cam_id_right, uint32_t cam_no_right, uint64_t timestamp, float roll, float pitch, uint32_t exposure, lcm_t* lcm)
	{
		// FIXME Calculate properly
		struct timeval tv;
		gettimeofday(&tv, NULL);
		uint64_t now = ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;
		uint64_t valid_until = now + (uint64_t)(100000);

		// Get image metadata
		char* data;
		int step;
		CvSize img_size;
		cvGetRawData(frame, (uchar**)(&data), &step, &img_size);
		char* data_right;
		int step_right;
		CvSize img_size_right;
		cvGetRawData(frame_right, (uchar**)(&data_right), &step_right, &img_size_right);

		// Create shared memory segment
		if (this->shmid == -1)
		{
			// Create shared memory segment
			if ((this->shmid = shmget(this->key, this->shm_size, IPC_CREAT | 0666)) < 0)
			{
				perror("shmget");
				exit(EXIT_FAILURE);
			}
		}

		// Attach shared memory segment
		if((this->shm = (char*)shmat(this->shmid, NULL, 0)) == (char*)-1) {
			perror("shmat");
			exit(EXIT_FAILURE);
		}

		// Copy image raw data into shared memory
		uint32_t size = this->shm_size / img_buf_size;
		memcpy(this->shm, 			 data, size);
		memcpy(this->shm + size, data_right, size);

		// Send out data at 1 Hz
		//sprintf(s, "Image #%d", img_seq);
		mavlink_image_available_t imginfo;
		imginfo.cam_id = cam_id;
		imginfo.cam_no = cam_no;
		imginfo.timestamp = timestamp;
		imginfo.valid_until = valid_until;
		imginfo.img_seq = this->img_seq;
		imginfo.img_buf_index = 2; //FIXME
		imginfo.width = img_size.width;
		imginfo.height = img_size.height;
		imginfo.depth = frame->depth;
		imginfo.channels = frame->nChannels;
		imginfo.key = (int)this->key;
		imginfo.exposure = exposure;
		imginfo.gain = 1;//gain;
		imginfo.roll = roll;
		imginfo.pitch = pitch;

		//std::cout << "cam #" << (int)imginfo.cam_no << " key: " << imginfo.key << " size: " << shm_size << " width: " << (int)imginfo.width << " height: " << (int)imginfo.height << " depth: " << (int)imginfo.depth << " channels: " << (int)imginfo.channels << " " << std::endl;

		mavlink_message_t msg;
		mavlink_msg_image_available_encode(this->sysid, this->compid, &msg, &imginfo);
		mavlink_message_t_publish (lcm, "IMAGES", &msg);

		this->img_seq++;

		// Detach shared memory segment
		shmdt(this->shm);
	}

protected:
	void attachSharedMem(int key, int shm_size, char** r_shm)
	{
	    this->key = key;
	    this->shm_size = shm_size;

		// Create shared memory segment
		if ((this->shmid = shmget(this->key, this->shm_size, IPC_CREAT | 0666)) < 0)
		{
			perror("shmget");
			exit(EXIT_FAILURE);
		}

		// Attach shared memory segment
		if((*r_shm = (char*)shmat(this->shmid, NULL, SHM_RDONLY)) == (char*)-1)
			//if((r_shm = (char*)shmat(shmid, NULL, 0)) == (char*)-1)
		{
			perror("shmat");
			exit(EXIT_FAILURE);
		}
		printf("\nRECEIVING IMAGES SUCCESSFULLY FROM NEW CAMERA, ATTACHED NEW BUFFER (key: %d, size: %d)\n", this->key, this->shm_size);
	}

private:
	int sysid;
	int compid;
	char* shm;
	key_t key;
	int shmid;
	int shm_size;
	unsigned int img_seq;
	int img_buf_index;
	struct shmid_ds shmid_ds;
};

#endif /* PxSharedMemServer_H_ */
