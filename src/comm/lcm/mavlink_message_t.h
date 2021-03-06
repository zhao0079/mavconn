/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by LCM
 **/

#include <stdint.h>
#include <stdlib.h>
#include <lcm/lcm_coretypes.h>
#include <lcm/lcm.h>

#ifndef _mavlink_message_t_h
#define _mavlink_message_t_h

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink.h"
/*
typedef struct _mavlink_message_t mavlink_message_t;
struct _mavlink_message_t
{
    int8_t     len;
    int8_t     seq;
    int8_t     sysid;
    int8_t     compid;
    int8_t     msgid;
    int8_t     payload[255];
    int8_t     ck_a;
    int8_t     ck_b;
};
*/
 
mavlink_message_t   *mavlink_message_t_copy(const mavlink_message_t *p);
void mavlink_message_t_destroy(mavlink_message_t *p);

typedef struct _mavlink_message_t_subscription_t mavlink_message_t_subscription_t;
typedef void (*mavlink_message_t_handler_t)(const lcm_recv_buf_t *rbuf, 
             const char *channel, const mavlink_message_t *msg, void *user);

int mavlink_message_t_publish(lcm_t *lcm, const char *channel, const mavlink_message_t *p);
mavlink_message_t_subscription_t* mavlink_message_t_subscribe (lcm_t *lcm, const char *channel, mavlink_message_t_handler_t f, void *userdata);
int mavlink_message_t_unsubscribe(lcm_t *lcm, mavlink_message_t_subscription_t* hid);

int  mavlink_message_t_encode(void *buf, int offset, int maxlen, const mavlink_message_t *p);
int  mavlink_message_t_decode(const void *buf, int offset, int maxlen, mavlink_message_t *p);
int  mavlink_message_t_decode_cleanup(mavlink_message_t *p);
int  mavlink_message_t_encoded_size(const mavlink_message_t *p);

// LCM support functions. Users should not call these
int64_t __mavlink_message_t_get_hash(void);
int64_t __mavlink_message_t_hash_recursive(const __lcm_hash_ptr *p);
int     __mavlink_message_t_encode_array(void *buf, int offset, int maxlen, const mavlink_message_t *p, int elements);
int     __mavlink_message_t_decode_array(const void *buf, int offset, int maxlen, mavlink_message_t *p, int elements);
int     __mavlink_message_t_decode_array_cleanup(mavlink_message_t *p, int elements);
int     __mavlink_message_t_encoded_array_size(const mavlink_message_t *p, int elements);
int     __mavlink_message_t_clone_array(const mavlink_message_t *p, mavlink_message_t *q, int elements);

#ifdef __cplusplus
}
#endif

#endif
