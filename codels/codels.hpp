/*
 * Copyright (c) 2020 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *                                                  Martin Jacquet - June 2020
 */
#ifndef H_CAMGAZEBO_CODELS
#define H_CAMGAZEBO_CODELS

#include "accamgazebo.h"

#include "camgazebo_c_types.h"

#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

#include <err.h>
#include <mutex>
#include <sys/time.h>

struct or_camera_pipe {
    gazebo::transport::NodePtr node;
    gazebo::transport::SubscriberPtr sub;
};

struct or_camera_data {
    uint64_t l;
    uint8_t* data;
    bool new_frame;
    std::mutex lock;
    timeval tv;

    or_camera_data(uint16_t w, uint16_t h)
    {
        this->set_size(w, h);
    }

    void set_size(uint16_t w, uint16_t h)
    {
        this->l = h * w * 3;
        this->data = new uint8_t[l];
        this->new_frame = false;
    }

    void cb(ConstImageStampedPtr &_msg)
    {
        std::lock_guard<std::mutex> guard(this->lock);

        if (_msg->image().data().length() != this->l)
            warnx("Skipping frame, incorrect size");
        else
        {
            gettimeofday(&(this->tv), NULL);
            memcpy(this->data, _msg->image().data().c_str(), _msg->image().data().length());
            this->new_frame = true;
        }
    }
};

#endif /* H_CAMGAZEBO_CODELS */
