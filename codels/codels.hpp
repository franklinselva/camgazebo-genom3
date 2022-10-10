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
#include <condition_variable>
#include <mutex>
#include <sys/time.h>

struct or_camera_pipe {
    gazebo::transport::NodePtr node;
    gazebo::transport::SubscriberPtr sub;
};

struct or_camera_data {
    uint64_t l;
    uint8_t* data;
    bool prompt_size_error;
    bool new_frame = false;
    std::mutex m;
    std::condition_variable cv;

    timeval tv;

    or_camera_data(uint16_t w, uint16_t h, uint16_t c) { set_size(w, h, c); }
    ~or_camera_data() { delete data; }

    void set_size(uint16_t w, uint16_t h, uint16_t c)
    {
        l = h * w * c;
        data = new uint8_t[l];
        prompt_size_error = false;
    }

    void cb(ConstImageStampedPtr &_msg)
    {
        if (_msg->image().data().length() == l)
        {
            std::unique_lock<std::mutex> lock(this->m);
            // the main thread releases the lock between codels wait and pub,
            // therefore the callback might retrieve it before the previous frame is published
            // (although its very unlikely)
            // if new_frame predicate is still true, wait for main thread to ping
            // wait for 16ms before dropping current frame (frame interval at 60Hz)
            if (new_frame)
                cv.wait_for(lock, std::chrono::duration<float>(16e-3));

            if (!new_frame)
            {
                gettimeofday(&(tv), NULL);
                memcpy(data, _msg->image().data().c_str(), l); // sizeof *this->data == 1
                new_frame = true;
                lock.unlock();
                cv.notify_all();
            }
            else
                warnx("frame dropped, is some processing too long?");   // should never prompt
        }
        else if (!prompt_size_error)
        {
            warnx("incorrect frame size; call set_format with values from gazebo model");
            prompt_size_error = true;
        }
    }
};

#endif /* H_CAMGAZEBO_CODELS */
