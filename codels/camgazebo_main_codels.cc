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
#include "accamgazebo.h"

#include "camgazebo_c_types.h"

#include "codels.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>

/* --- Global variables ------------------------------------------------- */
or_camera_data* cgz_data;

/* --- Callback --------------------------------------------------------- */
void camgz_cb(ConstImageStampedPtr& _msg) {
std::cout << "enter cb" << std::endl;
    if (_msg->image().width() != cgz_data->w ||
        _msg->image().height() != cgz_data->h) {
        cgz_data->w = _msg->image().width();
        cgz_data->h = _msg->image().height();
        // +1 for null terminate
        cgz_data->data = new uint8_t[_msg->image().data().length()];
    }
    memcpy(cgz_data->data, _msg->image().data().c_str(), _msg->image().data().length());
std::cout << "exit cb" << std::endl;
}

/* --- Task main -------------------------------------------------------- */


/** Codel camgz_start of task main.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_wait.
 */
genom_event
camgz_start(camgazebo_ids *ids, const camgazebo_frame *frame,
            const camgazebo_extrinsics *extrinsics,
            const genom_context self)
{
    // Init values for extrinsics
    *extrinsics->data(self) = {0,0,0, 0,0,0};
    extrinsics->write(self);

    // if (genom_sequence_reserve(&(frame->data(self)->pixels), 0) == -1) {
    //     camgazebo_e_mem_detail d;
    //     snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
    //     printf("camgazebo: %s\n", d.what);
    //     return camgazebo_e_mem(&d,self);
    // }

    cgz_data = new or_camera_data();

    ids->info.started = false;
    ids->pipe = new or_camera_pipe();
    return camgazebo_wait;
}


/** Codel camgz_wait of task main.
 *
 * Triggered by camgazebo_wait.
 * Yields to camgazebo_pause_wait, camgazebo_pub.
 */
genom_event
camgz_wait(bool started, const genom_context self)
{
    if (started)
        return camgazebo_pub;
    else
        return camgazebo_pause_wait;
}


/** Codel camgz_pub of task main.
 *
 * Triggered by camgazebo_pub.
 * Yields to camgazebo_pause_wait.
 */
genom_event
camgz_pub(const camgazebo_frame *frame, const genom_context self)
{
std::cout << "enter pub" << std::endl;
    or_sensor_frame* fdata = frame->data(self);
std::cout << "fdata ok" << std::endl;
    uint32_t l = cgz_data->h * cgz_data->w * cgz_data->bpp;
std::cout << "l ok" << std::endl;
    if (l > fdata->pixels._maximum)
        if (genom_sequence_reserve(&(fdata->pixels), l) == -1) {
            camgazebo_e_mem_detail d;
            snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
            printf("camgazebo: %s\n", d.what);
            return camgazebo_e_mem(&d,self);
        }
std::cout << "pub2" << std::endl;
    fdata->height = cgz_data->h;
    fdata->width = cgz_data->w;
    fdata->bpp = cgz_data->bpp;
    fdata->pixels._length = l;
    memcpy(fdata->pixels._buffer, cgz_data->data, l);
std::cout << "pub3" << std::endl;
    *(frame->data(self)) = *fdata;

    if (l>0) {
        cv::Mat cvRGB(cv::Size(fdata->width, fdata->height), CV_8UC3, (void*)cgz_data->data, cv::Mat::AUTO_STEP);
        cv::Mat cvBGR;
        cv::cvtColor(cvRGB, cvBGR, cv::COLOR_RGB2BGR);  // realsense data is RGB
        cv::imshow("frame", cvBGR);
        cv::waitKey(1);
    }
std::cout << "exit pub" << std::endl;
    return camgazebo_pause_wait;
}


/* --- Activity connect ------------------------------------------------- */

/** Codel camgz_connect of activity connect.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_connect(const char name[64], or_camera_pipe **pipe,
              const camgazebo_intrinsics *intrinsics, bool *started,
              const genom_context self)
{
    gazebo::client::setup();
    (*pipe)->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    (*pipe)->node->Init();

    char* topic = new char[64];
    snprintf(topic, 64, "/gazebo/mrsim/camera/link/%s/image", name);
    std::cout << topic << std::endl;
    (*pipe)->sub = (*pipe)->node->Subscribe(topic, camgz_cb);

    *started = true;

    return camgazebo_ether;
}


/* --- Activity disconnect ---------------------------------------------- */

/** Codel camgz_disconnect of activity disconnect.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_disconnect(bool *started, const genom_context self)
{
    gazebo::client::shutdown();
    *started = false;
    return camgazebo_ether;
}
