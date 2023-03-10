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

#include <condition_variable>
#include <mutex>
#include <chrono>
#include <opencv2/opencv.hpp>
using namespace cv;


/* --- Calib helper  ------------------------------------------------------ */
void compute_calib(or_sensor_intrinsics* intr, float hfov, or_camera_info_size_s size)
{
    float f = size.w/2/tan(hfov/2);
    intr->calib = {
        f, f, (float)size.w/2, (float)size.h/2, 0
    };
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
            const camgazebo_intrinsics *intrinsics,
            const genom_context self)
{
    ids->info.started = false;

    // These are the defaults values for the gazebo camera
    ids->hfov = 1.047;
    ids->info.size = {320, 240};
    snprintf(ids->info.format, sizeof(char)*8, "Y8");
    ids->info.compression_rate = -1;

    ids->data = new or_camera_data(ids->info.size.w, ids->info.size.h, 3);
    ids->pipe = new or_camera_pipe();

    // Publish initial calibration
    compute_calib(intrinsics->data(self), ids->hfov, ids->info.size);
    intrinsics->data(self)->disto = {0,0,0,0,0};
    *extrinsics->data(self) = {0,0,0,0,0,0};

    intrinsics->write(self);
    extrinsics->write(self);

    // Init frame ports
    frame->open("raw", self);
    frame->open("compressed", self);

    if (genom_sequence_reserve(&(frame->data("raw", self)->pixels), ids->data->l) == -1) {
        camgazebo_e_mem_detail d;
        snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
        warnx("%s", d.what);
        return camgazebo_e_mem(&d,self);
    }
    frame->data("raw", self)->pixels._length = ids->data->l;
    frame->data("raw", self)->height = ids->info.size.h;
    frame->data("raw", self)->width = ids->info.size.w;
    frame->data("raw", self)->bpp = 1;
    frame->data("raw", self)->compressed = false;

    (void)genom_sequence_reserve(&(frame->data("compressed", self)->pixels), 0);
    frame->data("compressed", self)->pixels._length = 0;
    frame->data("compressed", self)->height = ids->info.size.h;
    frame->data("compressed", self)->width = ids->info.size.w;
    frame->data("compressed", self)->bpp = 1;
    frame->data("compressed", self)->compressed = true;

    return camgazebo_wait;
}


/** Codel camgz_wait of task main.
 *
 * Triggered by camgazebo_wait.
 * Yields to camgazebo_pause_wait, camgazebo_wait, camgazebo_pub.
 */
genom_event
camgz_wait(bool started, or_camera_data **data,
           const genom_context self)
{
    if (!started)
        return camgazebo_pause_wait;

    std::unique_lock<std::mutex> lock((*data)->m);

    if (!(*data)->new_frame)
    {
        (*data)->cv.wait_for(lock, std::chrono::duration<int16_t>(camgazebo_poll_duration_sec));

        if (!(*data)->new_frame)
            return camgazebo_wait;
    }
    return camgazebo_pub;
}


/** Codel camgz_pub of task main.
 *
 * Triggered by camgazebo_pub.
 * Yields to camgazebo_wait.
 */
genom_event
camgz_pub(int16_t compression_rate, or_camera_data **data,
          const camgazebo_frame *frame, const genom_context self)
{
    or_sensor_frame* rfdata = frame->data("raw", self);

    std::unique_lock<std::mutex> lock((*data)->m);

    memcpy(rfdata->pixels._buffer, (*data)->data, rfdata->pixels._length); // sizeof *rfdata->pixels._buffer == 1

    rfdata->ts.sec = (*data)->tv.tv_sec;
    rfdata->ts.nsec = (*data)->tv.tv_usec * 1000;

    (*data)->new_frame = false;
    lock.unlock();
    (*data)->cv.notify_all();

    frame->write("raw", self);

    if (compression_rate != -1)
    {
        or_sensor_frame* cfdata = frame->data("compressed", self);

        Mat cvframe = Mat(
            Size(rfdata->width, rfdata->height),
            rfdata->bpp == 1 ? CV_8UC1 : CV_8UC3,
            rfdata->pixels._buffer,
            Mat::AUTO_STEP
        );

        std::vector<int32_t> compression_params;
        compression_params.push_back(IMWRITE_JPEG_QUALITY);
        compression_params.push_back(compression_rate);

        std::vector<uint8_t> buf;
        imencode(".jpg", cvframe, buf, compression_params);

        if (buf.size() > cfdata->pixels._maximum)
            if (genom_sequence_reserve(&(cfdata->pixels), buf.size())  == -1) {
                camgazebo_e_mem_detail d;
                snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
                warnx("%s", d.what);
                return camgazebo_e_mem(&d,self);
            }
        cfdata->pixels._length = buf.size();

        memcpy(cfdata->pixels._buffer, buf.data(), buf.size()); // sizeof *buf.data() == 1
        cfdata->ts = rfdata->ts;

        frame->write("compressed", self);
    }

    return camgazebo_wait;
}


/* --- Activity connect ------------------------------------------------- */

/** Codel camgz_connect of activity connect.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_connect(const char topic[256], or_camera_data **data,
              or_camera_pipe **pipe,
              const camgazebo_intrinsics *intrinsics, bool *started,
              const genom_context self)
{
    if (*started)
        warnx("already connected to gazebo, disconnect() first");
    else
    {
        gazebo::client::setup();
        (*pipe)->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
        (*pipe)->node->Init();

        std::lock_guard<std::mutex> guard((*data)->m);

        (*pipe)->sub = (*pipe)->node->Subscribe(topic, &or_camera_data::cb, *data);

        warnx("connected to %s", topic);
        *started = true;
    }

    return camgazebo_ether;
}


/* --- Activity disconnect ---------------------------------------------- */

/** Codel camgz_disconnect of activity disconnect.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_disconnect(or_camera_data **data, bool *started,
                 const genom_context self)
{
    std::lock_guard<std::mutex> guard((*data)->m);

    gazebo::client::shutdown();
    *started = false;

    warnx("disconnected from gazebo");

    return camgazebo_ether;
}


/* --- Activity get_K --------------------------------------------------- */

/** Codel camgz_get_K of activity get_K.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_get_K(const camgazebo_intrinsics *intrinsics, sequence5_float *K,
            const genom_context self)
{
    K->_length = 5;
    K->_buffer[0] = intrinsics->data(self)->calib.fx;
    K->_buffer[1] = intrinsics->data(self)->calib.fy;
    K->_buffer[2] = intrinsics->data(self)->calib.cx;
    K->_buffer[3] = intrinsics->data(self)->calib.cy;
    K->_buffer[4] = intrinsics->data(self)->calib.gamma;
    return camgazebo_ether;
}


/* --- Activity get_D --------------------------------------------------- */

/** Codel camgz_get_D of activity get_D.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_get_D(const camgazebo_intrinsics *intrinsics, sequence5_float *D,
            const genom_context self)
{
    D->_length = 5;
    D->_buffer[0] = intrinsics->data(self)->disto.k1;
    D->_buffer[1] = intrinsics->data(self)->disto.k2;
    D->_buffer[2] = intrinsics->data(self)->disto.k2;
    D->_buffer[3] = intrinsics->data(self)->disto.p1;
    D->_buffer[4] = intrinsics->data(self)->disto.p2;
    return camgazebo_ether;
}


/* --- Activity get_extrinsics ------------------------------------------ */

/** Codel camgz_get_extrinsics of activity get_extrinsics.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_get_extrinsics(const camgazebo_extrinsics *extrinsics,
                     sequence6_float *ext, const genom_context self)
{
    ext->_length = 5;
    ext->_buffer[0] = extrinsics->data(self)->trans.tx;
    ext->_buffer[1] = extrinsics->data(self)->trans.ty;
    ext->_buffer[2] = extrinsics->data(self)->trans.tz;
    ext->_buffer[3] = extrinsics->data(self)->rot.roll;
    ext->_buffer[4] = extrinsics->data(self)->rot.pitch;
    ext->_buffer[5] = extrinsics->data(self)->rot.yaw;
    return camgazebo_ether;
}


/* --- Activity set_extrinsics ------------------------------------------ */

/** Codel camgz_set_extrinsics of activity set_extrinsics.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_set_extrinsics(const sequence6_float *ext_values,
                     const camgazebo_extrinsics *extrinsics,
                     const genom_context self)
{
    *extrinsics->data(self) = {
        ext_values->_buffer[0],
        ext_values->_buffer[1],
        ext_values->_buffer[2],
        ext_values->_buffer[3],
        ext_values->_buffer[4],
        ext_values->_buffer[5]
    };

    extrinsics->write(self);

    warnx("new extrinsic calibration");
    return camgazebo_ether;
}


/* --- Activity set_hfov ------------------------------------------------ */

/** Codel camgz_set_hfov of activity set_hfov.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_set_hfov(float hfov_val, float *hfov,
               const or_camera_info_size_s *size,
               const camgazebo_intrinsics *intrinsics,
               const genom_context self)
{
    *hfov = hfov_val;

    compute_calib(intrinsics->data(self), *hfov, *size);
    intrinsics->write(self);

    warnx("set horizontal fov");
    return camgazebo_ether;
}


/* --- Activity set_format ---------------------------------------------- */

/** Codel camgz_set_fmt of activity set_format.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 * Throws camgazebo_e_io.
 */
genom_event
camgz_set_fmt(uint16_t w_val, uint16_t h_val, uint16_t c_val,
              or_camera_data **data, float hfov,
              or_camera_info_size_s *size, char format[8],
              const camgazebo_frame *frame,
              const camgazebo_intrinsics *intrinsics,
              const genom_context self)
{
    *size = {w_val, h_val};
    if (c_val == 1)
        snprintf(format, sizeof(char)*8, "Y8");
    if (c_val == 3)
        snprintf(format, sizeof(char)*8, "RBG8");

    (*data)->set_size(w_val, h_val, c_val);

    if (genom_sequence_reserve(&(frame->data("raw", self)->pixels), (*data)->l) == -1) {
        camgazebo_e_mem_detail d;
        snprintf(d.what, sizeof(d.what), "unable to allocate frame memory");
        warnx("%s", d.what);
        return camgazebo_e_mem(&d,self);
    }
    frame->data("raw", self)->pixels._length = (*data)->l;
    frame->data("raw", self)->height = h_val;
    frame->data("raw", self)->width = w_val;
    frame->data("raw", self)->bpp = c_val;

    (void)genom_sequence_reserve(&(frame->data("compressed", self)->pixels), 0);
    frame->data("compressed", self)->pixels._length = 0;
    frame->data("compressed", self)->height = h_val;
    frame->data("compressed", self)->width = w_val;
    frame->data("compressed", self)->bpp = c_val;

    compute_calib(intrinsics->data(self), hfov, *size);
    intrinsics->write(self);

    warnx("set image format");
    return camgazebo_ether;
}


/* --- Activity set_disto ----------------------------------------------- */

/** Codel camgz_set_disto of activity set_disto.
 *
 * Triggered by camgazebo_start.
 * Yields to camgazebo_ether.
 */
genom_event
camgz_set_disto(const sequence5_float *dist_values,
                const camgazebo_intrinsics *intrinsics,
                const genom_context self)
{
    intrinsics->data(self)->disto = {
        dist_values->_buffer[0],
        dist_values->_buffer[1],
        dist_values->_buffer[2],
        dist_values->_buffer[3],
        dist_values->_buffer[4]
    };

    intrinsics->write(self);

    warnx("set distortion coefficients");
    return camgazebo_ether;
}
