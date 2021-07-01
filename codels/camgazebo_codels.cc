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

#include <err.h>
#include <cstdio>

/* --- Attribute set_compression ---------------------------------------- */

/** Validation codel set_compression_rate of attribute set_compression.
 *
 * Returns genom_ok.
 * Throws camgazebo_e_io.
 */
genom_event
set_compression_rate(int16_t compression_rate,
                     const genom_context self)
{
    if (compression_rate >= -1 && compression_rate <= 100)
        return genom_ok;
    else
    {
        camgazebo_e_io_detail d;
        snprintf(d.what, sizeof(d.what), "%s", "unallowed compression rate");
        warnx("io error: %s", d.what);
        return camgazebo_e_io(&d,self);
    }
    return genom_ok;
}
