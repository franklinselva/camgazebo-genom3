#
# Copyright (c) 2020 LAAS/CNRS
# All rights reserved.
#
# Redistribution  and  use  in  source  and binary  forms,  with  or  without
# modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
# THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
# WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
# MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
# ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
# WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
# IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#                                                  Martin Jacquet - June 2020
#
lib_LTLIBRARIES = libcamgazebo_codels.la

libcamgazebo_codels_la_SOURCES  =	camgazebo_c_types.h
libcamgazebo_codels_la_SOURCES +=	camgazebo_codels.cc
libcamgazebo_codels_la_SOURCES +=	camgazebo_main_codels.cc

libcamgazebo_codels_la_CPPFLAGS =	$(requires_CFLAGS)
libcamgazebo_codels_la_LIBADD   =	$(requires_LIBS)
libcamgazebo_codels_la_CPPFLAGS+=	$(codels_requires_CFLAGS)
libcamgazebo_codels_la_LIBADD  +=	$(codels_requires_LIBS)
libcamgazebo_codels_la_LDFLAGS  =	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	camgazebo_c_types.h
CLEANFILES=	${BUILT_SOURCES}

camgazebo_c_types.h: ${top_srcdir}/camgazebo.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/camgazebo_c_types.h.d
