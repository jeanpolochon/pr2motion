#
# Copyright (c) 2015 CNRS/LAAS
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
lib_LTLIBRARIES = libpr2motion_codels.la

libpr2motion_codels_la_SOURCES=		pr2motion_c_types.h
libpr2motion_codels_la_SOURCES+=	pr2_state_client.hh pr2_state_client.cc
libpr2motion_codels_la_SOURCES+=	pr2motion_codels.cc
libpr2motion_codels_la_SOURCES+=	pr2motion_main_codels.cc
libpr2motion_codels_la_SOURCES+= 	pr2_gripper_sensor_client.hh pr2_gripper_sensor_client.cc
libpr2motion_codels_la_SOURCES+= 	pr2_gripper_client.hh pr2_gripper_client.cc
libpr2motion_codels_la_SOURCES+= 	pr2_torso_client.hh pr2_torso_client.cc
libpr2motion_codels_la_SOURCES+= 	pr2_head_client.hh pr2_head_client.cc
libpr2motion_codels_la_SOURCES+=        trajectories/PathG.h trajectories/TrajectoryG.h trajectories/PathG.cpp trajectories/TrajectoryG.cpp
libpr2motion_codels_la_SOURCES+= 	pr2_arm_client.hh pr2_arm_client.cc


libpr2motion_codels_la_CPPFLAGS=	$(requires_CFLAGS) $(sensor_msgs_CFLAGS) $(pr2_gripper_sensor_msgs_CFLAGS) $(actionlib_CFLAGS)  $(actionlib_msgs_CFLAGS) $(eigen3_CFLAGS) $(softmotion_CFLAGS)
libpr2motion_codels_la_LDFLAGS=		$(requires_LIBS) $(sensor_msgs_LIBS) $(pr2_gripper_sensor_msgs_LIBS) $(actionlib_LIBS) $(actionlib_msgs_LIBS) $(eigen3_LIBS) $(softmotion_LDFLAGS)
libpr2motion_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	pr2motion_c_types.h
CLEANFILES=	${BUILT_SOURCES}
pr2motion_c_types.h: ${top_srcdir}/pr2motion.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/pr2motion_c_types.h.d
