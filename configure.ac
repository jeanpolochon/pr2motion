dnl Autoconf file for building pr2motion codels library.
dnl
dnl Copyright (c) 2015 CNRS/LAAS
dnl
dnl Permission to use, copy, modify, and distribute this software for any
dnl purpose with or without fee is hereby granted, provided that the above
dnl copyright notice and this permission notice appear in all copies.
dnl
dnl THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
dnl WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
dnl MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
dnl ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
dnl WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
dnl ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
dnl OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
dnl

AC_PREREQ(2.59)

AC_INIT([pr2motion-genom3],[0.1],[openrobots@laas.fr])
AC_CONFIG_MACRO_DIR([autoconf])
AC_CONFIG_AUX_DIR([autoconf])
AC_CONFIG_HEADERS([autoconf/acpr2motion.h])
AM_INIT_AUTOMAKE([foreign no-define])

dnl Compilers
dnl
LT_INIT
AC_PROG_CC
AC_PROG_CXX

dnl External packages
PKG_CHECK_MODULES(requires, [
  genom3 >= 2.99.4
  sensor_msgs, pr2_controllers_msgs, pr2_gripper_sensor_msgs, actionlib, actionlib_msgs, eigen3, softMotion-libs, urdf
])

AC_PATH_PROG(GENOM3, [genom3], AC_MSG_ERROR([genom3 tool not found], 2))

dnl --enable-simu option
AC_ARG_ENABLE([simu], [AC_HELP_STRING( [--enable-simu], [used to turn on the SIMU flag])], [ AC_DEFINE([PR2_SIMU],[1],["Defined if we are in simu"]) ], [])

dnl --with-templates option
AG_OPT_TEMPLATES([$GENOM3 ],
    [$srcdir/pr2motion.gen])

dnl Output
AC_CONFIG_FILES([
	pr2motion-genom3.pc
	pr2motion-genom3-uninstalled.pc
	Makefile
	codels/Makefile
])
AC_OUTPUT
AG_OUTPUT_TEMPLATES

