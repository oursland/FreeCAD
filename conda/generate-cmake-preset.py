#!/bin/env python3

import os

SET(ENV{CC} "${CMAKE_SOURCE_DIR}/.conda/freecad/bin/clang")
SET(ENV{CFLAGS} "-ftree-vectorize -fPIC -fPIE -fstack-protector-strong -O2 -pipe -isystem ${CMAKE_SOURCE_DIR}/.conda/freecad/include")
SET(ENV{CXX} "${CMAKE_SOURCE_DIR}/.conda/freecad/bin/clang++")
SET(ENV{CXXFLAGS} "-ftree-vectorize -fPIC -fPIE -fstack-protector-strong -O2 -pipe -stdlib=libc++ -fvisibility-inlines-hidden -fmessage-length=0 -isystem ${CMAKE_SOURCE_DIR}/.conda/freecad/include")
SET(ENV{LDFLAGS} "-Wl,-pie -Wl,-headerpad_max_install_names -Wl,-dead_strip_dylibs -Wl,-rpath,${CMAKE_SOURCE_DIR}/.conda/freecad/lib -L${CMAKE_SOURCE_DIR}/.conda/freecad/lib")
SET(ENV{PATH} "${CMAKE_SOURCE_DIR}/.conda/freecad/bin:$ENV{PATH}")


print(os.environ['HOME'])
