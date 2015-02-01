# ============================================================================
#  vSMC/cmake/FindJEMALLOC.cmake
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013-2015, Yan Zhou
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#    Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
# ============================================================================

# Find jemalloc
#
# This module can be used to find jemalloc headers and libraries
#
# The following variables are set
#
# JEMALLOC_FOUND          - TRUE if jemalloc headers and libraries are found
# JEMALLOC_INCLUDE_DIR    - The directory containing jemalloc headers
# JEMALLOC_LINK_LIBRARIES - jemalloc libraries that shall be linked to
#
# The following variables affect the behavior of this module
#
# JEMALLOC_INC_PATH - The path CMake shall try to find headers first
# JEMALLOC_LIB_PATH - The path CMake shall try to find libraries first

IF (DEFINED JEMALLOC_FOUND)
    RETURN ()
ENDIF (DEFINED JEMALLOC_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindJEMALLOC.cpp JEMALLOC_TEST_SOURCE)

IF (NOT DEFINED JEMALLOC_LINK_LIBRARIES)
    FIND_LIBRARY (JEMALLOC_LINK_LIBRARIES jemalloc
        PATHS ${JEMALLOC_LIB_PATH} ENV LIBRARY_PATH ENV LIB NO_DEFAULT_PATH)
    FIND_LIBRARY (JEMALLOC_LINK_LIBRARIES jemalloc)
    IF (JEMALLOC_LINK_LIBRARIES)
        MESSAGE (STATUS "Found jemalloc libraries: ${JEMALLOC_LINK_LIBRARIES}")
    ELSE (JEMALLOC_LINK_LIBRARIES)
        MESSAGE (STATUS "NOT Found jemalloc libraries")
    ENDIF (JEMALLOC_LINK_LIBRARIES)
ENDIF (NOT DEFINED JEMALLOC_LINK_LIBRARIES)

IF (NOT DEFINED JEMALLOC_INCLUDE_DIR)
    FIND_PATH (JEMALLOC_INCLUDE_DIR jemalloc/jemalloc.h
        PATHS ${JEMALLOC_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
    FIND_PATH (JEMALLOC_INCLUDE_DIR jemalloc/jemalloc.h)
    IF (JEMALLOC_INCLUDE_DIR)
        MESSAGE (STATUS "Found jemalloc headers: ${JEMALLOC_INCLUDE_DIR}")
    ELSE (JEMALLOC_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found jemalloc headers")
    ENDIF (JEMALLOC_INCLUDE_DIR)
ENDIF (NOT DEFINED JEMALLOC_INCLUDE_DIR)

IF (JEMALLOC_LINK_LIBRARIES AND JEMALLOC_INCLUDE_DIR)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_INCLUDES  ${CMAKE_REQUIRED_INCLUDES})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES}
        ${JEMALLOC_INCLUDE_DIR})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
        ${JEMALLOC_LINK_LIBRARIES})
    CHECK_CXX_SOURCE_RUNS ("${JEMALLOC_TEST_SOURCE}" JEMALLOC_TEST_SOURCE_RUNS)
    IF (JEMALLOC_TEST_SOURCE_RUNS)
        MESSAGE (STATUS "Found jemalloc")
        SET (JEMALLOC_FOUND TRUE CACHE BOOL "Found jemalloc")
    ELSE (JEMALLOC_TEST_SOURCE_RUNS)
        MESSAGE (STATUS "NOT Found jemalloc")
        SET (JEMALLOC_FOUND FALSE CACHE BOOL "NOT Found jemalloc")
    ENDIF (JEMALLOC_TEST_SOURCE_RUNS)
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
ENDIF (JEMALLOC_LINK_LIBRARIES AND JEMALLOC_INCLUDE_DIR)
