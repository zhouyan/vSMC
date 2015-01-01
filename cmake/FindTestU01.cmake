# ============================================================================
#  vSMC/cmake/FindTestU01.cmake
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

# Find TestU01
#
# This module can be used to find TestU01 headers and libraries
#
# The following variables are set
#
# TESTU01_FOUND          - TRUE if TestU01 headers and libraries are found
#                          But it is untested by real OpenCL programs
# TestU01_INCLUDE_DIR    - The directory containing OpenCL headers
# TestU01_LINK_LIBRARIES - TBB libraries that shall be linked to
#
# The following variables affect the behavior of this module
#
# TestU01_INC_PATH - The path CMake shall try to find headers first
# TestU01_LIB_PATH - The path CMake shall try to find libraries first

IF (DEFINED TESTU01_FOUND)
    RETURN ()
ENDIF (DEFINED TESTU01_FOUND)

IF (NOT DEFINED TestU01_LINK_LIBRARIES)
    FIND_LIBRARY (TestU01_LINK_LIBRARIES testu01
        PATHS ${TestU01_LIB_PATH} ENV LIBRARY_PATH ENV LIB NO_DEFAULT_PATH)
    FIND_LIBRARY (TestU01_LINK_LIBRARIES testu01)
    IF (TestU01_LINK_LIBRARIES)
        MESSAGE (STATUS "Found TestU01 libraries: ${TestU01_LINK_LIBRARIES}")
    ELSE (TestU01_LINK_LIBRARIES)
        MESSAGE (STATUS "NOT Found TestU01 libraries")
    ENDIF (TestU01_LINK_LIBRARIES)
ENDIF (NOT DEFINED TestU01_LINK_LIBRARIES)

IF (NOT DEFINED TestU01_INCLUDE_DIR)
    FIND_PATH (TestU01_INCLUDE_DIR bbattery.h
        PATHS ${TestU01_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
    FIND_PATH (TestU01_INCLUDE_DIR bbattery.h)
    IF (TestU01_INCLUDE_DIR)
        MESSAGE (STATUS "Found TestU01 headers: ${TestU01_INCLUDE_DIR}")
    ELSE (TestU01_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found TestU01 headers")
        SET (TESTU01_FOUND FALSE)
    ENDIF (TestU01_INCLUDE_DIR)
ENDIF (NOT DEFINED TestU01_INCLUDE_DIR)

IF (TestU01_LINK_LIBRARIES AND TestU01_INCLUDE_DIR)
    MESSAGE (STATUS "Found TestU01")
    SET (TESTU01_FOUND TRUE CACHE BOOL "Found TestU01")
ELSE (TestU01_LINK_LIBRARIES AND TestU01_INCLUDE_DIR)
    MESSAGE (STATUS "NOT Found TestU01")
    SET (TESTU01_FOUND FALSE CACHE BOOL "Not Found TestU01")
ENDIF (TestU01_LINK_LIBRARIES AND TestU01_INCLUDE_DIR)
