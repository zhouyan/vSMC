# ============================================================================
#  vSMC/cmake/FindRandom123.cmake
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

# Find Random123
#
# This module can be used to find Random123 headers
#
# The following variables are set
#
# RANDOM123_FOUND       - TRUE if Random123 headers are found
# Random123_INCLUDE_DIR - The directory containing OpenCL headers
#
# The following variables affect the behavior of this module
#
# Random123_INC_PATH - The path CMake shall try to find headers first

IF (DEFINED RANDOM123_FOUND)
    RETURN ()
ENDIF (DEFINED RANDOM123_FOUND)

IF (NOT DEFINED Random123_INCLUDE_DIR)
    FIND_PATH (Random123_INCLUDE_DIR Random123/threefry.h
        PATHS ${Random123_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
    FIND_PATH (Random123_INCLUDE_DIR Random123/threefry.h)
    IF (Random123_INCLUDE_DIR)
        MESSAGE (STATUS "Found Random123 headers: ${Random123_INCLUDE_DIR}")
        SET (RANDOM123_FOUND TRUE CACHE BOOL "Found Random123")
    ELSE (Random123_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found Random123 headers")
        SET (RANDOM123_FOUND FALSE CACHE BOOL "Not Found Random123")
    ENDIF (Random123_INCLUDE_DIR)
ENDIF (NOT DEFINED Random123_INCLUDE_DIR)
