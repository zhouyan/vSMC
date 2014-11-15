# ============================================================================
#  vSMC/cmake/FindGCD.cmake
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013,2014, Yan Zhou
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

# Find Apple GCD support
#
# The following variable is set
#
# GCD_FOUND          - TRUE if Apple GCD is found and work correctly.
#                      But it is untested by real GCD programs
# GCD_INCLUDE_DIR    - The directory containing GCD headers
# GCD_LINK_LIBRARIES - GCD libraries that shall be linked to
#
# The following variables affect the behavior of this module
#
# GCD_INC_PATH - The path CMake shall try to find headers first
# GCD_LIB_PATH - The path CMake shall try to find libraries first

IF (DEFINED GCD_FOUND)
    RETURN ()
ENDIF (DEFINED GCD_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/FindGCD.cpp GCD_TEST_SOURCE)

IF (NOT DEFINED GCD_LINK_LIBRARIES)
    IF (APPLE)
        MESSAGE (STATUS "GCD link libraries not required (Mac OS X)")
        SET (GCD_LINK_LIBRARIES "" CACHE STRING "GCD link libraries")
    ELSE (APPLE)
        FIND_LIBRARY (GCD_LINK_LIBRARIES dispatch
            PATHS ${GCD_LIB_PATH} ENV LIBRARY_PATH ENV LIB NO_DEFAULT_PATH)
        FIND_LIBRARY (GCD_LINK_LIBRARIES dispatch)
        IF (GCD_LINK_LIBRARIES)
            MESSAGE (STATUS "Found GCD libraries: ${GCD_LINK_LIBRARIES}")
        ELSE (GCD_LINK_LIBRARIES)
            MESSAGE (STATUS "NOT Found GCD libraries")
        ENDIF (GCD_LINK_LIBRARIES)
    ENDIF (APPLE)
ENDIF (NOT DEFINED GCD_LINK_LIBRARIES)

IF (NOT DEFINED GCD_INCLUDE_DIR)
    FIND_PATH (GCD_INCLUDE_DIR dispatch/dispatch.h
        PATHS ${GCD_INC_PATH} ENV CPATH NO_DEFAULT_PATH)
    FIND_PATH (GCD_INCLUDE_DIR dispatch/dispatch.h)
    IF (GCD_INCLUDE_DIR)
        MESSAGE (STATUS "Found GCD headers: ${GCD_INCLUDE_DIR}")
    ELSE (GCD_INCLUDE_DIR)
        MESSAGE (STATUS "NOT Found GCD headers")
        SET (GCD_BASIC_FOUND FALSE)
    ENDIF (GCD_INCLUDE_DIR)
ENDIF (NOT DEFINED GCD_INCLUDE_DIR)

IF (APPLE)
    IF (GCD_INCLUDE_DIR)
        SET (GCD_BASIC_FOUND TRUE)
    ELSE (GCD_INCLUDE_DIR)
        SET (GCD_BASIC_FOUND FALSE)
    ENDIF (GCD_INCLUDE_DIR)
ELSE (APPLE)
    IF (GCD_LINK_LIBRARIES AND GCD_INCLUDE_DIR)
        SET (GCD_BASIC_FOUND TRUE)
    ELSE (GCD_LINK_LIBRARIES AND GCD_INCLUDE_DIR)
        SET (GCD_BASIC_FOUND FALSE)
    ENDIF (GCD_LINK_LIBRARIES AND GCD_INCLUDE_DIR)
ENDIF (APPLE)

IF (GCD_BASIC_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    SET (SAFE_CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES}
        ${GCD_INCLUDE_DIR})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
        ${GCD_LINK_LIBRARIES})
    CHECK_CXX_SOURCE_RUNS ("${GCD_TEST_SOURCE}" GCD_FOUND)
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
    IF (GCD_FOUND)
        MESSAGE (STATUS "Found GCD")
    ELSE (GCD_FOUND)
        MESSAGE (STATUS "NOT Found GCD")
    ENDIF (GCD_FOUND)
ENDIF (GCD_BASIC_FOUND)
