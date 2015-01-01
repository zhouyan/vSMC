# ============================================================================
#  vSMC/cmake/vSMCFindRandom.cmake
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

IF (DEFINED VSMC_RANDOM_FOUND)
    RETURN ()
ENDIF (DEFINED VSMC_RANDOM_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/vSMCFindRandom.cpp
    VSMC_RANDOM_TEST_SOURCE)

INCLUDE (CheckCXXSourceRuns)
SET (SAFE_CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS})
SET (SAFE_CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES})

IF (VSMC_ENABLE_CXX11)
    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
        -DVSMC_HAS_CXX11LIB_RANDOM=1 -DVSMC_HAS_RANDOM123=0)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_RANDOM_TEST_SOURCE}"
        VSMC_RANDOM_STD_FOUND)
ENDIF (VSMC_ENABLE_CXX11)
IF (NOT VSMC_RANDOM_STD_FOUND)
    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
        -DVSMC_HAS_CXX11LIB_RANDOM=0 -DVSMC_HAS_RANDOM123=0)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_RANDOM_TEST_SOURCE}"
        VSMC_RANDOM_BOOST_FOUND)
ENDIF (NOT VSMC_RANDOM_STD_FOUND)
IF (VSMC_RANDOM_STD_FOUND OR VSMC_RANDOM_BOOST_FOUND)
    SET (VSMC_RANDOM_FOUND TRUE CACHE BOOL "Found random")
ELSE (VSMC_RANDOM_STD_FOUND OR VSMC_RANDOM_BOOST_FOUND)
    SET (VSMC_RANDOM_FOUND FALSE CACHE BOOL "NOT Found random")
ENDIF (VSMC_RANDOM_STD_FOUND OR VSMC_RANDOM_BOOST_FOUND)

INCLUDE (FindRandom123)
IF (RANDOM123_FOUND AND VSMC_RANDOM_FOUND)
    UNSET (VSMC_RANDOM123_STD_FOUND CACHE)
    UNSET (VSMC_RANDOM123_BOOST_FOUND CACHE)
    SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES}
        ${Random123_INCLUDE_DIR})
    IF (VSMC_RANDOM_STD_FOUND)
        SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
            -DVSMC_HAS_CXX11LIB_RANDOM=1 -DVSMC_HAS_RANDOM123=1)
        CHECK_CXX_SOURCE_RUNS ("${VSMC_RANDOM_TEST_SOURCE}"
            VSMC_RANDOM123_STD_FOUND)
    ENDIF (VSMC_RANDOM_STD_FOUND)
    IF (NOT VSMC_RANDOM123_STD_FOUND)
        SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
            -DVSMC_HAS_CXX11LIB_RANDOM=0 -DVSMC_HAS_RANDOM123=1)
        CHECK_CXX_SOURCE_RUNS ("${VSMC_RANDOM_TEST_SOURCE}"
            VSMC_RANDOM123_BOOST_FOUND)
    ENDIF (NOT VSMC_RANDOM123_STD_FOUND)
    IF (VSMC_RANDOM123_STD_FOUND OR VSMC_RANDOM123_BOOST_FOUND)
        SET (VSMC_RANDOM123_FOUND TRUE CACHE BOOL "Found Random123")
    ELSE (VSMC_RANDOM123_STD_FOUND OR VSMC_RANDOM123_BOOST_FOUND)
        SET (VSMC_RANDOM123_FOUND FALSE CACHE BOOL "NOT Found Random123")
    ENDIF (VSMC_RANDOM123_STD_FOUND OR VSMC_RANDOM123_BOOST_FOUND)
ENDIF (RANDOM123_FOUND AND VSMC_RANDOM_FOUND)

SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS})
SET (CMAKE_REQUIRED_INCLUDES ${SAFE_CMAKE_REQUIRED_INCLUDES})
