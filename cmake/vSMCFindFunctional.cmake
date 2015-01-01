# ============================================================================
#  vSMC/cmake/vSMCFindFunctional.cmake
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

IF (DEFINED VSMC_FUNCTIONAL_FOUND)
    RETURN ()
ENDIF (DEFINED VSMC_FUNCTIONAL_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/vSMCFindFunctional.cpp
    VSMC_FUNCTIONAL_TEST_SOURCE)

INCLUDE (CheckCXXSourceRuns)
SET (SAFE_CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS})

IF (VSMC_ENABLE_CXX11)
    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
        -DVSMC_HAS_CXX11LIB_FUNCTIONAL=1)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_FUNCTIONAL_TEST_SOURCE}"
        VSMC_FUNCTIONAL_STD_FOUND)
ENDIF (VSMC_ENABLE_CXX11)

IF (NOT VSMC_FUNCTIONAL_STD_FOUND)
    SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS}
        -DVSMC_HAS_CXX11LIB_FUNCTIONAL=0)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_FUNCTIONAL_TEST_SOURCE}"
        VSMC_FUNCTIONAL_BOOST_FOUND)
ENDIF (NOT VSMC_FUNCTIONAL_STD_FOUND)

IF (VSMC_FUNCTIONAL_STD_FOUND OR VSMC_FUNCTIONAL_BOOST_FOUND)
    SET (VSMC_FUNCTIONAL_FOUND TRUE CACHE BOOL "Found functional")
ENDIF (VSMC_FUNCTIONAL_STD_FOUND OR VSMC_FUNCTIONAL_BOOST_FOUND)

SET (CMAKE_REQUIRED_DEFINITIONS ${SAFE_CMAKE_REQUIRED_DEFINITIONS})
