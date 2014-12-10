# ============================================================================
#  vSMC/cmake/vSMCFindOpenMP.cmake
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

IF (DEFINED VSMC_OPENMP_FOUND)
    RETURN ()
ENDIF (DEFINED VSMC_OPENMP_FOUND)

FILE (READ ${CMAKE_CURRENT_LIST_DIR}/vSMCFindOpenMP.cpp
    VSMC_OPENMP_TEST_SOURCE)

IF (NOT DEFINED OPENMP_FOUND)
    INCLUDE (FindOpenMP)
ENDIF (NOT DEFINED OPENMP_FOUND)
IF (OPENMP_FOUND)
    INCLUDE (CheckCXXSourceRuns)
    IF (NOT MSVC AND NOT "${OpenMP_CXX_FLAGS}" STREQUAL " ")
        SET (OpenMP_LINK_LIBRARIES ${OpenMP_CXX_FLAGS}
            CACHE STRING "OpenMP Link flags")
    ELSE (NOT MSVC AND NOT "${OpenMP_CXX_FLAGS}" STREQUAL " ")
        SET (OpenMP_LINK_LIBRARIES "" CACHE STRING "OpenMP Link flags")
    ENDIF (NOT MSVC AND NOT "${OpenMP_CXX_FLAGS}" STREQUAL " ")
    SET (SAFE_CMAKE_REQUIRED_FLAGS     ${CMAKE_REQUIRED_FLAGS})
    SET (SAFE_CMAKE_REQUIRED_LIBRARIES ${CMAKE_REQUIRED_LIBRARIES})
    SET (CMAKE_REQUIRED_FLAGS ${SAFE_CMAKE_REQUIRED_FLAGS} ${OpenMP_CXX_FLAGS})
    IF (OpenMP_LINK_LIBRARIES)
        SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES}
            ${OpenMP_LINK_LIBRARIES})
    ENDIF (OpenMP_LINK_LIBRARIES)
    CHECK_CXX_SOURCE_RUNS ("${VSMC_OPENMP_TEST_SOURCE}" VSMC_OPENMP_FOUND)
    SET (CMAKE_REQUIRED_FLAGS     ${SAFE_CMAKE_REQUIRED_FLAGS})
    SET (CMAKE_REQUIRED_LIBRARIES ${SAFE_CMAKE_REQUIRED_LIBRARIES})
ENDIF (OPENMP_FOUND)
