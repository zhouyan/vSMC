# ============================================================================
#  vSMC/lib/CMakeLists.txt
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
#  Copyright (c) 2013-2016, Yan Zhou
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
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
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

PROJECT(vSMCLib CXX)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

SET(VSMC_LIB_SOURCE_VSMCRNG ${PROJECT_SOURCE_DIR}/src/rng/rng.cpp)

SET(VSMC_LIB_SOURCE_VSMC ${VSMC_LIB_SOURCE_VSMCRNG}
    ${PROJECT_SOURCE_DIR}/src/core/monitor.cpp
    ${PROJECT_SOURCE_DIR}/src/core/particle.cpp
    ${PROJECT_SOURCE_DIR}/src/core/sampler.cpp
    ${PROJECT_SOURCE_DIR}/src/core/state_matrix.cpp
    ${PROJECT_SOURCE_DIR}/src/core/weight.cpp
    ${PROJECT_SOURCE_DIR}/src/resample/resample.cpp
    ${PROJECT_SOURCE_DIR}/src/smp/backend_seq.cpp
    ${PROJECT_SOURCE_DIR}/src/smp/backend_std.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/aligned_memory.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/covariance.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/hdf5.cpp
    ${PROJECT_SOURCE_DIR}/src/utility/stop_watch.cpp)

IF(OPENMP_FOUND)
    SET(VSMC_LIB_SOURCE_VSMC ${VSMC_LIB_SOURCE_VSMC}
        ${PROJECT_SOURCE_DIR}/src/smp/backend_omp.cpp)
ENDIF(OPENMP_FOUND)

IF(TBB_FOUND)
    SET(VSMC_LIB_SOURCE_VSMC ${VSMC_LIB_SOURCE_VSMC}
        ${PROJECT_SOURCE_DIR}/src/smp/backend_tbb.cpp)
ENDIF(TBB_FOUND)

SET(VSMC_LIB_TYPE shared static)
SET(VSMC_LIB_NAME vsmc vsmcrng)

IF(NOT DEFINED VSMC_INSTALL_LIB_DIR)
    IF(EXISTS ${CMAKE_INSTALL_PREFIX}/lib64)
        SET(VSMC_INSTALL_LIB_DIR lib64)
    ELSE(EXISTS ${CMAKE_INSTALL_PREFIX}/lib64)
        SET(VSMC_INSTALL_LIB_DIR lib)
    ENDIF(EXISTS ${CMAKE_INSTALL_PREFIX}/lib64)
ENDIF(NOT DEFINED VSMC_INSTALL_LIB_DIR)

FOREACH(name ${VSMC_LIB_NAME})
    ADD_CUSTOM_TARGET(lib${name})
    ADD_DEPENDENCIES(lib lib${name})
    FOREACH(type ${VSMC_LIB_TYPE})
        STRING(TOUPPER "${name}" NAME)
        STRING(TOUPPER "${type}" TYPE)
        ADD_LIBRARY(lib${name}_${type} ${TYPE} ${VSMC_LIB_SOURCE_${NAME}})
        SET_TARGET_PROPERTIES(lib${name}_${type}
            PROPERTIES OUTPUT_NAME ${name})
        VSMC_LINK_TARGET(lib${name}_${type})
        ADD_DEPENDENCIES(lib${name} lib${name}_${type})
        INSTALL(TARGETS lib${name}_${type} DESTINATION ${VSMC_INSTALL_LIB_DIR})
    ENDFOREACH(type ${VSMC_LIB_TYPE})
ENDFOREACH(name ${VSMC_LIB_NAME})
