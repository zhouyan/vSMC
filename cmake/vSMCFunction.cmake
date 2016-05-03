# ============================================================================
#  vSMC/cmake/vSMCFunction.cmake
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

FUNCTION(VSMC_ADD_EXECUTABLE exe src)
    ADD_EXECUTABLE(${exe} ${src})

    IF(DEFINED VSMC_LINK_LIBRARIES)
        TARGET_LINK_LIBRARIES(${exe} ${VSMC_LINK_LIBRARIES})
    ENDIF(DEFINED VSMC_LINK_LIBRARIES)

    GET_TARGET_PROPERTY(compile_flags ${exe} COMPILE_FLAGS)
    IF(NOT compile_flags)
        UNSET(compile_flags)
    ENDIF(NOT compile_flags)

    GET_TARGET_PROPERTY(link_flags ${exe} LINK_FLAGS)
    IF(NOT link_flags)
        UNSET(link_flags)
    ENDIF(NOT link_flags)

    IF(OPENMP_FOUND)
        SET(compile_flags "${compile_flags} ${OpenMP_CXX_FLAGS}")
        IF(NOT MSVC)
            SET(link_flags "${link_flags} ${OpenMP_CXX_FLAGS}")
        ENDIF(NOT MSVC)
    ENDIF(OPENMP_FOUND)

    IF(compile_flags)
        SET_TARGET_PROPERTIES(${exe} PROPERTIES COMPILE_FLAGS
            "${compile_flags}")
    ENDIF(compile_flags)

    IF(link_flags)
        SET_TARGET_PROPERTIES(${exe} PROPERTIES LINK_FLAGS
            "${link_flags}")
    ENDIF(link_flags)
ENDFUNCTION(VSMC_ADD_EXECUTABLE)

FUNCTION(VSMC_ADD_EXAMPLE basename)
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

    ADD_CUSTOM_TARGET(${basename})
    ADD_DEPENDENCIES(example ${basename})

    ADD_CUSTOM_TARGET(${basename}-check)
    ADD_DEPENDENCIES(check ${basename}-check)

    ADD_CUSTOM_TARGET(${basename}-files)
    ADD_DEPENDENCIES(${basename} ${basename}-files)
ENDFUNCTION(VSMC_ADD_EXAMPLE)

FUNCTION(VSMC_ADD_TEST basename testname)
    VSMC_ADD_EXECUTABLE(${basename}_${testname}
        ${PROJECT_SOURCE_DIR}/src/${basename}_${testname}.cpp)
    ADD_DEPENDENCIES(${basename} ${basename}_${testname})
    ADD_CUSTOM_TARGET(${basename}_${testname}-check
        DEPENDS ${basename}_${testname} ${basename}-files
        COMMAND ${basename}_${testname}
        COMMENT "Running ${basename}_${testname}"
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR})
    ADD_DEPENDENCIES(${basename}-check ${basename}_${testname}-check)
ENDFUNCTION(VSMC_ADD_TEST)

FUNCTION(VSMC_ADD_FILE basename filename)
    IF(UNIX)
        ADD_CUSTOM_COMMAND(
            OUTPUT  ${PROJECT_BINARY_DIR}/${filename}
            DEPENDS ${PROJECT_SOURCE_DIR}/${filename}
            COMMAND ${CMAKE_COMMAND} ARGS -E create_symlink
            ${PROJECT_SOURCE_DIR}/${filename}
            ${PROJECT_BINARY_DIR}/${filename})
    ELSE(UNIX)
        ADD_CUSTOM_COMMAND(
            OUTPUT  ${PROJECT_BINARY_DIR}/${filename}
            DEPENDS ${PROJECT_SOURCE_DIR}/${filename}
            COMMAND ${CMAKE_COMMAND} ARGS -E copy
            ${PROJECT_SOURCE_DIR}/${filename}
            ${PROJECT_BINARY_DIR}/${filename})
    ENDIF(UNIX)
    ADD_CUSTOM_TARGET(${basename}-${filename}
        DEPENDS ${PROJECT_BINARY_DIR}/${filename})
    ADD_DEPENDENCIES(${basename}-files ${basename}-${filename})
ENDFUNCTION(VSMC_ADD_FILE)

##############################################################################
# Essential
##############################################################################

SET(CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS}
    -D__STDC_CONSTANT_MACROS)
SET(CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS}
    -D__CL_ENABLE_EXCEPTIONS)
IF(APPLE)
    SET(CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS} -U__OBJC__)
    SET(CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS} -U__OBJC2__)
ENDIF(APPLE)

SET(VSMC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
SET(CMAKE_REQUIRED_INCLUDES ${CMAKE_REQUIRED_INCLUDES} ${VSMC_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${VSMC_INCLUDE_DIR})

##############################################################################
# Check backends
##############################################################################

# Sequential
SET(BACKENDS ${BACKENDS} "Sequential")

# Standard library
SET(BACKENDS ${BACKENDS} "Standard library")

# Thread
INCLUDE(FindThread)
IF(THREAD_FOUND)
    SET(VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${Thread_LINK_LIBRARIES})
ENDIF(THREAD_FOUND)

# Intel TBB
INCLUDE(FindTBB)
IF(TBB_FOUND)
    SET(BACKENDS ${BACKENDS} "Intel TBB")
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} ${TBB_DEFINITIONS})
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_TBB=1)
    SET(VSMC_INCLUDE_DIRS ${VSMC_INCLUDE_DIRS} ${TBB_INCLUDE_DIR})
    SET(VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${TBB_LINK_LIBRARIES})
ELSE(TBB_FOUND)
    UNSET(TBB_FOUND CACHE)
    SET(TBB_FOUND FALSE CACHE BOOL "NOT Found Intel TBB")
ENDIF(TBB_FOUND)

# OpenMP
IF(NOT DEFINED OPENMP_FOUND)
    INCLUDE(FindOpenMP)
ENDIF(NOT DEFINED OPENMP_FOUND)
IF(OPENMP_FOUND)
    SET(BACKENDS ${BACKENDS} "OpenMP")
    SET(OPENMP_FOUND TRUE CACHE BOOL "Found OpenMP")
ELSE(OPENMP_FOUND)
    UNSET(OPENMP_FOUND CACHE)
    SET(OPENMP_FOUND FALSE CACHE BOOL "NOT Found OpenMP")
ENDIF(OPENMP_FOUND)

##############################################################################
# Check features
##############################################################################

SET(FEATURES)

# Boost
INCLUDE(FindBoost)
IF(Boost_FOUND)
    SET(FEATURES ${FEATURES} "Boost")
    SET(VSMC_INCLUDE_DIRS ${VSMC_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
ENDIF(Boost_FOUND)

# 128-bits integer type
INCLUDE(FindInt128)
IF(INT128_FOUND)
    SET(FEATURES ${FEATURES} "128-bits integer type")
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_INT128=1)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_INT128=${INT128_TYPE})
ELSE(INT128_FOUND)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_INT128=0)
    UNSET(INT128_FOUND)
    SET(INT128_FOUND FALSE CACHE BOOL "NOT Found int128")
ENDIF(INT128_FOUND)

# MKL
INCLUDE(FindMKL)
IF(MKL_FOUND)
    SET(FEATURES ${FEATURES} "Intel MKL")
    SET(VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${MKL_LINK_LIBRARIES})
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_MKL=1)
    SET(VSMC_INCLUDE_DIRS ${VSMC_INCLUDE_DIRS} ${MKL_INCLUDE_DIR})
ELSE(MKL_FOUND)
    UNSET(MKL_FOUND CACHE)
    SET(MKL_FOUND FALSE CACHE BOOL "NOT Found Intel MKL")
    INCLUDE(FindBLAS)
    INCLUDE(FindLAPACK)
    IF(BLAS_FOUND AND LAPACK_FOUND)
        SET(VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${BLAS_LIBRARIES})
        SET(VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${LAPCK_LIBRARIES})
    ENDIF(BLAS_FOUND AND LAPACK_FOUND)
ENDIF(MKL_FOUND)

# TBB malloc
IF(TBB_FOUND AND TBB_MALLOC_LINK_LIBRARIES)
    SET(FEATURES ${FEATURES} "Intel TBB scalable allocator")
    SET(TBB_MALLOC_FOUND TRUE CACHE BOOL "Found Intel TBB malloc")
    SET(VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES}
        ${TBB_MALLOC_LINK_LIBRARIES})
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_TBB_MALLOC=1)
ELSE(TBB_FOUND AND TBB_MALLOC_LINK_LIBRARIES)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_TBB_MALLOC=0)
    UNSET(TBB_MALLOC_FOUND CACHE)
    SET(TBB_MALLOC_FOUND FALSE CACHE BOOL "NOT Found Intel TBB malloc")
ENDIF(TBB_FOUND AND TBB_MALLOC_LINK_LIBRARIES)

# AES-NI
INCLUDE(FindAESNI)
IF(AESNI_FOUND)
    SET(FEATURES ${FEATURES} "AES-NI")
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_AES_NI=1)
ELSE(AESNI_FOUND)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_AES_NI=0)
    UNSET(AESNI_FOUND CACHE)
    SET(AESNI_FOUND FALSE CACHE BOOL "NOT Found AES-NI")
ENDIF(AESNI_FOUND)

# AVX2
INCLUDE(FindAVX2)
IF(AVX2_FOUND)
    SET(FEATURES ${FEATURES} "AVX2")
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_AVX2=1)
ELSE(AVX2_FOUND)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_AVX2=0)
    UNSET(AVX2_FOUND CACHE)
    SET(AVX2_FOUND FALSE CACHE BOOL "NOT Found AVX2")
ENDIF(AVX2_FOUND)

# SSE2
INCLUDE(FindSSE2)
IF(SSE2_FOUND)
    SET(FEATURES ${FEATURES} "SSE2")
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_SSE2=1)
ELSE(SSE2_FOUND)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_SSE2=0)
    UNSET(SSE2_FOUND CACHE)
    SET(SSE2_FOUND FALSE CACHE BOOL "NOT Found SSE2")
ENDIF(SSE2_FOUND)

# RDRAND
INCLUDE(FindRDRAND)
IF(RDRAND_FOUND)
    SET(FEATURES ${FEATURES} "RDRAND")
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_RDRAND=1)
ELSE(RDRAND_FOUND)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_RDRAND=0)
    UNSET(RDRAND_FOUND CACHE)
    SET(RDRAND_FOUND FALSE CACHE BOOL "NOT Found RDRAND")
ENDIF(RDRAND_FOUND)

# Linux librt
IF(UNIX AND NOT APPLE AND NOT DEFINED LINUX_LIBRT)
    FIND_LIBRARY(LINUX_LIBRT rt)
ENDIF(UNIX AND NOT APPLE AND NOT DEFINED LINUX_LIBRT)
IF(LINUX_LIBRT)
    SET(VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${LINUX_LIBRT})
ENDIF(LINUX_LIBRT)

# HDF5
INCLUDE(FindHDF5)
IF(HDF5_FOUND)
    SET(FEATURES ${FEATURES} "HDF5")
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -DVSMC_HAS_HDF5=1)
    SET(VSMC_INCLUDE_DIRS ${VSMC_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS})
    SET(VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${HDF5_LIBRARIES})
ELSE(HDF5_FOUND)
    UNSET(HDF5_FOUND CACHE)
    SET(HDF5_FOUND FALSE CACHE BOOL "NOT Found HDF5")
ENDIF(HDF5_FOUND)

##############################################################################
# Macros
##############################################################################

# Make sure __STDC_CONSTANT_MACROS is defined
SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -D__STDC_CONSTANT_MACROS)

# Workaround for vanilla GCC on Mac OS X
IF(APPLE)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -U__OBJC__)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -U__OBJC2__)
ENDIF(APPLE)

# Disable MSVC iterator warning
IF(MSVC)
    SET(VSMC_DEFINITIONS ${VSMC_DEFINITIONS} -D_SCL_SECURE_NO_WARNINGS)
ENDIF(MSVC)

##############################################################################
# Defs, includes, and libs
##############################################################################

ADD_DEFINITIONS(${VSMC_DEFINITIONS})
INCLUDE_DIRECTORIES(SYSTEM ${VSMC_INCLUDE_DIRS})

MESSAGE(STATUS "=================== Definitions =======================")
FOREACH(def ${VSMC_DEFINITIONS})
    MESSAGE(STATUS "${def}")
ENDFOREACH(def ${VSMC_DEFINITIONS})

MESSAGE(STATUS "=================== Includes ==========================")
FOREACH(inc ${VSMC_INCLUDE_DIRS})
    MESSAGE(STATUS "${inc}")
ENDFOREACH(inc ${VSMC_INCLUDE_DIRS})

MESSAGE(STATUS "=================== Libraries ========================")
FOREACH(lib ${VSMC_LINK_LIBRARIES})
    MESSAGE(STATUS "${lib}")
ENDFOREACH(lib ${VSMC_LINK_LIBRARIES})
