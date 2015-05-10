# ============================================================================
#  vSMC/example/CMakeLists.txt
# ----------------------------------------------------------------------------
#                          vSMC: Scalable Monte Carlo
# ----------------------------------------------------------------------------
# Copyright (c) 2013-2015, Yan Zhou
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ============================================================================

PROJECT (vSMCExample C CXX)

INCLUDE (vSMCExampleFunctions)

# Required definitions
SET (CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS}
    -D__STDC_CONSTANT_MACROS)
SET (CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS}
    -D__CL_ENABLE_EXCEPTIONS)
IF (APPLE)
    SET (CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS} -U__OBJC__)
    SET (CMAKE_REQUIRED_DEFINITIONS ${CMAKE_REQUIRED_DEFINITIONS} -U__OBJC2__)
ENDIF (APPLE)

##############################################################################
# Check backends
##############################################################################

SET (SMP_EXECUTABLES)

# Sequential
SET (BACKENDS "Sequential")
SET (SMP_EXECUTABLES ${SMP_EXECUTABLES} seq)

# Thread
INCLUDE (FindThread)
IF (THREAD_FOUND)
    SET (VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${Thread_LINK_LIBRARIES})
ENDIF (THREAD_FOUND)

# Intel TBB
INCLUDE (FindTBB)
IF (TBB_FOUND)
    SET (BACKENDS ${BACKENDS} "Intel TBB")
    SET (SMP_EXECUTABLES ${SMP_EXECUTABLES} tbb)
    ADD_DEFINITIONS (${TBB_DEFINITIONS})
    ADD_DEFINITIONS (-DVSMC_HAS_TBB=1)
    INCLUDE_DIRECTORIES (SYSTEM ${TBB_INCLUDE_DIR})
    SET (VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${TBB_LINK_LIBRARIES})
ELSE (TBB_FOUND)
    UNSET (TBB_FOUND CACHE)
    SET (TBB_FOUND FALSE CACHE BOOL "NOT Found Intel TBB")
ENDIF (TBB_FOUND)

# OpenMP
INCLUDE (vSMCFindOpenMP)
IF (VSMC_OPENMP_FOUND)
    SET (BACKENDS ${BACKENDS} "OpenMP")
    SET (SMP_EXECUTABLES ${SMP_EXECUTABLES} omp)
    ADD_DEFINITIONS (-DVSMC_HAS_OMP=1)
ELSE (VSMC_OPENMP_FOUND)
    UNSET (VSMC_OPENMP_FOUND CACHE)
    SET (VSMC_OPENMP_FOUND FALSE CACHE BOOL "NOT Found OpenMP")
ENDIF (VSMC_OPENMP_FOUND)

# OpenCL
INCLUDE (FindOpenCL)
IF (OPENCL_FOUND)
    SET (BACKENDS ${BACKENDS} "OpenCL")
    ADD_CUSTOM_TARGET (example_cl)
    ADD_DEFINITIONS (${OpenCL_DEFINITIONS})
    ADD_DEFINITIONS (-DVSMC_HAS_OPENCL=1)
    INCLUDE_DIRECTORIES (SYSTEM ${OpenCL_INCLUDE_DIR})
ELSE (OPENCL_FOUND)
    UNSET (OPENCL_FOUND CACHE)
    SET (OPENCL_FOUND FALSE CACHE BOOL "NOT Found OpenCL")
ENDIF (OPENCL_FOUND)

# MPI
INCLUDE (vSMCFindMPI)
IF (VSMC_MPI_FOUND)
    SET (BACKENDS ${BACKENDS} "MPI")
    ADD_CUSTOM_TARGET (example_mpi)
    ADD_DEFINITIONS (-DVSMC_HAS_MPI=1)
    INCLUDE_DIRECTORIES (SYSTEM ${Boost_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES (SYSTEM ${MPI_CXX_INCLUDE_PATH})
ELSE (VSMC_MPI_FOUND)
    UNSET (VSMC_MPI_FOUND CACHE)
    SET (VSMC_MPI_FOUND FALSE CACHE BOOL "NOT Found MPI")
ENDIF (VSMC_MPI_FOUND)

# SMP executables
FOREACH (smp ${SMP_EXECUTABLES})
    ADD_CUSTOM_TARGET (example_${smp})
ENDFOREACH (smp)

##############################################################################
# Check features
##############################################################################

SET (FEATURES)

# CPUID
INCLUDE (FindCPUID)
IF (CPUID_FOUND)
    SET (FEATURES ${FEATURES} "Intel x86 CPUID")
ELSE (CPUID_FOUND)
    UNSET (CPUID_FOUND CACHE)
    SET (CPUID_FOUND FALSE CACHE BOOL "NOT Found CPUID")
ENDIF (CPUID_FOUND)

# RDTCSP
INCLUDE (FindRDTSCP)
IF (RDTSCP_FOUND)
    ADD_DEFINITIONS (-DVSMC_HAS_RDTSCP=1)
    SET (FEATURES ${FEATURES} "Intel x86 RDTSCP")
ELSE (RDTSCP_FOUND)
    ADD_DEFINITIONS (-DVSMC_HAS_RDTSCP=0)
    UNSET (RDTSCP_FOUND CACHE)
    SET (RDTSCP_FOUND FALSE CACHE BOOL "NOT Found RDTSCP")
ENDIF (RDTSCP_FOUND)

# 128-bits integer type
INCLUDE (FindInt128)
IF (INT128_FOUND)
    SET (FEATURES ${FEATURES} "128-bits integer type")
    ADD_DEFINITIONS (-DVSMC_HAS_INT128=1)
    ADD_DEFINITIONS (-DVSMC_INT128=${INT128_TYPE})
ELSE (INT128_FOUND)
    ADD_DEFINITIONS (-DVSMC_HAS_INT128=0)
    UNSET (INT128_FOUND)
    SET (INT128_FOUND FALSE CACHE BOOL "NOT Found int128")
ENDIF (INT128_FOUND)

# MKL
INCLUDE (FindMKL)
IF (MKL_FOUND)
    SET (FEATURES ${FEATURES} "Intel MKL")
    SET (VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${MKL_LINK_LIBRARIES})
    ADD_DEFINITIONS (-DVSMC_HAS_MKL=1)
    INCLUDE_DIRECTORIES (SYSTEM ${MKL_INCLUDE_DIR})
ELSE (MKL_FOUND)
    UNSET (MKL_FOUND CACHE)
    SET (MKL_FOUND FALSE CACHE BOOL "NOT Found Intel MKL")
ENDIF (MKL_FOUND)

# TBB malloc
IF (TBB_FOUND AND TBB_MALLOC_LINK_LIBRARIES)
    SET (FEATURES ${FEATURES} "Intel TBB scalable allocator")
    SET (TBB_MALLOC_FOUND TRUE CACHE BOOL "Found Intel TBB malloc")
    SET (VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES}
        ${TBB_MALLOC_LINK_LIBRARIES})
    ADD_DEFINITIONS (-DVSMC_HAS_TBB_MALLOC=1)
ELSE (TBB_FOUND AND TBB_MALLOC_LINK_LIBRARIES)
    ADD_DEFINITIONS (-DVSMC_HAS_TBB_MALLOC=0)
    UNSET (TBB_MALLOC_FOUND CACHE)
    SET (TBB_MALLOC_FOUND FALSE CACHE BOOL "NOT Found Intel TBB malloc")
ENDIF (TBB_FOUND AND TBB_MALLOC_LINK_LIBRARIES)

# AES-NI
INCLUDE (FindAESNI)
IF (AESNI_FOUND)
    SET (FEATURES ${FEATURES} "AES-NI")
    ADD_DEFINITIONS (-DVSMC_HAS_AES_NI=1)
ELSE (AESNI_FOUND)
    ADD_DEFINITIONS (-DVSMC_HAS_AES_NI=0)
    UNSET (AESNI_FOUND CACHE)
    SET (AESNI_FOUND FALSE CACHE BOOL "NOT Found AES-NI")
ENDIF (AESNI_FOUND)

# AVX2
INCLUDE (FindAVX2)
IF (AVX2_FOUND)
    SET (FEATURES ${FEATURES} "AVX2")
    ADD_DEFINITIONS (-DVSMC_HAS_AVX2=1)
ELSE (AVX2_FOUND)
    ADD_DEFINITIONS (-DVSMC_HAS_AVX2=0)
    UNSET (AVX2_FOUND CACHE)
    SET (AVX2_FOUND FALSE CACHE BOOL "NOT Found AVX2")
ENDIF (AVX2_FOUND)

# SSE2
INCLUDE (FindSSE2)
IF (SSE2_FOUND)
    SET (FEATURES ${FEATURES} "SSE2")
    ADD_DEFINITIONS (-DVSMC_HAS_SSE2=1)
ELSE (SSE2_FOUND)
    ADD_DEFINITIONS (-DVSMC_HAS_SSE2=0)
    UNSET (SSE2_FOUND CACHE)
    SET (SSE2_FOUND FALSE CACHE BOOL "NOT Found SSE2")
ENDIF (SSE2_FOUND)

# RDRAND
INCLUDE (FindRDRAND)
IF (RDRAND_FOUND)
    SET (FEATURES ${FEATURES} "RDRAND")
    ADD_DEFINITIONS (-DVSMC_HAS_RDRAND=1)
ELSE (RDRAND_FOUND)
    ADD_DEFINITIONS (-DVSMC_HAS_RDRAND=0)
    UNSET (RDRAND_FOUND CACHE)
    SET (RDRAND_FOUND FALSE CACHE BOOL "NOT Found RDRAND")
ENDIF (RDRAND_FOUND)

# Linux librt
IF (UNIX AND NOT APPLE AND NOT DEFINED LINUX_LIBRT)
    FIND_LIBRARY (LINUX_LIBRT rt)
ENDIF (UNIX AND NOT APPLE AND NOT DEFINED LINUX_LIBRT)
IF (LINUX_LIBRT)
    SET (VSMC_LINK_LIBRARIES ${VSMC_LINK_LIBRARIES} ${LINUX_LIBRT})
ENDIF (LINUX_LIBRT)

# HDF5
INCLUDE (FindHDF5)
IF (HDF5_FOUND)
    SET (FEATURES ${FEATURES} "HDF5")
    ADD_DEFINITIONS (-DVSMC_HAS_HDF5=1)
    INCLUDE_DIRECTORIES (SYSTEM ${HDF5_INCLUDE_DIRS})
ELSE (HDF5_FOUND)
    UNSET (HDF5_FOUND CACHE)
    SET (HDF5_FOUND FALSE CACHE BOOL "NOT Found HDF5")
ENDIF (HDF5_FOUND)

##############################################################################
# Macros
##############################################################################

# Make sure __STDC_CONSTANT_MACROS is defined
ADD_DEFINITIONS (-D__STDC_CONSTANT_MACROS)

# Make sure __CL_ENABLE_EXCEPTIONS is defined if using OpenCL
IF (OPENCL_FOUND)
    ADD_DEFINITIONS (-D__CL_ENABLE_EXCEPTIONS)
ENDIF (OPENCL_FOUND)

# Workaround for vanilla GCC on Mac OS X
IF (APPLE)
    ADD_DEFINITIONS (-U__OBJC__)
    ADD_DEFINITIONS (-U__OBJC2__)
ENDIF (APPLE)

# Disable MSVC iterator warning
IF (MSVC)
    ADD_DEFINITIONS (-D_SCL_SECURE_NO_WARNINGS)
ENDIF (MSVC)

ADD_DEFINITIONS (${EXAMPLE_DEFINITIONS})

##############################################################################
# Check examples
##############################################################################

ADD_CUSTOM_TARGET (example-files)
ADD_DEPENDENCIES (example example-files)

SET (EXAMPLES ${EXAMPLES} "vsmc")
ADD_SUBDIRECTORY (vsmc)

SET (EXAMPLES ${EXAMPLES} "paper")
ADD_SUBDIRECTORY (paper)

SET (EXAMPLES ${EXAMPLES} "pf")
ADD_SUBDIRECTORY (pf)

SET (EXAMPLES ${EXAMPLES} "rng")
ADD_SUBDIRECTORY (rng)

##############################################################################
# Enable backends
##############################################################################

MESSAGE (STATUS "=================== Enable backends ===================")
FOREACH (backend ${BACKENDS})
    MESSAGE (STATUS ${backend})
ENDFOREACH (backend ${BACKENDS})

##############################################################################
# Enable features
##############################################################################

MESSAGE (STATUS "=================== Enable features ===================")
FOREACH (feature ${FEATURES})
    MESSAGE (STATUS ${feature})
ENDFOREACH (feature ${FEATURES})

##############################################################################
# Enable examples
##############################################################################

MESSAGE (STATUS "=================== Enable examples ===================")
FOREACH (example ${EXAMPLES})
    MESSAGE (STATUS ${example})
ENDFOREACH (example ${EXAMPLES})

MESSAGE (STATUS "=======================================================")
