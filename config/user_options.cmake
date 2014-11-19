# ============================================================================
#  vSMC/config/user_options.cmake
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

# Unix-like systems usually don't need these settings
# However, on Windows, if the PATH are not properly set, you can configure the
# following options to help CMake find vSMC's dependencies

##############################################################################

# The root directory to find Boost
# Under this directory `include/boost` and `lib` directory shall be found
SET (BOOST_ROOT "C:/Program Files/Boost" CACHE PATH "Boost ROOT")

##############################################################################

# The inlude directory of Random123
# Under this directory `Random123/threefry.h` etc shall be found
SET (Random123_INC_PATH "C:/Program Files/Random123/include"
    CACHE PATH "Random123 include path")

##############################################################################

# Paths to Intel TBB
SET (TBB_ROOT "C:/Program Files/Intel/TBB/4.2")
SET (TBB_INC_PATH "${TBB_ROOT}/include" CACHE PATH "TBB include")
IF (MSVC_VERSION EQUAL 1600)
    SET (TBB_LIB_PATH "${TBB_ROOT}/lib/intel64/vc10" CACHE PATH "TBB lib")
ELSEIF (MSVC_VERSION EQUAL 1700)
    SET (TBB_LIB_PATH "${TBB_ROOT}/lib/intel64/vc11" CACHE PATH "TBB lib")
ELSEIF (MSVC_VERSION EQUAL 1800)
    SET (TBB_LIB_PATH "${TBB_ROOT}/lib/intel64/vc12" CACHE PATH "TBB lib")
ENDIF (MSVC_VERSION EQUAL 1600)

##############################################################################

# Paths to Intel OpenCL
SET (OpenCL_INC_PATH "C:/Program Files (x86)/Intel/OpenCL SDK/3.2/include"
    CACHE PATH "OpenCL include path")
SET (OpenCL_LIB_PATH "C:/Program Files (x86)/Intel/OpenCL SDK/3.2/lib/x64"
    CACHE PATH "OpenCL lib path")

##############################################################################

# Disable checking features
SET (AESNI_FOUND   FALSE CACHE BOOL "NO AES-NI")
SET (GCD_FOUND     FALSE CACHE BOOL "NO GCD")
SET (GSL_FOUND     FALSE CACHE BOOL "NO GSL")
SET (INT128_FOUND  FALSE CACHE BOOL "NO int128")
SET (MKL_FOUND     FALSE CACHE BOOL "NO MKL")
SET (RDRAND_FOUND  FALSE CACHE BOOL "NO RDRAND")
SET (TESTU01_FOUND FALSE CACHE BOOL "NO TestU01")

SET (VSMC_ENABLE_CXX11 ON CACHE BOOL "Try C++11")

IF (MSVC_VERSION LESS 1700)
    SET (VSMC_RANDOM_STD_FOUND FALSE CACHE BOOL "NO C++11 <random>")
    SET (CXX11LIB_THREAD_FOUND FALSE CACHE BOOL "NO C++11 <thread>")
ENDIF (MSVC_VERSION LESS 1700)

IF (MSVC_VERSION LESS 1800)
    SET (CXX11LIB_TUPLE_FOUND      FALSE CACHE BOOL "NO C++11 <tuple>")
    SET (VSMC_CXX11_MATH_STD_FOUND FALSE CACHE BOOL "NO C++11 <cmath>")
    SET (VSMC_CXX11_MATH_C99_FOUND FALSE CACHE BOOL "NO C99 <math.h>")
ENDIF (MSVC_VERSION LESS 1800)
