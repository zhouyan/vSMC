# ============================================================================
#  config/user_options_sample.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distributed under the 2-clauses BSD License.
#  See LICENSE for details.
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
