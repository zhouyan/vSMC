# ============================================================================
#  config/user_options_sample.cmake
# ----------------------------------------------------------------------------
#
#                          vSMC: Scalable Monte Carlo
#
#  This file is distribured under the 2-clauses BSD License.
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
SET (TBB_ROOT "C:/Program Files/Intel/TBB/4/2")
SET (TBB_INC_PATH "$TBB_ROOT/include" CACHE PATH "TBB include")
IF (MSVC_VERSION GREATER 1800)
    SET (VSMC_LIB_PATH "$TBB_ROOT/lib/vc12" CACHE PATH "TBB lib")
ELSEIF (MSVC_VERSION GREATER 1700)
    SET (VSMC_LIB_PATH "$TBB_ROOT/lib/vc11" CACHE PATH "TBB lib")
ELSEIF (MSVC_VERSION GREATER 1600)
    SET (VSMC_LIB_PATH "$TBB_ROOT/lib/vc10" CACHE PATH "TBB lib")
ENDIF (MSVC_VERSION GREATER 1800)

##############################################################################

# Paths to Intel OpenCL
SET (OpenCL_INC_PATH "C:/Program Files (x86)/Intel/OpenCL SDK/3.2/include"
    CACHE PATH "OpenCL include path")
SET (OpenCL_LIB_PATH "C:/Program Files (x86)/Intel/OpenCL SDK/3.2/lib/x64"
    CACHE PATH "OpenCL lib path")

##############################################################################

# Enable/Disable checking features
SET (RD_RAND_FOUND FALSE CACHE BOOL "NO RdRand")
SET (RD_SEED_FOUND FALSE CACHE BOOL "NO RdSeed")
SET (AESNI_FOUND   FALSE CACHE BOOL "NO AES-NI")
SET (INT128_FOUND  FALSE CACHE BOOL "NO int128")
SET (VSMC_CONSTEXPR_ENGINE_FOUND FALSE CACHE BOOL "NO constexpr min/max")
SET (VSMC_ENABLE_CXX11 ON CACHE BOOL "Try C++11")
SET (VSMC_ENABLE_CXX11LIB_THREAD ON CACHE BOOL "Try C++11 <thread>")
IF (MSVC_VERSION GREATER 1700)
    SET (VSMC_ENABLE_CXX11LIB_FUTURE ON CACHE BOOL "Try C++11 <future>")
ENDIF (MSVC_VERSION GREATER 1700)
