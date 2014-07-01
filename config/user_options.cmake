# ============================================================================
#  user_options_sample.cmake
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

# The Root directory to find Boost
# Under this directory `include/boost` and `lib` directory shall be found
SET (BOOST_ROOT "C:/Program Files/Boost" CACHE PATH "Boost ROOT")
# If BOOST_ROOT is set, one may also want to turn on Boost_NO_SYSTEM_PATHS
SET (Boost_NO_SYSTEM_PATHS OFF CACHE BOOL "Do not use system Boost search")
# Use Boost static libraries (vSMC itself does not use Boost runtime)
SET (Boost_USE_STATIC_LIBS ON CACHE BOOL "Boost use static libraries")
SET (Boost_USE_STATIC_RUNTIME ON CACHE BOOL "Boost use static runtime")

# For additional Boost related variables see the CMake official FindBoost.cmake

##############################################################################

# The inlude directory of Random123
# Under this directory `Random123/threefry.h` etc shall be found
SET (Random123_INC_PATH "C:/Program Files/Random123/include"
    CACHE PATH "Random123 include path")

##############################################################################

# The Root director to find Intel Threading Building Blocks
# Under this directory, on Windows, `bin`, `include` and `lib` shall be found
SET (TBB_ROOT "C:/Program Files/Intel/TBB/4.1" CACHE PATH "TBB ROOT")

# For additional TBB related variables see the vSMC's cmake/FindTBB.cmake

##############################################################################

# The path to find OpenCL/opencl.h (Apple) or CL/opencl.h (others)
SET (OpenCL_INC_PATH "C:/Program Files (x86)/Intel/OpenCL SDK/3.0/include"
    CACHE PATH "OpenCL include path")

# The path to libOpenCL.so (Unix like) or OpenCL.lib (Windows)
SET (OpenCL_LIB_PATH "C:/Program Files (x86)/Intel/OpenCL SDK/3.0/lib/x64"
    CACHE PATH "OpenCL lib path")

##############################################################################

# Enable MPI
SET (VSMC_ENABLE_MPI ON CACHE BOOL "Use MPI")
# The path to mpiexec
SET (VSMC_MPIEXEC "/usr/bin/mpiexec" CACHE STRING "mpiexec")

##############################################################################

# Disable checking features
SET (RD_RAND_FOUND FALSE CACHE BOOL "NO RdRand")
SET (RD_SEED_FOUND FALSE CACHE BOOL "NO RdSeed")
SET (AESNI_FOUND   FALSE CACHE BOOL "NO AES-NI")
