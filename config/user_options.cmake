# ============================================================================
#  vSMC/config/user_options.cmake
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

# Unix-like systems usually don't need these settings
# However, on Windows, if the PATH are not properly set, you can configure the
# following options to help CMake find vSMC's dependencies

##############################################################################

# The root directory to find Boost
# Under this directory `include/boost` and `lib` directory shall be found
SET (BOOST_ROOT "C:/Program Files/Boost" CACHE PATH "Boost ROOT")

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
