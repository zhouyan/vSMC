# ============================================================================
#  vSMC/example/rng/CMakeLists.txt
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

PROJECT (vSMCExample-rng CXX)
INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)

ADD_CUSTOM_TARGET (rng)
ADD_DEPENDENCIES (example rng)

ADD_CUSTOM_TARGET (rng-files)
ADD_DEPENDENCIES (example-files rng-files)

FUNCTION (ADD_RNG_TEST name)
    ADD_VSMC_EXECUTABLE (rng_${name}
        ${PROJECT_SOURCE_DIR}/src/rng_${name}.cpp)
    ADD_DEPENDENCIES (rng rng_${name})
    IF (MKL_FOUND AND VSMC_ENABLE_LIBRARY)
        ADD_VSMC_EXECUTABLE (rng_${name}_brng
            ${PROJECT_SOURCE_DIR}/src/rng_${name}.cpp)
        SET_TARGET_PROPERTIES (rng_${name}_brng PROPERTIES COMPILE_FLAGS
            "-DVSMC_RNG_TEST_BRNG=1")
        TARGET_LINK_LIBRARIES (rng_${name}_brng libvsmc)
        ADD_DEPENDENCIES (rng rng_${name}_brng)
    ENDIF (MKL_FOUND AND VSMC_ENABLE_LIBRARY)
ENDFUNCTION (ADD_RNG_TEST)

ADD_VSMC_EXECUTABLE (rng_dist ${PROJECT_SOURCE_DIR}/src/rng_dist.cpp)
ADD_DEPENDENCIES (rng rng_dist)

ADD_RNG_TEST(std)
ADD_RNG_TEST(philox)
ADD_RNG_TEST(threefry)
ADD_RNG_TEST(xorshift)

IF (AESNI_FOUND)
    ADD_RNG_TEST(aes)
    ADD_RNG_TEST(ars)
ENDIF (AESNI_FOUND)

IF (RDRAND_FOUND)
    ADD_RNG_TEST(rdrand)
ENDIF (RDRAND_FOUND)

IF (MKL_FOUND)
    ADD_RNG_TEST(mkl)
ENDIF (MKL_FOUND)

IF (OPENCL_FOUND)
    COPY_FILE (rng rng_cl.cl)
    COPY_FILE (rng rng_cl.R)
    ADD_VSMC_EXECUTABLE (rng_cl
        ${PROJECT_SOURCE_DIR}/src/rng_cl.cpp "OCL" "HDF5")
    ADD_DEPENDENCIES (rng rng_cl)
    ADD_DEPENDENCIES (example_cl rng_cl)
ENDIF (OPENCL_FOUND)
